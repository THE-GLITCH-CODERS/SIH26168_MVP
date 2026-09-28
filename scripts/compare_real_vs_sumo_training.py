"""Compare real-only and real-plus-SUMO speed-delta models fairly.

Both models use the same train-only IO-VNBD data and identical real-only
validation/test drives. Synthetic data is rejected if it contains a non-train
split. This evaluates speed-change prediction only; promotion still requires
the identical held-out real GNSS-outage trajectory replay.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_speed_delta_v1 import feature_names, metrics, motion_channels, segments, window_features

REAL_DEFAULT = ROOT / "data/processed/iovnbd/supervised_drives_v0.npz"
OUT_DEFAULT = ROOT / "outputs/speed_delta_real_sumo_compare"


def make_examples(
    data: np.lib.npyio.NpzFile,
    *,
    window_samples: int,
    horizon_seconds: float,
    max_gap_seconds: float,
    synthetic: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    raw = data["X"]
    speed = data["y_speed_mps"].astype(np.float64)
    elapsed = data["elapsed_s"].astype(np.float64)
    drives = data["drive_index"]
    split_codes = data["split_code"]
    if raw.ndim != 2 or raw.shape[1] != 12 or not (len(raw) == len(speed) == len(elapsed) == len(drives) == len(split_codes)):
        raise SystemExit("Dataset arrays must align and X must contain the expected 12 IMU channels")
    if synthetic and np.any(split_codes != 0):
        raise SystemExit("Synthetic dataset must be training-only (split_code 0 for every row)")

    all_x, all_y, all_split, all_drive, rates = [], [], [], [], []
    for drive in np.unique(drives):
        indices = np.flatnonzero(drives == drive)
        split = np.unique(split_codes[indices])
        if len(split) != 1 or split[0] not in (0, 1, 2):
            raise SystemExit(f"Drive {drive} has an invalid or inconsistent split")
        if synthetic and split[0] != 0:
            raise SystemExit("Synthetic samples may only participate in model fitting")
        for begin, end in segments(elapsed[indices], max_gap_seconds):
            rows = indices[begin:end]
            if len(rows) <= window_samples + 1:
                continue
            times = elapsed[rows]
            sample_dt = float(np.median(np.diff(times)))
            if not np.isfinite(sample_dt) or sample_dt <= 0:
                continue
            rates.append(1.0 / sample_dt)
            window_ends = np.arange(window_samples - 1, len(rows) - 1)
            future_time = times[window_ends] + horizon_seconds
            future = np.searchsorted(times, future_time, side="left")
            valid = future < len(rows)
            window_ends, future = window_ends[valid], future[valid]
            tolerance = max(sample_dt * 0.75, 0.03)
            valid = times[future] - times[window_ends] <= horizon_seconds + tolerance
            window_ends, future = window_ends[valid], future[valid]
            if not len(window_ends):
                continue
            summaries = window_features(motion_channels(raw[rows]), window_samples)
            x = summaries[window_ends - (window_samples - 1)]
            y = speed[rows[future]] - speed[rows[window_ends]]
            good = np.isfinite(y) & np.isfinite(x).all(axis=1)
            all_x.append(x[good])
            all_y.append(y[good])
            # Use a disjoint ID range so real and simulated drive counts are clear.
            drive_offset = 1_000_000 if synthetic else 0
            all_drive.append(np.full(int(good.sum()), int(drive) + drive_offset, dtype=np.int64))
            split_value = 0 if synthetic else int(split[0])
            all_split.append(np.full(int(good.sum()), split_value, dtype=np.uint8))
    if not all_x:
        raise SystemExit("Dataset did not produce any complete past-window/future-horizon examples")
    return (
        np.concatenate(all_x).astype(np.float32, copy=False),
        np.concatenate(all_y).astype(np.float64, copy=False),
        np.concatenate(all_split),
        np.concatenate(all_drive),
        float(np.median(rates)),
    )


def fit_and_score(
    train_x: np.ndarray,
    train_y: np.ndarray,
    real_x: np.ndarray,
    real_y: np.ndarray,
    real_split: np.ndarray,
    real_drives: np.ndarray,
    *,
    alpha: float,
    horizon_seconds: float,
    window_samples: int,
    feature_rate_hz: float,
) -> tuple[dict, dict]:
    mean = train_x.mean(axis=0, dtype=np.float64)
    scale = np.sqrt(np.mean((train_x.astype(np.float64) - mean) ** 2, axis=0))
    scale[scale < 1e-6] = 1.0
    target_mean = float(train_y.mean())
    standardized = (train_x.astype(np.float64) - mean) / scale
    centered = train_y - target_mean
    gram = standardized.T @ standardized
    rhs = standardized.T @ centered
    gram.flat[:: len(gram) + 1] += alpha
    coefficients = np.linalg.solve(gram, rhs)
    prediction = ((real_x.astype(np.float64) - mean) / scale) @ coefficients + target_mean
    prediction = np.clip(prediction, -4.0 * horizon_seconds, 4.0 * horizon_seconds)

    results = {}
    for name, code in (("train_real_diagnostic", 0), ("validation_real", 1), ("test_real", 2)):
        mask = real_split == code
        if not mask.any():
            if code == 0:
                continue
            raise SystemExit(f"The real dataset has no {name} samples")
        results[name] = {
            "real_drives": int(len(np.unique(real_drives[mask]))),
            "model": metrics(real_y[mask], prediction[mask]),
            "zero_delta_baseline": metrics(real_y[mask], np.zeros(int(mask.sum()))),
        }
    validation = real_split == 1
    validation_variance = float(np.mean((prediction[validation] - real_y[validation]) ** 2))
    model = {
        "format": "seamlessnav-linear-speed-delta-v1",
        "feature_names": feature_names(),
        "feature_mean": mean.tolist(),
        "feature_scale": scale.tolist(),
        "coefficients": coefficients.tolist(),
        "target_mean_mps": target_mean,
        "residual_variance_m2ps2": validation_variance,
        "window_samples": window_samples,
        "feature_rate_hz": feature_rate_hz,
        "horizon_seconds": horizon_seconds,
        "prediction_clip_mps": 4.0 * horizon_seconds,
        "eligible_for_fusion": False,
        "activation_note": "Speed score is not navigation proof. Require held-out real GNSS-outage trajectory improvement before fusion.",
    }
    return results, model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-dataset", type=Path, default=REAL_DEFAULT)
    parser.add_argument("--synthetic-dataset", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--window-samples", type=int, default=20)
    parser.add_argument("--horizon-seconds", type=float, default=1.0)
    parser.add_argument("--max-gap-seconds", type=float, default=0.2)
    parser.add_argument("--ridge-alpha", type=float, default=10.0)
    args = parser.parse_args()
    for path in (args.real_dataset, args.synthetic_dataset):
        if not path.exists():
            raise SystemExit(f"Dataset not found: {path}")
    real_data = np.load(args.real_dataset)
    synthetic_data = np.load(args.synthetic_dataset)
    real_x, real_y, real_split, real_drive, real_rate_hz = make_examples(
        real_data, window_samples=args.window_samples,
        horizon_seconds=args.horizon_seconds, max_gap_seconds=args.max_gap_seconds,
        synthetic=False,
    )
    sim_x, sim_y, sim_split, sim_drive, sim_rate_hz = make_examples(
        synthetic_data, window_samples=args.window_samples,
        horizon_seconds=args.horizon_seconds, max_gap_seconds=args.max_gap_seconds,
        synthetic=True,
    )
    if abs(sim_rate_hz - real_rate_hz) / real_rate_hz > 0.05:
        raise SystemExit(
            f"Feature rates differ: IO-VNBD {real_rate_hz:.3f} Hz vs synthetic {sim_rate_hz:.3f} Hz; "
            "regenerate SUMO features at the same cadence before comparing models"
        )
    train_real = real_split == 0
    if not train_real.any() or not (real_split == 1).any() or not (real_split == 2).any():
        raise SystemExit("Real data must include train, validation and held-out test drives")
    models = {
        "real_only": (real_x[train_real], real_y[train_real]),
        "real_plus_sumo": (np.concatenate((real_x[train_real], sim_x[sim_split == 0])), np.concatenate((real_y[train_real], sim_y[sim_split == 0]))),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "comparison": "real-only vs real-plus-SUMO synthetic training",
        "real_dataset": str(args.real_dataset.resolve()),
        "synthetic_dataset": str(args.synthetic_dataset.resolve()),
        "validation_and_test": "IO-VNBD real drives only; synthetic rows are never evaluated",
        "real_train_drives": int(len(np.unique(real_drive[train_real]))),
        "synthetic_train_drives": int(len(np.unique(sim_drive[sim_split == 0]))),
        "feature_rate_hz_median_real_and_synthetic": real_rate_hz,
        "window_duration_seconds_approx": args.window_samples / real_rate_hz,
        "test_is_used_for_model_selection": False,
        "models": {},
        "promotion": "Do not fuse based on these speed metrics alone; compare both on identical held-out real GNSS-outage windows.",
    }
    for name, (fit_x, fit_y) in models.items():
        results, model = fit_and_score(
            fit_x, fit_y, real_x, real_y, real_split, real_drive,
            alpha=args.ridge_alpha, horizon_seconds=args.horizon_seconds,
            window_samples=args.window_samples, feature_rate_hz=real_rate_hz,
        )
        out_dir = args.output_dir / name
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "speed_delta_v1.json").write_text(json.dumps(model, indent=2), encoding="utf-8")
        (out_dir / "metrics.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
        summary["models"][name] = results
    summary_path = args.output_dir / "comparison.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved comparison: {summary_path.resolve()}")


if __name__ == "__main__":
    main()
