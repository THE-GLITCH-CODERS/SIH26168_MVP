"""Train/evaluate a drive-held-out, past-IMU short-horizon speed-change model.

Vehicle speed is used only as the supervised target. Features use the same
past-only 2-second IMU window as speed_baseline_v0. The target is the change
from the window's final speed to speed one second later. This is an offline
candidate artifact, not a claim of deployable navigation accuracy.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA_DEFAULT = ROOT / "data/processed/iovnbd/supervised_drives_v0.npz"
OUT_DEFAULT = ROOT / "outputs/speed_delta_v1"


def motion_channels(raw: np.ndarray) -> np.ndarray:
    norms = np.column_stack((
        np.linalg.norm(raw[:, 0:3], axis=1),
        np.linalg.norm(raw[:, 6:9], axis=1),
        np.linalg.norm(raw[:, 9:12], axis=1),
    ))
    return np.concatenate((raw, norms), axis=1)


def feature_names() -> list[str]:
    channels = [
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
        "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
        "gyro_yaw_rps", "gyro_pitch_rps", "gyro_roll_rps",
        "accel_norm_mps2", "linear_accel_norm_mps2", "gyro_norm_rps",
    ]
    return [f"{stat}_{name}" for stat in ("mean", "std", "min", "max", "last") for name in channels]


def window_features(expanded: np.ndarray, window: int, batch_size: int = 4096) -> np.ndarray:
    """Bound temporary memory while producing summaries for every rolling window."""
    rows = len(expanded) - window + 1
    result = np.empty((rows, expanded.shape[1] * 5), dtype=np.float32)
    view = np.lib.stride_tricks.sliding_window_view(expanded, window_shape=window, axis=0)
    for start in range(0, rows, batch_size):
        end = min(rows, start + batch_size)
        windows = view[start:end].transpose(0, 2, 1)
        result[start:end] = np.concatenate((
            windows.mean(axis=1), windows.std(axis=1), windows.min(axis=1),
            windows.max(axis=1), windows[:, -1, :],
        ), axis=1)
    return result


def segments(elapsed: np.ndarray, max_gap: float) -> list[tuple[int, int]]:
    breaks = np.flatnonzero((np.diff(elapsed) <= 0.0) | (np.diff(elapsed) > max_gap)) + 1
    bounds = np.concatenate(([0], breaks, [len(elapsed)]))
    return [(int(bounds[i]), int(bounds[i + 1])) for i in range(len(bounds) - 1)]


def metrics(truth: np.ndarray, prediction: np.ndarray) -> dict:
    error = prediction - truth
    return {
        "samples": int(len(truth)),
        "mae_mps": float(np.mean(np.abs(error))),
        "rmse_mps": float(np.sqrt(np.mean(error**2))),
        "mean_error_mps": float(np.mean(error)),
        "truth_std_mps": float(np.std(truth)),
        "prediction_std_mps": float(np.std(prediction)),
        "correlation": float(np.corrcoef(truth, prediction)[0, 1]) if np.std(truth) > 1e-9 and np.std(prediction) > 1e-9 else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DATA_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--window-samples", type=int, default=20)
    parser.add_argument("--horizon-seconds", type=float, default=1.0)
    parser.add_argument("--max-gap-seconds", type=float, default=0.2)
    parser.add_argument("--ridge-alpha", type=float, default=10.0)
    args = parser.parse_args()
    if args.window_samples < 2 or args.horizon_seconds <= 0:
        raise SystemExit("window and horizon must be positive")
    if not args.dataset.exists():
        raise SystemExit(f"Dataset not found: {args.dataset}")

    data = np.load(args.dataset)
    raw, speed = data["X"], data["y_speed_mps"].astype(np.float64)
    elapsed, drives, split_codes = data["elapsed_s"], data["drive_index"], data["split_code"]
    all_x, all_y, all_split, all_drive = [], [], [], []
    for drive in np.unique(drives):
        indices = np.flatnonzero(drives == drive)
        split_values = np.unique(split_codes[indices])
        if len(split_values) != 1 or split_values[0] == 255:
            raise SystemExit(f"Drive {drive} has inconsistent or excluded split codes")
        for begin, end in segments(elapsed[indices], args.max_gap_seconds):
            row_indices = indices[begin:end]
            if len(row_indices) <= args.window_samples + 1:
                continue
            times = elapsed[row_indices]
            # Preserve the existing IO-VNBD drive split and avoid crossing gaps.
            sample_dt = float(np.median(np.diff(times)))
            if not np.isfinite(sample_dt) or sample_dt <= 0:
                continue
            horizon_samples = max(1, int(round(args.horizon_seconds / sample_dt)))
            current_end = np.arange(args.window_samples - 1, len(row_indices) - horizon_samples)
            if len(current_end) == 0:
                continue
            future_time = times[current_end] + args.horizon_seconds
            future_index = np.searchsorted(times, future_time, side="left")
            valid = future_index < len(row_indices)
            current_end, future_index = current_end[valid], future_index[valid]
            # The time target may not be exactly one row ahead on jittered logs.
            valid = (times[future_index] - times[current_end] <= args.horizon_seconds + max(sample_dt * 0.75, 0.03))
            current_end, future_index = current_end[valid], future_index[valid]
            if len(current_end) == 0:
                continue
            expanded = motion_channels(raw[row_indices])
            features = window_features(expanded, args.window_samples)
            window_row = current_end - (args.window_samples - 1)
            x = features[window_row]
            y = speed[row_indices[future_index]] - speed[row_indices[current_end]]
            valid = np.isfinite(y) & np.isfinite(x).all(axis=1)
            all_x.append(x[valid])
            all_y.append(y[valid])
            all_split.append(np.full(int(valid.sum()), split_values[0], dtype=np.uint8))
            all_drive.append(np.full(int(valid.sum()), drive, dtype=np.uint16))

    X = np.concatenate(all_x).astype(np.float32, copy=False)
    target = np.concatenate(all_y).astype(np.float64, copy=False)
    split, drive_ids = np.concatenate(all_split), np.concatenate(all_drive)
    train, validation, test = split == 0, split == 1, split == 2
    if not train.any() or not validation.any() or not test.any():
        raise SystemExit("Expected non-empty drive-held-out train, validation and test sets")

    mean = X[train].mean(axis=0, dtype=np.float64)
    scale = np.sqrt(np.mean((X[train].astype(np.float64) - mean) ** 2, axis=0))
    scale[scale < 1e-6] = 1.0
    target_mean = float(target[train].mean())
    standardized = (X[train].astype(np.float64) - mean) / scale
    centered = target[train] - target_mean
    gram = standardized.T @ standardized
    rhs = standardized.T @ centered
    gram.flat[:: len(gram) + 1] += args.ridge_alpha
    coefficients = np.linalg.solve(gram, rhs)
    prediction = ((X.astype(np.float64) - mean) / scale) @ coefficients + target_mean
    # A causal speed-change head is bounded so it cannot inject a huge single-step jump.
    prediction = np.clip(prediction, -4.0 * args.horizon_seconds, 4.0 * args.horizon_seconds)
    results = {}
    for name, mask in (("train", train), ("validation", validation), ("test", test)):
        results[name] = {
            "drives": int(len(np.unique(drive_ids[mask]))),
            "model": metrics(target[mask], prediction[mask]),
            "zero_speed_change_baseline": metrics(target[mask], np.zeros(int(mask.sum()))),
        }
    # Only validation residuals set the model's fixed uncertainty; held-out test remains untouched.
    residual_var = float(np.mean((prediction[validation] - target[validation]) ** 2))
    test_beats_zero = results["test"]["model"]["rmse_mps"] < results["test"]["zero_speed_change_baseline"]["rmse_mps"]
    metrics_doc = {
        "model": "past-only ridge speed-change candidate",
        "target": f"vehicle speed at t+{args.horizon_seconds:g}s minus speed at t, m/s",
        "input_features": "phone IMU only: accelerometer, gravity, linear acceleration, gyroscope and derived norms",
        "window_samples": args.window_samples,
        "horizon_seconds": args.horizon_seconds,
        "ridge_alpha": args.ridge_alpha,
        "train_drives": int(len(np.unique(drive_ids[train]))),
        "validation_drives": int(len(np.unique(drive_ids[validation]))),
        "test_drives": int(len(np.unique(drive_ids[test]))),
        "splits": results,
        "validation_residual_variance_m2ps2": residual_var,
        "heldout_test_beats_zero_change_baseline": bool(test_beats_zero),
        "deployment_status": "candidate_only; do not fuse unless drive-held-out and integrated outage results justify it",
        "limitations": [
            "IO-VNBD phone and vehicle clocks/labels are imperfect and the vehicle GNSS speed is not surveyed truth.",
            "Test set contains only its preassigned four drives; transfer to other phones and mounts is unproven.",
            "A lower speed-change MAE does not establish lower position drift; evaluate integrated outages separately.",
        ],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model_doc = {
        "format": "seamlessnav-linear-speed-delta-v1",
        "feature_names": feature_names(),
        "feature_mean": mean.tolist(),
        "feature_scale": scale.tolist(),
        "coefficients": coefficients.tolist(),
        "target_mean_mps": target_mean,
        "residual_variance_m2ps2": residual_var,
        "window_samples": args.window_samples,
        "horizon_seconds": args.horizon_seconds,
        "prediction_clip_mps": 4.0 * args.horizon_seconds,
        "eligible_for_fusion": False,
        "activation_note": "Requires improved held-out regression and integrated outage evaluation before any estimator fusion.",
    }
    (args.output_dir / "speed_delta_v1.json").write_text(json.dumps(model_doc, indent=2), encoding="utf-8")
    (args.output_dir / "metrics.json").write_text(json.dumps(metrics_doc, indent=2), encoding="utf-8")
    print(json.dumps(metrics_doc, indent=2))
    print(f"Saved model: {(args.output_dir / 'speed_delta_v1.json').resolve()}")


if __name__ == "__main__":
    main()
