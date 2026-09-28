"""Train a drive-held-out ridge baseline for phone-IMU vehicle speed."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA_DEFAULT = ROOT / "data/processed/iovnbd/supervised_drives_v0.npz"
META_DEFAULT = ROOT / "data/processed/iovnbd/supervised_drives_v0.json"
OUT_DEFAULT = ROOT / "outputs/speed_baseline_v0"


def motion_channels(raw: np.ndarray) -> np.ndarray:
    """Add orientation-tolerant norms to the raw 12 phone IMU channels."""
    accel_norm = np.linalg.norm(raw[:, 0:3], axis=1, keepdims=True)
    linear_norm = np.linalg.norm(raw[:, 6:9], axis=1, keepdims=True)
    gyro_norm = np.linalg.norm(raw[:, 9:12], axis=1, keepdims=True)
    return np.concatenate((raw, accel_norm, linear_norm, gyro_norm), axis=1)


def window_summary(values: np.ndarray, window: int) -> tuple[np.ndarray, list[str]]:
    from numpy.lib.stride_tricks import sliding_window_view

    expanded = motion_channels(values)
    windows = sliding_window_view(expanded, window_shape=window, axis=0).transpose(0, 2, 1)
    summary = np.concatenate(
        (
            windows.mean(axis=1),
            windows.std(axis=1),
            windows.min(axis=1),
            windows.max(axis=1),
            windows[:, -1, :],
        ),
        axis=1,
    ).astype(np.float32)
    names = [
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
        "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
        "gyro_yaw_rps", "gyro_pitch_rps", "gyro_roll_rps",
        "accel_norm_mps2", "linear_accel_norm_mps2", "gyro_norm_rps",
    ]
    stat_names = [f"{stat}_{name}" for stat in ("mean", "std", "min", "max", "last") for name in names]
    return summary, stat_names


def segment_indices(elapsed: np.ndarray, max_gap: float) -> list[tuple[int, int]]:
    if len(elapsed) == 0:
        return []
    breaks = np.flatnonzero((np.diff(elapsed) <= 0) | (np.diff(elapsed) > max_gap)) + 1
    bounds = np.concatenate(([0], breaks, [len(elapsed)]))
    return [(int(bounds[i]), int(bounds[i + 1])) for i in range(len(bounds) - 1)]


def regression_metrics(truth: np.ndarray, predicted: np.ndarray) -> dict:
    error = predicted - truth
    ss_total = float(np.sum((truth - np.mean(truth)) ** 2))
    ss_residual = float(np.sum(error**2))
    result = {
        "samples": int(len(truth)),
        "mae_mps": float(np.mean(np.abs(error))),
        "rmse_mps": float(np.sqrt(np.mean(error**2))),
        "median_absolute_error_mps": float(np.median(np.abs(error))),
        "r2": float(1 - ss_residual / ss_total) if ss_total > 0 else None,
        "mae_kmh": float(np.mean(np.abs(error)) * 3.6),
    }
    bins = [(0, 2), (2, 10), (10, 25), (25, 60)]
    result["speed_bins"] = {
        f"{low}-{high}_mps": {
            "samples": int(np.sum((truth >= low) & (truth < high))),
            "mae_mps": float(np.mean(np.abs(error[(truth >= low) & (truth < high)])))
            if np.any((truth >= low) & (truth < high))
            else None,
        }
        for low, high in bins
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DATA_DEFAULT)
    parser.add_argument("--metadata", type=Path, default=META_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--window-samples", type=int, default=20, help="Past-only window; 20 samples is about 2 seconds at 10 Hz.")
    parser.add_argument("--max-gap-seconds", type=float, default=0.2)
    parser.add_argument("--ridge-alpha", type=float, default=10.0)
    args = parser.parse_args()
    if not args.dataset.exists() or not args.metadata.exists():
        raise SystemExit("Training dataset/metadata not found; run audit and build_supervised_dataset.py first")
    if args.window_samples < 2:
        raise SystemExit("--window-samples must be at least 2")

    dataset = np.load(args.dataset)
    raw_x = dataset["X"]
    y = dataset["y_speed_mps"]
    elapsed = dataset["elapsed_s"]
    drive_index = dataset["drive_index"]
    split_code = dataset["split_code"]
    drive_ids = np.unique(drive_index)

    all_features, all_targets, all_splits, all_drives = [], [], [], []
    feature_names = None
    for drive in drive_ids:
        idx = np.flatnonzero(drive_index == drive)
        if not len(idx):
            continue
        drive_splits = np.unique(split_code[idx])
        if len(drive_splits) != 1 or drive_splits[0] == 255:
            raise SystemExit(f"Drive {drive} has inconsistent or excluded split codes")
        for start, end in segment_indices(elapsed[idx], args.max_gap_seconds):
            if end - start < args.window_samples:
                continue
            row_idx = idx[start:end]
            features, names = window_summary(raw_x[row_idx], args.window_samples)
            target_idx = row_idx[args.window_samples - 1 :]
            all_features.append(features)
            all_targets.append(y[target_idx])
            all_splits.append(np.full(len(features), drive_splits[0], dtype=np.uint8))
            all_drives.append(np.full(len(features), drive, dtype=np.uint16))
            feature_names = names

    X = np.concatenate(all_features, axis=0).astype(np.float32, copy=False)
    target = np.concatenate(all_targets).astype(np.float64)
    del all_features, all_targets
    split = np.concatenate(all_splits)
    model_drive = np.concatenate(all_drives)
    train = split == 0
    val = split == 1
    test = split == 2
    if not np.any(train) or not np.any(val) or not np.any(test):
        raise SystemExit("Need non-empty train, validation, and test splits")

    x_mean = X[train].mean(axis=0, dtype=np.float64)
    x_scale = np.sqrt(np.mean((X[train].astype(np.float64) - x_mean) ** 2, axis=0))
    x_scale[x_scale < 1e-6] = 1.0
    y_mean = float(target[train].mean())
    train_positions = np.flatnonzero(train)
    gram = np.zeros((X.shape[1], X.shape[1]), dtype=np.float64)
    rhs = np.zeros(X.shape[1], dtype=np.float64)
    for start in range(0, len(train_positions), 50_000):
        rows = train_positions[start : start + 50_000]
        standardized_batch = (X[rows].astype(np.float64) - x_mean) / x_scale
        centered_batch_y = target[rows] - y_mean
        gram += standardized_batch.T @ standardized_batch
        rhs += standardized_batch.T @ centered_batch_y
    gram.flat[:: len(gram) + 1] += args.ridge_alpha
    coefficients = np.linalg.solve(gram, rhs)

    predictions = np.empty(len(target), dtype=np.float64)
    for start in range(0, len(target), 100_000):
        end = min(start + 100_000, len(target))
        predictions[start:end] = ((X[start:end] - x_mean) / x_scale) @ coefficients + y_mean
    predictions = np.maximum(predictions, 0.0)
    constant_speed = float(np.median(target[train]))
    metrics = {
        "model": "past-only 2-second window ridge regression",
        "features": feature_names,
        "window_samples": args.window_samples,
        "window_duration_approx_seconds": args.window_samples * 0.1,
        "ridge_alpha": args.ridge_alpha,
        "train_drives": int(len(np.unique(model_drive[train]))),
        "validation_drives": int(len(np.unique(model_drive[val]))),
        "test_drives": int(len(np.unique(model_drive[test]))),
        "train": regression_metrics(target[train], predictions[train]),
        "validation": regression_metrics(target[val], predictions[val]),
        "test": regression_metrics(target[test], predictions[test]),
        "constant_train_median_baseline": {
            "validation": regression_metrics(target[val], np.full(np.sum(val), constant_speed)),
            "test": regression_metrics(target[test], np.full(np.sum(test), constant_speed)),
        },
        "interpretation": "Preliminary speed-regression metrics only. They do not measure dead-reckoning position drift or SIH benchmark performance.",
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    model_path = args.output_dir / "speed_ridge_v0.npz"
    metrics_path = args.output_dir / "metrics.json"
    np.savez_compressed(
        model_path,
        feature_mean=x_mean.astype(np.float32),
        feature_scale=x_scale.astype(np.float32),
        coefficients=coefficients.astype(np.float32),
        target_mean=np.float32(y_mean),
        window_samples=np.int32(args.window_samples),
    )
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    print(f"Saved model: {model_path.resolve()}")
    print(f"Saved metrics: {metrics_path.resolve()}")


if __name__ == "__main__":
    main()
