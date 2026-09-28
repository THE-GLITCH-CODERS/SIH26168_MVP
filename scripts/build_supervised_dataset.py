"""Build a compact, drive-separated IMU/speed dataset from audited IO-VNBD pairs."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
AUDIT_DEFAULT = ROOT / "outputs/iovnbd_all/iovnbd_pair_audit.csv"
OUTPUT_DEFAULT = ROOT / "data/processed/iovnbd/supervised_drives_v0.npz"
DAY = 86_400.0


def numeric(frame: pd.DataFrame, column: str) -> np.ndarray:
    return pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=float)


def find_axis(frame: pd.DataFrame, prefix: str) -> str:
    matches = [column for column in frame.columns if column.strip().startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f"expected one {prefix!r} column, got {matches}")
    return matches[0]


def split_for(phone_path: str, drive_id: str) -> tuple[str, str]:
    match = re.search(r"Driver ([A-Z])", phone_path)
    driver = f"Driver {match.group(1)}" if match else "Unknown"
    family = next((prefix for prefix in ("Vfa", "Vfb", "Vta", "Vtb", "Vw", "S", "M", "Y") if drive_id.startswith(prefix)), "other")
    if driver == "Driver A":
        return driver, "test"
    # IO-VNBD's synchronized pairs currently provide one passing driver group
    # outside Driver A. Hold out the complete Vw drive family for validation;
    # keep every complete drive in exactly one split.
    if driver == "Driver E" and family == "Vw":
        return driver, "validation"
    if driver != "Unknown":
        return driver, "train"
    return driver, "excluded"


def prepare_pair(row: pd.Series, clock_gate_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
    phone_path, vehicle_path = Path(row.phone_csv), Path(row.vehicle_csv)
    phone = pd.read_csv(phone_path, encoding="cp1252", low_memory=False)
    vehicle = pd.read_csv(vehicle_path, encoding="cp1252", low_memory=False)
    phone.columns, vehicle.columns = phone.columns.str.strip(), vehicle.columns.str.strip()

    date_col = "DATE (YYYY-MO-DD HH-MI-SS_SSS)"
    dates = pd.to_datetime(phone[date_col], format="%Y-%m-%d %H:%M:%S:%f", errors="coerce")
    if dates.isna().any():
        raise ValueError(f"{int(dates.isna().sum())} invalid phone timestamps")
    # Pandas datetime64 resolution depends on the source data (often us here),
    # so interpreting astype(int64) as nanoseconds silently compresses time.
    # Subtract datetimes first and ask pandas for seconds explicitly.
    phone_elapsed = (dates - dates.iloc[0]).dt.total_seconds().to_numpy(dtype=np.float64)
    phone_tod = (
        dates.dt.hour.to_numpy() * 3600
        + dates.dt.minute.to_numpy() * 60
        + dates.dt.second.to_numpy()
        + dates.dt.microsecond.to_numpy() / 1e6
    )
    vehicle_tod = numeric(vehicle, "Time Since Start of Day (seconds)")
    if not np.isfinite(vehicle_tod).all():
        raise ValueError("vehicle time-of-day has missing or invalid samples")
    vehicle_unwrapped = vehicle_tod.copy()
    for idx in range(1, len(vehicle_unwrapped)):
        if vehicle_unwrapped[idx] - vehicle_unwrapped[idx - 1] < -DAY / 2:
            vehicle_unwrapped[idx:] += DAY
    n = min(len(phone_tod), len(vehicle_tod))
    delta = phone_tod[:n] - vehicle_tod[:n]
    offset_samples = (delta + DAY / 2) % DAY - DAY / 2
    offset = float(np.median(offset_samples))
    residual = (offset_samples - offset + DAY / 2) % DAY - DAY / 2
    p95 = float(np.quantile(np.abs(residual), 0.95))
    if p95 > clock_gate_s:
        raise ValueError(f"clock p95 residual {p95:.4f}s exceeds gate {clock_gate_s:.3f}s")
    if np.any(np.diff(phone_elapsed) <= 0) or np.any(np.diff(vehicle_unwrapped) <= 0):
        raise ValueError("non-increasing sample timestamps")
    vehicle_elapsed = vehicle_unwrapped + offset - phone_tod[0]

    accel_cols = [find_axis(phone, f"ACCELEROMETER {axis} ") for axis in "XYZ"]
    gravity_cols = [find_axis(phone, f"GRAVITY {axis} ") for axis in "XYZ"]
    gyro_cols = [find_axis(phone, f"GYROSCOPE {axis} ") for axis in ("Yaw", "Pitch", "Roll")]
    accel = phone[accel_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    gravity = phone[gravity_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    gyro = phone[gyro_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    features = np.concatenate((accel, gravity, accel - gravity, gyro), axis=1)
    labels = numeric(vehicle, "Velocity (km/hr)") / 3.6
    valid = np.isfinite(labels)
    label_at_phone = np.interp(phone_elapsed, vehicle_elapsed[valid], labels[valid], left=np.nan, right=np.nan)
    valid_rows = np.isfinite(label_at_phone) & np.isfinite(features).all(axis=1)
    features = features[valid_rows].astype(np.float32)
    labels = label_at_phone[valid_rows].astype(np.float32)
    elapsed = phone_elapsed[valid_rows].astype(np.float32)
    driver, split = split_for(str(phone_path), str(row.drive_id))
    metadata = {
        "drive_id": str(row.drive_id),
        "driver_group": driver,
        "split": split,
        "rows_phone": int(len(phone)),
        "rows_vehicle": int(len(vehicle)),
        "rows_used": int(len(labels)),
        "clock_offset_s": offset,
        "clock_p95_residual_s": p95,
        "phone_duration_s": float(phone_elapsed[-1]),
        "family": next((prefix for prefix in ("Vfa", "Vfb", "Vta", "Vtb", "Vw", "S", "M", "Y") if str(row.drive_id).startswith(prefix)), "other"),
        "source_phone_csv": str(phone_path),
        "source_vehicle_csv": str(vehicle_path),
    }
    return features, labels, elapsed, metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-csv", type=Path, default=AUDIT_DEFAULT)
    parser.add_argument("--output", type=Path, default=OUTPUT_DEFAULT)
    parser.add_argument("--clock-gate-seconds", type=float, default=0.05)
    args = parser.parse_args()
    if not args.audit_csv.exists():
        raise SystemExit(f"Audit CSV not found: {args.audit_csv}; run scripts/audit_iovnbd_pairs.py first")
    audit = pd.read_csv(args.audit_csv)
    selected = audit[audit["clock_quality_pass_50ms"]].copy()
    if selected.empty:
        raise SystemExit("No pairs pass the current timestamp-quality gate")

    feature_parts, label_parts, elapsed_parts, pair_ids, split_codes = [], [], [], [], []
    metadata_rows = []
    split_number = {"train": 0, "validation": 1, "test": 2, "excluded": 255}
    feature_names = [
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
        "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
        "gyro_yaw_rps", "gyro_pitch_rps", "gyro_roll_rps",
    ]

    for pair_index, (_, row) in enumerate(selected.reset_index(drop=True).iterrows()):
        try:
            features, labels, elapsed, metadata = prepare_pair(row, args.clock_gate_seconds)
            if metadata["split"] == "excluded":
                continue
            feature_parts.append(features)
            label_parts.append(labels)
            elapsed_parts.append(elapsed)
            pair_ids.append(np.full(len(labels), pair_index, dtype=np.uint16))
            split_codes.append(np.full(len(labels), split_number[metadata["split"]], dtype=np.uint8))
            metadata["pair_index"] = pair_index
            metadata_rows.append(metadata)
            print(f"[{pair_index + 1}/{len(selected)}] {metadata['drive_id']}: {metadata['split']} ({len(labels):,} rows)")
        except Exception as exc:
            print(f"SKIP {row.drive_id}: {exc}")

    if not feature_parts:
        raise SystemExit("No drives were successfully prepared")
    X = np.concatenate(feature_parts)
    y = np.concatenate(label_parts)
    elapsed = np.concatenate(elapsed_parts)
    drive_index = np.concatenate(pair_ids)
    split_code = np.concatenate(split_codes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        X=X,
        y_speed_mps=y,
        elapsed_s=elapsed,
        drive_index=drive_index,
        split_code=split_code,
    )
    metadata_path = args.output.with_suffix(".json")
    metadata = {
        "feature_names": feature_names,
        "label_name": "vehicle_gnss_velocity_mps",
        "split_codes": split_number,
        "rows_total": int(len(y)),
        "rows_by_split": {name: int(np.sum(split_code == code)) for name, code in split_number.items() if code != 255},
        "drives_prepared": len(metadata_rows),
        "drives": metadata_rows,
        "provenance": "Phone IMU only as features; vehicle GNSS velocity used as labels after per-drive wall-clock offset estimation.",
        "caution": "Pretraining dataset only. Audit synchronization across all candidate drives and keep splits grouped by complete drive; no model metrics are implied by this file.",
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in metadata.items() if key != "drives"}, indent=2))
    print(f"Saved compact dataset: {args.output.resolve()}")
    print(f"Saved metadata: {metadata_path.resolve()}")


if __name__ == "__main__":
    main()
