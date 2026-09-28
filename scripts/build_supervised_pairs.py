"""Synchronize phone IMU inputs to vehicle-speed labels for one IO-VNBD pair.

Phone GNSS and all vehicle fields are deliberately excluded from the feature
columns. The phone wall-clock/vehicle time-of-day offset is estimated from the
paired timestamps and must be stable before rows are exported.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIR = ROOT / "data/raw/iovnbd/extracted/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1"
DAY_SECONDS = 24 * 60 * 60


def column_starting(frame: pd.DataFrame, prefix: str) -> str:
    found = [column for column in frame.columns if column.strip().startswith(prefix)]
    if len(found) != 1:
        raise SystemExit(f"Expected one column starting {prefix!r}, found {found}")
    return found[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-csv", type=Path, default=DEFAULT_DIR / "S-S1.csv")
    parser.add_argument("--vehicle-csv", type=Path, default=DEFAULT_DIR / "V-S1.csv")
    parser.add_argument("--output", type=Path, default=ROOT / "data/processed/iovnbd/S1_imu_speed_labels.csv")
    parser.add_argument("--max-clock-residual-seconds", type=float, default=0.05)
    args = parser.parse_args()

    phone = pd.read_csv(args.phone_csv, encoding="cp1252", low_memory=False)
    vehicle = pd.read_csv(args.vehicle_csv, encoding="cp1252", low_memory=False)
    phone.columns, vehicle.columns = phone.columns.str.strip(), vehicle.columns.str.strip()

    phone_datetime = pd.to_datetime(
        phone["DATE (YYYY-MO-DD HH-MI-SS_SSS)"],
        format="%Y-%m-%d %H:%M:%S:%f",
        errors="coerce",
    )
    if phone_datetime.isna().any():
        raise SystemExit(f"Phone timestamp parse failed for {int(phone_datetime.isna().sum())} rows")
    # datetime64 integer units follow the parsed dtype resolution (often us),
    # not necessarily ns. Subtract first and explicitly convert to seconds.
    phone_elapsed = (phone_datetime - phone_datetime.iloc[0]).dt.total_seconds().to_numpy(dtype=np.float64)
    phone_tod = (
        phone_datetime.dt.hour.to_numpy() * 3600
        + phone_datetime.dt.minute.to_numpy() * 60
        + phone_datetime.dt.second.to_numpy()
        + phone_datetime.dt.microsecond.to_numpy() / 1e6
    )

    vehicle_tod = pd.to_numeric(
        vehicle["Time Since Start of Day (seconds)"], errors="coerce"
    ).to_numpy(dtype=float)
    if not np.isfinite(vehicle_tod).all():
        raise SystemExit("Vehicle time-of-day contains missing or invalid samples")
    vehicle_unwrapped = vehicle_tod.copy()
    for idx in range(1, len(vehicle_unwrapped)):
        if vehicle_unwrapped[idx] - vehicle_unwrapped[idx - 1] < -DAY_SECONDS / 2:
            vehicle_unwrapped[idx:] += DAY_SECONDS
    if not np.all(np.diff(vehicle_unwrapped) > 0):
        raise SystemExit("Vehicle time-of-day is not strictly increasing after midnight handling")

    paired_rows = min(len(phone_tod), len(vehicle_tod))
    if paired_rows < 2:
        raise SystemExit("Phone and vehicle tables do not contain enough paired samples for clock estimation")
    raw_offset = phone_tod[:paired_rows] - vehicle_tod[:paired_rows]
    wrapped_offset = (raw_offset + DAY_SECONDS / 2) % DAY_SECONDS - DAY_SECONDS / 2
    clock_offset = float(np.median(wrapped_offset))
    residual = (wrapped_offset - clock_offset + DAY_SECONDS / 2) % DAY_SECONDS - DAY_SECONDS / 2
    p95_residual = float(np.quantile(np.abs(residual), 0.95))
    if p95_residual > args.max_clock_residual_seconds:
        raise SystemExit(
            f"Paired wall clocks are not stable enough: p95 residual={p95_residual:.4f}s "
            f"> {args.max_clock_residual_seconds:.4f}s. Do not create labels until alignment is resolved."
        )

    # Map vehicle time-of-day into the phone's relative wall-clock frame.
    vehicle_elapsed = vehicle_unwrapped + clock_offset - phone_tod[0]
    if np.any(np.diff(phone_elapsed) <= 0):
        raise SystemExit("Phone timestamps are not strictly increasing")

    acc_cols = [column_starting(phone, f"ACCELEROMETER {axis} ") for axis in "XYZ"]
    gravity_cols = [column_starting(phone, f"GRAVITY {axis} ") for axis in "XYZ"]
    gyro_cols = [column_starting(phone, f"GYROSCOPE {axis_label} ") for axis_label in ("Yaw", "Pitch", "Roll")]
    source_columns = acc_cols + gravity_cols + gyro_cols
    feature_frame = phone[source_columns].apply(pd.to_numeric, errors="coerce")
    if feature_frame.isna().any().any():
        raise SystemExit("Phone IMU feature channels contain missing or nonnumeric values")

    accel = feature_frame[acc_cols].to_numpy(dtype=float)
    gravity = feature_frame[gravity_cols].to_numpy(dtype=float)
    gyro = feature_frame[gyro_cols].to_numpy(dtype=float)
    linear_accel = accel - gravity
    label_speed = pd.to_numeric(vehicle["Velocity (km/hr)"], errors="coerce").to_numpy(dtype=float) / 3.6
    valid_label = np.isfinite(label_speed)
    if valid_label.sum() < 2:
        raise SystemExit("Vehicle speed has too few valid label samples")

    speed_at_phone_time = np.interp(
        phone_elapsed,
        vehicle_elapsed[valid_label],
        label_speed[valid_label],
        left=np.nan,
        right=np.nan,
    )
    in_vehicle_span = np.isfinite(speed_at_phone_time)
    columns = {
        "elapsed_s": phone_elapsed,
        "accel_x_mps2": accel[:, 0],
        "accel_y_mps2": accel[:, 1],
        "accel_z_mps2": accel[:, 2],
        "gravity_x_mps2": gravity[:, 0],
        "gravity_y_mps2": gravity[:, 1],
        "gravity_z_mps2": gravity[:, 2],
        "linear_accel_x_mps2": linear_accel[:, 0],
        "linear_accel_y_mps2": linear_accel[:, 1],
        "linear_accel_z_mps2": linear_accel[:, 2],
        "gyro_yaw_rps": gyro[:, 0],
        "gyro_pitch_rps": gyro[:, 1],
        "gyro_roll_rps": gyro[:, 2],
        "vehicle_gnss_speed_label_mps": speed_at_phone_time,
    }
    aligned = pd.DataFrame(columns).loc[in_vehicle_span].reset_index(drop=True)
    if aligned.empty:
        raise SystemExit("No phone samples overlap the vehicle label interval")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    aligned.to_csv(args.output, index=False)
    metadata = {
        "phone_csv": str(args.phone_csv.resolve()),
        "vehicle_csv": str(args.vehicle_csv.resolve()),
        "output_csv": str(args.output.resolve()),
        "rows_phone": int(len(phone)),
        "rows_vehicle": int(len(vehicle)),
        "rows_exported": int(len(aligned)),
        "estimated_phone_minus_vehicle_clock_offset_seconds": clock_offset,
        "clock_offset_p95_residual_seconds": p95_residual,
        "target": "vehicle GNSS velocity interpolated to phone wall-clock timestamps, m/s",
        "feature_channels": list(aligned.columns[1:-1]),
        "excluded_phone_fields": ["GPS position", "GPS speed", "GPS orientation", "magnetometer"],
        "caution": "This creates aligned rows, not train/validation windows. Validate offsets across more drives and split by drive before model training.",
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    print(f"Saved labels: {args.output.resolve()}")
    print(f"Saved metadata: {metadata_path.resolve()}")


if __name__ == "__main__":
    main()
