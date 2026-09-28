"""Audit wall-clock alignment and cadence across synchronized IO-VNBD pairs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = ROOT / "data/raw/iovnbd/extracted/Synchronised V abd S datasets"
DAY_SECONDS = 86_400.0
PHONE_DATE = "DATE (YYYY-MO-DD HH-MI-SS_SSS)"
VEHICLE_TIME = "Time Since Start of Day (seconds)"


def phone_clock(path: Path) -> tuple[np.ndarray, int]:
    frame = pd.read_csv(
        path,
        encoding="cp1252",
        usecols=lambda name: name.strip() == PHONE_DATE,
        low_memory=False,
    )
    dates = pd.to_datetime(frame.iloc[:, 0], format="%Y-%m-%d %H:%M:%S:%f", errors="coerce")
    valid = dates.notna().to_numpy()
    tod = (
        dates.dt.hour.to_numpy(dtype=float) * 3600
        + dates.dt.minute.to_numpy(dtype=float) * 60
        + dates.dt.second.to_numpy(dtype=float)
        + dates.dt.microsecond.to_numpy(dtype=float) / 1e6
    )
    # Carry dates forward through midnight into a continuous clock.
    unwrapped = tod.copy()
    for idx in range(1, len(unwrapped)):
        if unwrapped[idx] - unwrapped[idx - 1] < -DAY_SECONDS / 2:
            unwrapped[idx:] += DAY_SECONDS
    return unwrapped[valid], int((~valid).sum())


def vehicle_clock(path: Path) -> np.ndarray:
    frame = pd.read_csv(
        path,
        encoding="cp1252",
        usecols=lambda name: name.strip() == VEHICLE_TIME,
        low_memory=False,
    )
    values = pd.to_numeric(frame.iloc[:, 0], errors="coerce").to_numpy(dtype=float)
    values = values[np.isfinite(values)]
    unwrapped = values.copy()
    for idx in range(1, len(unwrapped)):
        if unwrapped[idx] - unwrapped[idx - 1] < -DAY_SECONDS / 2:
            unwrapped[idx:] += DAY_SECONDS
    return unwrapped


def audit_pair(phone_path: Path, vehicle_path: Path) -> dict:
    phone_time, phone_bad_time = phone_clock(phone_path)
    vehicle_time = vehicle_clock(vehicle_path)
    n = min(len(phone_time), len(vehicle_time))
    if n < 2:
        raise ValueError("fewer than two valid clock samples")

    raw_delta = phone_time[:n] - vehicle_time[:n]
    offset = float(np.median(raw_delta))
    residual = raw_delta - offset
    # Remove whole-day discontinuities if the streams span midnight.
    residual = (residual + DAY_SECONDS / 2) % DAY_SECONDS - DAY_SECONDS / 2
    p95_residual = float(np.quantile(np.abs(residual), 0.95))
    phone_dt = np.diff(phone_time)
    vehicle_dt = np.diff(vehicle_time)
    phone_start, vehicle_start = float(phone_time[0]), float(vehicle_time[0])
    vehicle_rel_start = vehicle_start + offset - phone_start
    duration_phone = float(phone_time[-1] - phone_start)
    duration_vehicle = float(vehicle_time[-1] - vehicle_start)
    common_duration = max(0.0, min(duration_phone, vehicle_rel_start + duration_vehicle) - max(0.0, vehicle_rel_start))
    return {
        "drive_id": phone_path.stem.removeprefix("S-"),
        "phone_csv": str(phone_path),
        "vehicle_csv": str(vehicle_path),
        "phone_rows_valid_clock": int(len(phone_time)),
        "vehicle_rows_valid_clock": int(len(vehicle_time)),
        "row_count_equal": len(phone_time) == len(vehicle_time),
        "estimated_phone_minus_vehicle_offset_s": offset,
        "clock_offset_p95_residual_s": p95_residual,
        "phone_bad_timestamp_rows": phone_bad_time,
        "phone_median_period_s": float(np.median(phone_dt[phone_dt > 0])) if np.any(phone_dt > 0) else None,
        "phone_nonpositive_dt_count": int(np.sum(phone_dt <= 0)),
        "vehicle_median_period_s": float(np.median(vehicle_dt[vehicle_dt > 0])) if np.any(vehicle_dt > 0) else None,
        "vehicle_nonpositive_dt_count": int(np.sum(vehicle_dt <= 0)),
        "phone_duration_s": duration_phone,
        "vehicle_duration_s": duration_vehicle,
        "estimated_vehicle_start_relative_to_phone_s": vehicle_rel_start,
        "common_duration_s": common_duration,
        "clock_quality_pass_50ms": (
            p95_residual <= 0.05
            and phone_bad_time == 0
            and int(np.sum(phone_dt <= 0)) == 0
            and int(np.sum(vehicle_dt <= 0)) == 0
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs")
    args = parser.parse_args()
    phone_files = sorted(args.root.rglob("S-*.csv"))
    pairs = [(phone, phone.with_name(phone.name.replace("S-", "V-", 1))) for phone in phone_files]
    pairs = [(phone, vehicle) for phone, vehicle in pairs if vehicle.exists()]
    if not pairs:
        raise SystemExit(f"No same-directory S-/V- CSV pairs found under {args.root}")

    rows = []
    errors = []
    for index, (phone_path, vehicle_path) in enumerate(pairs, start=1):
        try:
            rows.append(audit_pair(phone_path, vehicle_path))
        except Exception as exc:  # Keep the audit useful when one run is malformed.
            errors.append({"phone_csv": str(phone_path), "vehicle_csv": str(vehicle_path), "error": str(exc)})
        print(f"[{index}/{len(pairs)}] {phone_path.stem}: {'ok' if rows and rows[-1].get('phone_csv') == str(phone_path) else 'error'}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    table_path = args.output_dir / "iovnbd_pair_audit.csv"
    json_path = args.output_dir / "iovnbd_pair_audit.json"
    pd.DataFrame(rows).to_csv(table_path, index=False)
    passing = sum(row["clock_quality_pass_50ms"] for row in rows)
    result = {
        "dataset_root": str(args.root.resolve()),
        "phone_logs_found": len(phone_files),
        "paired_logs_found": len(pairs),
        "pairs_audited": len(rows),
        "pair_errors": errors,
        "passes_50ms_clock_residual_gate": int(passing),
        "clock_quality_gate_definition": "p95 absolute residual <= 50 ms, no phone parse failures, and strictly increasing phone and vehicle clocks",
        "table_csv": str(table_path.resolve()),
        "pair_results": rows,
    }
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "pair_results"}, indent=2))
    print(f"Saved pair table: {table_path.resolve()}")


if __name__ == "__main__":
    main()
