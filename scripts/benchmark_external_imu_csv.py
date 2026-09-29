"""Measure an external high-rate IMU CSV through the edge navigation engine.

Input cadence and navigation-output cadence come from the source timestamps.
Optional GNSS and truth columns enable an explicitly masked, map-free outage
score. Accelerometer columns must be gravity compensated in vehicle-forward
coordinates; adapt/rotate raw device axes before using this replay tool.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navcore.edge import (
    EdgeNavigationEngine, ExternalImuAdapter, ExternalImuConfig, ExternalImuSample,
)


def optional_float(row: dict[str, str], column: str | None) -> float | None:
    if not column or not row.get(column, "").strip():
        return None
    value = float(row[column])
    return value if math.isfinite(value) else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--timestamp", default="timestamp_s")
    parser.add_argument("--timestamp-unit", choices=("ns", "us", "ms", "s"), default="s")
    parser.add_argument("--accel-x", default="accel_forward_mps2")
    parser.add_argument("--gyro-z", default="yaw_rate_radps")
    parser.add_argument("--accel-unit", choices=("m/s2", "g"), default="m/s2")
    parser.add_argument("--gyro-unit", choices=("rad/s", "deg/s"), default="rad/s")
    parser.add_argument("--imu-hz", type=float, default=200.0, help="expected rate used only for gap diagnostics")
    parser.add_argument("--output-hz", type=float, default=200.0)
    parser.add_argument("--gnss-east", default="gnss_east_m")
    parser.add_argument("--gnss-north", default="gnss_north_m")
    parser.add_argument("--gnss-sigma", default="gnss_sigma_m")
    parser.add_argument("--truth-east", default="truth_east_m")
    parser.add_argument("--truth-north", default="truth_north_m")
    parser.add_argument("--outage-start-s", type=float)
    parser.add_argument("--outage-duration-s", type=float)
    args = parser.parse_args()
    if not args.input_csv.is_file():
        raise SystemExit(f"Input CSV does not exist: {args.input_csv}")
    if args.imu_hz <= 0 or args.output_hz <= 0:
        raise SystemExit("input and output rates must be positive")
    if (args.outage_start_s is None) != (args.outage_duration_s is None):
        raise SystemExit("provide both --outage-start-s and --outage-duration-s, or neither")
    if args.outage_duration_s is not None and args.outage_duration_s <= 0:
        raise SystemExit("outage duration must be positive")

    rows: list[dict[str, str]]
    with args.input_csv.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise SystemExit("CSV must include a header row")
        required = {args.timestamp, args.accel_x, args.gyro_z}
        missing = required - set(reader.fieldnames)
        if missing:
            raise SystemExit(f"CSV is missing required columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if len(rows) < 3:
        raise SystemExit("CSV must contain at least three IMU samples")

    scale_ns = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}[args.timestamp_unit]
    raw_times = [float(row[args.timestamp]) for row in rows]
    if any(not math.isfinite(t) for t in raw_times) or any(b <= a for a, b in zip(raw_times, raw_times[1:])):
        raise SystemExit("timestamps must be finite and strictly increasing")
    first_time = raw_times[0]
    elapsed_s = [(t - first_time) * scale_ns / 1e9 for t in raw_times]
    adapter = ExternalImuAdapter(ExternalImuConfig(
        acceleration_unit=args.accel_unit, angular_rate_unit=args.gyro_unit,
        timestamp_unit=args.timestamp_unit, acceleration_is_gravity_compensated=True,
        target_imu_hz=args.imu_hz,
        timestamp_offset_ns=-round(first_time * scale_ns),
    ))
    engine = EdgeNavigationEngine(adapter, output_hz=args.output_hz)
    output_rows: list[dict[str, object]] = []
    outage_begin = args.outage_start_s
    outage_end = None if outage_begin is None else outage_begin + args.outage_duration_s
    for source_time, elapsed, row in zip(raw_times, elapsed_s, rows):
        result = engine.process_imu(ExternalImuSample(
            timestamp=source_time,
            acceleration=(float(row[args.accel_x]), 0.0, 0.0),
            angular_rate=(0.0, 0.0, float(row[args.gyro_z])),
        ), quality=1.0)
        gnss_e = optional_float(row, args.gnss_east)
        gnss_n = optional_float(row, args.gnss_north)
        gnss_sigma = optional_float(row, args.gnss_sigma)
        in_outage = outage_begin is not None and outage_begin <= elapsed < outage_end
        if gnss_e is not None and gnss_n is not None and not in_outage:
            gnss_time_ns = round(source_time * scale_ns) - round(first_time * scale_ns)
            engine.update_gnss(gnss_time_ns, east_m=gnss_e, north_m=gnss_n,
                               horizontal_sigma_m=gnss_sigma or 5.0)
        if result is None:
            continue
        state = result.state
        truth_e = optional_float(row, args.truth_east)
        truth_n = optional_float(row, args.truth_north)
        error = None if truth_e is None or truth_n is None else math.hypot(state.east_m - truth_e, state.north_m - truth_n)
        output_rows.append({
            "elapsed_s": elapsed, "raw_east_m": state.east_m, "raw_north_m": state.north_m,
            "truth_east_m": truth_e, "truth_north_m": truth_n,
            "raw_position_error_m": error,
            "gnss_available": gnss_e is not None and gnss_n is not None and not in_outage,
            "mode": state.mode, "speed_mps": state.speed_mps,
            "horizontal_sigma_m": state.horizontal_sigma_m, "map_matched": False,
        })
    if not output_rows:
        raise SystemExit("No navigation outputs were emitted; check timestamps and output rate")

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(output_rows[0].keys()))
        writer.writeheader(); writer.writerows(output_rows)
    diagnostics = engine.diagnostics()
    report: dict[str, object] = {
        "experiment": "genuine external IMU CSV replay",
        "source_csv": str(args.input_csv.resolve()),
        "source_samples": len(rows),
        "source_duration_s": elapsed_s[-1] - elapsed_s[0],
        "measured_imu_input_hz": diagnostics.measured_input_hz,
        "target_navigation_output_hz": args.output_hz,
        "measured_navigation_output_hz": diagnostics.measured_output_hz,
        "navigation_output_samples": diagnostics.output_samples,
        "input_interval_median_ms": diagnostics.interval_median_ms,
        "input_interval_rms_jitter_ms": diagnostics.interval_rms_jitter_ms,
        "input_gap_count": diagnostics.intervals_over_gap_limit,
        "edge_processing_latency_p95_ms": diagnostics.processing_latency_p95_ms,
        "raw_outage_metrics": None,
        "map_aided_metrics": None,
        "limitations": [
            "Replay measures algorithm cadence and processing latency on this computer; it is not a target edge-device throughput claim.",
            "The supplied CSV must already be calibrated, time-aligned, gravity compensated, and rotated into forward-acceleration/yaw-rate coordinates.",
            "Raw outage drift is only computed when independent truth columns are present; map-matched output is never included in raw drift scores.",
        ],
    }
    if outage_begin is not None:
        outage_rows = [r for r in output_rows if outage_begin <= float(r["elapsed_s"]) < outage_end]
        scored = [r for r in outage_rows if r["raw_position_error_m"] is not None]
        if scored:
            errors = np.asarray([float(r["raw_position_error_m"]) for r in scored])
            start, end = scored[0], scored[-1]
            travel = sum(math.hypot(
                float(b["truth_east_m"]) - float(a["truth_east_m"]),
                float(b["truth_north_m"]) - float(a["truth_north_m"]),
            ) for a, b in zip(scored, scored[1:]))
            drift = math.hypot(
                (float(end["raw_east_m"]) - float(start["raw_east_m"])) - (float(end["truth_east_m"]) - float(start["truth_east_m"])),
                (float(end["raw_north_m"]) - float(start["raw_north_m"])) - (float(end["truth_north_m"]) - float(start["truth_north_m"])),
            )
            report["raw_outage_metrics"] = {
                "duration_s": args.outage_duration_s, "samples": len(scored),
                "absolute_error_rmse_m": float(np.sqrt(np.mean(errors**2))),
                "absolute_error_max_m": float(np.max(errors)),
                "relative_drift_m": drift, "truth_distance_m": travel,
                "drift_percent_of_total_distance": 100.0 * drift / travel if travel > 0 else None,
                "map_matched": False,
            }
        else:
            report["raw_outage_metrics"] = {"scored_samples": 0, "reason": "truth columns absent or no outputs in outage"}
    output_json = args.output_json or args.output_csv.with_suffix(".json")
    output_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
