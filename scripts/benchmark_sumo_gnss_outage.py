"""Run a controlled map-free GNSS outage replay over one SUMO FCD trajectory.

This is a filter and outage-state benchmark. It derives idealized vehicle-frame
IMU measurements from SUMO kinematics and injects configurable synthetic sensor
noise; it cannot substantiate real phone or FOG performance.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navcore.edge import (
    EdgeNavigationEngine, ExternalImuAdapter, ExternalImuConfig, ExternalImuSample,
)


def read_fcd(path: Path, vehicle_id: str | None) -> tuple[str, np.ndarray]:
    rows: dict[str, list[tuple[float, float, float, float, float]]] = {}
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag.rsplit("}", 1)[-1] == "timestep":
            t = float(element.attrib["time"])
            for vehicle in element:
                if vehicle.tag.rsplit("}", 1)[-1] != "vehicle":
                    continue
                try:
                    rows.setdefault(vehicle.attrib["id"], []).append((
                        t, float(vehicle.attrib["x"]), float(vehicle.attrib["y"]),
                        float(vehicle.attrib["angle"]), float(vehicle.attrib["speed"]),
                    ))
                except (KeyError, ValueError):
                    continue
            element.clear()
    if not rows:
        raise ValueError("FCD contains no usable vehicle trajectory")
    selected = vehicle_id or max(rows, key=lambda key: len(rows[key]))
    if selected not in rows:
        raise ValueError(f"vehicle id {selected!r} is not present in FCD")
    data = np.asarray(sorted(rows[selected]), dtype=np.float64)
    unique = np.concatenate(([True], np.diff(data[:, 0]) > 1e-6))
    data = data[unique]
    if len(data) < 5:
        raise ValueError("selected trajectory needs at least five unique samples")
    return selected, data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fcd", type=Path, required=True)
    parser.add_argument("--vehicle-id")
    parser.add_argument("--outage-start-s", type=float, required=True)
    parser.add_argument("--outage-duration-s", type=float, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--imu-hz", type=float, default=200.0)
    parser.add_argument("--gnss-hz", type=float, default=1.0)
    parser.add_argument("--gnss-noise-sigma-m", type=float, default=3.0)
    parser.add_argument("--accel-noise-sigma-mps2", type=float, default=0.08)
    parser.add_argument("--gyro-noise-sigma-radps", type=float, default=0.002)
    parser.add_argument("--seed", type=int, default=20260928)
    args = parser.parse_args()
    if not args.fcd.is_file():
        raise SystemExit(f"FCD file does not exist: {args.fcd}")
    if args.imu_hz <= 0 or args.gnss_hz <= 0 or args.outage_start_s < 0 or args.outage_duration_s <= 0:
        raise SystemExit("rates and outage duration must be positive; outage start cannot be negative")

    selected, source = read_fcd(args.fcd, args.vehicle_id)
    t_src, x_src, y_src, angle_src, speed_src = source.T
    t0, t1 = float(t_src[0]), float(t_src[-1])
    dt = 1.0 / args.imu_hz
    times = np.arange(t0, t1 + dt * 0.1, dt)
    if len(times) < 4:
        raise SystemExit("selected FCD trajectory is too short")
    x = np.interp(times, t_src, x_src); y = np.interp(times, t_src, y_src)
    speed = np.maximum(0.0, np.interp(times, t_src, speed_src))
    # Unwrap SUMO's north-clockwise heading before interpolation so turns across
    # 0/360 degrees do not invent a near-full-circle rotation.
    heading_source = np.unwrap(np.deg2rad(90.0 - angle_src))
    heading = np.interp(times, t_src, heading_source)
    accel = np.gradient(speed, dt, edge_order=2)
    yaw_rate = np.gradient(heading, dt, edge_order=2)
    truth_e = x - x[0]; truth_n = y - y[0]

    rng = np.random.default_rng(args.seed)
    accel_bias = rng.normal(0.0, 0.04)
    gyro_bias = rng.normal(0.0, 0.001)
    adapter = ExternalImuAdapter(ExternalImuConfig(
        acceleration_unit="m/s2", angular_rate_unit="rad/s", timestamp_unit="s",
        acceleration_is_gravity_compensated=True, target_imu_hz=args.imu_hz,
    ))
    engine = EdgeNavigationEngine(adapter, output_hz=args.imu_hz)
    output_rows: list[dict[str, object]] = []
    outage_begin = t0 + args.outage_start_s
    outage_end = outage_begin + args.outage_duration_s
    gnss_period = 1.0 / args.gnss_hz
    next_gnss_t = t0
    gnss_attempts = 0
    gnss_accepted = 0
    gnss_rejection_reasons: dict[str, int] = {}
    for index, timestamp in enumerate(times):
        course = (90.0 - math.degrees(float(heading[index]))) % 360.0
        sample = ExternalImuSample(
            timestamp=float(timestamp - t0),
            acceleration=(float(accel[index] + accel_bias + rng.normal(0.0, args.accel_noise_sigma_mps2)), 0.0, 0.0),
            angular_rate=(0.0, 0.0, float(yaw_rate[index] + gyro_bias + rng.normal(0.0, args.gyro_noise_sigma_radps))),
        )
        result = engine.process_imu(sample, quality=1.0, normal_driving=speed[index] > 1.0,
                                    stationary=speed[index] < 0.3)
        absolute_s = float(timestamp - t0)
        in_outage = outage_begin <= timestamp < outage_end
        if timestamp + 1e-8 >= next_gnss_t:
            if not in_outage:
                # Measurement is produced from SUMO truth, with GNSS-like position noise.
                gnss_attempts += 1
                accepted = engine.update_gnss(
                    round((timestamp - t0) * 1e9),
                    east_m=float(truth_e[index] + rng.normal(0.0, args.gnss_noise_sigma_m)),
                    north_m=float(truth_n[index] + rng.normal(0.0, args.gnss_noise_sigma_m)),
                    horizontal_sigma_m=max(1.0, args.gnss_noise_sigma_m),
                    speed_mps=float(speed[index]), course_deg_north_clockwise=course,
                )
                if accepted:
                    gnss_accepted += 1
                else:
                    reason = engine.filter.last_gnss_reason.split(":", 1)[0]
                    gnss_rejection_reasons[reason] = gnss_rejection_reasons.get(reason, 0) + 1
            # Advance the measurement clock during outages too. Otherwise the
            # first post-outage loop would incorrectly replay every missed fix.
            while next_gnss_t <= timestamp + 1e-8:
                next_gnss_t += gnss_period
        if result is None:
            continue
        # Apply the current-timestamp GNSS correction before logging the output.
        state = engine.filter.snapshot(result.state.timestamp_ns)
        error = math.hypot(state.east_m - truth_e[index], state.north_m - truth_n[index])
        output_rows.append({
            "elapsed_s": absolute_s,
            "truth_east_m": truth_e[index], "truth_north_m": truth_n[index],
            "raw_estimate_east_m": state.east_m, "raw_estimate_north_m": state.north_m,
            "raw_position_error_m": error, "gnss_available": not in_outage,
            "mode": state.mode, "speed_mps": state.speed_mps,
            "horizontal_sigma_m": state.horizontal_sigma_m,
            "map_matched": False,
        })

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(output_rows[0].keys()))
        writer.writeheader(); writer.writerows(output_rows)
    selected_outage = [row for row in output_rows if outage_begin - t0 <= float(row["elapsed_s"]) < outage_end - t0]
    if not selected_outage:
        raise SystemExit("configured outage does not overlap the selected FCD trajectory")
    errors = np.asarray([float(row["raw_position_error_m"]) for row in selected_outage])
    distance_m = sum(math.hypot(
        float(right["truth_east_m"]) - float(left["truth_east_m"]),
        float(right["truth_north_m"]) - float(left["truth_north_m"]),
    ) for left, right in zip(selected_outage, selected_outage[1:]))
    drift = math.hypot(
        (float(selected_outage[-1]["raw_estimate_east_m"]) - float(selected_outage[0]["raw_estimate_east_m"])) -
        (float(selected_outage[-1]["truth_east_m"]) - float(selected_outage[0]["truth_east_m"])),
        (float(selected_outage[-1]["raw_estimate_north_m"]) - float(selected_outage[0]["raw_estimate_north_m"])) -
        (float(selected_outage[-1]["truth_north_m"]) - float(selected_outage[0]["truth_north_m"])),
    )
    diagnostics = engine.diagnostics()
    report = {
        "experiment": "SUMO FCD controlled GNSS outage; map-free raw filter score",
        "vehicle_id": selected, "source_fcd": str(args.fcd.resolve()),
        "synthetic_imu_input_hz": args.imu_hz, "gnss_input_hz": args.gnss_hz,
        "outage_start_s": args.outage_start_s, "outage_duration_s": args.outage_duration_s,
        "outage_samples": len(selected_outage),
        "gnss_fix_attempts": gnss_attempts,
        "gnss_fixes_accepted": gnss_accepted,
        "gnss_rejection_counts_by_reason": gnss_rejection_reasons,
        "raw_outage_absolute_error_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "raw_outage_absolute_error_max_m": float(np.max(errors)),
        "raw_outage_relative_drift_m": drift,
        "outage_truth_distance_m": distance_m,
        "raw_outage_drift_percent_of_distance": 100.0 * drift / distance_m if distance_m > 0 else None,
        "map_aided_score": None,
        "edge_measured_input_hz": diagnostics.measured_input_hz,
        "edge_measured_output_hz": diagnostics.measured_output_hz,
        "edge_output_samples": diagnostics.output_samples,
        "edge_processing_latency_p95_ms": diagnostics.processing_latency_p95_ms,
        "limitations": [
            "FCD is interpolated to the configured rate; synthetic_imu_input_hz is not measured hardware cadence.",
            "IMU and GNSS measurements are generated from SUMO truth with configurable synthetic noise.",
            "This validates controlled filter behavior only; use genuine high-rate IMU and independent reference data for the edge benchmark.",
            "Reported drift is raw GNSS/INS output; no map matching is used in this score.",
        ],
    }
    output_json = args.output_json or args.output_csv.with_suffix(".json")
    output_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
