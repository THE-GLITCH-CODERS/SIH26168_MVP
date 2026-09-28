"""Turn SUMO FCD trajectories into synthetic phone-IMU training samples.

Expected input is SUMO ``--fcd-output`` XML with timestamped vehicle x/y,
navigation angle, and speed. This creates train-only examples in the same
12-channel layout as the audited IO-VNBD supervised dataset. The synthetic
signals are augmentation, not real sensor data or evidence of 200 Hz behavior.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "data/processed/sumo/synthetic_imu_train_v0.npz"
GRAVITY_MPS2 = 9.80665


def rotation_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """World-from-body rotation using Z(yaw) Y(pitch) X(roll)."""
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


def read_fcd(path: Path) -> dict[str, list[tuple[float, float, float, float, float]]]:
    """Read SUMO FCD XML into per-vehicle (t, x, y, angle, speed) rows."""
    trajectories: dict[str, list[tuple[float, float, float, float, float]]] = {}
    context = ET.iterparse(path, events=("start", "end"))
    _, root = next(context)
    for event, timestep in context:
        if event != "end":
            continue
        if timestep.tag.rsplit("}", 1)[-1] != "timestep":
            continue
        try:
            timestamp = float(timestep.attrib["time"])
        except (KeyError, ValueError):
            root.clear()
            continue
        for vehicle in timestep:
            if vehicle.tag.rsplit("}", 1)[-1] != "vehicle":
                continue
            try:
                row = (
                    timestamp,
                    float(vehicle.attrib["x"]),
                    float(vehicle.attrib["y"]),
                    float(vehicle.attrib["angle"]),
                    float(vehicle.attrib["speed"]),
                )
                if all(math.isfinite(value) for value in row):
                    trajectories.setdefault(vehicle.attrib["id"], []).append(row)
            except (KeyError, ValueError):
                continue
        root.clear()
    return trajectories


def trajectory_to_phone_features(
    rows: list[tuple[float, float, float, float, float]],
    rng: np.random.Generator,
    *,
    target_hz: float,
    vibration_probability_per_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Resample a trajectory and synthesize mounted-phone IMU in Android axes."""
    data = np.asarray(rows, dtype=np.float64)
    data = data[np.argsort(data[:, 0])]
    keep = np.concatenate(([True], np.diff(data[:, 0]) > 1e-6))
    data = data[keep]
    if len(data) < 4:
        raise ValueError("trajectory needs at least four distinct timestamps")
    source_dt = np.diff(data[:, 0])
    source_dt = source_dt[source_dt > 1e-6]
    if not len(source_dt):
        raise ValueError("trajectory has no positive time intervals")
    source_rate = 1.0 / float(np.median(source_dt))
    start, end = float(data[0, 0]), float(data[-1, 0])
    if end - start < 1.25:
        raise ValueError("trajectory is too short for the default speed-change target")
    dt = 1.0 / target_hz
    times = np.arange(start, end + dt * 0.25, dt, dtype=np.float64)
    if len(times) < 4:
        raise ValueError("trajectory does not cover enough target-rate samples")

    # SUMO FCD angle is navigation degrees: 0 north, increasing clockwise.
    heading_enu = np.unwrap(np.deg2rad(90.0 - data[:, 3]))
    heading = np.interp(times, data[:, 0], heading_enu)
    speed = np.maximum(0.0, np.interp(times, data[:, 0], data[:, 4]))
    velocity_world = np.column_stack((speed * np.cos(heading), speed * np.sin(heading), np.zeros_like(speed)))
    acceleration_world = np.gradient(velocity_world, dt, axis=0, edge_order=2)
    yaw_rate_world = np.gradient(heading, dt, edge_order=2)

    # Random but fixed phone placement for this vehicle: modest dashboard/holder
    # rotations plus occasional portrait/landscape reorientation.
    mount_yaw = rng.uniform(-math.pi, math.pi)
    mount_pitch = rng.uniform(-math.radians(35), math.radians(35))
    mount_roll = rng.uniform(-math.radians(35), math.radians(35))
    if rng.random() < 0.15:
        mount_yaw += rng.choice([-1.0, 1.0]) * math.pi / 2
    vehicle_from_phone = rotation_zyx(mount_yaw, mount_pitch, mount_roll)

    gravity_phone = np.empty((len(times), 3), dtype=np.float64)
    linear_accel_phone = np.empty_like(gravity_phone)
    gyro_phone = np.empty_like(gravity_phone)
    for index, theta in enumerate(heading):
        world_from_vehicle = rotation_zyx(float(theta), 0.0, 0.0)
        world_from_phone = world_from_vehicle @ vehicle_from_phone
        phone_from_world = world_from_phone.T
        gravity_phone[index] = phone_from_world @ np.array([0.0, 0.0, GRAVITY_MPS2])
        linear_accel_phone[index] = phone_from_world @ acceleration_world[index]
        gyro_phone[index] = phone_from_world @ np.array([0.0, 0.0, yaw_rate_world[index]])

    # Biases are session-level; random walks and high-frequency vibration/noise
    # vary within a session. Disturbances affect sensor input, never speed labels.
    accel_bias = rng.normal(0.0, 0.12, size=3)
    gyro_bias = rng.normal(0.0, 0.012, size=3)
    accel_walk = np.cumsum(rng.normal(0.0, 0.003 * math.sqrt(dt), size=linear_accel_phone.shape), axis=0)
    gyro_walk = np.cumsum(rng.normal(0.0, 0.0003 * math.sqrt(dt), size=gyro_phone.shape), axis=0)
    accel_noise = rng.normal(0.0, 0.08, size=linear_accel_phone.shape)
    gravity_noise = rng.normal(0.0, 0.025, size=gravity_phone.shape)
    gyro_noise = rng.normal(0.0, 0.002, size=gyro_phone.shape)
    # IO-VNBD training features are nominally 10 Hz. Keep synthetic frequencies
    # below that stream's 5 Hz Nyquist limit; the live sensor stream is separately
    # reduced to this model-feature cadence before inference.
    vibration_frequency = rng.uniform(0.5, min(4.5, target_hz * 0.45), size=3)
    vibration_phase = rng.uniform(-math.pi, math.pi, size=3)
    vibration_amplitude = rng.uniform(0.03, 0.7, size=3)
    vibration = np.sin(2.0 * math.pi * times[:, None] * vibration_frequency + vibration_phase) * vibration_amplitude
    shock_count = int(rng.poisson(max(0.0, (end - start) * vibration_probability_per_s)))
    if shock_count:
        shock_rows = rng.integers(0, len(times), size=shock_count)
        shock_axes = rng.integers(0, 3, size=shock_count)
        shock_values = rng.normal(0.0, 2.0, size=shock_count)
        vibration[shock_rows, shock_axes] += shock_values

    gravity_measured = gravity_phone + gravity_noise
    linear_measured = linear_accel_phone + accel_bias + accel_walk + accel_noise + vibration
    accel_measured = gravity_measured + linear_measured
    gyro_measured = gyro_phone + gyro_bias + gyro_walk + gyro_noise
    # Existing feature order is accel XYZ, gravity XYZ, linear XYZ, then
    # gyro yaw/pitch/roll (sensor Z/Y/X components).
    features = np.column_stack((
        accel_measured,
        gravity_measured,
        linear_measured,
        gyro_measured[:, 2], gyro_measured[:, 1], gyro_measured[:, 0],
    )).astype(np.float32)
    return features, speed.astype(np.float32), times.astype(np.float32), source_rate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fcd", type=Path, required=True, help="SUMO --fcd-output XML")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seed", type=int, default=20260928)
    parser.add_argument("--target-hz", type=float, default=10.0)
    parser.add_argument("--vibration-probability-per-s", type=float, default=0.12)
    args = parser.parse_args()
    if not args.fcd.exists():
        raise SystemExit(f"FCD file not found: {args.fcd}")
    if not math.isfinite(args.target_hz) or args.target_hz <= 0:
        raise SystemExit("--target-hz must be positive and finite")
    if args.vibration_probability_per_s < 0:
        raise SystemExit("--vibration-probability-per-s cannot be negative")

    rng = np.random.default_rng(args.seed)
    trajectories = read_fcd(args.fcd)
    feature_rows: list[np.ndarray] = []
    speed_rows: list[np.ndarray] = []
    time_rows: list[np.ndarray] = []
    drive_rows: list[np.ndarray] = []
    metadata: list[dict] = []
    for vehicle_index, (vehicle_id, rows) in enumerate(sorted(trajectories.items())):
        ordered = sorted(rows, key=lambda row: row[0])
        source_times = np.asarray([row[0] for row in ordered], dtype=np.float64)
        positive_dt = np.diff(source_times)
        positive_dt = positive_dt[positive_dt > 1e-6]
        if not len(positive_dt):
            print(f"SKIP {vehicle_id}: no positive timestamps")
            continue
        # Do not interpolate through SUMO vehicle disappearance/re-entry gaps.
        max_gap = max(0.25, 3.0 * float(np.median(positive_dt)))
        boundaries = np.concatenate(([0], np.flatnonzero(np.diff(source_times) > max_gap) + 1, [len(ordered)]))
        episode_index = 0
        for begin, end in zip(boundaries[:-1], boundaries[1:]):
            episode = ordered[int(begin):int(end)]
            if len(episode) < 4:
                continue
            try:
                features, speed, times, source_rate = trajectory_to_phone_features(
                    episode, rng, target_hz=args.target_hz,
                    vibration_probability_per_s=args.vibration_probability_per_s,
                )
                synthetic_drive = len(metadata)
                feature_rows.append(features)
                speed_rows.append(speed)
                time_rows.append(times - times[0])
                drive_rows.append(np.full(len(times), synthetic_drive, dtype=np.uint32))
                metadata.append({
                    "synthetic_drive_index": synthetic_drive,
                    "vehicle_id": vehicle_id,
                    "episode_index": episode_index,
                    "source_fcd_rows": len(episode),
                    "rows_generated": len(times),
                    "duration_s": float(times[-1] - times[0]),
                    "fcd_median_rate_hz": source_rate,
                    "synthetic_imu_rate_hz": args.target_hz,
                    "split": "train_only",
                })
                episode_index += 1
            except ValueError as exc:
                print(f"SKIP {vehicle_id} episode {episode_index}: {exc}")

    if not feature_rows:
        raise SystemExit("No usable vehicle trajectories found in SUMO FCD XML")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        X=np.concatenate(feature_rows),
        y_speed_mps=np.concatenate(speed_rows),
        elapsed_s=np.concatenate(time_rows),
        drive_index=np.concatenate(drive_rows),
        split_code=np.zeros(sum(map(len, drive_rows)), dtype=np.uint8),
    )
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps({
        "schema_version": 1,
        "source": "SUMO FCD trajectories with synthetic smartphone IMU sensor effects",
        "source_fcd": str(args.fcd.resolve()),
        "seed": args.seed,
        "target_hz": args.target_hz,
        "features": [
            "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
            "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
            "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
            "gyro_yaw_rps", "gyro_pitch_rps", "gyro_roll_rps",
        ],
        "label": "SUMO vehicle speed in m/s; training label only",
        "rows": int(sum(map(len, drive_rows))),
        "synthetic_drives": metadata,
        "limitations": [
            "Synthetic signals are not real phone or vehicle IMU measurements.",
            "All synthetic samples are train-only; validation and test must stay real and drive-held-out.",
            "Interpolation and injected vibration cannot establish physical 100 Hz or 200 Hz sensor performance.",
            "Tune sensor-effect distributions using training drives only, never the held-out test drives.",
        ],
    }, indent=2), encoding="utf-8")
    print(f"Saved synthetic training data: {args.output.resolve()}")
    print(f"Saved provenance: {metadata_path.resolve()}")
    print(f"Vehicles used: {len(metadata)}; rows: {sum(map(len, drive_rows)):,}")


if __name__ == "__main__":
    main()
