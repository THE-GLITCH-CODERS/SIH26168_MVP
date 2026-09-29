"""Replay a synthetic GNSS outage on an IO-VNBD drive.

Vehicle data is used only before the outage to calibrate the phone axes and at
the outage boundary for initial position/speed/heading. During the outage, the
navigation filters receive phone accelerometer and gyroscope data only. The
vehicle GNSS trajectory is used as an imperfect reference for scoring.

This is an initial physics baseline, not a learned model or a surveyed-truth
benchmark. It intentionally exports the uncorrected trajectory and its metrics.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navcore.fusion import VehicleFusionFilter
from navcore.speed_model import LinearSpeedDeltaModel
from navcore.portable_speed import PortableSpeedModel

DEFAULT_DIR = ROOT / "data/raw/iovnbd/extracted/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1"
G = 9.80665
DAY = 86400.0


def col(frame: pd.DataFrame, name: str) -> np.ndarray:
    return pd.to_numeric(frame[name], errors="coerce").to_numpy(dtype=np.float64)


def axis_columns(frame: pd.DataFrame, prefixes: tuple[str, ...]) -> list[str]:
    result = []
    for prefix in prefixes:
        matches = [name for name in frame.columns if name.startswith(prefix)]
        if len(matches) != 1:
            raise ValueError(f"Expected one column starting {prefix!r}; found {matches}")
        result.append(matches[0])
    return result


def robust_linear_fit(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float, float, int]:
    """Fit y = x @ coef + intercept with a small iterative MAD outlier gate."""
    valid = np.isfinite(y) & np.isfinite(x).all(axis=1)
    keep = valid.copy()
    coef = np.zeros(x.shape[1], dtype=np.float64)
    intercept = 0.0
    for _ in range(4):
        if int(keep.sum()) < max(30, x.shape[1] * 5):
            raise ValueError("Too few clean calibration samples")
        design = np.column_stack((x[keep], np.ones(int(keep.sum()))))
        solution, *_ = np.linalg.lstsq(design, y[keep], rcond=None)
        coef, intercept = solution[:-1], float(solution[-1])
        residual = y - (x @ coef + intercept)
        center = float(np.median(residual[keep]))
        mad = float(np.median(np.abs(residual[keep] - center)))
        # Some IO-VNBD vehicle channels are quantized, so MAD may be exactly
        # zero even when the window has real low-frequency variation. Keep a
        # fraction of the residual standard deviation as a scale floor rather
        # than rejecting every value outside the modal quantization bucket.
        residual_scale = float(np.std(residual[keep]))
        sigma = max(1.4826 * mad, 0.25 * residual_scale, 1e-3)
        next_keep = valid & (np.abs(residual - center) <= 4.5 * sigma)
        if np.array_equal(next_keep, keep):
            break
        keep = next_keep
    residual = y[keep] - (x[keep] @ coef + intercept)
    sse = float(np.sum(residual**2))
    centered = y[keep] - float(np.mean(y[keep]))
    sst = float(np.sum(centered**2))
    r2 = 1.0 - sse / sst if sst > 1e-12 else float("nan")
    rmse = math.sqrt(sse / max(int(keep.sum()), 1))
    return np.r_[coef, intercept], r2, rmse, int(keep.sum())


def local_xy(lat: np.ndarray, lon: np.ndarray, lat0: float, lon0: float) -> tuple[np.ndarray, np.ndarray]:
    east = 6371000.0 * np.radians(lon - lon0) * math.cos(math.radians(lat0))
    north = 6371000.0 * np.radians(lat - lat0)
    return east, north


def trajectory_metrics(e: np.ndarray, n: np.ndarray, ref_e: np.ndarray, ref_n: np.ndarray, distance: float, duration: float) -> dict:
    error = np.hypot(e - ref_e, n - ref_n)
    endpoint = float(error[-1])
    return {
        "horizontal_endpoint_error_m": endpoint,
        "horizontal_max_error_m": float(np.max(error)),
        "horizontal_rmse_m": float(np.sqrt(np.mean(error**2))),
        "reference_distance_m": float(distance),
        "outage_duration_s": float(duration),
        "endpoint_drift_percent_of_distance": 100.0 * endpoint / distance if distance > 0 else None,
        "max_error_percent_of_distance": 100.0 * float(np.max(error)) / distance if distance > 0 else None,
        "under_10_percent_endpoint_drift": bool(distance > 0 and endpoint / distance < 0.10),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-csv", type=Path, default=DEFAULT_DIR / "S-S1.csv")
    parser.add_argument("--vehicle-csv", type=Path, default=DEFAULT_DIR / "V-S1.csv")
    parser.add_argument("--outage-start-s", type=float, default=60.0, help="seconds from phone-log start")
    parser.add_argument("--outage-seconds", type=float, default=60.0)
    parser.add_argument("--calibration-seconds", type=float, default=30.0)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/iovnbd_outage_v0")
    parser.add_argument(
        "--evaluate-speed-model", type=Path,
        help="offline candidate JSON to evaluate on this outage; never loads it into Android/live navigation",
    )
    parser.add_argument("--evaluate-portable-speed", type=Path, help="Offline absolute-speed MLP what-if only; never promotes the artifact")
    args = parser.parse_args()
    if args.outage_start_s <= args.calibration_seconds or args.outage_seconds <= 0:
        raise SystemExit("Outage start must leave a complete, positive calibration window before it")
    run_name = args.phone_csv.stem.removeprefix("S-") or "drive"

    phone = pd.read_csv(args.phone_csv, encoding="cp1252", low_memory=False)
    vehicle = pd.read_csv(args.vehicle_csv, encoding="cp1252", low_memory=False)
    phone.columns, vehicle.columns = phone.columns.str.strip(), vehicle.columns.str.strip()

    phone_dates = pd.to_datetime(phone["DATE (YYYY-MO-DD HH-MI-SS_SSS)"], format="%Y-%m-%d %H:%M:%S:%f", errors="coerce")
    if phone_dates.isna().any():
        raise SystemExit("Phone wall-clock timestamps contain invalid rows")
    # The phone's session timer can reset when its logger restarts inside a
    # long drive file (observed in IO-VNBD S2). Use the recorded wall-clock
    # timestamps as the continuous replay axis, and retain the session timer
    # only as a diagnostic. This also keeps model windows from running
    # backwards across a logger restart.
    phone_timer_s = col(phone, "TIME SINCE START (ms)") / 1000.0
    phone_t = (phone_dates - phone_dates.iloc[0]).dt.total_seconds().to_numpy(dtype=np.float64)
    phone_timer_reset_count = int(np.count_nonzero(np.diff(phone_timer_s) <= 0.0))
    if np.any(~np.isfinite(phone_t)) or np.any(np.diff(phone_t) <= 0):
        raise SystemExit("Phone wall-clock timestamps are invalid or non-increasing")
    phone_tod = (phone_dates.dt.hour * 3600 + phone_dates.dt.minute * 60 + phone_dates.dt.second + phone_dates.dt.microsecond / 1e6).to_numpy(dtype=float)
    vehicle_tod = col(vehicle, "Time Since Start of Day (seconds)")
    vehicle_tod_unwrapped = vehicle_tod.copy()
    for i in range(1, len(vehicle_tod_unwrapped)):
        if vehicle_tod_unwrapped[i] - vehicle_tod_unwrapped[i - 1] < -DAY / 2:
            vehicle_tod_unwrapped[i:] += DAY
    paired = min(len(phone_tod), len(vehicle_tod_unwrapped))
    raw_offset = phone_tod[:paired] - vehicle_tod_unwrapped[:paired]
    wrapped_offset = (raw_offset + DAY / 2) % DAY - DAY / 2
    clock_offset = float(np.median(wrapped_offset))
    clock_residual = (wrapped_offset - clock_offset + DAY / 2) % DAY - DAY / 2
    clock_p95 = float(np.quantile(np.abs(clock_residual), 0.95))
    if clock_p95 > 0.05:
        raise SystemExit(f"Phone/vehicle clock p95 residual {clock_p95:.4f}s exceeds 50 ms; refusing replay")
    vehicle_t = vehicle_tod_unwrapped + clock_offset - phone_tod[0]
    if np.any(np.diff(vehicle_t) <= 0):
        raise SystemExit("Vehicle timestamps are non-increasing")

    accel_cols = axis_columns(phone, tuple(f"ACCELEROMETER {axis} " for axis in "XYZ"))
    gravity_cols = axis_columns(phone, tuple(f"GRAVITY {axis} " for axis in "XYZ"))
    gyro_cols = axis_columns(phone, tuple(f"GYROSCOPE {axis} " for axis in ("Yaw", "Pitch", "Roll")))
    accel = phone[accel_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    gravity = phone[gravity_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    gyro = phone[gyro_cols].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    linear_accel = accel - gravity
    speed_model = LinearSpeedDeltaModel.from_json(args.evaluate_speed_model) if args.evaluate_speed_model else None
    portable_model = PortableSpeedModel.load(args.evaluate_portable_speed) if args.evaluate_portable_speed else None
    model_features = np.column_stack((accel, gravity, linear_accel, gyro)).astype(np.float32) if (speed_model or portable_model) else None
    if portable_model and abs(1.0 / np.median(np.diff(phone_t)) - 10.0) > 0.5:
        raise SystemExit("Portable model requires nominal 10 Hz input")
    if speed_model is not None:
        measured_feature_rate = float(1.0 / np.median(np.diff(phone_t)))
        if abs(measured_feature_rate - speed_model.feature_rate_hz) / speed_model.feature_rate_hz > 0.05:
            raise SystemExit(
                f"Speed model requires {speed_model.feature_rate_hz:.3f} Hz feature input; "
                f"this phone CSV is {measured_feature_rate:.3f} Hz"
            )

    vehicle_speed = col(vehicle, "Velocity (km/hr)") / 3.6
    vehicle_heading = col(vehicle, "Heading (degrees)")
    vehicle_lat = col(vehicle, "Latitude (degrees)")
    vehicle_lon = col(vehicle, "Longitude (degrees)")
    vehicle_long_accel = col(vehicle, "Indicated Longitudinal Acceleration (g)") * G
    vehicle_yaw_rate = np.deg2rad(col(vehicle, "Yaw Rate (deg/sec)"))
    fields = [vehicle_speed, vehicle_heading, vehicle_lat, vehicle_lon, vehicle_long_accel, vehicle_yaw_rate]
    interp = [np.interp(phone_t, vehicle_t[np.isfinite(v)], v[np.isfinite(v)], left=np.nan, right=np.nan) for v in fields]
    speed, heading_deg, lat, lon, long_accel, yaw_rate = interp

    outage_start, outage_end = args.outage_start_s, args.outage_start_s + args.outage_seconds
    cal_mask = (phone_t >= outage_start - args.calibration_seconds) & (phone_t < outage_start)
    outage_mask = (phone_t >= outage_start) & (phone_t <= outage_end)
    if int(cal_mask.sum()) < 100 or int(outage_mask.sum()) < 20:
        raise SystemExit("Selected calibration/outage interval is outside available paired data")
    fit_acc, fit_acc_r2, fit_acc_rmse, fit_acc_n = robust_linear_fit(linear_accel[cal_mask], long_accel[cal_mask])
    fit_gyro, fit_gyro_r2, fit_gyro_rmse, fit_gyro_n = robust_linear_fit(gyro[cal_mask], yaw_rate[cal_mask])

    lat0, lon0 = float(lat[outage_mask][0]), float(lon[outage_mask][0])
    ref_e_all, ref_n_all = local_xy(lat, lon, lat0, lon0)
    ref_e = ref_e_all[outage_mask]
    ref_n = ref_n_all[outage_mask]
    ref_speed = speed[outage_mask]
    ref_heading = heading_deg[outage_mask]
    if not np.isfinite(np.r_[ref_e, ref_n, ref_speed, ref_heading]).all():
        raise SystemExit("Reference data is incomplete during the selected outage")
    distance = float(np.sum(np.hypot(np.diff(ref_e), np.diff(ref_n))))
    duration = float(phone_t[outage_mask][-1] - phone_t[outage_mask][0])
    if distance < 10.0:
        raise SystemExit(f"Selected reference path is only {distance:.1f} m; choose a moving outage window")

    # Initialization and frame calibration use pre-outage paired vehicle data.
    # No vehicle values are passed to either filter after outage_start.
    initial_heading = VehicleFusionFilter.course_deg_to_heading_rad(float(ref_heading[0]))
    initial_speed = max(0.0, float(ref_speed[0]))
    filters = {
        "imu_no_nhc": VehicleFusionFilter(output_hz=10.0, initial_east_m=0.0, initial_north_m=0.0, initial_speed_mps=initial_speed, initial_heading_rad=initial_heading, initial_position_sigma_m=1.0, initial_velocity_sigma_mps=0.5),
        "imu_nhc": VehicleFusionFilter(output_hz=10.0, initial_east_m=0.0, initial_north_m=0.0, initial_speed_mps=initial_speed, initial_heading_rad=initial_heading, initial_position_sigma_m=1.0, initial_velocity_sigma_mps=0.5),
        "gyro_only_constant_speed": VehicleFusionFilter(output_hz=10.0, initial_east_m=0.0, initial_north_m=0.0, initial_speed_mps=initial_speed, initial_heading_rad=initial_heading, initial_position_sigma_m=1.0, initial_velocity_sigma_mps=0.5),
    }
    if speed_model is not None:
        filters["imu_nhc_speed_model"] = VehicleFusionFilter(output_hz=10.0, initial_east_m=0.0, initial_north_m=0.0, initial_speed_mps=initial_speed, initial_heading_rad=initial_heading, initial_position_sigma_m=1.0, initial_velocity_sigma_mps=0.5)
    if portable_model is not None:
        filters["imu_nhc_portable_speed"] = VehicleFusionFilter(output_hz=10.0, initial_speed_mps=initial_speed, initial_heading_rad=initial_heading, initial_position_sigma_m=1.0, initial_velocity_sigma_mps=0.5)
    next_portable_s = -math.inf
    portable_predictions = 0
    portable_ood = 0
    tracks: dict[str, list] = {name: [] for name in filters}
    inference_ms: dict[str, list[float]] = {name: [] for name in filters}
    base_e, base_n = 0.0, 0.0
    hold_e, hold_n = [], []
    outage_indices = np.flatnonzero(outage_mask)
    calibration_norm = np.linalg.norm(linear_accel[cal_mask], axis=1)
    shock_center = float(np.median(calibration_norm))
    shock_mad = float(np.median(np.abs(calibration_norm - shock_center)))
    shock_limit = max(shock_center + 8.0 * 1.4826 * shock_mad, shock_center + 2.0)
    pending_speed_update: tuple[int, float, float] | None = None
    next_speed_prediction_s = outage_start
    virtual_speed_updates = 0
    virtual_speed_predictions = 0
    speed_model_inference_ms: list[float] = []
    for local_index, i in enumerate(outage_indices):
        timestamp_ns = int(round(phone_t[i] * 1e9))
        forward_accel = float(linear_accel[i] @ fit_acc[:-1] + fit_acc[-1])
        phone_yaw_rate = float(gyro[i] @ fit_gyro[:-1] + fit_gyro[-1])
        shock = float(np.linalg.norm(linear_accel[i]))
        quality = 0.2 if shock > shock_limit else 1.0
        hold_e.append(initial_speed * math.sin(math.radians(float(ref_heading[0]))) * (phone_t[i] - outage_start))
        hold_n.append(initial_speed * math.cos(math.radians(float(ref_heading[0]))) * (phone_t[i] - outage_start))
        for name, estimator in filters.items():
            tick = time.perf_counter_ns()
            virtual_speed = virtual_variance = None
            if name == "imu_nhc_speed_model" and pending_speed_update is not None:
                due_ns, predicted_speed, prediction_variance = pending_speed_update
                if timestamp_ns >= due_ns:
                    virtual_speed, virtual_variance = predicted_speed, prediction_variance
                    pending_speed_update = None
                    virtual_speed_updates += 1
            if name == "imu_nhc_portable_speed" and phone_t[i] >= next_portable_s and quality >= 0.7 and i >= 19:
                intervals = np.diff(phone_t[i-19:i+1])
                if np.all(intervals > 0) and np.max(intervals) <= 0.15 and np.isfinite(model_features[i-19:i+1]).all():
                    prediction = portable_model.predict(model_features[i-19:i+1])
                    portable_predictions += 1
                    if prediction["out_of_domain"]:
                        portable_ood += 1
                    else:
                        virtual_speed = prediction["speed_mps"]
                        virtual_variance = 4.0 * prediction["variance_m2ps2"]
                    next_portable_s = phone_t[i] + 1.0
            output = estimator.process_imu(
                timestamp_ns,
                forward_accel_mps2=(0.0 if name == "gyro_only_constant_speed" else forward_accel),
                yaw_rate_radps=phone_yaw_rate,
                quality=quality,
                normal_driving=(name in {"imu_nhc", "gyro_only_constant_speed", "imu_nhc_speed_model", "imu_nhc_portable_speed"}),
                virtual_speed_mps=virtual_speed,
                virtual_speed_variance=virtual_variance,
            )
            inference_ms[name].append((time.perf_counter_ns() - tick) / 1e6)
            if output is None:
                continue
            tracks[name].append((output.timestamp_ns, output.east_m, output.north_m, output.speed_mps, output.heading_rad_from_east_ccw, output.horizontal_sigma_m))
            if name == "imu_nhc_speed_model" and speed_model is not None and phone_t[i] >= next_speed_prediction_s and quality >= 0.7:
                begin = i - speed_model.window_samples + 1
                if begin >= 0:
                    window_times = phone_t[begin:i + 1]
                    model_intervals = np.diff(window_times)
                    if len(window_times) == speed_model.window_samples and np.all(model_intervals > 0) and np.max(model_intervals) <= 1.5 / speed_model.feature_rate_hz:
                        model_tick = time.perf_counter_ns()
                        prediction = speed_model.predict(model_features[begin:i + 1])
                        speed_model_inference_ms.append((time.perf_counter_ns() - model_tick) / 1e6)
                        predicted_future_speed = max(0.0, output.speed_mps + prediction.speed_delta_mps)
                        due_ns = timestamp_ns + int(round(prediction.horizon_seconds * 1e9))
                        pending_speed_update = (due_ns, predicted_future_speed, prediction.variance_m2ps2)
                        virtual_speed_predictions += 1
                        next_speed_prediction_s = phone_t[i] + prediction.horizon_seconds

    # Preserve one row per phone sample for easy inspection; filter outputs are
    # aligned at 10 Hz, while reference and constant-speed rows use phone cadence.
    rows = []
    for j, i in enumerate(outage_indices):
        row = {
            "elapsed_s": float(phone_t[i] - outage_start),
            "reference_east_m": float(ref_e[j]),
            "reference_north_m": float(ref_n[j]),
            "constant_speed_east_m": float(hold_e[j]),
            "constant_speed_north_m": float(hold_n[j]),
            "phone_forward_accel_mps2": float(linear_accel[i] @ fit_acc[:-1] + fit_acc[-1]),
            "phone_yaw_rate_radps": float(gyro[i] @ fit_gyro[:-1] + fit_gyro[-1]),
        }
        rows.append(row)
    result = pd.DataFrame(rows)
    for name, values in tracks.items():
        arr = np.asarray(values, dtype=float)
        if len(arr) == 0:
            raise SystemExit(f"Filter {name} emitted no states")
        output_t = arr[:, 0] / 1e9
        target_t = phone_t[outage_indices]
        result[f"{name}_east_m"] = np.interp(target_t, output_t, arr[:, 1])
        result[f"{name}_north_m"] = np.interp(target_t, output_t, arr[:, 2])
        result[f"{name}_speed_mps"] = np.interp(target_t, output_t, arr[:, 3])
        result[f"{name}_sigma_m"] = np.interp(target_t, output_t, arr[:, 5])

    metric_sets = {
        "constant_speed": trajectory_metrics(result.constant_speed_east_m.to_numpy(), result.constant_speed_north_m.to_numpy(), ref_e, ref_n, distance, duration),
        "imu_no_nhc": trajectory_metrics(result.imu_no_nhc_east_m.to_numpy(), result.imu_no_nhc_north_m.to_numpy(), ref_e, ref_n, distance, duration),
        "imu_nhc": trajectory_metrics(result.imu_nhc_east_m.to_numpy(), result.imu_nhc_north_m.to_numpy(), ref_e, ref_n, distance, duration),
        "gyro_only_constant_speed": trajectory_metrics(result.gyro_only_constant_speed_east_m.to_numpy(), result.gyro_only_constant_speed_north_m.to_numpy(), ref_e, ref_n, distance, duration),
    }
    if speed_model is not None:
        metric_sets["imu_nhc_speed_model"] = trajectory_metrics(
            result.imu_nhc_speed_model_east_m.to_numpy(), result.imu_nhc_speed_model_north_m.to_numpy(),
            ref_e, ref_n, distance, duration,
        )
    if portable_model is not None:
        metric_sets["imu_nhc_portable_speed"] = trajectory_metrics(result.imu_nhc_portable_speed_east_m.to_numpy(), result.imu_nhc_portable_speed_north_m.to_numpy(), ref_e, ref_n, distance, duration)
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{run_name}_outage_trajectory.csv"
    plot_path = output_dir / f"{run_name}_outage_trajectory.png"
    summary_path = output_dir / f"{run_name}_outage_metrics.json"
    result.to_csv(csv_path, index=False)

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    axes[0].plot(ref_e, ref_n, color="black", linewidth=2, label="Vehicle GNSS reference")
    axes[0].plot(result.constant_speed_east_m, result.constant_speed_north_m, label="Constant speed")
    axes[0].plot(result.imu_no_nhc_east_m, result.imu_no_nhc_north_m, label="Phone IMU, no NHC")
    axes[0].plot(result.imu_nhc_east_m, result.imu_nhc_north_m, label="Phone IMU + NHC")
    axes[0].plot(result.gyro_only_constant_speed_east_m, result.gyro_only_constant_speed_north_m, label="Gyro + constant speed + NHC")
    if speed_model is not None:
        axes[0].plot(result.imu_nhc_speed_model_east_m, result.imu_nhc_speed_model_north_m, label="Phone IMU + NHC + candidate speed")
    if portable_model is not None:
        axes[0].plot(result.imu_nhc_portable_speed_east_m, result.imu_nhc_portable_speed_north_m, label="IMU + NHC + absolute-speed MLP")
    axes[0].scatter([0], [0], marker="o", s=40, color="green", label="Outage start")
    axes[0].set_title(f"Synthetic GNSS outage trajectory · {run_name}")
    axes[0].set_xlabel("East from outage start (m)")
    axes[0].set_ylabel("North from outage start (m)")
    axes[0].axis("equal")
    axes[0].grid(alpha=0.25)
    axes[0].legend()
    elapsed = result.elapsed_s.to_numpy()
    plot_methods = [("constant_speed", "Straight constant speed"), ("imu_no_nhc", "Phone IMU, no NHC"), ("imu_nhc", "Phone IMU + NHC"), ("gyro_only_constant_speed", "Gyro + constant speed + NHC")]
    if speed_model is not None:
        plot_methods.append(("imu_nhc_speed_model", "Phone IMU + NHC + candidate speed"))
    if portable_model is not None:
        plot_methods.append(("imu_nhc_portable_speed", "IMU + NHC + absolute-speed MLP"))
    for name, title in plot_methods:
        e_col, n_col = f"{name}_east_m", f"{name}_north_m"
        error = np.hypot(result[e_col].to_numpy() - ref_e, result[n_col].to_numpy() - ref_n)
        axes[1].plot(elapsed, error, label=title)
    axes[1].set_title("Horizontal position error during outage")
    axes[1].set_xlabel("Outage elapsed time (s)")
    axes[1].set_ylabel("Error (m)")
    axes[1].grid(alpha=0.25)
    axes[1].legend()
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    summary = {
        "dataset": f"IO-VNBD synchronized categorized {run_name}",
        "phone_csv": str(args.phone_csv.resolve()),
        "vehicle_csv_reference_and_pre_outage_calibration": str(args.vehicle_csv.resolve()),
        "outage_start_s_from_phone_log": outage_start,
        "outage_duration_s": duration,
        "outage_samples": int(len(result)),
        "phone_sample_rate_hz_median": float(1.0 / np.median(np.diff(phone_t[outage_mask]))),
        "phone_elapsed_time_source": "monotonic wall-clock timestamps; session timer retained for diagnostics because it may reset",
        "phone_session_timer_reset_count": phone_timer_reset_count,
        "filter_output_rate_hz": 10.0,
        "phone_vehicle_clock_offset_s": clock_offset,
        "phone_vehicle_clock_residual_p95_s": clock_p95,
        "pre_outage_axis_calibration": {
            "duration_s": args.calibration_seconds,
            "forward_acceleration_phone_axis_coefficients": fit_acc[:-1].tolist(),
            "forward_acceleration_intercept_mps2": float(fit_acc[-1]),
            "forward_acceleration_calibration_r2": fit_acc_r2,
            "forward_acceleration_calibration_rmse_mps2": fit_acc_rmse,
            "forward_acceleration_calibration_samples_used": fit_acc_n,
            "yaw_rate_phone_axis_coefficients": fit_gyro[:-1].tolist(),
            "yaw_rate_intercept_radps": float(fit_gyro[-1]),
            "yaw_rate_calibration_r2": fit_gyro_r2,
            "yaw_rate_calibration_rmse_radps": fit_gyro_rmse,
            "yaw_rate_calibration_samples_used": fit_gyro_n,
        },
        "reference_distance_m": distance,
        "metrics_by_method": metric_sets,
        "offline_speed_model_evaluation": None if speed_model is None else {
            "source_model_json": str(args.evaluate_speed_model.resolve()),
            "format": "linear past-only speed delta",
            "window_samples": speed_model.window_samples,
            "feature_rate_hz": speed_model.feature_rate_hz,
            "horizon_seconds": speed_model.horizon_seconds,
            "predictions_scheduled": virtual_speed_predictions,
            "predicted_speed_updates_applied": virtual_speed_updates,
            "python_inference_latency_ms_p50": float(np.percentile(speed_model_inference_ms, 50)) if speed_model_inference_ms else None,
            "python_inference_latency_ms_p95": float(np.percentile(speed_model_inference_ms, 95)) if speed_model_inference_ms else None,
            "artifact_eligible_for_fusion": speed_model.eligible_for_fusion,
            "note": "Explicit offline what-if evaluation only. This replay does not authorize deployment; compare untouched held-out real outage windows and keep raw tracks separate.",
        },
        "portable_speed_what_if": None if portable_model is None else {
            "model": str(args.evaluate_portable_speed), "predictions": portable_predictions,
            "out_of_domain_rejections": portable_ood, "variance_multiplier": 4.0,
            "artifact_eligible_for_fusion": portable_model.doc.get("eligible_for_fusion", False),
            "note": "Offline experiment overrides promotion flag only here; no deployment authorization"
        },
        "desktop_filter_call_latency_ms": {
            name: {
                "p50": float(np.percentile(samples, 50)),
                "p95": float(np.percentile(samples, 95)),
                "max": float(np.max(samples)),
                "note": "Python desktop replay timing only; not a smartphone or 200 Hz edge benchmark.",
            }
            for name, samples in inference_ms.items()
        },
        "limitations": [
            "GNSS outage is simulated in replay, not a real GNSS-denied collection.",
            "Vehicle GNSS/odometry is an onboard reference, not surveyed ground truth.",
            "Vehicle labels are used before outage to calibrate phone-to-vehicle sensor axes and initialize position, speed and heading; they are not inputs during the outage.",
            "The speed model, when supplied, is evaluated offline as a separate what-if candidate. Map matching is not used to score the raw trajectory.",
            "The selected IO-VNBD smartphone stream is nominally 10 Hz and cannot establish 200 Hz edge performance.",
        ],
        "artifacts": {"trajectory_csv": str(csv_path), "trajectory_plot": str(plot_path)},
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved trajectory CSV: {csv_path}")
    print(f"Saved plot: {plot_path}")
    print(f"Saved metrics: {summary_path}")


if __name__ == "__main__":
    main()
