"""Vendor-neutral external IMU adapter and high-rate edge navigation wrapper.

This module defines the boundary between a sensor-specific SDK/logger and the
shared vehicle fusion filter. It does not infer mount alignment or attitude:
those must be calibrated by the caller and described in ``ExternalImuConfig``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Literal

from .fusion import NavigationOutput, VehicleFusionFilter
from .map_matching import ConfidenceGatedMapMatcher, MapMatchResult, MapObservation


AccelerationUnit = Literal["m/s2", "g"]
AngularRateUnit = Literal["rad/s", "deg/s"]
TimestampUnit = Literal["ns", "us", "ms", "s"]


@dataclass(frozen=True)
class ExternalImuSample:
    """One timestamped sensor-frame accelerometer and gyroscope observation.

    ``timestamp`` uses the unit declared by the adapter config. Acceleration
    and angular rate are three-axis sensor-frame vectors in the corresponding
    declared units. If the accelerometer includes gravity, provide either the
    per-sample ``gravity_specific_force_sensor`` or a static value in config.
    """

    timestamp: int | float
    acceleration: tuple[float, float, float]
    angular_rate: tuple[float, float, float]
    gravity_specific_force_sensor: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class ExternalImuConfig:
    """Metadata required to normalize one external IMU source.

    ``sensor_to_vehicle_rotation`` is a row-major proper rotation matrix. Its
    output axes are vehicle x-forward, y-left, z-up. Bias and gravity vectors
    are expressed in SI units in the source sensor frame. ``timestamp_offset_ns``
    maps the source clock into the same monotonic nanosecond clock used by GNSS.
    """

    acceleration_unit: AccelerationUnit
    angular_rate_unit: AngularRateUnit
    timestamp_unit: TimestampUnit
    sensor_to_vehicle_rotation: tuple[float, ...] = (
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    )
    accelerometer_bias_mps2_sensor: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_bias_radps_sensor: tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration_is_gravity_compensated: bool = False
    gravity_specific_force_sensor_mps2: tuple[float, float, float] | None = None
    timestamp_offset_ns: int = 0
    target_imu_hz: float = 200.0


@dataclass(frozen=True)
class VehicleImuSample:
    """Normalized inputs consumed by ``VehicleFusionFilter``."""

    timestamp_ns: int
    forward_accel_mps2: float
    yaw_rate_radps: float


@dataclass(frozen=True)
class EdgeDiagnostics:
    samples_seen: int
    measured_input_hz: float | None
    interval_median_ms: float | None
    interval_rms_jitter_ms: float | None
    intervals_over_gap_limit: int
    output_samples: int = 0
    measured_output_hz: float | None = None
    processing_latency_p95_ms: float | None = None


@dataclass(frozen=True)
class EdgeNavigationResult:
    state: NavigationOutput
    diagnostics: EdgeDiagnostics
    processing_latency_ms: float


_ACCEL_SCALE = {"m/s2": 1.0, "g": 9.80665}
_GYRO_SCALE = {"rad/s": 1.0, "deg/s": math.pi / 180.0}
_TIME_SCALE_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}


def _vector3(values: tuple[float, float, float], name: str) -> tuple[float, float, float]:
    if len(values) != 3 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{name} must contain three finite values")
    return tuple(float(value) for value in values)


class ExternalImuAdapter:
    """Convert external IMU observations to vehicle-forward SI samples."""

    def __init__(self, config: ExternalImuConfig) -> None:
        if config.acceleration_unit not in _ACCEL_SCALE:
            raise ValueError(f"unsupported acceleration unit: {config.acceleration_unit}")
        if config.angular_rate_unit not in _GYRO_SCALE:
            raise ValueError(f"unsupported angular-rate unit: {config.angular_rate_unit}")
        if config.timestamp_unit not in _TIME_SCALE_NS:
            raise ValueError(f"unsupported timestamp unit: {config.timestamp_unit}")
        if not math.isfinite(config.target_imu_hz) or config.target_imu_hz <= 0:
            raise ValueError("target_imu_hz must be positive and finite")
        if len(config.sensor_to_vehicle_rotation) != 9:
            raise ValueError("sensor_to_vehicle_rotation must be a row-major 3x3 matrix")
        matrix = tuple(float(value) for value in config.sensor_to_vehicle_rotation)
        if not all(math.isfinite(value) for value in matrix):
            raise ValueError("sensor_to_vehicle_rotation must be finite")
        rows = (matrix[0:3], matrix[3:6], matrix[6:9])
        for i in range(3):
            for j in range(3):
                dot = sum(rows[i][k] * rows[j][k] for k in range(3))
                if abs(dot - (1.0 if i == j else 0.0)) > 1e-4:
                    raise ValueError("sensor_to_vehicle_rotation must be orthonormal")
        determinant = (
            rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
            - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
            + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
        )
        if abs(determinant - 1.0) > 1e-4:
            raise ValueError("sensor_to_vehicle_rotation must be a proper rotation (determinant +1)")
        if config.acceleration_is_gravity_compensated and config.gravity_specific_force_sensor_mps2 is not None:
            raise ValueError("do not configure gravity subtraction when acceleration is already compensated")
        self.config = config
        self._matrix = rows
        self._accel_bias = _vector3(config.accelerometer_bias_mps2_sensor, "accelerometer bias")
        self._gyro_bias = _vector3(config.gyro_bias_radps_sensor, "gyroscope bias")
        self._gravity = (
            None if config.gravity_specific_force_sensor_mps2 is None
            else _vector3(config.gravity_specific_force_sensor_mps2, "gravity specific force")
        )
        self._last_timestamp_ns: int | None = None

    def normalize(self, sample: ExternalImuSample) -> VehicleImuSample:
        """Normalize units, gravity, axes, biases and timestamp for one sample."""
        if not math.isfinite(float(sample.timestamp)):
            raise ValueError("sample timestamp must be finite")
        source_timestamp_ns = round(float(sample.timestamp) * _TIME_SCALE_NS[self.config.timestamp_unit])
        timestamp_ns = source_timestamp_ns + self.config.timestamp_offset_ns
        if timestamp_ns < 0:
            raise ValueError("normalized timestamp must be non-negative")
        if self._last_timestamp_ns is not None and timestamp_ns <= self._last_timestamp_ns:
            raise ValueError("external IMU timestamps must be strictly increasing after clock mapping")

        accel_scale = _ACCEL_SCALE[self.config.acceleration_unit]
        gyro_scale = _GYRO_SCALE[self.config.angular_rate_unit]
        accel = tuple(float(v) * accel_scale - b for v, b in zip(_vector3(sample.acceleration, "acceleration"), self._accel_bias))
        gyro = tuple(float(v) * gyro_scale - b for v, b in zip(_vector3(sample.angular_rate, "angular rate"), self._gyro_bias))
        if not self.config.acceleration_is_gravity_compensated:
            gravity = sample.gravity_specific_force_sensor or self._gravity
            if gravity is None:
                raise ValueError("missing gravity vector for this gravity-included accelerometer sample")
            gravity = _vector3(gravity, "per-sample gravity specific force")
            accel = tuple(accel[i] - gravity[i] for i in range(3))

        accel_vehicle = tuple(sum(self._matrix[i][j] * accel[j] for j in range(3)) for i in range(3))
        gyro_vehicle = tuple(sum(self._matrix[i][j] * gyro[j] for j in range(3)) for i in range(3))
        self._last_timestamp_ns = timestamp_ns
        return VehicleImuSample(timestamp_ns, accel_vehicle[0], gyro_vehicle[2])


class EdgeNavigationEngine:
    """High-rate adapter + fusion wrapper; defaults to the 200 Hz edge target.

    Call ``process_imu`` for every external sensor observation. The estimator
    propagates at each delivered sample and emits states on its configured
    timestamp grid. A 200 Hz configuration does not prove 200 Hz operation:
    use ``diagnostics`` and wall-clock processing latency on genuine source data.
    """

    def __init__(
        self,
        adapter: ExternalImuAdapter,
        *,
        output_hz: float = 200.0,
        map_matcher: ConfidenceGatedMapMatcher | None = None,
    ) -> None:
        if not math.isfinite(output_hz) or output_hz <= 0:
            raise ValueError("output_hz must be positive and finite")
        self.adapter = adapter
        self.filter = VehicleFusionFilter(output_hz=output_hz)
        self.map_matcher = map_matcher
        self._nominal_period_ns = 1e9 / adapter.config.target_imu_hz
        self._timestamps = deque(maxlen=256)
        self._output_timestamps = deque(maxlen=256)
        self._latencies_ms = deque(maxlen=512)
        self._samples_seen = 0
        self._outputs_seen = 0
        self._gap_count = 0

    def diagnostics(self) -> EdgeDiagnostics:
        intervals_ms = [
            (right - left) / 1e6
            for left, right in zip(self._timestamps, list(self._timestamps)[1:])
        ]
        if not intervals_ms:
            return EdgeDiagnostics(self._samples_seen, None, None, None, self._gap_count,
                                   self._outputs_seen, None, self._latency_p95())
        ordered = sorted(intervals_ms)
        mid = len(ordered) // 2
        median = ordered[mid] if len(ordered) % 2 else (ordered[mid - 1] + ordered[mid]) / 2.0
        mean = sum(intervals_ms) / len(intervals_ms)
        jitter = math.sqrt(sum((interval - mean) ** 2 for interval in intervals_ms) / len(intervals_ms))
        duration_ns = self._timestamps[-1] - self._timestamps[0]
        measured_hz = (len(self._timestamps) - 1) * 1e9 / duration_ns if duration_ns > 0 else None
        output_hz = None
        if len(self._output_timestamps) > 1 and self._output_timestamps[-1] > self._output_timestamps[0]:
            output_hz = (len(self._output_timestamps) - 1) * 1e9 / (self._output_timestamps[-1] - self._output_timestamps[0])
        return EdgeDiagnostics(self._samples_seen, measured_hz, median, jitter, self._gap_count,
                               self._outputs_seen, output_hz, self._latency_p95())

    def _latency_p95(self) -> float | None:
        if not self._latencies_ms:
            return None
        ordered = sorted(self._latencies_ms)
        return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]

    def process_imu(
        self,
        sample: ExternalImuSample,
        *,
        quality: float = 1.0,
        normal_driving: bool = True,
        stationary: bool = False,
    ) -> EdgeNavigationResult | None:
        """Consume one sample; return a navigation update when its cadence is due."""
        started_ns = time.perf_counter_ns()
        normalized = self.adapter.normalize(sample)
        if self._timestamps:
            interval_ns = normalized.timestamp_ns - self._timestamps[-1]
            if interval_ns > self._nominal_period_ns * 1.5:
                self._gap_count += 1
        self._timestamps.append(normalized.timestamp_ns)
        self._samples_seen += 1
        state = self.filter.process_imu(
            normalized.timestamp_ns,
            forward_accel_mps2=normalized.forward_accel_mps2,
            yaw_rate_radps=normalized.yaw_rate_radps,
            quality=quality,
            normal_driving=normal_driving,
            stationary=stationary,
        )
        if state is None:
            return None
        latency_ms = (time.perf_counter_ns() - started_ns) / 1e6
        self._output_timestamps.append(state.timestamp_ns)
        self._latencies_ms.append(latency_ms)
        self._outputs_seen += 1
        return EdgeNavigationResult(state, self.diagnostics(), latency_ms)

    def update_gnss(
        self,
        timestamp_ns: int,
        *,
        east_m: float,
        north_m: float,
        horizontal_sigma_m: float,
        speed_mps: float | None = None,
        course_deg_north_clockwise: float | None = None,
    ) -> bool:
        """Apply an already projected local ENU GNSS fix to the edge filter."""
        return self.filter.update_gnss(
            timestamp_ns,
            east_m=east_m,
            north_m=north_m,
            horizontal_sigma_m=horizontal_sigma_m,
            speed_mps=speed_mps,
            course_deg_north_clockwise=course_deg_north_clockwise,
        )

    def match_map_observation(self, observation: MapObservation) -> MapMatchResult | None:
        """Return an optional, separate road-matched fix without altering the filter state."""
        if self.map_matcher is None:
            return None
        return self.map_matcher.update(observation)
