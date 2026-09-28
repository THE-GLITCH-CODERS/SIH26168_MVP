"""Small 2D vehicle error-state filter reference implementation.

Inputs are already normalized into a local ENU frame and vehicle-forward axis.
This is an offline/edge reference core, not yet wired into the Android app.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class NavigationOutput:
    timestamp_ns: int
    east_m: float
    north_m: float
    east_velocity_mps: float
    north_velocity_mps: float
    speed_mps: float
    heading_rad_from_east_ccw: float
    horizontal_sigma_m: float
    velocity_sigma_mps: float
    mode: str


class VehicleFusionFilter:
    """Asynchronous GNSS/IMU fusion with conditional vehicle constraints.

    State is local ENU meters and radians, ordered as east, north, east velocity,
    north velocity, heading (east/CCW), gyro-z bias, and forward-acceleration
    bias. Call ``process_imu`` for every sample and ``update_gnss`` when a fix
    arrives. Timestamps must be monotonically ordered on one monotonic clock.
    """

    _N = 7

    def __init__(
        self,
        *,
        output_hz: float = 10.0,
        initial_east_m: float = 0.0,
        initial_north_m: float = 0.0,
        initial_speed_mps: float = 0.0,
        initial_heading_rad: float = 0.0,
        initial_position_sigma_m: float = 10.0,
        initial_velocity_sigma_mps: float = 3.0,
    ) -> None:
        if not math.isfinite(output_hz) or output_hz <= 0:
            raise ValueError("output_hz must be positive and finite")
        self.output_period_ns = max(1, int(1e9 / output_hz))
        self.x = np.array(
            [
                initial_east_m,
                initial_north_m,
                initial_speed_mps * math.cos(initial_heading_rad),
                initial_speed_mps * math.sin(initial_heading_rad),
                self._wrap(initial_heading_rad),
                0.0,
                0.0,
            ],
            dtype=np.float64,
        )
        self.P = np.diag(
            [
                initial_position_sigma_m**2,
                initial_position_sigma_m**2,
                initial_velocity_sigma_mps**2,
                initial_velocity_sigma_mps**2,
                math.radians(30.0) ** 2,
                math.radians(2.0) ** 2,
                1.0**2,
            ]
        )
        self.last_timestamp_ns: int | None = None
        self.next_output_ns: int | None = None
        self.last_gnss_ns: int | None = None
        self.last_gnss_accepted = False
        self.last_gnss_reason = "no_fix"

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def course_deg_to_heading_rad(course_deg_north_clockwise: float) -> float:
        """Convert GNSS course (0° north, clockwise) to ENU math heading."""
        return VehicleFusionFilter._wrap(math.pi / 2.0 - math.radians(course_deg_north_clockwise))

    def _predict(self, dt: float, forward_accel: float, yaw_rate: float, quality: float) -> None:
        e, n, ve, vn, yaw, gyro_bias, accel_bias = self.x
        c, s = math.cos(yaw), math.sin(yaw)
        accel = forward_accel - accel_bias
        east_accel, north_accel = accel * c, accel * s

        self.x[0] = e + ve * dt + 0.5 * east_accel * dt * dt
        self.x[1] = n + vn * dt + 0.5 * north_accel * dt * dt
        self.x[2] = ve + east_accel * dt
        self.x[3] = vn + north_accel * dt
        self.x[4] = self._wrap(yaw + (yaw_rate - gyro_bias) * dt)

        F = np.eye(self._N, dtype=np.float64)
        F[0, 2] = dt
        F[1, 3] = dt
        F[0, 4] = -0.5 * accel * s * dt * dt
        F[1, 4] = 0.5 * accel * c * dt * dt
        F[2, 4] = -accel * s * dt
        F[3, 4] = accel * c * dt
        F[0, 6] = -0.5 * c * dt * dt
        F[1, 6] = -0.5 * s * dt * dt
        F[2, 6] = -c * dt
        F[3, 6] = -s * dt
        F[4, 5] = -dt

        q = min(max(quality, 0.0), 1.0)
        accel_sigma = 0.35 + (1.0 - q) * 4.0
        gyro_sigma = math.radians(0.5 + (1.0 - q) * 8.0)
        Q = np.diag(
            [
                (0.5 * accel_sigma * dt * dt) ** 2,
                (0.5 * accel_sigma * dt * dt) ** 2,
                (accel_sigma * dt) ** 2,
                (accel_sigma * dt) ** 2,
                (gyro_sigma * dt) ** 2,
                (math.radians(0.02) * math.sqrt(dt)) ** 2,
                (0.01 * math.sqrt(dt)) ** 2,
            ]
        )
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def _update(self, residual: np.ndarray, H: np.ndarray, R: np.ndarray, gate: float | None) -> tuple[bool, float]:
        innovation = H @ self.P @ H.T + R
        try:
            weighted = np.linalg.solve(innovation, residual)
        except np.linalg.LinAlgError:
            return False, math.inf
        distance = float(residual.T @ weighted)
        if not math.isfinite(distance) or (gate is not None and distance > gate):
            return False, distance
        K = np.linalg.solve(innovation, H @ self.P).T
        self.x = self.x + K @ residual
        self.x[4] = self._wrap(float(self.x[4]))
        identity_minus_kh = np.eye(self._N) - K @ H
        # Joseph form keeps covariance symmetric positive semidefinite under roundoff.
        self.P = identity_minus_kh @ self.P @ identity_minus_kh.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return True, distance

    def update_gnss(
        self,
        timestamp_ns: int,
        *,
        east_m: float,
        north_m: float,
        horizontal_sigma_m: float,
        speed_mps: float | None = None,
        course_deg_north_clockwise: float | None = None,
        speed_sigma_mps: float = 0.7,
        course_sigma_deg: float = 10.0,
    ) -> bool:
        """Apply a quality-weighted GNSS position and optional velocity fix."""
        vals = (east_m, north_m, horizontal_sigma_m)
        if not all(math.isfinite(v) for v in vals) or horizontal_sigma_m <= 0:
            raise ValueError("GNSS position and positive accuracy must be finite")
        if self.last_gnss_ns is not None and timestamp_ns < self.last_gnss_ns:
            raise ValueError("GNSS fixes must be delivered in timestamp order")
        if self.last_timestamp_ns is not None and timestamp_ns > self.last_timestamp_ns:
            raise ValueError("propagate IMU through the GNSS timestamp before applying that fix")
        sigma = min(max(horizontal_sigma_m, 1.0), 100.0)
        H = np.zeros((2, self._N))
        H[0, 0] = H[1, 1] = 1.0
        residual = np.array([east_m - self.x[0], north_m - self.x[1]])
        accepted, distance = self._update(residual, H, np.eye(2) * sigma**2, 9.21)
        self.last_gnss_accepted = accepted
        self.last_gnss_reason = "accepted" if accepted else f"innovation_gate:{distance:.2f}"
        if accepted:
            self.last_gnss_ns = timestamp_ns

        if accepted and speed_mps is not None and math.isfinite(speed_mps) and speed_mps >= 0:
            speed_sigma = min(max(speed_sigma_mps, 0.2), 10.0)
            if course_deg_north_clockwise is not None and speed_mps >= 1.0:
                course = math.radians(course_deg_north_clockwise)
                observed_velocity = np.array([speed_mps * math.sin(course), speed_mps * math.cos(course)])
                Hv = np.zeros((2, self._N))
                Hv[0, 2] = Hv[1, 3] = 1.0
                # Course uncertainty contributes cross-track velocity uncertainty.
                lateral_sigma = max(speed_mps * math.radians(course_sigma_deg), speed_sigma)
                along = np.array([math.sin(course), math.cos(course)])
                cross = np.array([math.cos(course), -math.sin(course)])
                basis = np.column_stack((along, cross))
                Rv = basis @ np.diag([speed_sigma**2, lateral_sigma**2]) @ basis.T
                self._update(observed_velocity - self.x[2:4], Hv, Rv, 13.82)
            else:
                heading = self.x[4]
                Hs = np.zeros((1, self._N))
                Hs[0, 2], Hs[0, 3] = math.cos(heading), math.sin(heading)
                Hs[0, 4] = -math.sin(heading) * self.x[2] + math.cos(heading) * self.x[3]
                predicted_speed = self.x[2] * math.cos(heading) + self.x[3] * math.sin(heading)
                self._update(np.array([speed_mps - predicted_speed]), Hs, np.array([[speed_sigma**2]]), 9.0)
        return accepted

    def update_virtual_speed(self, speed_mps: float, variance_m2ps2: float) -> bool:
        """Fuse a future learned speed head only in proportion to its uncertainty."""
        if not math.isfinite(speed_mps) or not math.isfinite(variance_m2ps2) or variance_m2ps2 <= 0:
            raise ValueError("Virtual speed and positive variance must be finite")
        yaw = self.x[4]
        H = np.zeros((1, self._N))
        H[0, 2], H[0, 3] = math.cos(yaw), math.sin(yaw)
        H[0, 4] = -math.sin(yaw) * self.x[2] + math.cos(yaw) * self.x[3]
        predicted_speed = self.x[2] * math.cos(yaw) + self.x[3] * math.sin(yaw)
        return self._update(
            np.array([speed_mps - predicted_speed]),
            H,
            np.array([[max(variance_m2ps2, 0.04)]]),
            9.0,
        )[0]

    def process_imu(
        self,
        timestamp_ns: int,
        *,
        forward_accel_mps2: float,
        yaw_rate_radps: float,
        quality: float = 1.0,
        normal_driving: bool = True,
        stationary: bool = False,
        virtual_speed_mps: float | None = None,
        virtual_speed_variance: float | None = None,
    ) -> NavigationOutput | None:
        """Propagate one IMU sample; return an output when its cadence is due.

        ``quality`` ranges from 0 (poor/shocked) to 1 (nominal) and scales process
        noise. ``normal_driving`` controls the non-holonomic constraint; callers
        should disable it during impact, slip, or uncertain motion. ``stationary``
        enables a zero-velocity update. A learned virtual speed and variance are
        optional until a model has been trained and validated.
        """
        if not math.isfinite(forward_accel_mps2) or not math.isfinite(yaw_rate_radps):
            raise ValueError("IMU inputs must be finite")
        if timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        if self.last_timestamp_ns is None:
            if self.last_gnss_ns is not None and timestamp_ns < self.last_gnss_ns:
                raise ValueError("IMU timestamp precedes the latest GNSS update")
            self.last_timestamp_ns = timestamp_ns
            self.next_output_ns = timestamp_ns
            dt = 0.0
        else:
            dt = (timestamp_ns - self.last_timestamp_ns) / 1e9
            if dt <= 0:
                raise ValueError("IMU timestamps must be strictly increasing")
            self.last_timestamp_ns = timestamp_ns
            # Substep long gaps; raise process noise for the whole interval.
            gap_quality = min(quality, 0.25 if dt > 0.25 else quality)
            steps = max(1, math.ceil(dt / 0.05))
            for _ in range(steps):
                self._predict(dt / steps, forward_accel_mps2, yaw_rate_radps, gap_quality)

        if stationary:
            H = np.zeros((2, self._N))
            H[0, 2] = H[1, 3] = 1.0
            self._update(-self.x[2:4].copy(), H, np.eye(2) * 0.04, None)
            if quality >= 0.5:
                # At rest, gravity-compensated forward acceleration and yaw rate
                # should be near zero; estimate persistent IMU biases cautiously.
                Hb = np.zeros((2, self._N))
                Hb[0, 5] = Hb[1, 6] = 1.0
                bias_sigma = 0.03 + (1.0 - min(max(quality, 0.0), 1.0)) * 0.25
                self._update(
                    np.array([yaw_rate_radps - self.x[5], forward_accel_mps2 - self.x[6]]),
                    Hb,
                    np.eye(2) * bias_sigma**2,
                    16.0,
                )
        elif normal_driving and math.hypot(self.x[2], self.x[3]) > 1.0:
            yaw = self.x[4]
            ve, vn = self.x[2], self.x[3]
            lateral_velocity = -math.sin(yaw) * ve + math.cos(yaw) * vn
            H = np.zeros((1, self._N))
            H[0, 2] = -math.sin(yaw)
            H[0, 3] = math.cos(yaw)
            H[0, 4] = -math.cos(yaw) * ve - math.sin(yaw) * vn
            constraint_sigma = 0.15 + (1.0 - min(max(quality, 0.0), 1.0)) * 2.0
            self._update(np.array([-lateral_velocity]), H, np.array([[constraint_sigma**2]]), None)

        if virtual_speed_mps is not None and virtual_speed_variance is not None:
            self.update_virtual_speed(virtual_speed_mps, virtual_speed_variance)

        if self.next_output_ns is None or timestamp_ns < self.next_output_ns:
            return None
        while self.next_output_ns <= timestamp_ns:
            self.next_output_ns += self.output_period_ns
        return self.snapshot(timestamp_ns)

    def snapshot(self, timestamp_ns: int | None = None) -> NavigationOutput:
        """Return state, uncertainty and GNSS/DR mode at the supplied time."""
        timestamp = self.last_timestamp_ns if timestamp_ns is None else timestamp_ns
        if timestamp is None:
            raise RuntimeError("No timestamp available; process an IMU sample first")
        gnss_age = math.inf if self.last_gnss_ns is None else max(0, timestamp - self.last_gnss_ns) / 1e9
        mode = "GNSS_AIDED" if gnss_age <= 1.5 else "DEAD_RECKONING"
        ve, vn = float(self.x[2]), float(self.x[3])
        return NavigationOutput(
            timestamp_ns=int(timestamp),
            east_m=float(self.x[0]),
            north_m=float(self.x[1]),
            east_velocity_mps=ve,
            north_velocity_mps=vn,
            speed_mps=math.hypot(ve, vn),
            heading_rad_from_east_ccw=float(self.x[4]),
            horizontal_sigma_m=math.sqrt(max(float(self.P[0, 0] + self.P[1, 1]), 0.0)),
            velocity_sigma_mps=math.sqrt(max(float(self.P[2, 2] + self.P[3, 3]), 0.0)),
            mode=mode,
        )
