"""Portable inference reader for exported past-only speed-delta candidates.

Candidates can be evaluated from recorded 10 Hz feature windows, but models
marked ``eligible_for_fusion: false`` are never silently activated in a filter.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SpeedDeltaPrediction:
    speed_delta_mps: float
    variance_m2ps2: float
    horizon_seconds: float
    feature_rate_hz: float
    eligible_for_fusion: bool


class LinearSpeedDeltaModel:
    """Load the project's compact 75-coefficient speed-delta JSON artifact."""

    def __init__(self, model_document: dict) -> None:
        if model_document.get("format") != "seamlessnav-linear-speed-delta-v1":
            raise ValueError("unsupported speed model format")
        self.feature_mean = np.asarray(model_document.get("feature_mean"), dtype=np.float64)
        self.feature_scale = np.asarray(model_document.get("feature_scale"), dtype=np.float64)
        self.coefficients = np.asarray(model_document.get("coefficients"), dtype=np.float64)
        self.target_mean = float(model_document["target_mean_mps"])
        self.variance = float(model_document["residual_variance_m2ps2"])
        self.window_samples = int(model_document["window_samples"])
        self.feature_rate_hz = float(model_document.get("feature_rate_hz", 10.0))
        self.horizon_seconds = float(model_document["horizon_seconds"])
        self.prediction_clip_mps = float(model_document["prediction_clip_mps"])
        self.eligible_for_fusion = bool(model_document.get("eligible_for_fusion", False))
        expected_features = 75
        if any(array.shape != (expected_features,) for array in (self.feature_mean, self.feature_scale, self.coefficients)):
            raise ValueError(f"model must contain {expected_features} standardized features")
        channels = [
            "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
            "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
            "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
            "gyro_yaw_rps", "gyro_pitch_rps", "gyro_roll_rps",
            "accel_norm_mps2", "linear_accel_norm_mps2", "gyro_norm_rps",
        ]
        expected_names = [f"{stat}_{name}" for stat in ("mean", "std", "min", "max", "last") for name in channels]
        if model_document.get("feature_names") != expected_names:
            raise ValueError("model feature names/order do not match this runtime")
        if not np.isfinite(self.feature_mean).all() or not np.isfinite(self.feature_scale).all() or not np.isfinite(self.coefficients).all():
            raise ValueError("model arrays must be finite")
        if np.any(self.feature_scale <= 0) or not math.isfinite(self.target_mean):
            raise ValueError("model scaling/target mean is invalid")
        if not math.isfinite(self.variance) or self.variance <= 0:
            raise ValueError("model residual variance must be positive and finite")
        if self.window_samples < 2 or self.feature_rate_hz <= 0 or self.horizon_seconds <= 0:
            raise ValueError("model window/rate/horizon must be positive")

    @staticmethod
    def from_json(path: Path) -> "LinearSpeedDeltaModel":
        return LinearSpeedDeltaModel(json.loads(path.read_text(encoding="utf-8")))

    def predict(self, raw_window: np.ndarray) -> SpeedDeltaPrediction:
        """Predict future-minus-current speed from one exact past IMU window.

        Input rows use the training order: accel XYZ, gravity XYZ, linear
        acceleration XYZ, gyro yaw/pitch/roll. Input cadence must equal the
        model's declared ``feature_rate_hz`` and the window must end at inference
        time; this class does not resample, filter, or fuse the prediction.
        """
        window = np.asarray(raw_window, dtype=np.float64)
        if window.shape != (self.window_samples, 12) or not np.isfinite(window).all():
            raise ValueError(f"raw_window must be finite with shape ({self.window_samples}, 12)")
        expanded = np.column_stack((
            window,
            np.linalg.norm(window[:, 0:3], axis=1),
            np.linalg.norm(window[:, 6:9], axis=1),
            np.linalg.norm(window[:, 9:12], axis=1),
        ))
        vector = np.concatenate((
            expanded.mean(axis=0),
            expanded.std(axis=0),
            expanded.min(axis=0),
            expanded.max(axis=0),
            expanded[-1],
        ))
        standardized = (vector - self.feature_mean) / self.feature_scale
        delta = float(standardized @ self.coefficients + self.target_mean)
        delta = float(np.clip(delta, -self.prediction_clip_mps, self.prediction_clip_mps))
        return SpeedDeltaPrediction(
            speed_delta_mps=delta,
            variance_m2ps2=max(self.variance, 1e-6),
            horizon_seconds=self.horizon_seconds,
            feature_rate_hz=self.feature_rate_hz,
            eligible_for_fusion=self.eligible_for_fusion,
        )
