"""Small device-rotation-invariant IMU model; identical feature order on Android.

Inputs: 20 consecutive 10 Hz rows, accelerometer XYZ, gravity XYZ, linear
acceleration XYZ and gyro XYZ, in SI units. Norms avoid assuming IO-VNBD's
gyro channel naming equals Android's axis naming. Outputs are experimental.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

CHANNELS = ["accel_norm", "gravity_norm", "linear_norm", "gyro_norm"]
FEATURES = [f"{stat}_{channel}" for stat in ("mean", "std", "min", "max", "last") for channel in CHANNELS]

def invariant_channels(raw):
    raw = np.asarray(raw, dtype=np.float64)
    return np.column_stack([np.linalg.norm(raw[:, i:i+3], axis=1) for i in (0, 3, 6, 9)])

def features(raw):
    c = invariant_channels(raw)
    return np.concatenate([c.mean(0), c.std(0), c.min(0), c.max(0), c[-1]])

class PortableSpeedModel:
    def __init__(self, document):
        if document.get("format") != "seamlessnav-invariant-mlp-v1" or document.get("feature_names") != FEATURES:
            raise ValueError("unsupported model contract")
        self.doc = document
        self.mean = np.array(document["mean"])
        self.scale = np.array(document["scale"])
        self.w1 = np.array(document["w1"])
        self.b1 = np.array(document["b1"])
        self.w2 = np.array(document["w2"])
        self.b2 = float(document["b2"])
        self.noise = np.array(document["noise_coefficients"])
        arrays = [self.mean,self.scale,self.w1,self.b1,self.w2,self.noise,np.array([self.b2,document['target_mean'],document['target_scale'],document['variance_calibration']])]
        if any(not np.isfinite(a).all() for a in arrays) or np.any(self.scale <= 0):
            raise ValueError("nonfinite weights or invalid scaling")
        if self.mean.shape != (20,) or self.scale.shape != (20,) or self.w1.shape != (20,len(self.b1)) or self.w2.shape != self.b1.shape or self.noise.shape != (21,):
            raise ValueError("invalid weight shapes")
        if document.get('window_samples') != 20 or document.get('feature_rate_hz') != 10 or document['target_scale'] <= 0 or document['variance_calibration'] <= 0:
            raise ValueError("invalid temporal contract or output scaling")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def predict(self, raw):
        raw = np.asarray(raw, dtype=float)
        if raw.shape != (20,12) or not np.isfinite(raw).all():
            raise ValueError("requires a finite 20 x 12 IMU window")
        return self.predict_channels(invariant_channels(raw))

    def predict_channels(self, channels):
        c = np.asarray(channels, dtype=float)
        if c.shape != (20,4) or not np.isfinite(c).all():
            raise ValueError("requires a finite 20 x 4 magnitude window")
        f = np.concatenate([c.mean(0), c.std(0), c.min(0), c.max(0), c[-1]])
        z = (f-self.mean)/self.scale
        ood = bool(np.any(np.abs(z)>8))
        z = np.clip(z,-8,8)
        normalized = np.tanh(z@self.w1+self.b1)@self.w2+self.b2
        speed = float(np.clip(normalized*self.doc["target_scale"]+self.doc["target_mean"],0,60))
        log_var = np.clip(z@self.noise[:-1]+self.noise[-1],-4,8)
        variance = float(np.clip(np.exp(log_var)*self.doc["variance_calibration"],0.25,400))
        return {"speed_mps":speed,"variance_m2ps2":variance,"out_of_domain":ood,
                "eligible_for_fusion":bool(self.doc.get("eligible_for_fusion",False)) and not ood,
                "vibration_rms_mps2":float(c[:,2].std())}


class PortableSpeedStream:
    """Same causal 100 ms completed-bin inference as the Android implementation."""
    def __init__(self, model):
        from collections import deque
        self.model = model
        self.window = deque(maxlen=20)
        self.bucket = None
        self.last_ns = None
        self.total = np.zeros(4)
        self.count = 0
        self.latest = None

    def add_sample(self, timestamp_ns, raw):
        raw = np.asarray(raw, dtype=float)
        if raw.shape != (12,) or not np.isfinite(raw).all():
            raise ValueError("invalid raw speed-model sample")
        if self.last_ns is not None and timestamp_ns <= self.last_ns:
            raise ValueError("model timestamps must increase")
        if self.last_ns is not None and timestamp_ns - self.last_ns > 250_000_000:
            self.window.clear(); self.count=0; self.total.fill(0); self.latest=None; self.bucket=None
        self.last_ns = timestamp_ns
        current = timestamp_ns // 100_000_000
        prediction = None
        if self.bucket is not None and current != self.bucket:
            if current == self.bucket + 1 and self.count:
                self.window.append(self.total / self.count)
                if len(self.window) == 20:
                    prediction = self.model.predict_channels(self.window)
                    prediction['timestamp_ns'] = current * 100_000_000
                    self.latest = prediction
            else:
                self.window.clear(); self.latest=None
            self.count=0; self.total.fill(0)
        self.bucket=current
        self.total += np.linalg.norm(raw.reshape(4,3), axis=1)
        self.count += 1
        return prediction
