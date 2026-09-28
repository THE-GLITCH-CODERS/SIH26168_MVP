# Desktop / edge navigation core (prototype)

`VehicleFusionFilter` is the shared planar GNSS/IMU reference filter. It consumes timestamp-ordered vehicle-forward, gravity-compensated acceleration and vehicle-up yaw rate, propagates on every IMU event, and applies gated GNSS position/velocity corrections. Conditional ZUPT/NHC updates depend on caller-supplied motion quality. Learned virtual speed is still held out of the live navigation path.

`ExternalImuAdapter` in `edge.py` is the vendor-neutral external sensor boundary. It converts declared acceleration/gyro units, sensor-to-vehicle rotation, SI sensor biases, optional gravity specific force, and source timestamps plus clock offset into the filter's input. It rejects non-monotonic timestamps and invalid frame transforms rather than silently repairing them. The source clock must be mapped to the same monotonic nanosecond timebase used by GNSS.

Example for a device that already reports gravity-compensated linear acceleration:

```python
from navcore.edge import EdgeNavigationEngine, ExternalImuAdapter, ExternalImuConfig

adapter = ExternalImuAdapter(ExternalImuConfig(
    acceleration_unit="m/s2",
    angular_rate_unit="rad/s",
    timestamp_unit="ns",
    sensor_to_vehicle_rotation=(1, 0, 0, 0, 1, 0, 0, 0, 1),
    acceleration_is_gravity_compensated=True,
    target_imu_hz=200,
))
engine = EdgeNavigationEngine(adapter, output_hz=200)
```

For a raw accelerometer, set `acceleration_is_gravity_compensated=False` and provide its gravity specific-force vector either in the config or per sample. A constant vector is only appropriate when the mount/attitude justifies it; use a per-sample attitude-derived vector when pitch/roll changes matter. Rotation maps sensor axes to vehicle x-forward, y-left, z-up. PPC's published IMU convention is x-forward, y-right, z-down, so its sensor-to-vehicle rotation is `diag(1, -1, -1)`; its CSV units and gravity handling still need to be confirmed from the downloaded file before replay. The current 2D filter uses forward linear acceleration and vehicle-up yaw rate only; full 3D strapdown attitude is not implemented.

`EdgeNavigationEngine.process_imu` must receive every sensor sample. It defaults to a 200 Hz output time grid, reports measured input cadence, median interval, RMS interval jitter, long-gap count, and per-update processing latency, and accepts GNSS already projected to local east/north meters. Set output to 10 Hz for the phone contract. The requested/selected rate is not evidence of achieved rate: validate using genuine external-IMU data and measured cadence/latency. IO-VNBD's nominal 10 Hz data cannot prove 200 Hz; PPC's 100 Hz IMU can validate the adapter below the target but is not a FOG benchmark.

`LinearSpeedDeltaModel` reads the exported compact JSON and predicts from the exact 10 Hz feature-window contract. It returns the candidate delta and validation variance, but does not fuse it. Exported candidates remain explicitly `eligible_for_fusion: false`; `scripts/replay_iovnbd_outage.py --evaluate-speed-model ...` supports an offline what-if position evaluation before any promotion decision.

`ConfidenceGatedMapMatcher` reads a local OSM XML extract or prepared regional road-graph JSON. It returns a separate matched coordinate with confidence/abstention metadata and never rewrites the fusion output. The Android app can load prepared graph JSON and display a separate distance/heading-scored match; only Python/edge has route-continuity HMM scoring. Sensor-specific SDK/serial adapters, full 3D gravity compensation, route-level map validation, and a verified SIH under-10% outage result remain open. This remains a prototype reference, not a validated navigation product.
