# IO-VNBD synthetic GNSS outage replay v0

Run with `python scripts/replay_iovnbd_outage.py`. The default replay uses the
categorized synchronized S1 pair, a 30-second pre-outage calibration window,
and a 59.9-second outage starting 60 seconds after the phone log begins. The
phone stream's median sample cadence is 10 Hz, and the filter publishes at 10
Hz.

## Data use and leakage boundary

Before the simulated outage, the paired vehicle data calibrates a linear phone
accelerometer-to-forward-acceleration projection and phone-gyro-to-yaw-rate
projection. Vehicle position, speed, and heading initialize the state at the
outage boundary. During the outage, the filter receives only phone IMU values;
vehicle GNSS is used offline as the reference trajectory. The vehicle reference
is an onboard GNSS solution, not surveyed ground truth. This per-drive
pre-outage calibration uses information not available to a phone-only model
unless it is replaced with the intended online/self-calibration method, so this
replay does not establish cross-drive model generalization.

## Default S1 result

The vehicle-reference path covers 348.17 m during the 59.9-second outage.

| Method | Endpoint error | Trajectory RMSE | Endpoint error / distance |
|---|---:|---:|---:|
| Straight constant speed | 508.17 m | 245.43 m | 145.95% |
| Phone IMU, no NHC | 1,015.48 m | 484.52 m | 291.66% |
| Phone IMU + NHC | 458.90 m | 253.90 m | 131.80% |
| Phone gyro + constant speed + NHC | 447.29 m | 245.93 m | 128.47% |

None approaches the challenge's stated less-than-10% drift target. NHC helps
relative to the unconstrained accelerometer filter, but the remaining error is
large. The segment includes substantial speed changes; the current filters
have no validated stop/slowdown detector or virtual-speed correction. The
straight constant-speed baseline also cannot follow turns.

## Interpretation and next work

Treat this as a pipeline milestone and a failure baseline. It proves that the
timestamp alignment can drive a 10 Hz offline outage replay and produce
trajectory/error artifacts; it does not prove navigation readiness. Next,
improve stationary/slow-motion detection and phone-to-vehicle calibration,
evaluate multiple fixed outage windows on held-out drives, and compare
confidence-gated learned speed corrections. Keep pre-map dead-reckoning scores
separate from any map-matched track. The 10 Hz phone data cannot validate the
separate 200 Hz FOG edge target.

The run writes `S1_outage_trajectory.csv`, `S1_outage_trajectory.png`, and
`S1_outage_metrics.json` under the ignored
`outputs/iovnbd_outage_v0/` directory. The JSON contains exact configuration,
calibration diagnostics, clock residual, position metrics, and desktop timing.
