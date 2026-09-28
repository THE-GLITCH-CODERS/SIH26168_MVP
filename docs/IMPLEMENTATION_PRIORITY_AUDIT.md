# Implementation audit and priority order

Checked against the SIH expected capabilities and current source on 2026-09-28, after connecting the first Kotlin phone navigation path.

## Current state

| Expected capability | Current implementation | Gap / consequence |
|---|---|---|
| Phone IMU/GNSS collection | Android app enumerates sensors, requests 100 Hz, records monotonic events and GNSS fixes, reports observed rates, and exports CSV/profile ZIP. | Latest OnePlus capture contains 33 GPS callbacks over 35.6 s, with a final reported accuracy of 3.79 m. It verifies stationary GNSS acquisition, not moving-route navigation. |
| Vehicle alignment and motion quality | Kotlin processor estimates stationary gyro bias, rotation-vector tilt, GNSS-course yaw offset, motion events, and a quality score. Those quality outputs now adjust filter noise and gate NHC/ZUPT; recent motion state carries into a GNSS outage until stationary evidence appears. | Heuristic prototype only. No validated full phone-to-vehicle 3D transform or multi-device calibration. |
| Phone speed / stop estimation | Phone filter uses GNSS velocity while available and physical IMU propagation during outages. A corrected ridge speed model exists only as an offline candidate. | Test speed MAE is 5.94 m/s, test R² is -0.78, and outage integration fails the SIH limit. Candidate remains disconnected. Pre-fix speed-delta artifacts are invalid due the timestamp bug. |
| SUMO synthetic training branch | SUMO scenario runner, FCD-to-phone-IMU generator and leakage-guarded real-only vs real-plus-SUMO trainer are implemented. Synthetic rows are train-only and validation/test remain real IO-VNBD drives. | SUMO is not installed here, so no simulated data or comparison metrics exist yet. Candidate still requires held-out real outage-position gains before fusion. |
| Dead reckoning and 10 Hz output | Kotlin phone filter consumes phone IMU and publishes GNSS-anchored position, velocity, heading, uncertainty and GNSS/DR mode on a 10 Hz target. | App was built, installed and launched; no moving-route or outage capture has verified state quality or actual output cadence. |
| GNSS/INS fusion and transition | GNSS position/speed/course updates feed the phone filter; mode changes to DR when the last accepted GNSS update ages out. | Needs live reacquisition/outage validation and better GNSS integrity/hysteresis. |
| Offline map matching | Python edge/offline HMM plus Android prepared-JSON loading and a separate confidence-gated map hypothesis beside live navigation. Neither path overwrites the raw filter state. | Android currently scores spatial candidates by distance/heading without graph continuity; no OSM map extract, rendered map, or route-validation result is included. |
| External-IMU edge adapter | `navcore/edge.py` normalizes declared units, timestamps/offset, sensor-to-vehicle rotation, bias and gravity input, then forwards vehicle-forward acceleration and yaw rate to the reference filter. Cadence, interval jitter, long gaps and processing latency are surfaced. | Sensor-specific hardware/data adapters and real-source replay remain. The edge output grid defaults to 200 Hz, but no genuine 200 Hz/FOG cadence or latency result exists yet. |
| Under-10% outage drift | One 59.9 s synthetic S1 outage was evaluated. Phone IMU + NHC endpoint error is 458.9 m over a 348.2 m reference path (131.8%). | Fails the target by a wide margin. This is a negative replay against onboard vehicle GNSS, not surveyed truth. |

## Priority implementation plan

### P0 — complete the phone smoke run

The recent capture verifies phone sensor logging and stationary GPS acquisition. Next, capture a securely mounted moving route and controlled GNSS outage only when safe. Check that live output advances near 10 Hz, GNSS-aided/DR mode changes sensibly, and exported capture timestamps remain monotonic.

### P1 — validate the live filter before tuning models

Compare phone filter output to a GNSS reference on the moving capture; inspect heading, acceleration projection, uncertainty growth, and GNSS reacquisition. Replay the exact same held-out outage intervals in Python and Kotlin. Do not fuse learned speed unless it improves held-out speed and integrated endpoint, maximum-error, and RMSE metrics.

### P1 — complete the external-IMU target

The vendor-neutral normalization and cadence-monitoring interface is implemented in `navcore/edge.py`. Replay PPC's real 100 Hz source first, then obtain a genuine 200 Hz/FOG stream and measure cadence/latency. Do not claim 200 Hz from a configured output grid, phone input, or upsampled data.

### P2 — validate confidence-gated offline map matching

Prepare a small OSM extract for the recorded route, compare the Android display-only candidates with the Python route-continuity HMM, and log both map-free and post-map tracks so matching cannot hide the raw drift score.

## Device status

The updated debug APK was installed and launched on OnePlus CPH2723 (`1d52435c`). The 2026-09-28 capture verifies GPS fixes in an outdoor/stationary session. Live moving-route output, GNSS outage transition and actual 10 Hz cadence still need a controlled drive capture.
