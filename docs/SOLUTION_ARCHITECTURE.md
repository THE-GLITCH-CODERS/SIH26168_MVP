# Proposed solution architecture and feature status

## What the problem asks us to build

For the latest implemented blocks, commands and evidence, see [29 September delivery](ARCHITECTURE_IMPLEMENTATION_20260929.md). Earlier milestone status below is historical where it differs from that report.


The brief describes two deployables that share one navigation design:

1. **Smartphone app:** live phone accelerometer, gyroscope, magnetometer and GNSS; vehicle-relative calibration; local filtering and fusion; uninterrupted position during GNSS loss; offline road constraints; smooth position display. The stated GNSS/INS position output target is 10 Hz.
2. **Edge software engine:** the same estimator interface accepting an external IMU, including a high-grade FOG stream around 200 Hz. The inertial state must propagate at the sensor cadence and provide correspondingly high-rate state output; GNSS corrections may arrive more slowly.

The brief also asks for preliminary model artifacts and a position plot on a subset of IO-VNBD for proposal screening, then additional data for later evaluation. The public phone logs inspected in this repository are nominally 10 Hz. They can support 10 Hz replay and a baseline, but cannot demonstrate 200 Hz behavior or characterize a 200 Hz FOG.

### Sensor variation across phones

Android's `SensorManager` is a standardized API, not a promise that every phone contains the same physical IMU. Android documents that manufacturers are not required to include particular sensor types. In the Android 16 compatibility definition, a 3-axis accelerometer and gyroscope are strongly recommended on handheld devices; if present, the spec defines minimum reporting capabilities. A 3-axis magnetometer is also strongly recommended rather than guaranteed. Gravity, linear-acceleration and rotation-vector readings may be software/composite sensors. Check availability at runtime; never build model logic around one assumed phone, vendor, or sensor list.

Use explicit capability profiles: (A) accelerometer + gyroscope, with magnetometer optional, for inertial propagation; (B) accelerometer without gyroscope, as a reduced, GNSS-dominant profile with weaker outage heading; (C) no accelerometer, as GNSS-only, clearly reporting inertial dead reckoning unavailable; and (D) external IMU, normalized by an adapter that declares units, axes, timestamp clock, calibration, and measured rate. Gravity/linear-acceleration/rotation-vector streams can help orientation, but they may already fuse the same accelerometer/gyro; do not treat them as statistically independent measurements in the estimator.

For reproducible model cohorts, archive device/build fingerprint and each sensor's type, name, vendor, version, resolution, range and advertised minimum delay where available. The logger now exports this inventory and event-timestamp-derived per-sensor rates in a `session_profile.json` sidecar inside the capture ZIP; actual delivery still needs a real-device check. Use event timestamps—not callback arrival time—for propagation, and compute delivered rate/jitter from those timestamps. For apps targeting Android 12+, common listener APIs are rate-limited to 200 Hz; higher rates require the high-sampling-rate permission and remain subject to device/platform constraints. Our phone prototype requests 100 Hz. The edge FOG path is a distinct external-sensor adapter; it cannot be validated by pretending a phone or the 10 Hz IO-VNBD log is a 200 Hz IMU.

For model portability, normalize units and axes, calibrate bias/mount per session, augment training with rotations, scale/bias/noise/rate jitter and vibration, and report leave-one-device/phone-model-out results. Use sensor metadata to audit cohorts and choose capability fallbacks; do not let an OEM-name lookup determine a confident navigation answer.

### What “smart automatic detection” means in this project

It is several automatic stages, not a hard-coded list of phone brands:

| Stage | Automatic behavior | Current status |
|---|---|---|
| Hardware discovery | Query the Android sensor service for available sensors; handle absent types; choose full, reduced or GNSS-only capability mode. | Implemented in the logger UI. |
| Device/session profiling | Record manufacturer/model/OS, sensor inventory and capabilities, successful listener registration, and observed event rate/intervals. | Implemented in the capture ZIP sidecar. |
| Sensor health | Detect timestamp gaps/jitter, unstable bias/noise, and calibration reliability; lower confidence rather than blindly trusting a weak stream. | Android records per-sensor cadence, jitter, gaps, sample age and sensor inventory; motion-quality heuristics feed the phone filter. Multi-device calibration remains pending. |
| Vehicle self-calibration | Use a quiet stationary interval to estimate gravity direction and gyro bias; during a reliable straight GNSS-aided drive infer phone-forward/yaw alignment. Recalibrate or flag low confidence if the mount changes. | Planned; needs real-phone captures to set and validate thresholds. |
| Motion/event understanding | Distinguish ordinary motion, stationary/engine vibration, turns, pothole-like shocks and possible slip; adapt filter noise and disable invalid constraints. | Planned; statistical detector first, learned event model after labeled diverse-device data. |
| Model portability | Normalize axes/units/rates; train and report holdout-phone results; use model uncertainty and a safe GNSS-dominant fallback on unfamiliar/weak devices. | Plan established; not trained or benchmarked yet. |

Stationary detection needs care in a running vehicle: engine vibration can make accelerometer variance large even when GNSS says the car is stopped. Combine GNSS speed, gyro, gravity consistency and robust vibration statistics; if the evidence conflicts, postpone bias calibration instead of learning the vibration as sensor bias. Yaw cannot be determined from gravity alone: use straight, reliable GNSS course when moving, with magnetometer only as an optional, disturbance-checked aid.

## Recommended estimator: learn corrections, keep navigation physical

Avoid a black-box network that predicts latitude and longitude directly. Use a multi-rate, uncertainty-aware estimator. Physics integrates motion sample by sample; small learned heads estimate quantities the phone cannot measure cleanly and report their confidence. The filter decides how much to trust them and each conventional sensor.

```text
Phone sensors ─┐
External IMU ───┴─> time/frame normalizer ─> mount alignment + sensor health
                                            │
                     event/quality scores <─┤
                                            ├─> local vehicle frame
                                            │     ├─ learned Δspeed + uncertainty
                                            │     ├─ gyro/accel bias and attitude
                                            │     └─ shock / stop / turn / slip flags
                                            v
                                strapdown / vehicle kinematics
                                + error-state Kalman correction
                              ┌─────────────┴──────────────┐
                         GNSS updates                 constraints
                    robust residual gate        ZUPT / adaptive NHC
                              └─────────────┬──────────────┘
                                            v
                            position + velocity + heading
                             uncertainty + operating mode
                                            │
                          optional confidence-gated map matcher
                                            │
                          app renderer / edge output interface
```

### Rate contract

| Path | Input and propagation | Position/state output | Meaning |
|---|---|---|---|
| Smartphone | Consume each delivered phone IMU sample; the logger currently requests 100 Hz and measures the actual accelerometer callback rate | 10 Hz target (one navigation output every 100 ms) | Do not claim a 100 Hz phone sensor callback, or an edge-grade IMU, from the request alone. |
| Edge with FOG | Consume and propagate at the actual external IMU cadence, nominally 200 Hz (one sample every 5 ms) | High-rate state output, targeting about 200 Hz as the brief requests | GNSS and learned corrections are asynchronous; they must not throttle inertial propagation. |
| IO-VNBD offline replay | Use measured timestamps from the provided phone/vehicle rows, nominally 10 Hz | Score at a declared 10 Hz resampling grid | This validates the lower-rate path only. Do not upsample 10 Hz observations and call it a 200 Hz test. |

The application output cadence and the IMU propagation cadence are separate clocks. Both must be measured from timestamps and reported. Missing samples, jitter, duplicate timestamps, and sensor rate limits are data-quality signals, not a reason to silently invent measurements.

For external-IMU evaluation, the [PPC 2024 dataset](https://github.com/taroz/PPC-Dataset) is a strong next addition: it provides a vehicle-mounted 100 Hz ADIS16505 MEMS IMU, 5 Hz multi-GNSS and reference trajectories. It can exercise the sensor-neutral adapter and higher-rate filter path, but it is not a 200 Hz FOG dataset. The official [KITTI raw data](https://www.cvlibs.net/datasets/kitti/raw_data.php) also publishes synchronized 10 Hz GPS/IMU streams; it is useful for a separate automotive reference check, not a phone-sensor transfer claim, and downloading requires site login. Neither dataset is currently included in project training. For the current milestone, all eligible synchronized IO-VNBD pairs were rebuilt first; its unsynchronized archive should remain unlabeled until an independent clock/trajectory alignment is established.

## Modules and how they address the brief

1. **Time and sensor-frame adapter.** Normalize elapsed-realtime timestamps, units, axis handedness, sample quality, and source identity. Android and external-IMU adapters map into the same typed event interface. Preserve raw input for replay and audit.
2. **Vehicle alignment and calibration.** Estimate vertical from gravity during stable intervals; infer forward/side axes from a safe straight-motion calibration using GNSS course and inertial response; refine yaw during reliable forward GNSS motion. Treat the magnetometer as a fallible aid because vehicle metal/electronics can disturb it. Detect phone movement after calibration and lower confidence/recalibrate rather than silently retaining a stale mount transform.
3. **Signal quality and motion events.** Start with robust statistics and explicit state logic for stationary, smooth travel, turn, high vibration, impact, and possible slip. A compact temporal classifier can later refine those labels. Raise process noise and suppress unsafe constraints on impact/slip; never integrate pothole vibration as a confident vehicle acceleration.
4. **Learned virtual speed change.** Replace the weak v0 direct-speed ridge with a causal, small temporal CNN/TCN that predicts forward Δspeed over a short interval plus an uncertainty (or distribution scale), optionally with motion class. Vehicle speed from audited, synchronized IO-VNBD pairs is a training label only. At run time this head sees phone/external IMU channels only. Seed speed from GNSS when available; integrate bounded Δspeed during outages; use model uncertainty to reduce its influence. Add zero-speed classification/updates for stops. Train by drive/session split, with device/mount/noise/bias/rotation augmentation and balanced speed/turn/stop coverage. Quantize and benchmark the same model used on device.
5. **Vehicle kinematics and error-state filter.** Propagate position, velocity and attitude at the input sample cadence, estimate IMU biases, and publish uncertainty. Use a 2D vehicle state with non-holonomic lateral/vertical velocity constraints only when the motion-quality score says the vehicle is rolling normally. Apply zero-velocity updates only when stationary detection is confident. These are conditional observations, not universal truths during impact, skid, loose surface or aggressive maneuver.
6. **GNSS integrity and fusion.** Gate GNSS by reported accuracy, fix status/age and the filter innovation. Adapt measurement covariance to observed quality; reject isolated outliers and soften multipath rather than snapping to them. Keep a clear GNSS-good / degraded / outage / reacquisition state machine with hysteresis. On reacquisition, feed GNSS residuals through the filter and smoothly converge; do not jump the display or retroactively hide the raw outage error.
7. **Road/map matcher.** Offline OSM roads provide candidate segments. A Hidden Markov/Viterbi matcher scores candidate distance, heading, route continuity and feasible vehicle motion. Keep several hypotheses near intersections; commit only above a confidence threshold. Use a road candidate as a soft filter constraint, not an unconditional snap. Log pre-map and post-map tracks separately so map correction cannot disguise dead-reckoning drift.
8. **Presentation and edge API.** Publish timestamp, geodetic/local position, speed, heading, covariance/uncertainty, sensor ages, GNSS state, motion flags, and whether map matching affected the fix. The app renders a smooth icon and exposes capture/quality/mode; the edge adapter publishes the same state contract at its higher cadence.

**Transition behavior now in the phone and Python reference/edge filters:** IMU propagation is called for every sample irrespective of GNSS mode. The phone rejects fixes older than five seconds, and both filters reject fixes above 35 m reported horizontal accuracy and gate remaining fixes against state innovation. A 1.5 s degraded-age threshold and 3 s dead-reckoning timeout avoid flicker around a nominal 1 Hz fix stream; accuracy uses a 20 m entry / 12 m exit hysteresis. After an outage, GNSS correction variance starts at 9x, then 4x, then nominal over three uninterrupted accepted fixes; an accuracy-gate or innovation rejection restarts that streak. This is a software freshness policy, not millisecond knowledge of when RF reception physically stopped. The phone emits on its 10 Hz state grid; external edge cadence is governed by sensor inputs and its configured output grid.

## What is implemented now

| Feature | Status |
|---|---|
| IO-VNBD archive downloaded and kept out of Git | Done |
| Paired phone/vehicle timestamp audit | Done across all 72 synchronized pairs; 64 pass 50 ms residual plus strict-monotonicity checks. Eight require clock review. This is a clock-consistency check, not proof every sensor is physically aligned. |
| Full synchronized training data | Done: 64 eligible drives and 726,618 phone samples; 40 train, 20 validation, 4 held-out test drives. The Vw validation drives are held out by drive within Driver E; this is not a cross-driver validation. |
| Timestamp normalization | Fixed in both paired-data builders. Parsed datetime differences are explicitly converted to seconds; all earlier model artifacts trained on compressed timelines are invalid. |
| Drive-held-out speed models | Corrected absolute-speed baseline: 5.94 m/s test MAE vs 8.87 m/s for a constant-train-median baseline, with test R² -0.78. Fresh speed-delta candidate test RMSE is 0.756 m/s vs 0.752 m/s zero-change; mixed SUMO candidate is 0.772 m/s. None qualifies for live fusion. |
| SUMO synthetic-IMU branch | `scripts/generate_synthetic_imu_from_sumo.py` accepts multiple FCD files and combines them into distinct train-only phone-like IMU drives. Real-only and mixed candidates are evaluated on identical real IO-VNBD validation/test drives, followed by identical outage replays. | Current augmentation degraded real held-out speed RMSE and average outage drift; synthetic data is not promoted. SUMO is a scenario harness, not a physical sensor benchmark. |
| Android sensor/GNSS logger | Source and debug APK build successfully. Session auto-starts after location permission handling, requests 100 Hz, displays per-sensor observed rate/sample count/interval range/jitter/age and hardware metadata, records monotonic sensor/GNSS rows and exports CSV/profile locally. |
| Shared navigation filter and learned model | Python 2D reference remains; a separate Kotlin phone filter is now wired to the live app. The learned speed model is not connected. |
| Android calibration and motion-quality block | Initial heuristic wired to live sensor/GNSS events and now feeds signal quality, stationary, and normal-driving gates into the phone filter. Needs mounted-drive validation. |
| Learned speed correction | Not used. Current speed-delta candidate is slightly worse than zero-change on test RMSE and worsens average integrated outage drift across all four test drives. The real-plus-SUMO candidate performs worse again. |
| Phone navigation output | Implemented as a 10 Hz target with continuous accelerometer-driven propagation and asynchronous GNSS position/velocity corrections. The 2026-09-28 vivo capture measured 1 Hz before the course-gating fix; source now uses retained fused heading, then lower-confidence phone azimuth before alignment. It auto-starts a local session after permission handling. A new on-device capture is still required to confirm cadence and behavior. |
| GNSS outage and recovery policy | Phone and Python reference/edge filters keep IMU propagation continuous, gate by fix age/accuracy/innovation, use hysteresis, and require three consistent accepted fixes with variance ramping on return. | Algorithmic policy implemented; physical loss-detection latency, phone behavior, and edge behavior remain unvalidated on recorded outages. |
| OSM map matching | Python and Android use confidence-gated beam HMMs with directed road-graph transitions. OSM XML-to-JSON preparation preserves road name/type, tunnel, bridge and layer tags. Android renders roads/tunnels/bridges and keeps raw and accepted matched tracks separate. | No regional road graph is bundled; import an OSM extract for the demo route. Android confidence and road continuity need route-level validation. Matching does not alter raw navigation output. |
| External-IMU edge input adapter | Implemented in `navcore/edge.py`: declares units, timebase/offset, SI biases, gravity handling and sensor-to-vehicle rotation; reports measured input/output cadence, interval jitter/gaps, and p95 processing latency. Sensor-specific hardware adapters are still needed. |
| Edge output target / 200 Hz FOG evaluation | Edge diagnostics and external CSV replay now measure actual input/output timestamps, gaps/jitter and p95 processing latency. No genuine 200 Hz FOG input or target-edge benchmark result is bundled. PPC's 100 Hz IMU validates adapter behavior only below target. |
| Preliminary outage benchmark | Four equal 60 s IO-VNBD test outages: raw IMU+NHC endpoint drift 43.6–80.0%; mean 66.6%. Current two-scenario SUMO outages: 59.5% and 100.3%, with synthetic 200 Hz cadence and 0.086–0.108 ms p95 desktop filter latency. None meets <10%; simulation is not hardware proof, and IO-VNBD reference is onboard vehicle GNSS/odometry rather than surveyed truth. |

The logger is a data-acquisition prototype. It is useful because it lets us verify sensor availability, timestamps, phone-specific sampling and mount behavior, and collect project-phone recordings. It should not be presented as the final expected solution.

## What is innovative and why it can score better

These are proposed differentiators, not already proven results or claims that no other team has ever used them:

- **Confidence-aware learned virtual odometer:** predict short-horizon speed change and uncertainty rather than asking a net to hallucinate an absolute position. A filter can fall back when the phone is shaken or the model is unsure.
- **Motion-conditioned constraints:** use stop updates, non-holonomic constraints, and shock suppression only when their assumptions hold; detect when a pothole or skid invalidates those assumptions.
- **Self-checking mount alignment:** calibrate from vehicle motion and gravity, watch for a shifted holder/phone, and surface confidence. This directly attacks phone placement variation that weakens an IMU-only model.
- **Integrity-aware GNSS return:** use innovation consistency, hysteresis and gradual correction for multipath and outage recovery. Show an uncertainty/mode indicator alongside the icon.
- **Map matching that preserves evidence:** maintain route hypotheses and confidence; report map-free DR separately from the road-constrained track.
- **One algorithm, two cadence-specific adapters:** the phone publishes at the required 10 Hz while a FOG edge pipeline propagates/publishes around 200 Hz. Models may update slower than propagation. Prove these with latency and drop/jitter measurements, not nominal configuration values.
- **Ablation-first evidence:** compare raw inertial, +virtual speed, +adaptive vehicle constraints, +robust GNSS, and +map hypotheses on exactly the same held-out outages. This tells evaluators what each innovation contributes.

## Benchmark plan

The brief says dead-reckoning drift should be below 10% of distance, with examples under 5 m over 50 m and under 100 m over 1 km during GNSS denial. It does not unambiguously define “drift” as endpoint, maximum error or RMSE. Report all three, along with horizontal distance, outage duration, along-track and cross-track errors, and the ratio used; request the official scoring definition from organizers.

For GNSS/INS, report smartphone state output at 10 Hz and edge state output/propagation near 200 Hz separately. Include p50/p95 processing latency, missed/deferred outputs, CPU/memory and power where measurable. Compare each version on identical drive-held-out outage windows. IO-VNBD vehicle GNSS/odometry is an available reference, not surveyed truth; label its limitations. Never score the map-matched track as raw DR.

### Next implementation order

1. Capture a safely mounted moving route and controlled GNSS outage on the connected physical phone; compare live state and event timestamps against GNSS and measure 10 Hz output cadence.
2. Replay the external adapter with PPC's real 100 Hz data, then a genuine 200 Hz/FOG stream; measure propagation/output cadence, jitter, gaps and p50/p95 latency. The adapter implementation alone is not a 200 Hz result.
3. Fix phone-to-vehicle frame calibration and GNSS update ordering; add filter replay tests against the live Kotlin equations before relying on the UI state.
4. Improve the learned speed candidate only after correcting targets/features and evaluating drive-held-out speed error plus integrated outage drift. Keep it disconnected unless both improve.
5. Validate the implemented GNSS freshness/accuracy/innovation gates and three-fix recovery ramp on held-out drives and multiple phones; tune thresholds from those results. Keep raw DR metrics separate from GNSS-aided and map-matched tracks.
6. Connect offline maps and confidence-gated route hypotheses; report pre-map and post-map metrics.
