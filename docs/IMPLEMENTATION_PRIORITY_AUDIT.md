# Implementation audit and priority order

Checked against the SIH expected capabilities and current source on 2026-09-29.

## Latest delivery: map and portable runtime

See [29 September implementation and evidence](ARCHITECTURE_IMPLEMENTATION_20260929.md) for current changes, commands and remaining limits.

- Interactive location-following OSM map, regional graph download/import, offline vector view, tunnel/bridge styling, separate raw/matched paths and absolute match gating are implemented and emulator-checked.
- Route selection now plans over directed connected OSM arcs in the imported regional graph and displays the route, next-turn distance, remaining distance and off-route state. An ongoing location foreground service keeps the Activity-owned session active while the user switches apps; persistence across Activity/process destruction is still not implemented.
- Mount rotation is estimated from gravity and signed straight acceleration, with a near-level-road assumption and an observability gate.
- A portable absolute-speed MLP with learned variance runs on Android and edge diagnostics. It remains excluded from live fusion. Python/Kotlin inference parity passes.
- Across the same four held-out outages, candidate mean drift is 61.60% versus raw IMU+NHC 66.58%; S3a regresses. None meet 10%. This does not establish validated AI fusion.
- The Android provider-outage emulator run exercised GNSS aided, degraded, DR and reacquisition while IMU remained active. This is not physical-phone validation.
- The edge JSONL package and synthetic 200 Hz replay work. Genuine 200 Hz FOG input and target-hardware performance remain unverified.
- Seven Kotlin module checks, APK assembly and lint pass; lint has non-blocking warnings. Navigation JSONL exports support cadence analysis.

## Earlier baseline audit (before this delivery)

| Expected capability | Current implementation | Gap / consequence |
|---|---|---|
| Phone IMU/GNSS collection | Android app enumerates sensors, requests 100 Hz, automatically starts a local foreground session after permission resolution, and records monotonic events/GNSS fixes with cadence diagnostics and CSV/profile ZIP export. A location foreground service keeps the session active while the Activity is paused and the user changes apps. | Latest vivo V2416/V2418 capture spans about 62.2 s: accel/gyro delivered about 52 Hz, but had a 2.26 s maximum callback gap; 27 GPS callbacks ended at 19.2 m accuracy with 8/59 satellites used/visible. GPS speed was zero, so this is not a moving-route validation. Foreground-service capture was smoke-checked on the emulator; OEM/device background rates and continuity across Activity/process destruction remain unverified. Data stays local. |
| Vehicle alignment and motion quality | Kotlin processor estimates stationary gyro bias, rotation-vector tilt, GNSS-course yaw offset, motion events, and a quality score. Those quality outputs now adjust filter noise and gate NHC/ZUPT; recent motion state carries into a GNSS outage until stationary evidence appears. | Heuristic prototype only. No validated full phone-to-vehicle 3D transform or multi-device calibration. |
| Phone speed / stop estimation | Phone filter uses GNSS velocity while available and physical IMU propagation during outages. A ridge speed-change candidate is trained offline from the complete drive-separated IO-VNBD dataset. | Fresh training covers 40 train, 20 validation, and 4 test drives. On test drives, candidate speed-change RMSE is 0.756 m/s versus 0.752 m/s for the zero-change baseline. In a common 60 s outage replay, mean endpoint drift is 66.6% for IMU+NHC and 84.5% with the real-only candidate. The candidate lowers drift on only 2/4 drives and remains disconnected. |
| SUMO synthetic training branch | SUMO traffic, multi-FCD synthetic phone-IMU generation, leakage-guarded real-only versus real-plus-SUMO training, held-out outage replay, and one-command controlled-outage suite are implemented. | Two FCD files generated 39 usable train-only synthetic drives (11,989 rows). Real-plus-SUMO test speed RMSE was 0.772 m/s versus 0.756 m/s for real-only and 0.752 m/s zero-change. On four identical held-out outages, mean candidate drift was 93.8% for real-plus-SUMO versus 66.6% raw IMU+NHC; it lowered drift on only 1/4 drives. Do not use this augmentation in the live model. |
| Dead reckoning and 10 Hz output | Kotlin phone filter consumes phone IMU and publishes a GNSS-anchored navigation state on a 100 ms schedule; IMU propagation is not gated on a fresh GNSS course. Capture CSV/profile include output cadence. | The newest available device capture predates the propagation fix and measured 1.00 Hz output while stationary. A new APK capture is still required to verify 10 Hz. The old capture also contained 2.26 s IMU callback gaps. |
| GNSS/INS fusion and transition | IMU propagation runs on accelerometer events in every mode; GNSS corrects the state asynchronously. Filters reject stale fixes, gate by reported accuracy and innovation, use 1.5 s degraded and 3 s outage thresholds, apply quality hysteresis, and ramp returning-fix variance 9x → 4x → 1x across three accepted fixes. | Implemented as an initial policy; mounted-route behavior and reacquisition smoothness still need device validation. It is not millisecond detection of physical RF loss. |
| Offline map matching and route guidance | Python and Android use confidence-gated, directed-graph continuity matching. Android auto-restores an imported graph or loads a bundled `offline-roads-v1.json` asset when present. Raw and matched tracks stay separate; OSM road type, name, tunnel, bridge and layer tags are preserved and styled. Route selection, graph routing and next-turn/remaining-distance guidance now use the loaded directed graph. | No national graph is bundled; road download/import is required for the selected area. Route planning is regional and does not yet include global route search, place lookup, voice prompts or route-level physical validation. Matching is display-only and never changes raw drift metrics. |
| External-IMU edge adapter | `navcore/edge.py` normalizes declared units, timestamps/offset, sensor-to-vehicle rotation, bias and gravity input, then forwards vehicle-forward acceleration and yaw rate to the reference filter. Input/output cadence, jitter/gaps, and p95 latency are surfaced. | `scripts/benchmark_external_imu_csv.py` can replay a genuine calibrated high-rate CSV. No genuine 200 Hz/FOG source or target-hardware result is bundled. |
| Controlled SUMO outage | `scripts/run_sumo_demo_suite.py` automates SUMO traffic generation and explicit GNSS-outage replay, reporting raw map-free results. | A fresh two-scenario 10 s replay measured 200 Hz synthetic input/output and p95 desktop processing of 0.086–0.108 ms. Raw relative drift was 59.5% and 100.3%. Interpolated synthetic IMU/GNSS do not establish physical sensor performance. |
| Under-10% outage drift | All four preassigned IO-VNBD test drives were replayed over the same 60 s interval after 30 s calibration. IMU+NHC endpoint drift was 80.0% (S1), 73.5% (S2), 43.6% (S3a), and 69.3% (S3c). | Fails the target on every drive. The candidate speed model reduced drift on two drives and worsened it on two; mean drift rose from 66.6% to 84.5%. The reference is onboard vehicle GNSS/odometry, not surveyed truth. |

## Priority implementation plan

### P0 — complete the phone smoke run

The latest capture verifies phone sensor logging and stationary GPS acquisition, and reveals the prior 1 Hz state-publication bug. Source now propagates IMU without a fresh course and auto-starts after permission resolution. Install the rebuilt app and repeat a securely mounted moving route, then a controlled GNSS outage only when safe. Verify measured output near 10 Hz, continuous DR propagation, smooth GNSS reacquisition, automatic start, and monotonic capture timestamps. Investigate the observed 2.26 s sensor callback gap before treating rate as stable.

### P1 — validate the live filter before tuning models

Install the rebuilt APK and capture a securely mounted moving route; verify the 10 Hz phone output, continuous DR, and smooth GNSS reacquisition. The available capture predates the propagation fix. Do not fuse learned speed: full-dataset held-out speed and integrated outage checks do not show a consistent improvement.

### P1 — complete the external-IMU target

The vendor-neutral normalization and cadence-monitoring interface is implemented in `navcore/edge.py`. Replay PPC's real 100 Hz source first, then obtain a genuine 200 Hz/FOG stream and measure cadence/latency. Do not claim 200 Hz from a configured output grid, phone input, or upsampled data.

### P2 — validate confidence-gated offline map matching

Prepare or import a regional OSM extract for the chosen demo route, verify bridge/tunnel tags, and inspect Android raw and matched tracks together. Map matching remains presentation-only; continue scoring the raw path independently.

## Device status

The earlier debug APK was installed on OnePlus CPH2723. The latest 2026-09-28 capture is from vivo V2416/V2418 and verifies outdoor/stationary GPS acquisition. Its measured navigation cadence was 1.00 Hz before the fresh-course propagation fix; the current source builds successfully, but no Android device is connected for a new live capture. Live moving-route quality, outage transition, OSM route coverage and actual 10 Hz cadence remain unverified.

## 2026-09-29 verification addendum

- Rebuilt the Android debug APK after porting directed-road beam continuity to the on-device matcher. `python -m compileall -q navcore scripts` and `git diff --check` also pass.
- Fixed IO-VNBD replay elapsed-time handling to use continuous date timestamps when the dataset's session timer resets (one reset in S2); each test-drive replay now completes. Per-drive reports are in `outputs/iovnbd_candidate_all_test/`.
- Retrained the offline candidate on the current 64-drive split: 40 train, 20 validation, 4 test. Candidate test speed-delta RMSE (0.756 m/s) is slightly worse than zero-change (0.752 m/s); integrated candidate drift is worse on average. It remains offline-only.
- Exercised SUMO augmentation using 39 train-only synthetic drives and the same real held-out validation/test splits. Real-plus-SUMO degrades test RMSE to 0.772 m/s; its outage candidate averages 93.8% drift and helps only 1 of 4 drives. Synthetic augmentation is not promoted.
- Re-ran two SUMO scenarios against the current filter. Results are synthetic harness measurements, not hardware results; both exceed the SIH 10% drift limit.

## 2026-09-29 Android route/background update

- Added graph-constrained destination selection and turn/distance guidance to the Android map. A Kotlin route-planner test covers connected streets, a turn, and along-route guidance.
- Added an ongoing Android location foreground service so a session can continue while the app is paused for another map app. On the emulator, Home left `NavigationSessionService` active with `isForeground=true`; relaunch returned to the same task. Activity/process-death recovery is not provided.
- `:app:testDebugUnitTest` and `:app:assembleDebug` pass. The physical phone's ADB connection dropped during APK installation, so this iteration is emulator-verified only.
- No local OSM road pack, connected Android device, or genuine external 200 Hz IMU/FOG capture is available in this workspace. Those results therefore cannot be manufactured by implementation alone.
