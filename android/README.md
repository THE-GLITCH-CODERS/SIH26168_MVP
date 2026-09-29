# NAVIS Android phone navigation prototype (0.4)

The app discovers the phone's available accelerometer, gyroscope, magnetometer, gravity, linear-acceleration, and rotation-vector sensors; records timestamped observations and GNSS fixes; exports a CSV/profile bundle; and runs a Kotlin phone navigation filter. During capture it estimates gyro bias after a stable stop, phone tilt from the rotation vector, a yaw offset from sufficiently accurate GNSS course while moving, and basic stationary/turn/impact/vibration events. Those signal-quality outputs scale inertial process noise and gate stationary/NHC updates. The filter publishes position, speed, heading, horizontal uncertainty, and GNSS-aided/dead-reckoning mode at a 10 Hz target. It holds the GNSS anchor instead of integrating arbitrary phone motion until a course reference or calibrated vehicle heading is available. A sensor-neutral SI-unit input type remains the adapter boundary for future external IMUs.

The navigation panel displays a location-following OpenStreetMap raster basemap when the phone has a sufficiently accurate position and internet access. It requests only tiles in the visible area, caches them according to server cache headers, shows cached tiles when a refresh is unavailable, and displays OpenStreetMap attribution in the map. This is not a guaranteed full offline basemap: a new area needs internet tile access. Position can come from the navigation filter or a phone fix suitable for map centering; a coarse network estimate is never fused as GNSS and estimates worse than 250 m are not used to center the map. For offline road matching, the panel can also import a prepared `seamlessnav-offline-roads-v1` JSON through Android's document picker. After one import, it is copied to app-private storage and restored automatically on later launches; a bundled `offline-roads-v1.json` asset also loads automatically when present. To package an OSM XML area for zero-touch loading, run `python scripts/prepare_offline_osm.py --osm-xml <regional-extract.osm>`; the default output is the Android asset. The optional graph renders separately from the raw track; accepted map-matched hypotheses are shown separately, and OSM tunnel segments are dashed amber while bridges are teal.

**Latest delivery:** [Map, model, calibration, edge interface and evidence](../docs/ARCHITECTURE_IMPLEMENTATION_20260929.md).

The app opens to an interactive location-following OSM map. Download nearby
roads while online or import a road graph, then select **Offline road map**.
The app restores saved regional roads on launch. Use **Set destination** and tap
the map to plan over connected directed roads in that loaded region; the route,
next-turn distance, remaining distance, and off-route indication are drawn
separately from the raw and map-matched tracks. Online tiles and offline vector
roads are separate resources. The bundled speed MLP runs as a diagnostic and
cannot correct navigation until promoted. Exported ZIPs include
`*_navigation.jsonl` alongside the sensor CSV and profile.

Older milestone details below are historical where they differ from the linked report.

## Open and run

1. Open this `android/` folder as a project in Android Studio. API 37 needs Android Studio Panda 3 (2025.3.3 Patch 1) or newer; the installed build here is Android Studio 2026.1.4 (`AI-261.26222.65.2614.16379836`), so it is new enough.
2. Use JDK 17. The project uses Android Gradle Plugin 9.1.1 and Gradle 9.3.1. Let Android Studio sync and download Gradle/plugin dependencies.
3. Connect an Android phone with USB debugging enabled, accept the debugging prompt, and press Run. A real phone is needed for meaningful IMU/GNSS values.
4. On first launch, grant location permission. A local capture/navigation session starts automatically; GNSS permission denial still allows IMU-only capture. With location permission, an ongoing foreground-service notification keeps the session active while you switch to another map app. Tap the notification to return; use **Stop capture** in NAVIS to end the session. Mount the phone securely and drive only in a safe/legal setting. If Android destroys the app activity/process, the current activity-owned capture ends; background continuity across process death is not implemented. After ending a session, export the locally saved capture bundle if needed.

The log is stored privately in app files while recording; there is no automatic cloud upload. Export creates a user-selected ZIP containing the CSV and `session_profile.json`. The CSV includes timestamped sensor, GNSS, and target-10-Hz navigation rows; navigation rows store raw position, uncertainty, speed, and heading. The sidecar records phone/build identity, sensor inventory/specifications, listener-registration results, requested cadence, measured per-sensor rates, sample counts, mean/min/max event interval, interval jitter, and navigation-output sample count/rate/interval bounds. The live sensor panel shows those measured statistics, sample age, and the device-reported resolution/range/minimum delay. Raw filter state and map-matched display positions remain separate.

## Current boundaries

- The capture request is 100 Hz; Android/device hardware may deliver another rate. The screen reports event-timestamp-derived rates per sensor. That is sensor input cadence, not the required 10 Hz navigation output rate or the 200 Hz FOG edge pipeline.
- The foreground service keeps the live activity-owned capture running while the app is paused, and the screen-on flag is cleared so the display may sleep. If Android destroys the activity/process, capture stops; durable service-owned navigation across process death is not implemented. Background IMU rates, battery impact, and behavior across OEM power-management policies still need physical-phone measurement.
- CSV rows are flushed periodically; write failures are shown in the app and in the session profile rather than silently ignored. Confirm that the exported row count and per-sensor rates look reasonable before using a capture for model work.
- The alignment and event logic is an initial heuristic. It is not yet a full phone-to-vehicle 3D transform and has not been validated across mounts, phone models, or real drive sessions. The UI reports phone tilt relative to gravity and an observed yaw offset; it does not claim vehicle-relative pitch/roll calibration.
- The timestamp-unit bug in earlier prepared data compressed phone timelines by 1,000×. Those earlier model artifacts/metrics are invalid. The corrected model results are recorded in `docs/SPEED_MODEL_V0.md`.
- The speed model is deliberately not connected to the navigation filter. It must improve held-out speed error and integrated outage position metrics before it can influence state estimates.
- The phone filter uses fresh fix timestamps, accuracy and innovation gates, a 1.5 s degraded-age threshold, a 3 s dead-reckoning threshold, and a 9x → 4x → 1x GNSS measurement-variance ramp across three consistent reacquisition fixes. Once degraded, the GNSS status remains degraded until three consecutive accepted fixes at 12 m accuracy or better; poor, rejected, or borderline fixes restart that recovery streak. IMU propagation continues after a trustworthy heading exists; before that, the filter holds its anchor and labels heading as unaligned. The GNSS map badge follows filter acceptance rather than merely displaying the latest raw receiver callback. These are configured transitions, not evidence of millisecond physical RF-loss detection or a successful outage benchmark.
- The Android view shows a vehicle marker, raw DR track, confidence-gated route-continuity HMM track, GNSS/DR mode, confidence, and OSM bridge/tunnel styling. Destination routing and maneuver guidance run on the loaded directed regional graph; it is not yet full nationwide route search, place-name search, or turn-by-turn voice guidance. The OSM importer retains road name/type, tunnel, bridge, and layer tags. Map matching remains experimental and display-only; route validation and an under-10%-drift result remain pending.
- The Python edge diagnostics report measured input/output timestamp cadence, interval jitter/gaps, and p95 processing latency. `scripts/benchmark_external_imu_csv.py` measures a genuine external source when supplied a correctly calibrated, aligned CSV; no genuine 200 Hz FOG result is bundled. `scripts/benchmark_sumo_gnss_outage.py` runs a controlled synthetic outage replay, but SUMO-derived/interpolated signals cannot establish real hardware performance.
- One command runs SUMO traffic generation and controlled outage evaluation: `python scripts/run_sumo_demo_suite.py`. The suite uses SUMO from PATH and produces FCD scenarios, map-free outage CSVs, per-run reports and a summary under `outputs/sumo_demo_suite/`. For model augmentation only, use `scripts/run_sumo_training_scenarios.py`; keep simulated trajectories out of held-out real-drive evaluation.
- Android sensor APIs provide common type/unit/timestamp interfaces, not identical physical sensor quality. The app profiles each handset at runtime; advertised maximum rate is only a capability field and does not replace measuring delivered timestamps. It currently reports sensor vendor and advertised rate on screen and saves the broader inventory in the profile sidecar.

See the repository root README and `docs/SPEED_MODEL_V0.md` for the data/model groundwork and known limitations.

## Phone smoke test

Build the debug APK from Android Studio with **Run**, or from PowerShell with
Android Studio's bundled JDK and SDK configured. The APK is written to
`app/build/outputs/apk/debug/app-debug.apk`.

For ADB installation, enable Developer options and USB debugging on the phone,
connect it, approve the computer prompt on the phone, then verify it appears in
`adb devices -l` before installing. The app requests location access on first
launch. Record a short stationary capture first, then a securely mounted
in-vehicle capture with straight motion and turns when safe. Keep the app in the
foreground; stop and export the ZIP. Check sensor availability, observed sample
rates, timestamps, row count, and GNSS fix rate.

This iteration's debug APK built and the unit tests passed. It was installed
and launched on the Android emulator; the log showed the navigation service
foregrounded after the app was sent to Home. The connected physical phone
disconnected during ADB installation, so physical-device UI, location
permissions, tile loading, background IMU cadence, and real-drive behavior are
not yet verified. Report measured 10 Hz state cadence separately from sensor
input rate. The live path still needs a GNSS-denied drive comparison before
making any SIH positional benchmark claim.
