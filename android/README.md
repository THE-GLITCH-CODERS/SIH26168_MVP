# Android phone navigation prototype (0.3)

The app discovers the phone's available accelerometer, gyroscope, magnetometer, gravity, linear-acceleration, and rotation-vector sensors; records timestamped observations and GNSS fixes; exports a CSV/profile bundle; and runs a Kotlin phone navigation filter. During capture it estimates gyro bias after a stable stop, phone tilt from the rotation vector, a yaw offset from sufficiently accurate GNSS course while moving, and basic stationary/turn/impact/vibration events. Those signal-quality outputs scale inertial process noise and gate stationary/NHC updates. The filter publishes position, speed, heading, horizontal uncertainty, and GNSS-aided/dead-reckoning mode at a 10 Hz target. A sensor-neutral SI-unit input type remains the adapter boundary for future external IMUs.

The navigation panel can load a prepared `seamlessnav-offline-roads-v1` JSON through Android's document picker. The app indexes regional road segments off the UI thread and displays a separate confidence-gated road hypothesis beside the raw filter state. The Android matcher is a spatial candidate/emission-score implementation; graph-continuity HMM matching remains in Python/edge. It does not modify the navigation filter or present a rendered street map.

## Open and run

1. Open this `android/` folder as a project in Android Studio. API 37 needs Android Studio Panda 3 (2025.3.3 Patch 1) or newer; the installed build here is Android Studio 2026.1.4 (`AI-261.26222.65.2614.16379836`), so it is new enough.
2. Use JDK 17. The project uses Android Gradle Plugin 9.1.1 and Gradle 9.3.1. Let Android Studio sync and download Gradle/plugin dependencies.
3. Connect an Android phone with USB debugging enabled, accept the debugging prompt, and press Run. A real phone is needed for meaningful IMU/GNSS values.
4. Grant location permission. Mount the phone securely, press **Start capture**, drive only in a safe/legal setting, press **Stop capture**, then **Export capture bundle (.zip)**.

The log is stored privately in app files while recording. Export creates a user-selected ZIP containing the CSV and `session_profile.json`. Each CSV row includes a sensor name, elapsed-realtime nanoseconds, a derived UTC millisecond timestamp, three sensor values, and optional GNSS position/accuracy/speed/bearing. Empty columns mean that field does not apply to the row. The sidecar records phone/build identity, available sensor inventory and specifications, listener-registration results, units/axis frame, request cadence, and event-timestamp-derived per-sensor sample counts/rates/interval bounds.

## Current boundaries

- The capture request is 100 Hz; Android/device hardware may deliver another rate. The screen reports event-timestamp-derived rates per sensor. That is sensor input cadence, not the required 10 Hz navigation output rate or the 200 Hz FOG edge pipeline.
- Foreground-only prototype: it keeps the screen awake during capture and stops/closes the capture when the app leaves the foreground. Background/foreground-service behavior comes after this on-device smoke run.
- CSV rows are flushed periodically; write failures are shown in the app and in the session profile rather than silently ignored. Confirm that the exported row count and per-sensor rates look reasonable before using a capture for model work.
- The alignment and event logic is an initial heuristic. It is not yet a full phone-to-vehicle 3D transform and has not been validated across mounts, phone models, or real drive sessions. The UI reports phone tilt relative to gravity and an observed yaw offset; it does not claim vehicle-relative pitch/roll calibration.
- The timestamp-unit bug in earlier prepared data compressed phone timelines by 1,000×. Those earlier model artifacts/metrics are invalid. The corrected model results are recorded in `docs/SPEED_MODEL_V0.md`.
- The speed model is deliberately not connected to the navigation filter. It must improve held-out speed error and integrated outage position metrics before it can influence state estimates.
- The phone filter is a first live prototype. It has not yet been validated on a moving route or a real GNSS outage; the current 10 Hz state target is not an achieved under-10%-drift result.
- GNSS outage/reacquisition has a basic mode switch, but production-grade hysteresis, a rendered map/navigation icon, validated road matching, the external-IMU hardware adapter and measured 200 Hz edge performance remain open. The Android road matcher is currently a display-only hypothesis.
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

The current connected-phone build compiles, installs, and starts. Use the smoke
run to inspect sensor inventory/rates and check live navigation outdoors while
moving safely. Report the 10 Hz state cadence separately from the sensor input
rate. The live path still needs a GNSS-denied drive comparison before making
any SIH positional benchmark claim.
