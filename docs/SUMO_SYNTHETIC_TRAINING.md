# SUMO synthetic IMU training branch

This branch augments the audited IO-VNBD supervised windows. It does not replace the real-drive split or certify navigation performance.

## Flow

1. With SUMO and `netgenerate` on PATH (or `SUMO_HOME` set), generate varied grid/traffic scenarios with:

   ```powershell
   python scripts/run_sumo_training_scenarios.py --scenarios 5 --duration-s 900
   ```

   It writes network, route, FCD and provenance-manifest files under `data/processed/sumo/scenarios/`. FCD must include timestep, vehicle id, x/y coordinates, navigation angle and speed. SUMO documents angle as navigation degrees (0° north, clockwise) and speed in m/s: [FCD output documentation](https://sumo.dlr.de/docs/Simulation/Output/FCDOutput.html).
2. Convert FCD trajectories to phone-like sensor samples:

   ```powershell
   python scripts/generate_synthetic_imu_from_sumo.py `
     --fcd data/processed/sumo/scenario_01.fcd.xml `
     --fcd data/processed/sumo/scenario_02.fcd.xml `
     --output data/processed/sumo/scenario_01_imu.npz `
     --seed 101 --target-hz 10
   ```

   Repeat `--fcd` for independent traffic scenarios. It simulates fixed phone mount rotations, gravity, accelerometer/gyro biases, noise, low-rate vibration and sparse shocks. Episodes are split at large timestamp gaps; generated trajectory IDs are unique across FCD files and marked train-only in provenance.

3. Compare the real-only and real-plus-SUMO models using identical real IO-VNBD validation and test drives:

   ```powershell
   python scripts/compare_real_vs_sumo_training.py `
     --real-dataset data/processed/iovnbd/supervised_drives_v0.npz `
     --synthetic-dataset data/processed/sumo/scenario_01_imu.npz
   ```

   The comparison refuses synthetic validation/test rows and refuses a synthetic feature cadence more than 5% away from the real training cadence. It exports separate model candidates and per-split speed metrics under `outputs/speed_delta_real_sumo_compare/`.

4. Run each candidate through the same held-out real GNSS outage, using the same outage interval and pre-outage calibration:

   ```powershell
   python scripts/replay_iovnbd_outage.py `
     --evaluate-speed-model outputs/speed_delta_real_sumo_compare/real_plus_sumo/speed_delta_v1.json `
     --output-dir outputs/iovnbd_outage_real_plus_sumo
   ```

   Repeat for the real-only candidate, multiple untouched test drives and outage windows. The replay injects predicted speed only into that offline what-if filter, schedules each delta at its one-second target horizon, and reports the map-free trajectory metrics. This is evaluation—not enabling the candidate in Android or production edge fusion.

5. Run a full SUMO simulation plus controlled GNSS-blackout smoke suite in one command:

   ```powershell
   python scripts/run_sumo_demo_suite.py --scenarios 2 --duration-s 120
   ```

   The suite generates traffic/FCD scenarios, applies explicit 10 s GNSS outages, and writes each raw map-free replay plus a summary JSON under `outputs/sumo_demo_suite/`. This is a workflow check; it is not an accuracy claim.

   To replay one existing FCD trajectory with a custom outage interval:

   ```powershell
   python scripts/benchmark_sumo_gnss_outage.py `
     --fcd data/processed/sumo/scenarios/scenario_01.fcd.xml `
     --outage-start-s 120 --outage-duration-s 60 `
     --output-csv outputs/sumo_outage/raw_track.csv
   ```

   The runner derives idealized forward acceleration/yaw rate from SUMO kinematics, adds configurable synthetic sensor noise, samples GNSS at 1 Hz, and suppresses those updates over the requested time interval. Its JSON reports measured replay input/output cadence, p95 processing latency, outage RMSE/max error, endpoint-relative drift, and drift as a percentage of total reference distance traveled during the outage. It does not run map matching, so map snapping cannot improve the raw score.

6. Measure a **genuine external IMU CSV** with `scripts/benchmark_external_imu_csv.py`. The default columns are `timestamp_s`, `accel_forward_mps2`, and `yaw_rate_radps`; optional `gnss_east_m`, `gnss_north_m`, `gnss_sigma_m`, `truth_east_m`, and `truth_north_m` columns support fusion/outage scoring. Declare time and sensor units explicitly when they differ from defaults. The input must already be gravity compensated, calibrated, time aligned, and expressed in the vehicle-forward/yaw axes. Example:

   ```powershell
   python scripts/benchmark_external_imu_csv.py `
     --input-csv data/external/imu_200hz.csv `
     --output-csv outputs/external_imu/edge_track.csv `
     --timestamp timestamp_ns --timestamp-unit ns `
     --accel-x forward_accel --gyro-z yaw_rate `
     --outage-start-s 30 --outage-duration-s 20
   ```

   The JSON measures source-timestamp input Hz, emitted navigation Hz, interval jitter/gaps, and replay processing latency. Drift metrics require independent truth columns and are computed from the raw filter only. Wall-clock replay latency on a development computer is not target-hardware throughput.

## Important limits

- Current IO-VNBD phone samples are nominally 10 Hz. Synthetic examples default to the same cadence and keep injected vibration below its 5 Hz Nyquist limit. A live phone stream must be anti-aliased/decimated to the trained feature cadence before model inference; the current Android app does not yet load this model.
- For high-frequency vibration learning, collect real 100 Hz phone data with vehicle speed/reference labels. Resampling a 10 Hz dataset cannot recreate those measurements.
- Calibrate synthetic noise and bias ranges from training drives only. Do not tune them using held-out test drives.
- Synthetic speed labels are generated from SUMO and are useful for model augmentation only. They are not real vehicle sensor reference or surveyed truth.
- The generated model is always `eligible_for_fusion: false`. It must also improve integrated position error on identical, held-out real GNSS-outage windows before it can be considered for navigation.
- SUMO 1.27.1 is on PATH. A fresh two-scenario 60 s run with controlled 10 s outages reported raw relative drift of 100.3% and 59.5%; these synthetic/interpolated results fail the SIH <10% target. The two FCD files generated 39 usable synthetic training drives (11,989 rows). Compared on real-only held-out IO-VNBD drives, real-plus-SUMO test speed RMSE was 0.772 m/s versus 0.756 m/s for real-only and 0.752 m/s for the zero-change baseline. Across four identical 60 s real outages, the real-plus-SUMO speed candidate averaged 93.8% endpoint drift and reduced drift on only 1/4 drives. Do not promote synthetic augmentation from this experiment.
- Synthetic 10 Hz trajectories cannot demonstrate the external 200 Hz edge/FOG benchmark.
- SUMO outage replay uses kinematics-derived synthetic IMU and GNSS. It is a reproducible controlled filter exercise, not physical sensor validation or evidence that the SIH under-10% drift target is met.
