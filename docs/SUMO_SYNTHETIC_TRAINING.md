# SUMO synthetic IMU training branch

This branch augments the audited IO-VNBD supervised windows. It does not replace the real-drive split or certify navigation performance.

## Flow

1. Install SUMO and set `SUMO_HOME` (or pass binary/tool paths), then generate varied grid/traffic scenarios with:

   ```powershell
   python scripts/run_sumo_training_scenarios.py --scenarios 5 --duration-s 900
   ```

   It writes network, route, FCD and provenance-manifest files under `data/processed/sumo/scenarios/`. FCD must include timestep, vehicle id, x/y coordinates, navigation angle and speed. SUMO documents angle as navigation degrees (0° north, clockwise) and speed in m/s: [FCD output documentation](https://sumo.dlr.de/docs/Simulation/Output/FCDOutput.html).
2. Convert FCD trajectories to phone-like sensor samples:

   ```powershell
   python scripts/generate_synthetic_imu_from_sumo.py `
     --fcd data/processed/sumo/scenario_01.fcd.xml `
     --output data/processed/sumo/scenario_01_imu.npz `
     --seed 101 --target-hz 10
   ```

   It simulates fixed phone mount rotations, gravity, accelerometer/gyro biases, noise, low-rate vibration and sparse shocks. Episodes are split at large timestamp gaps. Each generated trajectory is marked train-only in its provenance JSON.

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

## Important limits

- Current IO-VNBD phone samples are nominally 10 Hz. Synthetic examples default to the same cadence and keep injected vibration below its 5 Hz Nyquist limit. A live phone stream must be anti-aliased/decimated to the trained feature cadence before model inference; the current Android app does not yet load this model.
- For high-frequency vibration learning, collect real 100 Hz phone data with vehicle speed/reference labels. Resampling a 10 Hz dataset cannot recreate those measurements.
- Calibrate synthetic noise and bias ranges from training drives only. Do not tune them using held-out test drives.
- Synthetic speed labels are generated from SUMO and are useful for model augmentation only. They are not real vehicle sensor reference or surveyed truth.
- The generated model is always `eligible_for_fusion: false`. It must also improve integrated position error on identical, held-out real GNSS-outage windows before it can be considered for navigation.
- SUMO is not installed in the current workspace. The converter is ready for SUMO FCD exports; actual scenario generation and real-plus-synthetic metrics remain pending until a SUMO run is available.
- Synthetic 10 Hz trajectories cannot demonstrate the external 200 Hz edge/FOG benchmark.
