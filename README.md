# Seamless Vehicle Navigation (SIH 26168)

An edge-first prototype for vehicle dead reckoning and GNSS/INS fusion using smartphone or external IMU data. The first milestone is to make IO-VNBD data inspection and evaluation repeatable before training any model.

## Current project status

- Git repository initialized locally.
- The synchronized IO-VNBD archive is downloaded and extracted under `data/raw/iovnbd/` (ignored by Git).
- `scripts/inspect_iovnbd.py` reports phone-GNSS timing and produces a phone-fix diagnostic plot.
- `scripts/plot_iovnbd_pair.py` plots the paired vehicle-GNSS reference with phone-GNSS fixes and summarizes continuity.
- These first plots are dataset sanity checks, not dead-reckoning results or claims against the SIH drift benchmark.
- `scripts/audit_iovnbd_pairs.py` now scans all phone/vehicle pairs in the synchronized archive; only pairs with stable, strictly increasing clocks are used for labels.

## Dataset

Source: [IO-VNBD GitHub repository](https://github.com/onyekpeu/IO-VNBD), synchronized smartphone and vehicle logs.

The downloaded archive is the upstream `Synchronised V abd S datasets.zip` file (203,606,286 bytes). The upstream files are distributed through Git LFS. The archive has 442 entries, including 288 CSVs. Keep source data local; do not commit it or redistribute it. Check the upstream repository for current terms before sharing derived or raw data. The GitHub repository currently does not provide a clear dataset license file.

The paired phone log includes GPS, accelerometer, gravity, gyroscope, magnetometer, and orientation fields. The vehicle log includes GPS, vehicle velocity and heading, wheel speeds, yaw rate, and indicated longitudinal/lateral acceleration. The sample logs inspected here report a nominal 10 Hz phone cadence and 10 Hz vehicle cadence. Paired logs are useful for labels and comparisons, but do not treat vehicle measurements as phone-only model inputs.

## Setup

Use Python 3.10+ in a virtual environment, then install:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## Inspect a paired run

From the repository root:

```powershell
python scripts/inspect_iovnbd.py
```

The paired-run view uses S1 by default and writes `outputs/s-s1_paired_gnss_overview.png` plus a JSON summary:

```powershell
python scripts/plot_iovnbd_pair.py
```

Audit all synchronized phone/vehicle pairs before training:

```powershell
python scripts/audit_iovnbd_pairs.py
```

Check the phone/vehicle row-lag relationship separately before creating training labels:

```powershell
python scripts/analyze_pair_alignment.py
```

For S1, the phone wall-clock and vehicle time-of-day streams have a stable offset. The timestamp-pairing script uses that offset to create a phone-IMU/vehicle-speed label table under the ignored `data/processed/` directory:

```powershell
python scripts/build_supervised_pairs.py
```

To prepare the full eligible synchronized archive and train the first ridge baseline:

```powershell
python scripts/build_supervised_dataset.py
python scripts/train_speed_baseline.py
```

The preprocessing converts parsed datetimes using explicit elapsed seconds. Earlier model outputs used an incorrect nanosecond assumption and are invalid. Current full-archive preparation yields 64 eligible drives (40 train, 20 validation, 4 test), 726,618 paired phone samples; eight of 72 pairs fail the clock/monotonicity gate. The validation set holds out complete Vw drives from the same IO-VNBD driver as training, so it measures drive variation but not cross-driver generalization.

Run a synthetic GNSS-outage replay on held-out IO-VNBD S1. The script calibrates
phone sensor axes on the pre-outage segment, initializes at the outage boundary,
then passes phone IMU only to the navigation filters while scoring against the
vehicle GNSS reference:

```powershell
python scripts/replay_iovnbd_outage.py
```

Defaults: 30 s pre-outage calibration, outage from 60–120 s, and 10 Hz output.
It writes a trajectory CSV, plot, and JSON metrics under
`outputs/iovnbd_outage_v0/`. Vehicle data is used for pre-outage calibration and
initial state, and as an imperfect reference; it is not an input during the
outage. This is an initial physics baseline, not a learned-model or surveyed-
truth result. Use the `--outage-start-s`, `--outage-seconds`, and
`--calibration-seconds` arguments to inspect other windows.

The baseline currently does not beat a constant-speed predictor on the held-out test split. See [the recorded metrics and next model direction](docs/SPEED_MODEL_V0.md). It is a reference point, not a navigation-quality model.

## SUMO synthetic-training branch

The new architecture now has scripts to generate varied SUMO grid-traffic scenarios, convert FCD trajectories into synthetic phone-IMU features, and compare real-only against real-plus-synthetic speed models. SUMO itself is not installed in the current workspace, so this workflow has not yet produced synthetic data or model metrics. See [the step-by-step workflow and safeguards](docs/SUMO_SYNTHETIC_TRAINING.md).

The comparison keeps all synthetic samples in training and scores both candidates on the same real validation/test drives. Those speed metrics alone cannot promote a model; it must also improve held-out real GNSS-outage position error. The current IO-VNBD model features are nominally 10 Hz, so live 100 Hz phone input needs a matching anti-aliased model-input path before any on-device use. No model is currently loaded by Android.

## Offline road matching

The edge/offline core can prepare a regional OSM XML extract as a compact road graph and run a confidence-gated route matcher while retaining the unmodified raw track. The Android app can now load the prepared JSON and display a separate match hypothesis beside the raw navigation state. See [offline map preparation and matching](docs/OFFLINE_MAP_MATCHING.md). The repository contains no OSM extract, so the phone handoff still needs a real map and route validation.

Check the phone/vehicle row-lag relationship separately before creating training labels:

```powershell
python scripts/analyze_pair_alignment.py
```

It uses vehicle GNSS as an onboard reference stream. The phone stream is sampled with the IMU rows and appears noisier/delayed; timing must be validated before making training labels. This is a data-quality overview, not surveyed ground truth and not a dead-reckoning score. The phone-only diagnostic can be run separately:

```powershell
python scripts/inspect_iovnbd.py --phone-csv "data/raw/iovnbd/extracted/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/S-S1.csv"
```

## Intended architecture

1. **Portable navigation core:** timestamp normalization, sensor-frame/mount alignment, IMU quality and motion-event estimates, error-state inertial propagation, adaptive non-holonomic constraints, GNSS measurement gating, and uncertainty output.
2. **Learned virtual speed sensor:** a compact model estimates forward speed and confidence from phone IMU windows. Vehicle wheel speed/odometry can supervise training where synchronized labels exist; it must not leak into phone-only inference.
3. **GNSS transition logic:** quality-aware fusion lowers GNSS weight during multipath/outages and restores it gradually after recovery.
4. **Optional map matcher:** road constraints are applied only when map coverage and candidate confidence support them; map snapping must not hide or inflate the uncorrected dead-reckoning metric.
5. **Two deployment adapters:** Android app (Kotlin/Android sensor APIs) and an edge adapter for external IMU streams. The core model and navigation interfaces remain sensor-vendor independent.

The Python 2D fusion reference is in [`navcore/`](navcore/README.md). A Kotlin phone-side 10 Hz navigation state is now connected to live IMU/GNSS events. It reports local GNSS-anchored position, speed, heading, uncertainty, and GNSS-aided/dead-reckoning mode, with signal-quality-scaled propagation and conditional constraints. This is a first live prototype; phone route and outage behavior still need validation. The learned speed candidate remains disconnected.

## Android prototype

The mobile app at [`android/`](android/README.md) logs sensor data and now includes a first GNSS/INS navigation state at a 10 Hz target. It reports observed accelerometer rate and records monotonic timestamps with IMU and GNSS rows. The Kotlin path builds and is installed on the connected OnePlus; a moving-route test is still needed. Neither the under-10%-drift result nor the 200 Hz external-IMU path has been demonstrated.

See [the proposed solution architecture and feature-status matrix](docs/SOLUTION_ARCHITECTURE.md) for how the expected phone and edge outputs map to implementation, innovation ideas, and benchmarks.

The phone dataset is nominally 10 Hz, so it can support a 10 Hz phone demonstration. It cannot by itself validate a 200 Hz FOG edge pipeline. That requires a separate high-rate IMU replay/recording and timing benchmark; the edge estimator should propagate at sensor rate while slower learned corrections update asynchronously.

## Evaluation discipline

Report horizontal position error against an independent reference, distance traveled, outage duration, drift ratio (`horizontal endpoint/along-track error` and `trajectory RMSE`, stated separately), update rate, and inference latency. Include raw inertial, constrained inertial, and GNSS/INS results on identical held-out outage intervals. Split by drive/session, not randomly by row. Never score a trajectory after map snapping as if it were unconstrained dead reckoning.

SIH's stated threshold is under 10% positional drift during GNSS denial, with example cases of under 5 m over 50 m and under 100 m over 1 km. The exact evaluation definition (endpoint vs. maximum vs. RMSE) should be confirmed against the current official problem statement; show all three where possible.
#   S I H 2 6 1 6 8 _ M V P  
 