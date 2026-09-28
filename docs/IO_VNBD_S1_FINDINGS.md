# IO-VNBD S1 data check

Source archive: [IO-VNBD GitHub repository](https://github.com/onyekpeu/IO-VNBD), synchronized subset, downloaded from the upstream Git LFS media endpoint. The raw archive and extracted CSVs live under `data/raw/iovnbd/` and are excluded from Git.

## Files examined

Run S1 contains `S-S1.csv` (smartphone) and `V-S1.csv` (vehicle). Each has 51,746 rows and covers about 5,174.5 seconds. Their recorded sample intervals are approximately 0.1 seconds. The phone CSV has GPS, acceleration, gravity, gyro, magnetic field, and orientation fields. The vehicle CSV has GNSS, velocity, heading, wheel speeds, yaw rate, and longitudinal/lateral acceleration among other ECU fields.

The phone samples include lat/lon, GPS speed, GPS accuracy, orientation, and the IMU fields in the same table. The vehicle stream provides independent onboard speed/position signals that can supervise training, but those vehicle-only fields must not enter the phone inference feature set.

## Alignment caution

Do not join phone and vehicle rows blindly. Both S1 files have the same row count and nominal cadence, but use different clocks. The phone's first displayed wall-clock time is `2019-09-08 10:07:49:546`, while vehicle time-of-day is 32,869.0 s (09:07:49.0). Across all 51,746 rows, phone time-of-day minus vehicle time-of-day is 3,600.545 s at the median; 95% of residuals from that offset are within 1 ms. This supports aligning the IMU rows to vehicle labels by their recorded timestamps for S1 rather than assuming the raw time origins match.

An exploratory GNSS-only row-lag sweep found strongest phone-GPS-speed/vehicle-velocity correlation around a 4.2 s GNSS channel lag (correlation about 0.929); shifting GNSS positions similarly reduced their median separation from about 24 m to 13 m. This does not override the row wall-clock alignment: phone GNSS values can be stale/delayed within the 10 Hz IMU table. Keep the phone `DATE` timestamps for the IMU rows, and treat GNSS channel latency separately. These diagnostics are not surveyed truth, so check them across other paired drives before generalizing.

## S1 plot and measured values

Run `python scripts/plot_iovnbd_pair.py` to regenerate `outputs/s-s1_paired_gnss_overview.png` and its JSON summary. The plot overlays phone GNSS fixes with the vehicle GNSS stream. Raw polyline sums are about 37.2 km (phone) and 38.0 km (vehicle); these sums include GNSS jitter and are not benchmark distance truth. The vehicle GNSS trace is an onboard reference, not surveyed ground truth.

## Reproducible scripts

- `python scripts/plot_iovnbd_pair.py` creates the paired spatial overview.
- `python scripts/analyze_pair_alignment.py` measures GNSS lag diagnostics; do not use this GNSS lag as the IMU timestamp offset.
- `python scripts/build_supervised_pairs.py` maps S1 phone timestamps to vehicle speed labels using the stable wall-clock offset and excludes phone GNSS/speed from model inputs.

## Next data gate

Before training:

1. Read the full upstream dataset description and identify how synchronized runs and outages were curated.
2. Analyze several paired runs, not only S1; estimate and document per-stream offsets and data-quality flags.
3. Validate the timestamp-offset method across the selected drives; resample only after synchronization, keeping phone IMU features separate from vehicle labels.
4. Split train/validation/test by drive or route, never by random rows from the same drive.
5. Report the raw/unconstrained DR score separately from GNSS/INS and map-matched outputs.
