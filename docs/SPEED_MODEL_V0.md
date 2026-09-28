# Speed model status

The supervised speed pipeline uses phone IMU features and synchronized vehicle GNSS speed labels from IO-VNBD. Phone position/speed and vehicle sensor inputs are excluded from model features.

## Corrected data and baseline (2026-09-28)

The preprocessing previously divided `datetime64` integer values by 1e9 even when pandas had retained microsecond units. That compressed phone time by 1,000x and misaligned labels. Both `build_supervised_dataset.py` and `build_supervised_pairs.py` now subtract parsed timestamps and convert the resulting timedelta explicitly to seconds. Metrics and artifacts created before this correction are invalid and must not be cited.

The full synchronized archive contains 72 phone/vehicle pairs. Sixty-four pass a 50 ms clock-residual and strictly increasing timestamp gate; eight remain excluded for clock or monotonicity problems. The prepared set contains 726,618 phone samples across 64 complete drives: 40 train, 20 validation, and 4 held-out test. Validation holds out the Vw drive family from the same Driver E; this supports drive-level validation but does not establish cross-driver generalization. The test split is Driver A.

The past-only, two-second ridge speed regressor produced:

| Split | Drives | Ridge MAE | Train-median constant MAE | Ridge R² |
|---|---:|---:|---:|---:|
| Train | 40 | 5.85 m/s | — | 0.39 |
| Validation | 20 | 6.82 m/s | 8.40 m/s | 0.32 |
| Test | 4 | 5.94 m/s | 8.87 m/s | -0.78 |

The test MAE improves over this weak constant baseline, but negative R² and large speed-bin errors indicate substantial domain/speed-distribution mismatch. This is only speed regression, not a trajectory result. The model is not loaded by Android and is not fused into navigation.

## Navigation evidence and eligibility

A separate 59.9 s S1 synthetic outage traversed 348.2 m according to the onboard vehicle GNSS reference. The physics NHC filter ended 458.9 m from the reference (131.8% of distance); the constant-speed baseline ended 508.2 m away (146.0%). Both fail the stated <10% endpoint criterion. The reference is vehicle GNSS, not surveyed truth.

A speed correction can enter fusion only after it improves both drive-held-out validation/test and integrated outage position metrics on untouched windows. It must also report uncertainty and be gated off during shocks, turns with poor alignment, mount shifts, and unfamiliar device profiles. The old speed-delta v1 artifacts were trained on the time-compressed data and are invalid; do not use them.

## Next model work

1. Review the eight synchronization failures and validate alignment on a new mounted-phone capture.
2. Examine test errors by drive, speed bin, stop/turn/shock event and phone/device profile. Preserve complete drives when splitting.
3. Retrain a causal forward-speed-change or acceleration model and calibrate its uncertainty on validation drives.
4. Replay the same held-out GNSS outage intervals with and without the candidate. Keep it disabled unless integrated endpoint, max-error, and RMSE metrics improve without hiding GNSS reacquisition errors.
