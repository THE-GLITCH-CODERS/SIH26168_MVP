"""Diagnose row-lag alignment in a synchronized IO-VNBD phone/vehicle pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIR = ROOT / "data/raw/iovnbd/extracted/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-csv", type=Path, default=DEFAULT_DIR / "S-S1.csv")
    parser.add_argument("--vehicle-csv", type=Path, default=DEFAULT_DIR / "V-S1.csv")
    parser.add_argument("--max-lag-seconds", type=float, default=10.0)
    parser.add_argument("--sample-period-seconds", type=float, default=0.1)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/S-S1_alignment.json")
    args = parser.parse_args()

    phone = pd.read_csv(args.phone_csv, encoding="cp1252", low_memory=False)
    vehicle = pd.read_csv(args.vehicle_csv, encoding="cp1252", low_memory=False)
    phone.columns, vehicle.columns = phone.columns.str.strip(), vehicle.columns.str.strip()
    phone_speed = pd.to_numeric(phone["GPS SPEED (Kmh)"], errors="coerce").to_numpy()
    vehicle_speed = pd.to_numeric(vehicle["Velocity (km/hr)"], errors="coerce").to_numpy()
    phone_lat = np.radians(pd.to_numeric(phone["GPS LATITUDE (degrees)"], errors="coerce").to_numpy())
    phone_lon = np.radians(pd.to_numeric(phone["GPS LONGITUDE (degrees)"], errors="coerce").to_numpy())
    vehicle_lat = np.radians(pd.to_numeric(vehicle["Latitude (degrees)"], errors="coerce").to_numpy())
    vehicle_lon = np.radians(pd.to_numeric(vehicle["Longitude (degrees)"], errors="coerce").to_numpy())
    count = min(map(len, (phone_speed, vehicle_speed, phone_lat, vehicle_lat)))
    phone_speed, vehicle_speed = phone_speed[:count], vehicle_speed[:count]
    phone_lat, phone_lon = phone_lat[:count], phone_lon[:count]
    vehicle_lat, vehicle_lon = vehicle_lat[:count], vehicle_lon[:count]

    max_rows = round(args.max_lag_seconds / args.sample_period_seconds)
    rows = []
    for lag in range(-max_rows, max_rows + 1):
        # Positive lag compares phone[i] with vehicle[i + lag].
        p0, v0 = max(0, -lag), max(0, lag)
        length = count - abs(lag)
        p_speed = phone_speed[p0 : p0 + length]
        v_speed = vehicle_speed[v0 : v0 + length]
        mask = np.isfinite(p_speed) & np.isfinite(v_speed)
        corr = float(np.corrcoef(p_speed[mask], v_speed[mask])[0, 1]) if mask.sum() > 2 else None

        plat, vlat = phone_lat[p0 : p0 + length], vehicle_lat[v0 : v0 + length]
        plon, vlon = phone_lon[p0 : p0 + length], vehicle_lon[v0 : v0 + length]
        geo_mask = np.isfinite(plat) & np.isfinite(vlat) & np.isfinite(plon) & np.isfinite(vlon)
        a = np.sin((plat[geo_mask] - vlat[geo_mask]) / 2) ** 2
        a += np.cos(plat[geo_mask]) * np.cos(vlat[geo_mask]) * np.sin((plon[geo_mask] - vlon[geo_mask]) / 2) ** 2
        distance = 2 * 6_371_000.0 * np.arcsin(np.sqrt(np.clip(a, 0, 1)))
        median_distance = float(np.median(distance)) if len(distance) else None
        rows.append({
            "lag_samples": lag,
            "lag_seconds": lag * args.sample_period_seconds,
            "phone_i_vs_vehicle_i_plus_lag_speed_correlation": corr,
            "median_phone_vehicle_gnss_separation_m": median_distance,
        })

    by_corr = [row for row in rows if row["phone_i_vs_vehicle_i_plus_lag_speed_correlation"] is not None]
    by_distance = [row for row in rows if row["median_phone_vehicle_gnss_separation_m"] is not None]
    result = {
        "phone_csv": str(args.phone_csv.resolve()),
        "vehicle_csv": str(args.vehicle_csv.resolve()),
        "row_count_phone_vehicle": [int(len(phone)), int(len(vehicle))],
        "nominal_sample_period_seconds": args.sample_period_seconds,
        "sign_convention": "Positive lag compares phone row i to vehicle row i+lag.",
        "best_speed_correlation": max(by_corr, key=lambda row: row["phone_i_vs_vehicle_i_plus_lag_speed_correlation"]),
        "smallest_median_gnss_separation": min(by_distance, key=lambda row: row["median_phone_vehicle_gnss_separation_m"]),
        "top_speed_correlation_candidates": sorted(
            by_corr,
            key=lambda row: row["phone_i_vs_vehicle_i_plus_lag_speed_correlation"],
            reverse=True,
        )[:10],
        "caution": "Diagnostic only. GNSS smoothing/latency and measurement noise can shift these optima; do not use the estimated row lag as IMU-label truth without validating source timestamps.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"Saved alignment diagnostic: {args.output.resolve()}")


if __name__ == "__main__":
    main()
