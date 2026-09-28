"""Plot the synchronized IO-VNBD vehicle GNSS reference with phone GNSS fixes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIR = ROOT / "data/raw/iovnbd/extracted/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1"


def load_gps(path: Path, time_column: str, lat_column: str, lon_column: str, time_scale: float) -> dict:
    frame = pd.read_csv(path, encoding="cp1252", low_memory=False)
    frame.columns = frame.columns.str.strip()
    required = [time_column, lat_column, lon_column]
    missing = [column for column in required if column not in frame.columns]
    if missing:
        raise SystemExit(f"Missing columns in {path.name}: {missing}")
    time = pd.to_numeric(frame[time_column], errors="coerce").to_numpy(dtype=float) / time_scale
    lat = pd.to_numeric(frame[lat_column], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(frame[lon_column], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(time) & np.isfinite(lat) & np.isfinite(lon)
    valid &= (lat >= -90) & (lat <= 90) & (lon >= -180) & (lon <= 180)
    time, lat, lon = time[valid], lat[valid], lon[valid]
    order = np.argsort(time, kind="stable")
    time, lat, lon = time[order], lat[order], lon[order]
    if len(time) < 2:
        raise SystemExit(f"Fewer than two valid GPS fixes in {path}")

    dt = np.diff(time)
    lat1, lat2 = np.radians(lat[:-1]), np.radians(lat[1:])
    dlat = lat2 - lat1
    dlon = np.radians(lon[1:] - lon[:-1])
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    step_m = 2 * 6_371_000.0 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return {
        "time": time,
        "lat": lat,
        "lon": lon,
        "rows": int(len(frame)),
        "valid_fixes": int(len(time)),
        "duration_seconds": float(time[-1] - time[0]),
        "median_sample_interval_seconds": float(np.median(dt[dt > 0])) if np.any(dt > 0) else None,
        "gps_path_length_m_unfiltered": float(np.sum(step_m)),
        "note": "Path length is a raw GNSS polyline sum; GNSS jitter can inflate it.",
        "largest_sample_gap_seconds": float(np.max(dt)) if len(dt) else None,
    }


def local_xy(track: dict, lat0: float, lon0: float) -> tuple[np.ndarray, np.ndarray]:
    east = 6_371_000.0 * np.radians(track["lon"] - lon0) * math.cos(lat0)
    north = 6_371_000.0 * np.radians(track["lat"] - math.degrees(lat0))
    return east, north


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-csv", type=Path, default=DEFAULT_DIR / "S-S1.csv")
    parser.add_argument("--vehicle-csv", type=Path, default=DEFAULT_DIR / "V-S1.csv")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs")
    args = parser.parse_args()
    phone_path, vehicle_path = args.phone_csv.resolve(), args.vehicle_csv.resolve()
    for path in (phone_path, vehicle_path):
        if not path.exists():
            raise SystemExit(f"File not found: {path}; extract IO-VNBD or pass the CSV paths.")

    phone = load_gps(phone_path, "TIME SINCE START (ms)", "GPS LATITUDE (degrees)", "GPS LONGITUDE (degrees)", 1000.0)
    vehicle = load_gps(vehicle_path, "Time Since Start of Day (seconds)", "Latitude (degrees)", "Longitude (degrees)", 1.0)
    lat0, lon0 = math.radians(float(vehicle["lat"][0])), float(vehicle["lon"][0])
    phone_e, phone_n = local_xy(phone, lat0, lon0)
    vehicle_e, vehicle_n = local_xy(vehicle, lat0, lon0)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = phone_path.stem.lower().replace(" ", "_")
    plot_path = args.output_dir / f"{stem}_paired_gnss_overview.png"
    summary_path = args.output_dir / f"{stem}_paired_gnss_summary.json"
    fig, ax = plt.subplots(figsize=(9, 7), constrained_layout=True)
    colored_reference = ax.scatter(
        vehicle_e,
        vehicle_n,
        c=vehicle["time"] - vehicle["time"][0],
        s=2,
        cmap="viridis",
        label="vehicle GNSS reference",
        rasterized=True,
    )
    ax.scatter(phone_e, phone_n, s=3, color="darkorange", alpha=0.25, label="phone GNSS fixes", rasterized=True)
    ax.scatter(vehicle_e[0], vehicle_n[0], marker="o", s=55, color="green", label="start", zorder=3)
    ax.scatter(vehicle_e[-1], vehicle_n[-1], marker="X", s=65, color="red", label="end", zorder=3)
    ax.set_title(f"IO-VNBD synchronized GNSS overview — {phone_path.stem}")
    ax.set_xlabel("East from first vehicle fix (m)")
    ax.set_ylabel("North from first vehicle fix (m)")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend(markerscale=3)
    fig.colorbar(colored_reference, ax=ax, label="Vehicle elapsed time (s)")
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    summary = {
        "phone_csv": str(phone_path),
        "vehicle_csv": str(vehicle_path),
        "phone_gnss": {key: value for key, value in phone.items() if key not in {"time", "lat", "lon"}},
        "vehicle_gnss_reference": {key: value for key, value in vehicle.items() if key not in {"time", "lat", "lon"}},
        "interpretation": (
            "Data-quality overview only. Vehicle GNSS is an onboard reference stream, not surveyed ground truth. "
            "The spatial overlay does not assert sample-level time alignment and is not a dead-reckoning result."
        ),
        "plot": str(plot_path),
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved plot: {plot_path}")


if __name__ == "__main__":
    main()
