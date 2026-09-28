"""Inspect an IO-VNBD phone log and save a phone-GNSS diagnostic plot and summary."""

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
DEFAULT_PHONE = (
    ROOT
    / "data/raw/iovnbd/extracted/Synchronised V abd S datasets/"
    "Categorised IOVNB Dataset/S (Driver A)/S1/S-S1.csv"
)


def haversine_m(lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
    """Return point-to-point great-circle distance in metres."""
    radius = 6_371_000.0
    lat1 = np.radians(lat[:-1])
    lat2 = np.radians(lat[1:])
    dlat = lat2 - lat1
    dlon = np.radians(lon[1:] - lon[:-1])
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    return 2 * radius * np.arctan2(np.sqrt(a), np.sqrt(1 - a))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phone-csv", type=Path, default=DEFAULT_PHONE)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs")
    parser.add_argument("--gps-gap-seconds", type=float, default=3.0)
    args = parser.parse_args()

    phone_path = args.phone_csv.resolve()
    if not phone_path.exists():
        raise SystemExit(
            f"Phone CSV not found: {phone_path}\n"
            "Download and extract the synchronized IO-VNBD archive first, or pass --phone-csv."
        )

    # Upstream CSVs contain legacy Windows-1252 characters (for example m/s²).
    frame = pd.read_csv(phone_path, encoding="cp1252", low_memory=False)
    frame.columns = frame.columns.str.strip()
    required = [
        "TIME SINCE START (ms)",
        "GPS LATITUDE (degrees)",
        "GPS LONGITUDE (degrees)",
    ]
    missing = [name for name in required if name not in frame.columns]
    if missing:
        raise SystemExit(f"Missing required columns in {phone_path.name}: {missing}")

    t = pd.to_numeric(frame[required[0]], errors="coerce").to_numpy(dtype=float) / 1000.0
    lat = pd.to_numeric(frame[required[1]], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(frame[required[2]], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    valid &= (lat >= -90) & (lat <= 90) & (lon >= -180) & (lon <= 180)
    t, lat, lon = t[valid], lat[valid], lon[valid]
    if len(t) < 2:
        raise SystemExit("Need at least two valid timestamped GPS fixes to plot a trajectory.")

    order = np.argsort(t, kind="stable")
    t, lat, lon = t[order], lat[order], lon[order]
    dt = np.diff(t)
    positive_dt = dt[dt > 0]
    steps = haversine_m(lat, lon)
    # GPS fixes are repeated across the 10 Hz IMU stream, so per-row implied
    # speeds are not a reliable outlier test. Keep the raw polyline sum explicit.
    path_length_m = float(np.sum(steps))
    gaps = np.flatnonzero(dt > args.gps_gap_seconds)

    # Local tangent-plane approximation, suitable for visualizing a single drive.
    lat0, lon0 = math.radians(float(lat[0])), float(lon[0])
    east = 6_371_000.0 * np.radians(lon - lon[0]) * math.cos(lat0)
    north = 6_371_000.0 * np.radians(lat - lat[0])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = phone_path.stem.replace(" ", "_")
    plot_path = args.output_dir / f"{stem.lower()}_gps_reference.png"
    summary_path = args.output_dir / f"{stem.lower()}_summary.json"

    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    line = ax.scatter(east, north, c=t - t[0], s=3, cmap="viridis", rasterized=True)
    ax.scatter(east[0], north[0], marker="o", s=55, color="green", label="start", zorder=3)
    ax.scatter(east[-1], north[-1], marker="X", s=65, color="red", label="end", zorder=3)
    ax.set_title(f"IO-VNBD phone GNSS diagnostic — {phone_path.stem}")
    ax.set_xlabel("East from first fix (m)")
    ax.set_ylabel("North from first fix (m)")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.colorbar(line, ax=ax, label="Elapsed time (s)")
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    summary = {
        "source_csv": str(phone_path),
        "rows_total": int(len(frame)),
        "valid_gps_fixes": int(len(t)),
        "duration_seconds": float(t[-1] - t[0]),
        "median_phone_gps_interval_seconds": float(np.median(positive_dt)) if len(positive_dt) else None,
        "gps_gap_threshold_seconds": args.gps_gap_seconds,
        "gps_gaps_over_threshold": int(len(gaps)),
        "longest_gps_gap_seconds": float(np.max(dt)) if len(dt) else None,
        "gps_path_length_approx_m": path_length_m,
        "path_length_note": "Raw GNSS polyline sum; receiver jitter can inflate distance.",
        "first_fix_lat_lon": [float(lat[0]), float(lon[0])],
        "last_fix_lat_lon": [float(lat[-1]), float(lon[-1])],
        "interpretation": "Phone GNSS diagnostic only; not surveyed truth or a dead-reckoning estimate.",
        "plot": str(plot_path),
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved plot: {plot_path}")


if __name__ == "__main__":
    main()
