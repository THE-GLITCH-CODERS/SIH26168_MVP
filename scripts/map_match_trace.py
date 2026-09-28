"""Offline confidence-gated map matching for a CSV navigation/GNSS track."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navcore.map_matching import ConfidenceGatedMapMatcher, MapObservation, OfflineRoadNetwork


def optional_float(row: dict[str, str], name: str) -> float | None:
    value = row.get(name, "").strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--osm-xml", type=Path, help="offline OSM .osm/.xml extract")
    source.add_argument("--offline-map-json", type=Path, help="prepared graph from prepare_offline_osm.py")
    parser.add_argument("--track-csv", type=Path, required=True, help="CSV containing timestamps and geographic positions")
    parser.add_argument("--output-jsonl", type=Path, required=True, help="separate matched track output; raw input is unchanged")
    parser.add_argument("--confidence-threshold", type=float, default=0.80)
    parser.add_argument("--timestamp-column", default="timestamp_elapsed_ns")
    parser.add_argument("--latitude-column", default="latitude_deg")
    parser.add_argument("--longitude-column", default="longitude_deg")
    parser.add_argument("--accuracy-column", default="horizontal_accuracy_m")
    parser.add_argument("--speed-column", default="speed_mps")
    parser.add_argument("--heading-column", default="bearing_deg")
    args = parser.parse_args()
    map_path = args.osm_xml or args.offline_map_json
    for path in (map_path, args.track_csv):
        if not path.exists():
            raise SystemExit(f"Input not found: {path}")

    roads = OfflineRoadNetwork.from_osm_xml(args.osm_xml) if args.osm_xml else OfflineRoadNetwork.from_json(args.offline_map_json)
    matcher = ConfidenceGatedMapMatcher(roads, confidence_threshold=args.confidence_threshold)
    seen = accepted = 0
    args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
    # Fresh file per run, with one JSON object per location observation.
    with args.track_csv.open("r", newline="", encoding="utf-8-sig") as source, args.output_jsonl.open("w", encoding="utf-8") as output:
        reader = csv.DictReader(source)
        required = (args.latitude_column, args.longitude_column)
        if not reader.fieldnames or any(name not in reader.fieldnames for name in required):
            raise SystemExit(f"Track CSV must contain columns: {', '.join(required)}")
        for row in reader:
            if row.get("sensor") and row["sensor"].strip().lower() not in {"gnss", "navigation", "nav_state"}:
                continue
            try:
                timestamp = int(float(row[args.timestamp_column]))
                # Android session exports may use elapsed nanoseconds or UTC ms.
                if args.timestamp_column.endswith("_utc_ms"):
                    timestamp *= 1_000_000
                latitude = float(row[args.latitude_column])
                longitude = float(row[args.longitude_column])
                accuracy = float(row.get(args.accuracy_column, "") or 15.0)
            except (KeyError, TypeError, ValueError):
                continue
            observation = MapObservation(
                timestamp_ns=timestamp,
                latitude_deg=latitude,
                longitude_deg=longitude,
                horizontal_sigma_m=max(1.0, accuracy),
                speed_mps=optional_float(row, args.speed_column),
                heading_deg_north_clockwise=optional_float(row, args.heading_column),
            )
            result = matcher.update(observation)
            output.write(json.dumps(asdict(result), separators=(",", ":")) + "\n")
            seen += 1
            accepted += int(result.accepted)
    print(f"Loaded {len(roads.arcs):,} directed road segments from {map_path}")
    print(f"Matched {accepted:,}/{seen:,} observations with confidence >= {args.confidence_threshold:.2f}")
    print(f"Saved separate matched track: {args.output_jsonl.resolve()}")


if __name__ == "__main__":
    main()
