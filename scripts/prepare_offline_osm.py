"""Prepare an offline, car-road graph from a regional OSM XML extract."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ANDROID_MAP = ROOT / "android/app/src/main/assets/offline-roads-v1.json"
sys.path.insert(0, str(ROOT))
from navcore.map_matching import OfflineRoadNetwork


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--osm-xml", type=Path, required=True, help="local .osm/.xml extract; no network access is used")
    parser.add_argument(
        "--output-json", type=Path, default=DEFAULT_ANDROID_MAP,
        help="compact offline graph; defaults to Android's auto-loaded asset path",
    )
    parser.add_argument("--cell-size-m", type=float, default=60.0)
    args = parser.parse_args()
    network = OfflineRoadNetwork.from_osm_xml(args.osm_xml, cell_size_m=args.cell_size_m)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    network.write_json(args.output_json)
    print(f"Road graph origin: {network.origin_lat:.6f}, {network.origin_lon:.6f}")
    print(f"Directed road segments: {len(network.arcs):,}")
    print(f"Nodes: {len(network.nodes):,}")
    print(f"OSM attribution: {network.attribution}")
    print(f"Saved offline map: {args.output_json.resolve()}")


if __name__ == "__main__":
    main()
