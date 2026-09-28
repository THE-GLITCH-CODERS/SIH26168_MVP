"""Generate reproducible SUMO grid-traffic FCD scenarios for IMU augmentation.

Requires an installed Eclipse SUMO distribution, ``netgenerate``, and the
distribution's ``tools/randomTrips.py``. This script creates files only under
the selected output directory; it does not install SUMO or download road data.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "data/processed/sumo/scenarios"


def locate_binary(explicit: str | None, name: str, sumo_home: Path | None) -> str:
    if explicit:
        candidate = Path(explicit)
        if candidate.exists():
            return str(candidate)
        raise SystemExit(f"Configured {name} binary does not exist: {candidate}")
    suffixes = (".exe", "") if os.name == "nt" else ("",)
    if sumo_home:
        for suffix in suffixes:
            candidate = sumo_home / "bin" / f"{name}{suffix}"
            if candidate.exists():
                return str(candidate)
    found = shutil.which(name)
    if found:
        return found
    raise SystemExit(f"Could not find {name}; install Eclipse SUMO or pass --{name}-bin")


def run(command: list[str]) -> None:
    print("RUN:", subprocess.list2cmdline(command))
    subprocess.run(command, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scenarios", type=int, default=5)
    parser.add_argument("--duration-s", type=float, default=900.0)
    parser.add_argument("--grid-number", type=int, default=4)
    parser.add_argument("--grid-length-m", type=float, default=300.0)
    parser.add_argument("--step-length-s", type=float, default=0.1)
    parser.add_argument("--base-trip-period-s", type=float, default=2.0)
    parser.add_argument("--base-seed", type=int, default=26168)
    parser.add_argument("--sumo-bin")
    parser.add_argument("--netgenerate-bin")
    parser.add_argument("--random-trips", type=Path)
    args = parser.parse_args()
    if args.scenarios < 1 or args.duration_s <= 2 or args.grid_number < 2 or args.grid_length_m <= 0:
        raise SystemExit("Need >=1 scenario, duration >2s, grid number >=2 and positive grid length")
    if args.step_length_s <= 0 or args.base_trip_period_s <= 0:
        raise SystemExit("Step length and trip period must be positive")

    sumo_home_text = os.environ.get("SUMO_HOME")
    sumo_home = Path(sumo_home_text) if sumo_home_text else None
    sumo = locate_binary(args.sumo_bin, "sumo", sumo_home)
    netgenerate = locate_binary(args.netgenerate_bin, "netgenerate", sumo_home)
    random_trips = args.random_trips
    if random_trips is None and sumo_home:
        random_trips = sumo_home / "tools" / "randomTrips.py"
    if random_trips is None or not random_trips.exists():
        raise SystemExit("Could not find SUMO tools/randomTrips.py; pass --random-trips or set SUMO_HOME")

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    network = output / "grid.net.xml"
    run([
        netgenerate, "--grid", f"--grid.number={args.grid_number}",
        f"--grid.length={args.grid_length_m}", f"--output-file={network}",
    ])
    scenarios = []
    # Vary traffic demand and random seed to produce distinct stop/go profiles.
    demand_scales = (0.55, 0.8, 1.0, 1.35, 1.8)
    for index in range(args.scenarios):
        seed = args.base_seed + index
        scale = demand_scales[index % len(demand_scales)]
        period = args.base_trip_period_s / scale
        routes = output / f"scenario_{index + 1:02d}.rou.xml"
        fcd = output / f"scenario_{index + 1:02d}.fcd.xml"
        run([
            sys.executable, str(random_trips), "-n", str(network), "-r", str(routes),
            "--begin", "0", "--end", str(args.duration_s), "--period", f"{period:.6f}",
            "--seed", str(seed),
        ])
        run([
            sumo, "--net-file", str(network), "--route-files", str(routes),
            "--end", str(args.duration_s), "--step-length", str(args.step_length_s),
            "--fcd-output", str(fcd), "--seed", str(seed), "--no-step-log", "true",
        ])
        scenarios.append({
            "scenario_index": index,
            "seed": seed,
            "traffic_trip_period_s": period,
            "fcd_file": str(fcd),
            "intended_use": "synthetic training only",
        })

    manifest = {
        "generator": "Eclipse SUMO grid + randomTrips.py",
        "grid_number": args.grid_number,
        "grid_length_m": args.grid_length_m,
        "simulation_duration_s": args.duration_s,
        "simulation_step_s": args.step_length_s,
        "base_seed": args.base_seed,
        "scenarios": scenarios,
        "limitations": [
            "These artificial grid routes are training augmentation, not held-out evaluation.",
            "Use separate real drives for validation/test; never report simulation-only navigation accuracy as real-world performance.",
            "FCD trajectory cadence is not an IMU hardware timing benchmark.",
        ],
    }
    manifest_path = output / "scenario_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Saved SUMO scenario manifest: {manifest_path}")


if __name__ == "__main__":
    main()
