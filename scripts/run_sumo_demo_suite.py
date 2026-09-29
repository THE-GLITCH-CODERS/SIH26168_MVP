"""One-command SUMO simulation plus controlled GNSS-outage evaluation.

The generated FCD/IMU/GNSS are synthetic. This suite checks the edge adapter,
GNSS masking and benchmark outputs; it does not validate real sensor cadence or
the SIH drift target.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/sumo_demo_suite")
    parser.add_argument("--scenarios", type=int, default=2)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--grid-number", type=int, default=3)
    parser.add_argument("--grid-length-m", type=float, default=200.0)
    parser.add_argument("--step-length-s", type=float, default=0.1)
    parser.add_argument("--outage-start-s", type=float, default=15.0)
    parser.add_argument("--outage-duration-s", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=200.0)
    parser.add_argument("--gnss-hz", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=26168)
    args = parser.parse_args()
    if args.scenarios < 1 or args.duration_s <= args.outage_start_s + args.outage_duration_s:
        raise SystemExit("Need >=1 scenario and simulation duration beyond the configured outage")

    sumo = shutil.which("sumo")
    netgenerate = shutil.which("netgenerate")
    if not sumo or not netgenerate:
        raise SystemExit("SUMO and netgenerate must be installed and available on PATH")
    output = args.output_dir.resolve()
    scenario_dir = output / "scenarios"
    outage_dir = output / "outage_replays"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    outage_dir.mkdir(parents=True, exist_ok=True)

    scenario_command = [
        sys.executable, str(ROOT / "scripts/run_sumo_training_scenarios.py"),
        "--output-dir", str(scenario_dir), "--scenarios", str(args.scenarios),
        "--duration-s", str(args.duration_s), "--grid-number", str(args.grid_number),
        "--grid-length-m", str(args.grid_length_m), "--step-length-s", str(args.step_length_s),
        "--base-seed", str(args.seed), "--sumo-bin", sumo, "--netgenerate-bin", netgenerate,
    ]
    subprocess.run(scenario_command, check=True)

    runs = []
    for index in range(1, args.scenarios + 1):
        fcd = scenario_dir / f"scenario_{index:02d}.fcd.xml"
        csv_path = outage_dir / f"scenario_{index:02d}_raw_track.csv"
        json_path = outage_dir / f"scenario_{index:02d}_outage.json"
        benchmark_command = [
            sys.executable, str(ROOT / "scripts/benchmark_sumo_gnss_outage.py"),
            "--fcd", str(fcd), "--outage-start-s", str(args.outage_start_s),
            "--outage-duration-s", str(args.outage_duration_s), "--output-csv", str(csv_path),
            "--output-json", str(json_path), "--imu-hz", str(args.imu_hz),
            "--gnss-hz", str(args.gnss_hz), "--seed", str(args.seed + index - 1),
        ]
        subprocess.run(benchmark_command, check=True)
        report = json.loads(json_path.read_text(encoding="utf-8"))
        runs.append({
            "scenario": index,
            "fcd": str(fcd),
            "raw_track_csv": str(csv_path),
            "outage_report_json": str(json_path),
            "drift_percent_of_distance": report.get("raw_outage_drift_percent_of_distance"),
            "edge_measured_input_hz": report.get("edge_measured_input_hz"),
            "edge_measured_output_hz": report.get("edge_measured_output_hz"),
            "processing_latency_p95_ms": report.get("edge_processing_latency_p95_ms"),
        })

    summary = {
        "suite": "SUMO simulated driving with explicit GNSS outages",
        "sumo_executable": sumo,
        "scenario_count": args.scenarios,
        "simulation_duration_s": args.duration_s,
        "outage_start_s": args.outage_start_s,
        "outage_duration_s": args.outage_duration_s,
        "synthetic_imu_target_hz": args.imu_hz,
        "synthetic_gnss_hz": args.gnss_hz,
        "runs": runs,
        "limitations": [
            "SUMO FCD is interpolated to the selected synthetic IMU cadence.",
            "Synthetic IMU/GNSS does not prove physical sensor performance or SIH drift compliance.",
            "The outage score is raw map-free filter output; map matching is assessed separately.",
        ],
    }
    summary_path = output / "sumo_demo_suite_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
