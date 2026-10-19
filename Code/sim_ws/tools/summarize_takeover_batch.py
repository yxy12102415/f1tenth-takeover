#!/usr/bin/env python3
"""Aggregate immutable per-trial summaries selected by a run-id prefix."""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results_root", type=Path)
    parser.add_argument("run_prefix")
    args = parser.parse_args()
    trials = []
    for path in sorted(args.results_root.glob("*/*/summary.json")):
        payload = json.loads(path.read_text())
        if str(payload.get("run_id", "")).startswith(args.run_prefix):
            trials.append(payload)
    if not trials:
        raise SystemExit(f"No summaries match run prefix {args.run_prefix!r}")

    grouped = defaultdict(list)
    rows = []
    for trial in trials:
        grouped[trial["scenario"]].append(trial)
        rows.append({
            "scenario": trial["scenario"],
            "run_id": trial["run_id"],
            "seed": trial["seed"],
            "passed": trial["passed"],
            "collisions": trial["safety"]["collision_events"],
            "boundary_violations": trial["safety"]["boundary_violation_events"],
            "overtakes": trial["overtaking"]["completed"],
            "minimum_body_clearance_m": trial["safety"]["minimum_body_clearance_m"],
            "tracking_position_rmse_m": trial["tracking"]["position_rmse_m"],
            "planning_p95_ms": trial["runtime"]["planning_p95_ms"],
            "mpc_p95_ms": trial["runtime"]["mpc_p95_ms"],
        })

    aggregate = {}
    for scenario, values in sorted(grouped.items()):
        aggregate[scenario] = {
            "trials": len(values),
            "pass_rate": sum(bool(v["passed"]) for v in values) / len(values),
            "collision_trials": sum(v["safety"]["collision_events"] > 0 for v in values),
            "boundary_violation_trials": sum(
                v["safety"]["boundary_violation_events"] > 0 for v in values
            ),
            "overtake_success_rate": sum(
                v["overtaking"]["completed"] > 0 for v in values
            ) / len(values),
            "minimum_body_clearance_m": min((
                v["safety"]["minimum_body_clearance_m"] for v in values
                if v["safety"]["minimum_body_clearance_m"] is not None
            ), default=None),
            "maximum_planning_p95_ms": max(v["runtime"]["planning_p95_ms"] for v in values),
            "maximum_mpc_p95_ms": max(v["runtime"]["mpc_p95_ms"] for v in values),
        }
    output = args.results_root / f"batch_{args.run_prefix}.json"
    output.write_text(json.dumps({"run_prefix": args.run_prefix, "scenarios": aggregate}, indent=2) + "\n")
    csv_path = args.results_root / f"batch_{args.run_prefix}.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(aggregate, indent=2))
    print(f"Wrote {output} and {csv_path}")


if __name__ == "__main__":
    main()
