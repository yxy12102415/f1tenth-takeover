#!/usr/bin/env python3
"""Summarize run-level corridor-versus-visibility comparison trials."""

import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path


def mean(values):
    finite = [value for value in values if value is not None and math.isfinite(value)]
    return sum(finite) / len(finite) if finite else None


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: summarize_visibility_comparison.py DESIGN.csv SUMMARY.json"
        )
    design_path = Path(sys.argv[1]).resolve()
    output_path = Path(sys.argv[2]).resolve()
    rows = []
    with design_path.open(newline="", encoding="utf-8") as handle:
        design = list(csv.DictReader(handle))
    if not design:
        raise SystemExit("comparison design contains no trials")

    grouped = defaultdict(list)
    missing = []
    for item in design:
        summary_path = Path(item["summary_path"])
        if not summary_path.is_file():
            missing.append(str(summary_path))
            continue
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        risk = summary["risk_aware_planning"]
        expected_side = item["expected_side"]
        selection_counts = risk["candidate_selection_counts"]
        row = {
            "block": item["block"],
            "expected_side": expected_side,
            "map_name": item["map_name"],
            "method": item["method"],
            "run_id": item["run_id"],
            "selected_expected_side": selection_counts.get(expected_side, 0) > 0,
            "target_loss_count": summary["tracking"]["target_loss_count"],
            "position_rmse_m": summary["tracking"]["position_rmse_m"],
            "following_distance_rmse_m": summary["following"]["distance_rmse_m"],
            "minimum_lead_distance_m": summary["following"]["minimum_lead_distance_m"],
            "planner_success_rate": summary["runtime"]["planner_success_rate"],
            "no_collision_checked_path_count": risk[
                "no_collision_checked_path_count"
            ],
            "corridor_refresh_requested_count": risk.get(
                "corridor_refresh_requested_count", 0
            ),
            "planning_p95_ms": summary["runtime"]["planning_time_ms"]["p95"],
        }
        rows.append(row)
        grouped[(expected_side, item["method"])].append(row)

    aggregates = {}
    for (expected_side, method), group in sorted(grouped.items()):
        aggregates[f"{expected_side}:{method}"] = {
            "trial_count": len(group),
            "expected_side_selection_rate": mean([
                float(row["selected_expected_side"]) for row in group
            ]),
            "mean_target_loss_count": mean([
                row["target_loss_count"] for row in group
            ]),
            "mean_position_rmse_m": mean([
                row["position_rmse_m"] for row in group
            ]),
            "mean_following_distance_rmse_m": mean([
                row["following_distance_rmse_m"] for row in group
            ]),
            "mean_planner_success_rate": mean([
                row["planner_success_rate"] for row in group
            ]),
            "mean_no_collision_checked_path_count": mean([
                row["no_collision_checked_path_count"] for row in group
            ]),
            "mean_corridor_refresh_requested_count": mean([
                row["corridor_refresh_requested_count"] for row in group
            ]),
            "mean_planning_p95_ms": mean([
                row["planning_p95_ms"] for row in group
            ]),
        }

    result = {
        "design": {
            "type": "blocked mirrored run-level comparison",
            "unit_of_replication": "complete trial",
            "methods": ["corridor", "visibility"],
            "conditions": ["left", "right"],
            "trial_count_planned": len(design),
            "trial_count_complete": len(rows),
            "missing_summaries": missing,
        },
        "aggregates": aggregates,
        "trials": rows,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    temporary.replace(output_path)
    print(json.dumps(aggregates, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
