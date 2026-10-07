#!/usr/bin/env python3
"""Package compact, reviewable evidence while leaving full ROS logs in-place."""

import shutil
from pathlib import Path


SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parents[1]
SOURCE = SIM / "results" / "takeover"
DESTINATION = ROOT / "results" / "takeover_sim_20261007"


def copy_files(source, destination, names):
    destination.mkdir(parents=True, exist_ok=True)
    for name in names:
        path = source / name
        if path.exists():
            shutil.copy2(path, destination / name)


def main():
    DESTINATION.mkdir(parents=True, exist_ok=True)
    copy_files(SOURCE, DESTINATION, (
        "batch_verified_20261007.json", "batch_verified_20261007.csv",
        "batch_final_20261007.json", "batch_final_20261007.csv"
    ))
    for scenario in (
        "constant_speed", "accel_brake", "stop_go", "short_occlusion", "no_space"
    ):
        for seed in (1, 2, 3):
            run = SOURCE / scenario / f"verified_20261007_seed{seed}"
            copy_files(run, DESTINATION / scenario / f"seed{seed}", (
                "config.json", "metadata.json", "summary.json", "trajectory.csv",
                "trajectory_en.png", "prediction_en.png",
            ))
    for scenario, run_id in (
        ("v8_clear", "verified_v8_clear_20261007"),
        ("v8_static", "verified_v8_static_20261007"),
    ):
        copy_files(
            SOURCE / scenario / run_id,
            DESTINATION / "regression" / scenario,
            ("config.json", "metadata.json", "summary.json", "trajectory.csv", "trajectory_en.png"),
        )
    # Keep both successful and failed Fitzpatrick attempts visible.  The strict
    # straight-window run is the current stage-4 result; older full-map runs
    # explain why the map is not marked accepted.
    fitz_source = SOURCE / "fitzpatrick_constant_speed"
    for run in sorted(fitz_source.glob("*")):
        if run.is_dir():
            copy_files(run, DESTINATION / "fitzpatrick" / run.name, (
                "config.json", "metadata.json", "summary.json",
                "trajectory_en.png", "prediction_en.png",
            ))
    copy_files(
        SOURCE / "accel_brake" / "acceptance_accel_brake_seed1",
        DESTINATION / "documented_failures" / "pre_fix_accel_brake",
        ("config.json", "metadata.json", "summary.json", "trajectory_en.png", "prediction_en.png"),
    )
    print(f"Packaged compact evidence at {DESTINATION}")


if __name__ == "__main__":
    main()
