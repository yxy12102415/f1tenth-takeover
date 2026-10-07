#!/usr/bin/env python3
"""Create deterministic Gym adapters for the existing Fitzpatrick map/route."""

import csv
from pathlib import Path

import yaml
from PIL import Image


SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parents[1]
MAPS = SIM / "src" / "f1tenth_gym_ros" / "maps"
WAYPOINTS = SIM / "src" / "f1tenth_mpc-main" / "mpc" / "waypoints"
SOURCE_MAP = ROOT / "real_vehicle" / "maps" / "Fitzpatrick.pgm"
SOURCE_YAML = ROOT / "real_vehicle" / "maps" / "Fitzpatrick.yaml"
SOURCE_ROUTE = ROOT / "real_vehicle" / "routes" / "Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv"


def main():
    metadata = yaml.safe_load(SOURCE_YAML.read_text())
    image_name = "Fitzpatrick_sim.png"
    Image.open(SOURCE_MAP).convert("L").save(MAPS / image_name, optimize=False)
    metadata["image"] = image_name
    (MAPS / "Fitzpatrick_sim.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    with SOURCE_ROUTE.open(newline="") as source, (
        WAYPOINTS / "Fitzpatrick_sim_mpc.csv"
    ).open("w", newline="") as destination:
        reader = csv.DictReader(source)
        destination.write("# x_m,y_m,yaw_rad,v_ref_mps\n")
        writer = csv.writer(destination, lineterminator="\n")
        for row in reader:
            writer.writerow((row["x"], row["y"], row["yaw"], row["speed_mps"]))
    print("Imported Fitzpatrick_sim map and route adapters")


if __name__ == "__main__":
    main()
