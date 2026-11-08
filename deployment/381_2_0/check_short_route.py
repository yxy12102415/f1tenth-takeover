#!/usr/bin/env python3
"""Reproducible 1-D distance screen, not a trajectory acceptance test."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def screen(cap, lead, acceleration, braking, gap, ahead, delay):
    # Ego starts at rest; lead moves at constant speed, including the delay.
    # Profile: accelerate, cruise if needed, brake to rest. Braking is allowed
    # during the pass. This deliberately ignores lateral dynamics and geometry.
    gain = gap + ahead + lead * delay
    reciprocal = 1.0 / acceleration + 1.0 / braking
    ramp_gain = 0.5 * cap * cap * reciprocal - lead * cap * reciprocal
    if ramp_gain >= gain:
        # Triangular profile reaches the required final relative displacement.
        peak = lead + (lead * lead + 2.0 * gain / reciprocal) ** 0.5
        cruise_time = 0.0
    else:
        peak = cap
        cruise_time = (gain - ramp_gain) / (peak - lead)
    moving_time = peak * reciprocal + cruise_time
    distance = 0.5 * peak * peak * reciprocal + peak * cruise_time
    final_gap = distance - (gap + lead * (delay + moving_time))
    assert abs(final_gap - ahead) < 1e-9
    return {
        "ego_cap_mps": cap, "lead_mps": lead,
        "initial_stationary_delay_s": delay, "peak_speed_mps": peak,
        "cruise_time_s": cruise_time, "elapsed_time_s": delay + moving_time,
        "ego_distance_to_stop_m": distance, "final_ahead_gap_m": final_gap,
    }


def main():
    config = json.loads((HERE / "candidate_config.json").read_text())
    length = config["route_length_m"]
    rows = []
    for cap in (1.5, 1.8, 2.0):
        for delay in (0.0, 0.6, 1.0):
            row = screen(cap, config["lead_speed_candidate_mps"],
                         config["ego_acceleration_limit_mps2"],
                         config["ego_deceleration_limit_mps2"],
                         config["initial_gap_candidate_m"], 1.2, delay)
            row["remaining_distance_m"] = length - row["ego_distance_to_stop_m"]
            row["fits_1d_length_only"] = row["remaining_distance_m"] >= 0
            rows.append(row)
    report = {
        "route_length_m": length, "lead_speed_confirmed": False,
        "physical_motion_ready": False,
        "scope": "Ideal 1-D accelerate/cruise/brake screen; NOT a collision-free maneuver or terminal-stop certificate.",
        "assumptions": [
            "Ego starts stationary; opponent maintains 0.5 m/s throughout.",
            "Initial reference-point gap 1.5 m; final ego lead 1.2 m.",
            "Acceleration 1.0 m/s^2 and braking 0.8 m/s^2 are ideal constant capabilities, not measured coast performance.",
            "Braking may overlap the overtake; lateral displacement, steering lag, curvature speed limits, margins and coast policy are omitted.",
            "The 0.6/1.0 s delays are sensitivity examples, not measured startup latency.",
            "A stopped ego can subsequently be caught by a continuing opponent; coordinated end-of-test stopping is not modeled.",
        ],
        "cases": rows,
    }
    output = HERE / "reports" / "short_route_longitudinal.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
