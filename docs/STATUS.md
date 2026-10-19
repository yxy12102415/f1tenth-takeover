# Current status — source export 2026-10-06

## Development snapshot — 2026-10-20 (not release acceptance)

Track-assisted prediction, time-indexed references to the retained v8 MPC,
cancel/rejoin recovery and control-clock freshness-triggered replanning are
implemented on `codex/track-aware-timed-mpc-20261016`. Latest evidence is from
2026-10-18, not a new physical test. The latest right-side dropout/noise trial
completed in full with zero collision, boundary violation or MPC failure.
The preceding 12-run campaign still contained a collision and an additional
fail-stop trial; both remain archived. Full multi-seed validation and static-v8
regressions on the latest code are **pending**. No real-car deployment approval.

See [the progress/evidence report](../results/track_aware_timed_20261018/REPORT.md)
and [the implementation contract](TRACK_AWARE_TIMED_CONTROL.md). Historical
results below describe their own code versions, not acceptance of this branch.

The most recent physical evidence in this workspace is dated **2026-09-27**, not the export date.

| Entry point on the commissioned Jetson | Active release | Evidence scope |
|---|---|---|
| `fitzpatrick_avoidance_dynamic_mpc.sh` | `ground_fitzpatrick_joint_20260927_v8` | One full Fitzpatrick A/B lap passed; operator confirmation saved. |
| `fitzpatrick_avoidance_pid.sh` | `ground_fitzpatrick_joint_20260927_v2` | Separate earlier PID branch. The recorded `20260927_212615` result has `passed=false`; do not count the v8 MPC run as PID acceptance. |

Historical PP/PID/MPC tests on the 381 map are separate experiments. They do not establish Fitzpatrick dual-obstacle acceptance for those controllers.

## v8 change

A previous run stopped on the return from obstacle B with solver status 4. Replacing lateral velocity with zero and yaw rate with a kinematic estimate every tick was inconsistent with the optimizer's dynamic prediction. v8 propagates those estimates continuously using the same backward-Euler model. The user also authorized the lateral prediction bound to change from 1.50 to 1.65 m/s². Collision clearances, steering limits, L1 behavior, 500 ms input age and 45 ms compute deadline were retained.

## Evidence

- Unmodified recorded inputs: 220-frame and 428-frame fault replays passed; observer equation/reset checks passed.
- Four two-obstacle full-lap model cases and three actuator-response cases passed.
- Non-actuating ROS obstacle/fault checks and three endpoint checks passed.
- Field run `20260927_232018-closed-frenet-ground-attempt`: 44.7315 m in 27.9834 s, endpoint error 0.07453 m, no active invalid controller messages. Final ERPM/current/fault were zero.
- Operator: both obstacles passed without contact, little oscillation, natural stop.

B was passed on the opposite side from the previous failed trial. A single field run does not isolate the effects of state estimation, constraint margin and planner choice. Repeatability at other placements remains to be tested.

Published JSON files preserve selected original evidence. `RAW_LOG_HASHES.json` identifies the larger logs retained on the Jetson. Plots use PF-estimated vehicle positions and accumulated LiDAR returns; neither is an independent survey or a measurement of physical body clearance.

## Next work

1. Repeat the same configuration at unchanged A/B positions.
2. Test additional static-obstacle positions with adequate maneuvering space.
3. Evaluate PID separately before claiming comparable results.
4. Add moving-obstacle prediction and behavior selection before overtaking tests.

## Laptop simulation update — 2026-10-08

This does not replace or extend the physical evidence above. On the existing
Fitzpatrick map/route, the new headless simulation campaign ran two 15-trial
batches and preserved all failures. The final batch passed 13/15 configured
checks: route cruise 3/3, 1.5 m/s overtaking 3/3, 2.0 m/s overtaking 3/3,
follow/wait 3/3, dropout-plus-noise 1/3. All 15 were free of sampled collision
and boundary violation. The 2.0 m/s runs still each triggered one MPC fail-stop;
dropout recovery remains incomplete. This is **not** a physical overtaking gate.

See [the new report](../results/fitzpatrick_sim_20261008/REPORT.md), including
compressed raw trajectories/predictions and separate English plots. Prefer
`./run_takeover_trial.sh fitzpatrick_1p5 '' 1 my_trial false` from `Code/sim_ws`.
