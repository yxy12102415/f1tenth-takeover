# Dynamic-Obstacle and Overtaking Simulation Report — 2026-10-07

## Outcome

Stages 1–3 are implemented and reproducible on the validated Ubuntu laptop.
The final standardized batch contains 15 independent trials (five scenarios ×
three seeds). All 15 met their scenario-level acceptance condition with no
collision and no map-boundary violation. Stage 4 is **not accepted**: the
Fitzpatrick map has usable geometric space, but a strict straight-window run
did not complete before the prediction horizon reached the next bend, and one
earlier full-map seed collided in the bend.

| Scenario | Trials passed | Overtake success | Collision trials | Minimum body clearance | Worst planning P95 | Worst MPC P95 |
|---|---:|---:|---:|---:|---:|---:|
| Constant-speed lead | 3/3 | 3/3 | 0 | 0.506 m | 0.233 ms | 1.808 ms |
| Acceleration/braking lead | 3/3 | 3/3 | 0 | 0.507 m | 0.225 ms | 1.736 ms |
| Stop-and-go lead | 3/3 | 3/3 | 0 | **0.406 m** | 0.193 ms | 1.804 ms |
| Injected 0.6 s LiDAR occlusion | 3/3 | 3/3 | 0 | 0.505 m | 0.210 ms | 1.714 ms |
| Insufficient passing space | 3/3 | 0/3 (expected) | 0 | 0.504 m | 0.171 ms | 2.000 ms |

Constant-speed completion time was 12.75–15.08 s (mean 13.58 s). The no-space
case did not force a pass through the static center obstruction. Native v8
no-obstacle and static-obstacle regression trials also passed without collision
or boundary violation.

The no-space trials logged 27–28 rejected/infeasible planner cycles and one
recovered MPC failure each while settling into safe following. These counts are
reported as runtime diagnostics rather than hidden; the vehicle remained safe
and the configured outcome was satisfied.

## Evidence

- Current-code aggregate results: `batch_verified_20261007.json` and `.csv`.
  `batch_final_20261007.*` is retained as an earlier development batch.
- Each scenario/seed directory contains exact config, metadata, summary, raw
  20 Hz trajectory/control/runtime CSV, and two separate English figures.
- Full per-cycle predictions and ROS logs remain in the ignored working output
  at `Code/sim_ws/results/takeover/` (228 MiB at delivery time).
- `fitzpatrick_space_audit.json` records footprint and curvature feasibility.
- `documented_failures/pre_fix_accel_brake/` preserves the rear-end failure that
  exposed unsafe stopping after a completed pass.
- `fitzpatrick/fitzpatrick_final_20261007_seed3/` preserves the failed bend run.

No failed run was deleted or reclassified. Development failures remain in the
full working output. The aggregate `verified_20261007` batch was rerun after
the final evaluator, ground-truth frame, and expected-behavior checks.

## Important interpretation

The successful behavior result is stronger than the tracking result. Once the
lead is alongside/behind, a forward LiDAR naturally loses it; whole-trial EKF
position RMSE is several metres in many passing runs because of stale or false
post-pass associations. The behavior layer rejects large identity jumps and
uses a bounded CV continuation to finish an already committed pass. This is
sufficient for the current simulation behavior test, not a publication-ready
tracker.

The current stop-and-go minimum is 0.406 m, but an earlier preserved run reached
only 0.170 m. This sensitivity is a physical-deployment blocker: require a
larger explicit threshold, recorded-scan tracker validation, shadow mode, and
low-speed closed-course tests before actuation.
