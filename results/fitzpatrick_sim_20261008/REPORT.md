# Fitzpatrick laptop simulation — 2026-10-08

This is newly executed **simulation** evidence, not a physical vehicle result.
All trials run headless and sequentially on the laptop. No vehicle SSH session,
hardware driver, or physical command publisher was started.

**Verdict: partial simulation acceptance, not a real-car deployment gate.**
The revised batch passed 13/15 configured behavioral checks. Basic fixed-route
cruise and constant-speed overtaking are repeatable in these three runs; the
dropout/noise case remains unresolved. Prefer the 1.5 m/s demo, not 2.0 m/s.

## Final batch results

Batch `fitz_bottleneck_20261008`, saved in `after_side_fix/`. Each case has three
separate 40-second trials. Every trial had zero sampled body collision and zero
map-boundary violation; the two behavioral failures remain exported.

| Scenario | Passed | Smallest body gap | MPC failures per run |
|---|---:|---:|---|
| No lead, route cruise | 3/3 | Not applicable | 0 / 0 / 0 |
| Overtake at ego cap 1.5 m/s | 3/3 | 0.389 m | 0 / 0 / 0 |
| Overtake at ego cap 2.0 m/s | 3/3 | 0.402 m | 1 / 1 / 1 |
| 0.6 s dropout + 0.02 m noise, cap 1.5 | 1/3 | 0.408 m | 1 / 1 / 1 |
| Follow/wait, cap 0.9 m/s | 3/3 | 0.229 m | 0 / 0 / 1 |

For nominal 1.5 m/s overtaking, completion took **13.36–13.52 s** and post-rejoin
nearest-route-point RMSE was **0.080–0.082 m**. TRACKED-only target position RMSE
was 0.073–0.076 m, but whole-trial RMSE was about 8.2 m because stale estimates
continue after the lead leaves the forward sensor field. Do not quote the
TRACKED-only metric as whole-trial tracking accuracy.

Across the final batch, the worst per-trial planning P95 was **0.160 ms** and
MPC P95 was **1.921 ms**. These are laptop compute-time samples, not end-to-end
latency, worst-case execution-time bounds, or Jetson performance.

At 2.0 m/s all three trials had one solver rejection during rejoin and a
fail-stop command step of about 40 m/s² (speed-command derivative). This is a
control-smoothness failure despite the behavioral pass. In the follow case,
waiting is permitted by the current checker and the minimum gap is only about
0.23 m; this does not establish production-quality adaptive cruise control.

Dropout/noise seeds 2 and 3 cancelled after `curvature_limit` rejection at about
5.8/6.2 s, accumulated 157/156 planner failures, and eventually entered WAIT
after losing the target. Seed 2 initially chose left; seed 3 initially chose
right. Thus improved side scoring helps, but **does not solve noisy trajectory
feasibility or cancellation recovery**. No rejection threshold was relaxed.

Next implementation priorities:

1. Generate a curvature-feasible continuation/abort path from the actual pose
   and heading after noise/dropout, without switching sides while committed.
2. Make longitudinal cap transitions and time-indexed collision prediction
   consistent with MPC acceleration/jerk limits; preserve fail-stop if no
   feasible solution exists.
3. Add stronger follow progress/headway criteria, separate dropout-only from
   noise-only cases, and test varied initial gaps, lateral positions and lead
   acceleration/braking before expanding physical operation.

Offline verification: 5 standalone C++ candidate/footprint checks and 7 Python
full-footprint evaluator checks passed. This includes a small occupied map cell
inside a vehicle with all four corners in free space.

Three additional Gym smoke regressions, `fitz_regression_20261008_seed1`, passed
and are exported in `regressions/`: original v8 clear-route mode (12 s), original
v8 static-obstacle mode (14 s), and standard-map constant-speed overtaking (28 s).
These are newly run, short simulation checks, not new real-v8 full-lap evidence.
Total documented campaign: **33 trials**, including the 15-run before-fix batch,
15-run after-fix batch and these three regressions. Earlier exploratory runs
and startup failures remain under `Code/sim_ws/results/takeover/` and are not
silently included in the final acceptance denominator.

## Configuration and scope

- Existing Fitzpatrick occupancy map and approximately 44.73 m fixed route.
- Start: ego `(0.613857424, -0.347118616, 0)`, lead
  `(2.824191660, -0.027387507, 0.287312773)` in map coordinates.
- Forty-second trials. Lead target speed 0.8 m/s. Ego caps 0.9, 1.5 or 2.0 m/s.
- Single lead vehicle; map-assisted LiDAR clustering/EKF, world-frame CV
  prediction, side-committed Frenet offsets, shared real-v8 dynamic MPC.
- Known-map filtering removes clusters explained by walls; a 1.5 m reference-
  route association band rejects distant clutter. This assumption must be
  revisited for off-route traffic and noisy physical localization.
- Opponent truth is used only for its own controller and evaluation. Ego
  localization uses Gym odometry, not a noisy PF simulator.
- `fitzpatrick_occlusion` injects a 0.6 s **measurement dropout** starting at
  4 s, plus 0.02 m Gaussian measurement noise. It is not a moving physical
  occluder or a general natural-occlusion benchmark.
- Seeds 1–3 control tracker perturbations. Gym's LiDAR uses its fixed default
  seed 12345. Counts below measure repetition on one layout, not independent
  traffic/map randomization or a statistically established success probability.

## Changes under test

1. Headless runner defaults to Fitzpatrick, bounds build/BLAS/Numba parallelism,
   serializes trials, and isolates DDS to localhost/domain 73. Verbose build and
   ROS output goes to trial files. No deletion of old logs was needed.
2. No-target behavior follows the original v8 route planner. Static map/route
   gates prevent wall clusters becoming fictitious leads. Stale post-pass
   tracks do not retrigger FOLLOW after route progress wraps.
3. Receding-horizon lateral paths use measured start position and heading with
   a finite remaining Hermite transition, avoiding residual-offset snaps.
4. Final side ranking uses the smaller of map and inflated predicted-body
   clearance; validity gates, footprint sizes, inflation and collision limits
   are unchanged. Ranking wall clearance alone preferred the failing side.
5. Evaluation checks occupied cells under the entire oriented body, not just
   corners, and requires an independent truth-confirmed pass and route rejoin.
   Whole-trial RMSE remains reported, including stale/lost estimates; an
   additional TRACKED-only RMSE is clearly labeled.

## Preserved pre-selection-fix batch

Batch `fitz_accept_20261008`: 15 completed trials; no sampled collision or map
boundary violation. All results, including six failed trials, are archived in
`before_side_fix/`.

| Scenario | Passed / repetitions | Interpretation |
|---|---:|---|
| No lead, ego cap 1.5 m/s | 3/3 | More than 90% route-sample progress |
| Lead 0.8, ego cap 2.0 m/s | 2/3 | Failed repetition initially selected right |
| Lead 0.8, ego cap 1.5 m/s | 1/3 | Failed repetitions initially selected right |
| Dropout/noise, ego cap 1.5 m/s | 0/3 | No completed pass; preserved failures |
| Follow, ego cap 0.9 m/s | 3/3 | No pass; FOLLOW/WAIT/CANCEL observed |

The failure diagnosis is supported by logged initial side and subsequent
solver failures. An approximate offline candidate reconstruction additionally
showed greater wall clearance on the right but less clearance to the inflated
opponent prediction. This reconstruction is not an exact solver replay because
the CSV does not retain the complete EKF covariance at each callback.

## Interpretation limits and next work

- Behavioral `passed=true` does **not** require zero MPC failures or smooth
  commands. Inspect runtime rejection counts and command-rate metrics before
  drawing deployment conclusions. Safety stopping on solver failure remains
  active; do not suppress it to improve completion counts.
- Forty-second, 20 Hz footprint checks detect observed collisions, not all
  possible continuous-time contacts. Clear-route acceptance is not an endpoint
  stop test. Following acceptance currently allows waiting and has no minimum
  progress/maximum waiting-time threshold.
- CV prediction can be inaccurate through bends; timed planner references are
  reduced to a geometric path and cap at the MPC interface. The exact future
  s(t) used in collision checking is not enforced by MPC.
- Final-command acceleration is a finite-difference **command** metric, not
  measured body acceleration. Emergency zeros can create large command jumps.
- The tracker can lose the lead after passing beyond its forward field of
  view. Continuing on the route is not evidence of rear-facing perception.
  The post-pass identity jump gate also needs multi-target/reacquisition tests.
- No new Jetson, dynamic physical overtaking, or interactive RViz acceptance
  is claimed. Desktop freeze root cause remains unproven; memory observations
  during these headless runs showed about 27 GiB available and no swap use.

## Reproduction

From the repository's `Code/sim_ws` directory:

```bash
./run_takeover_trial.sh fitzpatrick_1p5 '' 1 my_fitz_trial false
bash run_fitzpatrick_batch.sh 3 my_fitz_batch
bash tools/test_takeover_offline.sh
```

Use a fresh run ID: existing runs cannot be overwritten. Each trial saves
configuration, source commit/patch, summary, trajectory and prediction CSVs,
launch logs, and separate English trajectory/prediction PNGs. Exported raw CSVs
are gzip-compressed losslessly; `MANIFEST.json` records archive hashes.
The complete batch is currently expected to exit nonzero because dropout/noise
does not meet acceptance. A nonzero result is intentionally not masked.

Host: Ubuntu 22.04 x86_64, ROS Humble, acados v0.4.0
`43354449192aa51557abda37d1bc17d907077354`. Imported F1TENTH Gym:
`4fdb9c7e6fb7c701290f4dc18377d07c1681724f` (clean external checkout), Gym 0.19.0,
NumPy 1.21.5, Numba 0.55.1, SciPy 1.8.0. The editable Gym import on this host
comes from the outer workspace's `Code/sim_ws/external/f1tenth_gym`, not a Jetson
binary. Wall-clock ROS scheduling prevents bit-identical reruns.
