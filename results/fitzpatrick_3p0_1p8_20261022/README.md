# Fitzpatrick simulation: 3.0 / 1.8 m/s maximum speeds

Simulation-only work on 2026-10-22. Ego speed cap is 3.0 m/s and opponent
speed cap is 1.8 m/s; both slow for curvature. The opponent's configured
lateral acceleration limit is 1.2 m/s². These are maximum speeds, not claims
of constant-speed motion around the entire track.

Acceptance requires two independently evaluated pass-and-rejoin events in
one continuous 180 s trial, zero scored collisions and boundary violations.
MPC rejections, SQP timeouts and invalid plans remain visible separately.
No collision threshold or body dimensions were relaxed. Opponent truth is
exclusive to its own control and evaluation; a live graph audit confirmed
the ego tracker uses LiDAR. This is the numerical SQP adapter described in
`../../docs/UPSTREAM_SQP_ADAPTER.md`, not the entire Predictive Spliner stack.

## Preserved development trials

- `sqp_3p0_1p8_20261022_seed1`: full 180 s constant-lead trial, one pass,
  zero collisions/boundary violations, 19 MPC solver rejections. Observed
  maxima were approximately 3.000 and 1.800 m/s. Failed two-pass acceptance.
- `sqp_braking_20261022_clear`: full 40 s empty-track trial, zero collisions
  or boundary events, but 587 MPC solver rejections. Failed route-cruise
  acceptance. The old geometric-only fallback could not provide a sufficiently
  long, consistent high-speed reference.
- `sqp_braking_20261022_seed1`: full 180 s constant-lead trial, zero passes,
  zero collisions/boundary violations, one MPC dynamic-conflict rejection.
  SQP geometry still contained unnecessary curvature oscillations. Failed.

## Current implementation and validation

- Shared C++ MPC accepts an explicit speed limit. Its default remains 2 m/s.
  The simulation opts into 3 m/s. Measured-state bounds, command integration,
  warm-start reset, all 24 prediction stages and terminal bounds use that limit.
  The default real-v8 construction and generated solver defaults are retained.
- The lead controller receives the configured maximum rather than a hidden
  1 m/s limit. Its new curvature/braking governor is opt-in for these scenarios.
- High-speed timed planning previews braking distance including jerk/response
  allowance, preserving acceleration and steering optimization in the MPC.
- SQP geometric curvature allowance includes the known route bend and measured
  initial curvature, while full Cartesian steering, timing, lateral acceleration,
  map and body checks still decide feasibility.
- The adapter penalizes spatial C2 second derivatives in physical units; the
  original unscaled knot cost alone became weak at the adapter's dense spacing.
- Cancel recovery preserves initial curvature. Explicitly empty scenarios use
  the same timed planner; target freshness is still required when a lead is enabled.

Offline candidate, timing, curved braking, cancel curvature, target freshness,
MPC 2.8 m/s solving/default-limit isolation, socket and SQP Jacobian checks pass.

## Subsequent development results (retained, not a pooled acceptance batch)

All full trials below ran approximately 182 wall-clock seconds, including
shutdown. Collision counts refer to the independent 20 Hz oriented-body score,
not a continuous-time certificate. Short smoke tests are explicitly separate.

| Run prefix | Scenario | Truth passes | Collisions | Controller rejections | Result |
| --- | --- | ---: | ---: | ---: | --- |
| `sqp_smooth_20261022` | Constant | 1 | 2 | 0 | Failed |
| `sqp_recovery_20261022` | Constant | 0 | 1 | 10 | Failed; planned lateral trial not run |
| `sqp_feedback_20261022` | Constant / lateral | 0 / 0 | 0 / 1 | 11 / 9 | Failed |
| `sqp_gym_model_20261022` | Constant | 1 | 0 | 9 | Failed |
| `sqp_full_lidar_20261022` | Constant / lateral | 0 / 0 | 2 / 0 | 38 / 23 | Failed |
| `sqp_side_geometry_20261022` | Constant / lateral | 2 / 0 | 0 / 0 | 0 / 29 | Constant passed; lateral failed |
| `sqp_accept_3p0_1p8_20261022_seed1..3` | Constant, 3 repeats | 1 / 1 / 1 | 0 / 0 / 0 | 1 / 6 / 2 | All failed; batch stopped before lateral |

The earlier `official_gym` development model matched Gym's 0.3302 m wheelbase
but retained the real-v8 0.269 rad steering limit. It is not the current
`official_gym_native` profile, which uses Gym's native 0.4189 rad limit in both
the generated optimizer and the C++ planner/command checks. The original
real-v8 model/defaults remain separate and unchanged by profile selection.

Tracker fixes retain front-only initial acquisition, but permit a recent known
identity to use actual side/rear scan beams and up to 300 points per side-panel
cluster. The old 140-point cap rejected close, fully visible side panels.
Retiming now uses 3 m/s³ jerk; full-return clearance and known-map steering
feasibility are checked before committing to the maneuver.

`sqp_return_guard_20261022_smoke` and
`sqp_native_steer_20261022_smoke` are **35-second, one-pass diagnostic tests**.
Both passed that limited criterion with zero controller rejections or scored
collision/boundary events. Neither proves two-pass acceptance. Full trials of
the native profile are stored under the distinct `sqp_native_pair_20261023`
prefix; consult their completed summaries rather than treating this historical
development table as current acceptance.

## Reproduction

From `Code/sim_ws`, using fresh run IDs:

```bash
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_clear_3p0 '' 1 NEW_CLEAR false
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_double_3p0_1p8 '' 1 NEW_RUN false
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_double_3p0_1p8_lateral '' 1 NEW_LATERAL false
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_double_overtake_batch.sh 3 NEW_BATCH 3p0_1p8
```

Set the final argument to `true` for a single GUI trial. The batch runs headless,
serially and on localhost. Each trial saves raw CSV, config/source snapshot,
summary, and separate English actual-trajectory and forecast figures under
`Code/sim_ws/results/takeover/<scenario>/<run_id>/`.
