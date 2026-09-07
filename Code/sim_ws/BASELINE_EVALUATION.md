# Gym Pure-LiDAR Baseline Evaluation

Run a complete timed trial from `Code/sim_ws`:

```bash
./run_baseline_trial.sh constant_speed 60
```

Arguments are `scenario`, `duration_seconds`, optional `run_id`, optional
`map_name`, and optional `method`. Supported lead scenarios are
`constant_speed`, `accel_brake`, `turning`, and `stop_go`. The method defaults
to `nominal`; paper-method ablations are `corridor`, `visibility`, `adaptive`,
and `full`.

Each run writes `results/<scenario>/<run_id>/` with `tracking.csv`,
`following.csv`, `control.csv`, `runtime.csv`, `decision.csv`, `metadata.json`, and
`summary.json`. The evaluator compares the LiDAR EKF output against Gym ground
truth; ground truth is not subscribed to by the tracker, planner, or MPC.

The ego stack also records `/ego_tracker/status`. `TRACKED` means the current
scan supplied an associated LiDAR cluster, `COASTING` means that the published
state is a bounded EKF prediction, and `LOST` causes the controller to stop.
Predicted output does not reset measurement freshness, so coasting cannot hide
a stale target from the safety checks or evaluator.

Generate the deterministic obstacle maps once (or regenerate them identically):

```bash
./generate_baseline_maps.py
```

Available maps are `Melbourne_map_obstacle_center`,
`Melbourne_map_obstacle_left`, and `Melbourne_map_obstacle_right`. For example:

```bash
./run_baseline_trial.sh constant_speed 60 obstacle_center_01 \
  Melbourne_map_obstacle_center
```

These maps only prepare the next evaluation phase. No obstacle-avoidance tuning
is part of this baseline.

## Standardized benchmark maps

The parameterized benchmark maps are generated with:

```bash
./generate_standardized_maps.py
```

They include `benchmark_straight`, primary `benchmark_occlusion_a`, held-out
`benchmark_occlusion_b`, and center/left/right obstacle variants. The trial
runner automatically selects the matching starting poses and lead waypoint file:

The current obstacle-free baseline uses only the three base map names. The
straight stadium has gentle `4.0 m`-radius end turns; Map A is a larger rounded
square. Direct `gym.sh` launches of a benchmark map also select its correct
spawn poses, rather than retaining the out-of-bounds Melbourne defaults.

```bash
./run_baseline_trial.sh turning 30 map_a_center_01 \
  benchmark_occlusion_a_obstacle_center
```

See [STANDARDIZED_BENCHMARK_MAPS.md](STANDARDIZED_BENCHMARK_MAPS.md) for the
geometry, generated artifacts, physical-course interpretation, and experiment
rules.

## Stabilized baseline profile

The repeatable baseline provides both a low-speed control and a
curvature-aware higher-speed condition:

- `constant_speed`: lead fixed at `1.2 m/s` as the control condition;
- `turning`: waypoint speed is capped at `2.4 m/s` and decreases with
  curvature according to a `0.65 m/s^2` nominal lateral-acceleration limit;
- lead longitudinal acceleration limit: `1.0 m/s^2`;
- ego speed and acceleration limits: `3.0 m/s` and `1.8 m/s^2`;
- nominal following-distance parameter: `1.0 m`.

The stability repair uses LiDAR position-only EKF updates, covariance-aware
association, short bounded prediction, a velocity-informed rear anchor, one
second of MPC preview, and continuous yaw unwrapping at the `+pi/-pi`
boundary. Safety timeouts and the final command arbiter remain active.

The nominal controller foundation for the later paper method now predicts a
constant-turn-rate target pose over the one-second MPC horizon instead of
planning only to the target's current rear anchor. Its nominal reference is
`d_ref = clip(1.0 + 0.15*v_ego, 1.0, 1.8) m`, with bounded spacing feedback and
a CBF-like speed cap `v_ego <= v_lead + gamma*(d-d_ref)`. A compact LiDAR
cluster matched to the freshly associated lead, plus its configured vehicle
footprint, is excluded from static occupancy so the moving-anchor path does not
treat the lead as a permanent wall. No exclusion is made while the tracker is
coasting.

Representative low-speed obstacle-free Map A stability regressions were:

| Scenario | Duration | Target losses | Position RMSE | Velocity RMSE | Planner success |
| --- | ---: | ---: | ---: | ---: | ---: |
| constant speed | 25.1 s | 0 | 0.039 m | 0.067 m/s | 100% |
| acceleration/braking | 25.1 s | 0 | 0.033 m | 0.064 m/s | 100% |
| turning | 30.0 s | 0 | 0.035 m | 0.048 m/s | 100% |
| stop-and-go | 30.1 s | 0 | 0.042 m | 0.084 m/s | 93.9% |

These are smoke/stability trials, not final paper statistics. Use repeated runs
with fixed durations and run IDs for the final comparison. Map B is retained as
a held-out hard case: its tighter blind turns still cause natural occlusion,
association loss, and planner failures. That failure is a useful motivation and
test case for the later risk-aware method rather than part of the nominal
baseline operating envelope.

The final moving-anchor curvature-aware smoke tests produced:

| Map | Duration | Target losses | Fixed-1m distance RMSE | Dynamic-reference RMSE | Minimum distance | Planner success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| enlarged straight | 35.1 s | 0 | 0.650 m | 0.408 m | 1.275 m | 100% |
| Map A | 35.1 s | 0 | 0.673 m | 0.471 m | 1.388 m | 100% |
| Map A stop-and-go | 30.0 s | 0 | 0.485 m | 0.368 m | 0.847 m | 100% |

Run this variable-speed condition with `scenario=turning`. The name is retained
for command compatibility, but its semantics are now “follow the curvature
speed profile,” not “drive the whole lap at one turning speed.”

The nominal dynamic reference remains a speed/headway controller and is not
itself the paper's uncertainty-aware contribution. The `0.847 m` stop-and-go
minimum shows why the paper layer must be validated against minimum distance
and target retention, not distance RMSE alone.

The paper layer is now implemented behind the `method` argument without
changing the default nominal behavior. See
[RISK_AWARE_METHOD.md](RISK_AWARE_METHOD.md) for its equations, candidate
planner, visibility model, fail-closed behavior, ablation modes, and current
integration-test status. Final repeated experiments and statistical comparison
remain separate from implementation completion.

For `adaptive` and `full`, the evaluator records both the historical fixed-gap
error and error relative to the controller's dynamic following reference. It
also logs raw risk, filtered risk, and the applied positive-closing-speed
scale. Use `dynamic_distance_rmse_m` as the primary gap-tracking metric for
these modes. Retaining `distance_rmse_m` makes the cost of intentionally
increasing the safety gap visible, but that offset is not itself a control
error.

The September 5 corridor stabilization additionally uses motion-confirmed
global target reacquisition, variable-offset curved corridors, Euclidean grid
inflation, side-labelled committed paths, side locking, and forward/heading-
consistent suffix trimming. The former Map A center-obstacle failure now passes
three consecutive 15-second trials with zero target losses and `100%` planner
success. This does not change the rule that `/opp_racecar/odom` is evaluator-
only for the ego stack.

The subsequent visibility V1 implementation keeps collision and line-of-sight
occupancy separate, connects adjacent observed surface returns, evaluates a
five-point weighted target belief over a three-second CV horizon only when
visibility can affect candidate selection, and uses a `0.075` decision
deadband. Each result now includes event-complete `decision_events.csv` in
addition to the 20 Hz sampled decision log.

Focused mirrored maps `benchmark_straight_visibility_left` and
`benchmark_straight_visibility_right` are available for the visibility
hypothesis. Run a balanced corridor-versus-visibility batch with:

```bash
./run_visibility_comparison.sh 12 5 visibility_confirmatory_01
```

The frozen-parameter `visibility_confirmatory_v2` batch completed all 20
planned trials (five repetitions per method and mirrored condition).
Visibility selected the expected side in all `10/10` visibility trials with
zero target losses. Mean position RMSE was `0.0204 m` left and `0.0186 m`
right; mean planner success was `100%` left and `98.78%` right. The right
condition still averaged `2.6` transient fail-closed, same-side refresh cycles
per trial. Visibility actively changed the geometry-preferred left pass to the
expected right pass in all five right-condition trials. This passes the focused
simulation integration gate but is not a paper-level generalization result.
The curved and held-out cases remain open.
See [RISK_AWARE_METHOD.md](RISK_AWARE_METHOD.md) for the exact current gate.

The current `full` development configuration subsequently reduced the minimum
corridor offset to `0.58 m` and clears only the non-nominal ego start footprint
in collision space. A same-cycle, same-side geometry refresh is accepted only
after a new collision check. Six mirrored development runs had zero target
losses, about `99.84%/99.38%` mean planner success, and `0.506/0.537 m` mean
dynamic-gap RMSE on left/right. A few fail-closed cycles remain. Consequently
`full` is implemented and diagnosable, but not yet stability-validated or
ready for autonomous real-vehicle driving.

For physical transfer, use
[REAL_VEHICLE_DEPLOYMENT.md](REAL_VEHICLE_DEPLOYMENT.md). The dedicated real
launch disables the simulated lead controller and ground-truth evaluator,
uses SensorData QoS for LiDAR, maps ego odometry to `/odom`, and defaults to a
non-actuating `/drive_shadow` output with reduced command limits.
