# Optional upstream SQP adapter — experimental, simulation only

## Identity and scope

Backend `upstream_sqp` actually executes the numerical methods of
[ForzaETH/predictive-spliner](https://github.com/ForzaETH/predictive-spliner),
commit `ddbfbb2312eccda7bb5d84469ec0980ddb07a5e9`. It is **not a full reproduction
of Predictive Spliner**. Specifically, we do not import its ROS 1 nodes, learned
Gaussian-process opponent model, ROC node, state machine or controller.

The original nine numerical methods are loaded from a separately obtained
checkout after SHA-256 verification, without executing the ROS imports or node
constructor. No upstream source is copied into this repository. The upstream
checkout inspected on 2026-10-22 has no LICENSE and its package.xml says TODO;
the parent race_stack MIT license must not be presumed to license a separate
submodule. Resolve licensing with the authors before redistribution/publication
of an upstream-derived release. This is a local evaluation integration.

Local adaptations (not claims about the unmodified upstream):

- Phase-specific start/end constraints: start at measured lateral position,
  finish parallel to the passing corridor instead of returning prematurely.
- Add analytic cost/constraint Jacobians, checked against numerical derivatives
  of the original functions, and tighter optimization tolerance; reject residual
  violations even if SLSQP reports success.
- Run a single-threaded, time-limited linear necessary-condition check before
  SLSQP. It is an outer relaxation including validation tolerances, not a
  collision certificate. An infeasible/failed check rejects the proposal;
  all successful proposals still require original constraints and C++ checks.
- Map-derived lateral bounds and committed-side obstacle constraints.
- Our positive-left Frenet sign convention is adapted for upstream curvature.
- C2 spline endpoint constraints match measured position, heading and curvature;
  dense interpolant-curvature constraints are included inside optimization.
  No unoptimized post-solve geometry correction is used by the final adapter.
- A soft progress/continuity term `1000 * sum((d - d_reference)^2)` augments
  upstream's objective. The reference is a maneuver seed on first entry and
  the previous valid SQP spatial path combined with the stable maneuver goal
  thereafter. This avoids endlessly
  deferring the lane change; hard obstacle/map constraints remain dominant.
- A bounded transition-length search retries infeasible geometry; it does not
  enumerate the old fixed lateral offsets or select baseline overtaking paths.
- Time parameterization, swept-map checks, same-time inflated vehicle-body
  checks and live MPC-prediction validation remain in the existing C++ stack.
- New-pass admission requires distinct fresh measurements and 0.6 s of stable
  estimated velocity/covariance. It does not alter an already committed pass,
  uncertainty inflation, target-loss timeouts or the default candidate backend.
- Penalize the C2 interpolant's spatial second derivative in physical units,
  in addition to the original knot-difference cost. This keeps the smoothing
  effect from vanishing when knot spacing is reduced.
- A C1 Hermite route interpolant and consistent measured-pose anchor remove
  polyline-knot curvature spikes during soft starts. Cancel recovery also
  preserves initial curvature. The map and full-body checks still run on the
  resulting Cartesian geometry.
- SQP map bounds and timed footprint checks reserve an additional 0.10 m for
  execution error (an engineering margin, not a formal robust-control tube).
  A separate, non-actuating MPC instance checks every valid
  SQP candidate before admission. Failed previews are recorded separately;
  the live MPC still performs its own checks and may reject later execution.
- SQP passing bounds match the larger of planner/MPC sampling inflation and
  reserve another 0.10 m of predicted body clearance. The MPC preview checks
  that reserve before a proposal is admitted.

The `3p0_1p8` simulation profile sets ego/lead maxima to 3.0/1.8 m/s. It adds
curvature-aware lead speed and braking-distance preview for ego time
parameterization. The shared MPC's default speed limit remains 2.0 m/s;
the simulation opts into runtime limits through every shooting stage and the
terminal state. Simulation takeover also uses Gym's measured ego yaw rate
instead of the real-v8 command-derived yaw observer. Lateral velocity remains
model-estimated because this Gym version publishes a zero placeholder. The
optimization formulation, hard lateral-acceleration limit and real-v8 defaults
are retained. `controller_model=official_gym_native` selects a separate generated model using Gym's
wheelbase, nominal tires and actuator-response approximations. Both preview and
live MPC use that same model, feedback mode and native 0.4189 rad steering bound;
the real-v8 default remains 0.269 rad. A known-map curvature admission check
rejects a passing corridor that exceeds the selected vehicle's geometric
steering capability. Return admission also validates the full four-second
return and reserves 0.10 m predicted body clearance. The common retimer uses
3 m/s³ jerk, consistent with the future MPC acceleration-increment constraint.
The initial return trigger uses physical longitudinal body separation; only a
candidate that also passes the complete inflated-body/return/MPC checks may
execute. Requiring an additional worst-case axis-aligned inflated separation
before considering a gradual diagonal return needlessly missed feasible return
windows. No collision inflation or return-clearance threshold is removed.
The current candidate retains a bounded curvature/arc-length progress budget
for high-speed SQP entry. It is an engineering admission heuristic, not a
completion certificate; it remains overly conservative for some weave
encounters. Experiments removing that budget, considering a return before
physical longitudinal separation, and extending the SQP spatial window did
not improve full-trial acceptance and were rolled back. Their failed results
are retained in the dated report. High-speed adaptive FOLLOW/PREPARE add
0.45 s lead-speed headway,
0.40 s closing-response allowance and relative braking distance; the retimer
uses that same gap law at future times. The original <=2 m/s/default candidate
following law remains unchanged.
See the laptop setup for exact
dependencies and remaining model differences. Return blend sizing uses measured
speed; future acceleration is limited by retiming, rather than assuming every
return must already be executed at the maximum cruise speed.

Front-only target acquisition is retained. With explicit opt-in, a recently
measured identity can use the full side/rear LiDAR mask and a larger beam-count
limit appropriate to a complete side panel. It is still gated by the existing
association and static-map rejection. The rear mask expires after 0.3 s without
a measurement; it does not extend the target-loss deadline.

Native Gym runs use a physics-step ROS `/clock` and publish only fresh sensor
states at 50 Hz. Predictor age, planner/controller timers and evaluation use
the same simulation clock. SQP/MPC solve costs and deadlines remain measured
on the wall clock. Simulation-time trial completion is bounded by an external
wall-time supervisor. Default historical profiles retain wall-time mode.

Only OVERTAKE lateral geometry uses SQP. FOLLOW/CRUISE and recovery/REJOIN remain
the existing implementation. This hybrid boundary is intentional and visible in
configuration and status, not a silent switch between two overtaking planners.
An invalid SQP proposal can cause FOLLOW/CANCEL/WAIT; it cannot silently become
an old finite-offset OVERTAKE candidate.

## Data flow

LiDAR → clustering → Cartesian CV Kalman tracker → track-assisted motion
prediction (Cartesian fallback off-route) → existing behavior/side commitment
→ upstream SQP lateral knots → C2 geometry → bounded acceleration/jerk/lag
time parameterization → time-aligned body/map validation → existing dynamic
MPC (longitudinal acceleration + steering) → safety arbiter → simulated drive.

No lead ground-truth input is added. The numerical worker receives only the
geometric optimization problem derived by the ego planner, not ROS topics. The
lead odometry remains exclusive to its own controller and independent scoring.

## Setup and launch

Keep the normal laptop/ROS 2/acados setup in `LAPTOP_TAKEOVER_SIM.md`. This
optional worker additionally needs the already available NumPy and SciPy.
From the repository root, obtain the external source if absent:

```bash
git clone https://github.com/ForzaETH/predictive-spliner.git .deps/predictive-spliner-upstream
git -C .deps/predictive-spliner-upstream checkout --detach ddbfbb2312eccda7bb5d84469ec0980ddb07a5e9
cd Code/sim_ws
OPENBLAS_NUM_THREADS=1 python3 tools/test_upstream_sqp.py
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_double_fast '' 1 '' false
```

Do not run clone/checkout over somebody's existing modified reference checkout.
The adapter refuses a different source-file hash. The default backend remains
`frenet_candidates`. Use the final argument `true` to request the existing GUI;
headless runs are preferred on this laptop. Fixed seed is not a claim of
bit-for-bit ROS wall-clock determinism.

Outputs are under `Code/sim_ws/results/takeover/<scenario>/<run_id>/` and include
configuration, dirty-source patch, new adapter-source copies, CSVs, summary and
separate English trajectory/forecast figures. Use unique run IDs; failed runs
are retained. The two-pass criterion is unchanged: two independently verified
pass-and-rejoin events in the same uninterrupted trial, not two reset trials.

## Runtime and safety limits

The local worker runs outside the control process, single-threaded CPU math,
over a private Unix socket. The caller waits at most 40 ms for one solve and
discards late results. An unavailable proposal is not sent to the MPC. The
existing independent stale-command safety arbiter remains active.

This is **not yet a hard real-time scheduling guarantee**: multiple side
proposals can cost more than one 50 ms control period. Deployment would require
an asynchronous planning schedule, state/time-stamped result validation and
measured Jetson deadlines before any actuator access. No hardware deployment
is authorized by a successful laptop trial.

## Acceptance

Integration and offline tests alone are not acceptance. Consult the dated
[result report](../results/upstream_sqp_20261022/README.md) for actual new-backend trials. Earlier adaptive Frenet results
must never be attributed to this adapter. Full upstream GP prediction,
complete method reproduction and real-vehicle validation remain out of scope
of this backend adapter.
