# Track-assisted prediction and timed MPC integration

Implementation updated through 2026-10-18. Simulation only; nothing is installed or
enabled on the commissioned vehicle. See the test report for measured results,
including intermediate failures. A successful launch is not physical approval.

## What remains unchanged

- LiDAR clustering and the existing Cartesian constant-velocity Kalman tracker
  remain the measurement/identity source. Despite its historical EKF name, its
  current motion update is linear CV, not the MPC vehicle model.
- The reference route, static-v8 planner, generated acados solver, ten-state
  dynamic vehicle model, steering/acceleration bounds and safety arbiter remain.
- Opponent truth feeds only its own controller and the independent evaluator.
  The ego still uses `/scan`, ego odometry and the LiDAR tracker estimate.
- No real-vehicle connection, actuator command, NN model or imported ForzaETH
  implementation is part of this change.

## Prediction

The separate predictor projects the estimated target onto the reference route
to obtain longitudinal progress, lateral offset, tangential speed and lateral
speed. It advances progress along the curved route and exponentially damps
lateral **velocity** over one second; it does not force lateral position to
zero. It accounts for the offset path's different arc length. Position/velocity
variance and process noise inflate future footprints.

The route prior is gated: lateral offset must be under 1.2 m and velocity
heading must differ from the route by less than 0.65 rad. Otherwise the predictor
uses Cartesian CV, including crossing/off-route motion. During a short loss,
the forecast is anchored to the last observed track instead of repeatedly
extending the filter's straight coasting output. No filter state is overwritten.

This is a single-target, map-assisted, non-interactive prior. It is not an
intention predictor or a calibrated collision probability. The inherited
uncertainty inflation cap is heuristic. Target age limits remain 1.2 s normally
and 4 s during a committed maneuver. Expiry still stops an unfinished maneuver;
after a fully completed pass, expected rear-field-of-view loss permits route
cruise. New fresh targets are checked again.

## Recovery

Cancellation preserves the chosen side and initially holds the measured lateral
offset. A lower speed creates room behind the lead before returning, or the car
returns after it is fully ahead. Reacquisition following a mid-maneuver WAIT
enters this recovery logic rather than restarting side selection. Recovering
behind the lead is not counted as a completed overtake.
An entry is cancelled while still near the reference route if the lead's
acceleration removes the speed advantage. This addresses premature commitment
while the lead/velocity estimate is still accelerating at startup.

Return completion is checked before stale-target rejection. The 20 Hz control
loop also requests an immediate replan when observation age crosses the current
freshness limit between 5 Hz planning ticks. This uses the unchanged limit:
an aligned, fully passed vehicle can enter CRUISE; an unfinished maneuver still
enters WAIT. The counter `freshness_triggered_replans` records this event.

Return entry now requires longitudinal body separation (half the combined
vehicle lengths plus the existing body margin) and a feasible, fully checked
return candidate, rather than waiting for a fixed 1 m gap before considering a
return. The completion gap of 1.20 m, alignment criteria, footprint inflation,
steering bounds and observation-age limits are unchanged.

Hermite transitions retain measured position and heading. A smoothly faded
projection residual removes tiny discontinuities at near-zero-speed starts.
Transition length depends on offset and speed/lateral-acceleration budget;
longer blends are tried for curvature rejection without changing the steering
limit. If no candidate is feasible, the existing stop remains active.

## Planner/control contract

1. At 5 Hz the planner produces `[t, s, d, v, x, y, yaw]`. It uses bounded
   acceleration/deceleration, jerk, the MPC's 0.15 s longitudinal response,
   curvature-dependent speed preview and Frenet arc-length conversion.
   FOLLOW/PREPARE also update the future speed target from the forecast gap,
   instead of maintaining today's catch-up speed for the whole four seconds.
2. At 20 Hz the same v8 MPC receives the reference at `plan_age + k * 0.05`.
   Position and speed come from that one time-indexed trajectory. Invalid,
   non-monotone, non-finite or expired time references are rejected.
3. The speed bound is the configured absolute vehicle limit. A changed desired
   speed is expressed by the smooth reference, not an instantly tightened
   command-state bound. Timed mode executes the solver's first command-state
   transition and does not independently truncate its solved speed.
4. A numerical warm-start retry in timed mode preserves command and actuator
   estimates. It cannot make an infeasible maneuver appear feasible by resetting
   those estimates. The legacy untimed interface remains available.
5. Before publication the MPC's **own predicted motion** is checked against
   current time-aligned target footprints, uncertainty, sampling inflation and
   map clearance. A failed check or solve still emits a stop.

This is consistent reference/command integration with closed-loop revalidation,
not an equality constraint guaranteeing exact arrival at every future timestamp
or a formal recursive-feasibility proof. Both the planner and MPC run receding
horizons. Prediction/model error and future sensor loss remain limitations.

Static-v8 calls omit the optional `TimedReference` argument and retain their
existing geometric/terminal controller interface. `/takeover/reference` is a
visualization path; the simulation's actual timed handoff is in-process C++.
A future split-node Jetson adapter must preserve the time origin and all fields.

## Run and audit

From `Code/sim_ws`, with the pinned acados/Humble setup described in
`LAPTOP_TAKEOVER_SIM.md`:

```bash
bash tools/test_takeover_offline.sh
./run_takeover_trial.sh fitzpatrick_constant_speed '' 1 my_timed_trial false
./run_takeover_trial.sh fitzpatrick_occlusion '' 2 my_dropout_trial false
bash run_fitzpatrick_batch.sh 3 my_timed_batch

# Restrict entry candidates to the right for a regression (0=automatic,
# +1=left, -1=right). This never bypasses feasibility/collision checks.
TAKEOVER_PASS_SIDE=-1 ./run_takeover_trial.sh fitzpatrick_occlusion '' 3 right_regression false
python3 tools/report_timed_consistency.py results/takeover my_timed_batch \
  --expected-runs 15 --require-clean

# Explicit ablations; unique run IDs prevent overwriting evidence.
TAKEOVER_TRACK_PREDICTION=false ./run_takeover_trial.sh fitzpatrick_occlusion '' 2 cv_timed false
TAKEOVER_TIMED_CONTROL=false ./run_takeover_trial.sh fitzpatrick_constant_speed '' 1 route_untimed false
```

The flags default to `true`, are recorded in `config.json`, and are launch
parameters too. Ablations are not the original frozen release: recovery fixes
remain enabled. Use the preserved pre-change backup/commit for historical replay.

`trajectory.csv` includes current timed position/speed error, horizon reference
deviation, reference age, MPC footprint-check results and transition length.
`predictions.csv` records the planner time origin, time-indexed reference,
opponent forecast, and MPC predicted positions/speeds. MPC times are relative to
`status_stamp_s`; planner/lead times are relative to `plan_stamp_s`. Null fields
are retained when static mode, startup or a rejected solve has no such data.
`summary.json` separates behavior/safety acceptance from planner/controller
failure counters. Inspect both: the older `passed` flag alone is not a stability
claim. Post-pass target loss is expected and must not be confused with loss
while following; stale-estimate RMSE across the entire run is not tracked-only
estimation accuracy.

The runner is localhost/domain isolated, serial, headless by default and writes
logs to disk. It saves the full tracked-source patch including the shared MPC
header. Preserve rejected trials; do not relax collision tests to improve scores.

## Before a vehicle deployment

Rebuild on the Jetson architecture and validate deadlines, the time origin,
input freshness, frame conventions, covariance, actual steering/speed response,
final-command watchdog and emergency stop. Gym and the v8 actuator model differ;
these laptop timings and ideal ego localization do not establish vehicle safety.
Begin with non-actuating shadow comparisons and controlled low-speed checks.
