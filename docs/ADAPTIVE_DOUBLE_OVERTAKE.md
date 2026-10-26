# Fitzpatrick: adaptive same-side overtaking

Simulation-only extension of the existing LiDAR tracker, route-assisted
prediction, timed Frenet planner and unchanged dynamic MPC. No real-vehicle
connection or actuation is involved.

Latest measured status (2026-10-22): **6/6 higher-speed trials completed two
passes**, but only **3/6 passed the strict uninterrupted-motion audit**. Constant
lead passed 3/3 strictly; lateral drift still caused brief protected planning/
control interruptions. See the [higher-speed test report](../results/fitzpatrick_double_fast_20261022/README.md)
for raw paths, failures, exact speeds and limitations. Scenario availability
does not imply that the entire dynamic-obstacle problem is finished.

## What changed

New scenarios enable `adaptive_offset`; existing scenarios leave it disabled
for a fixed-offset regression. Entry evaluates left/right passing candidates.
Once committed, subsequent plans search **only the selected side**, considering
the configured nominal offset, 0.62/1.02/1.22 m route offsets, an opponent-relative
offset (bounded to 0.50–1.25 m), and the last accepted offset. This is a bounded
candidate search, not continuous SQP optimization or a behavior-learning model.

Adaptive-mode return paths use quintic blends preserving measured position,
heading and yaw-rate-derived curvature at each replan. Passing paths and the
fixed-offset regression retain their cubic blends.
This avoids an instantaneous curvature change when starting a return on a bend.
Candidate generation preserves time-indexed speed,
acceleration/jerk/lag, uncertainty-inflated footprint checks and map limits.
Adaptive candidates additionally enforce the v8 MPC's 0.25 m swept center map
clearance. Entry must have enough lateral room for the existing inflated bodies
at passing speed: a low-speed entry is not accepted merely because the opponent
is still far away. Before commitment a static-map corridor preview covers the
estimated passing distance (entry/return distance plus closing-distance divided
by speed advantage); a side that narrows downstream is rejected. This is a
conservative map-only precheck, not an extension of the target prediction or
observation expiry. Ranking uses capped bottleneck clearance, lateral excursion,
curvature and offset-change penalties. Feasible previous offsets get hysteresis;
an infeasible previous candidate is never retained for smoothness.
Adaptive side selection and offset search now use the same capped-clearance
score, including progress along the route over their common time horizon. This
avoids rewarding surplus wall clearance indefinitely on a slower outer bend.
An aborted adaptive maneuver's return forecast does not exceed lead speed
(subject to the existing 0.70 m/s running floor) until aligned. This cap enters
the timed trajectory and MPC reference; it is not a post-solve command override.

Before committing a return, a separate, non-actuating instance of the **same**
MPC copies the live actuator/observer state and checks candidate executability
and its actual time-indexed predicted footprints. A rejected return tries a
longer blend, or keeps overtaking on the locked side while looking for a feasible
return. Return admission requires an additional 0.10 m reserve beyond the
already inflated predicted body; millimetre-clearance previews are rejected.
That reserve is an **entry** condition, not a second collision boundary: once
returning, the original inflated-footprint and MPC constraints remain mandatory
every cycle, but a safe reduction from 0.100 m to 0.093 m does not itself cancel
the maneuver. This state-dependent admission avoids threshold chatter.
Before beginning a return it must also predict alignment within the remaining
observation-validity window, reserving one planner period. Neither condition
relaxes collision checks or extends target freshness.
Rejections are counted separately in `return_preview_rejections`; they
are not erased or mistaken for commands sent to the vehicle. The live solver
is not mutated by a preview. Unit tests verify that isolation.

The MPC model/costs/bounds, safety arbiter, observation expiry and truth isolation
are unchanged. The optimization result's speed is not independently overridden. This does not
guarantee safety against arbitrary sudden opponent cut-ins.

## Continuous two-pass acceptance

One slow opponent continuously drives the closed Fitzpatrick route. Ego must
catch/pass it, rejoin, continue around the route, then lap/pass it again. Neither
car is reset or teleported and no ground truth is injected into ego planning.

The evaluator's independent truth counter requires each distinct lap-relative
encounter to begin 1.35–7 m behind, include the OVERTAKE state, and finish more
than 1.20 m ahead with route error <0.10 m and heading error <0.15 rad, in CRUISE
for at least 0.5 s. Repeated completion messages cannot increment this counter.
Acceptance requires at least two controller-reported **and** truth-confirmed
passes, no ego/lead body collision, and no boundary violations by either car.
Planner/controller failures remain separately reported; acceptance is not a
claim of solver stability. Tests preserve all failed runs.

`fitzpatrick_double_lateral` adds a smooth 0.22 m-amplitude, two-period spatial
sine offset to the opponent's own route. Its steering controller drives that
path physically in Gym. It is repeatable lateral drift, not an adversarial
instantaneous lane cut-in. Ego knows only the unchanged reference map/route
and its LiDAR observations, not the perturbed opponent route.

## Run

From `Code/sim_ws` (Humble and pinned acados already installed):

```bash
bash tools/test_takeover_offline.sh
./run_takeover_trial.sh fitzpatrick_double_overtake '' 1 my_double_pass false
./run_takeover_trial.sh fitzpatrick_double_lateral '' 1 my_double_drift false
# GUI: same isolated trial, with RViz (one instance only)
./run_takeover_trial.sh fitzpatrick_double_lateral '' 1 my_double_gui true
# Frozen fixed-offset and static-v8 regression scenarios
./run_takeover_trial.sh fitzpatrick_1p5 '' 1 my_fixed_regression false
./run_takeover_trial.sh v8_static '' 1 my_static_regression false
# Sequential repeated trials of both new scenarios (three seeds each)
bash run_double_overtake_batch.sh 3 my_double_release
# Higher-speed stress tests: ego limit 1.8 m/s, lead target 1.0 m/s
./run_takeover_trial.sh fitzpatrick_double_fast '' 1 my_fast_trial false
./run_takeover_trial.sh fitzpatrick_double_fast_lateral '' 1 my_fast_drift false
bash run_double_overtake_batch.sh 3 my_fast_release fast
# GUI for the higher-speed constant-lead case
./run_takeover_trial.sh fitzpatrick_double_fast '' 1 my_fast_gui true
```

Both new scenarios initially specify a 110 s window, ego limit 1.5 m/s and
opponent target 0.8 m/s. The separate `fitzpatrick_double_fast` and
`fitzpatrick_double_fast_lateral` scenarios retain the same 110 s window,
spawn poses, 0.22 m drift amplitude (lateral only), and two-pass/safety
criteria, increasing ONLY ego/lead speed settings to 1.8/1.0 m/s. These are
limits/targets, not constant ego speed; curvature and maneuver constraints
still slow the car. High-speed configuration availability is not acceptance.
Unique run IDs are required; existing results cannot
be overwritten. Localhost-only DDS/domain 73 isolates the simulation.

Outputs include full CSV trajectories and time-indexed predictions, independent
`overtaking.truth_events`, source/config snapshots, two separate English plots,
and adaptive `target_offset_m` / `evaluated_candidates` diagnostics. Ground truth
is restricted to opponent control and scoring.

## Evidence interpretation and limits

- This tests one opponent lapped twice, not overtaking two independently
  controlled opponents. A success is a complete pass **and** reference-route
  rejoin; moving alongside is insufficient.
- New scenarios use 1.5 m/s ego limit, 0.8 m/s opponent target, and no injected
  measurement-noise/dropout stress. The lateral case uses a 0.22 m spatial sine
  amplitude. Abrupt cut-ins, arbitrary multi-object traffic and 2.0 m/s are not
  accepted by these tests. The 1.8/1.0 m/s stress cases are reported separately;
  no result from 1.5/0.8 m/s is counted as their acceptance.
- The tracker and route prior, bounded covariance inflation, model mismatch,
  and finite prediction horizon do not constitute a formal safety guarantee.
  ROS wall-clock scheduling means fixed seeds reproduce scenario conditions,
  not bit-identical trajectories.
- Ego Gym odometry supplies localization. Real PF error, actuator calibration,
  communication latency and Jetson worst-case timing still need separate
  validation. No real vehicle is contacted or commanded here.
- Startup invalid plans remain in the raw logs. The separate clean-motion
  audit requires zero invalid samples after 2 s, zero controller failures and
  zero solver timeouts. Collision/boundary checks cover the **entire** trial,
  including startup; their thresholds are not relaxed.
- CSV acceleration/steering-rate estimates use differences between evaluator
  samples. They can spike with message-timing jitter or final shutdown; they
  are not direct measurements of physical acceleration or steering rate.

Plots are standalone English development-report figures, not a journal-format
certification. Actual trajectories use all recorded positions without smoothing.
Squares mark independent truth-confirmed rejoin times. Prediction figures show
the first recorded model forecast of each OVERTAKE entry, with separate line/
marker styles and explicit "not ground truth" labels. Raw CSVs remain beside
the figures. The color contrast and rendered output were reviewed using the
scientific-visualization workflow; grayscale distinctions rely on line styles,
not color alone.
