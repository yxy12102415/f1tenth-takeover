# Timed passing-speed experiment — 2026-11-05

Simulation-only development. The accepted real-v8 export is unchanged.

## Question and prospective screen

Does releasing passing speed inside the timed forecast, with model-informed
braking preview, improve complete passing and rejoining without more safety
interventions? Freeze the implementation before the screen, keep ego/lead caps
3.0/1.5 m/s, command bounds [-3,+3] m/s² and the existing evaluator. Use four
Fitzpatrick spawn positions, 35 simulated seconds each, one fresh-process trial
per arm per position. Shuffle blocks and arm order with seed 20261105. Both arms
use the same recovery fixes; the control is the current legacy speed policy,
not the original historical commit. This is an exploratory engineering screen,
not a powered or deterministic success-rate estimate. Keep every failed run.

```bash
cd Code/sim_ws
python3 tools/run_stability_comparison.py --profile-comparison \
  --prefix speed_profile_screen_20261105 --schedule-seed 20261105 --plan-only
python3 tools/run_stability_comparison.py --profile-comparison \
  --prefix speed_profile_screen_20261105 --schedule-seed 20261105
```

## Implementation

`TAKEOVER_PREDICTIVE_SPEED_PROFILE=true` with `upstream_sqp` enables the
current revised candidate policy:

- Try the requested passing cap, an intermediate cap, then the lead-speed cap.
  Recompute timing and require full inflated-body/map and MPC admission for
  each proposal. Select the fastest admitted cap, including at side selection.
  This removes the fixed current-offset speed gate without bypassing collision
  checks. Acceleration remains jerk/actuator limited, not an instantaneous jump.
- Forecast braking with the 50 ms command integrator, jerk 3 m/s³, configured
  acceleration/braking response and current positive acceleration/command lead.
  Reserve a 0.2 s planning tick plus a 0.5 s speed-reference response allowance.
  Invert this braking-distance estimate to get a curve-approach speed cap.
  Integrate the actual Cartesian spline arc length for the available braking
  distance. A metre of reference-route progress is not necessarily a metre of
  physical travel on an offset bend. The route preview allows the existing
  0.5 minimum progress metric so an inner curve is not overlooked.
  This is a model-informed proposal, not a proof that the full MPC always
  realizes the assumed braking policy. The nonlinear MPC preview and runtime
  rejection remain mandatory. Curve lateral budget stays 1.2 m/s².
- Bound the spline optimizer's spatial curvature by the vehicle steering
  limit, then apply the separate speed-dependent lateral budget during timing.
  The former current-speed budget at every future knot could declare a path
  infeasible even if braking made it executable. Cartesian curvature and MPC
  checks are retained, including their original tolerances.
- Preserve legacy curve preview during CANCEL and REJOIN. Require inflated
  longitudinal separation plus the existing reserve before considering a
  diagonal return. Faster passing must not force an earlier merge.
- Sampling inflation uses at least the current relative speed and also the
  future relative speed. Higher predicted acceleration cannot reduce this margin.

Defaults remain legacy until evidence warrants promotion. The earlier progressive
and early-return flags remain independent, false in both comparison arms.

The first screen `speed_profile_screen_20261105` used a per-stage lateral
speed-release ramp instead of the revised speed-proposal search and used route
progress in the new braking estimate. It is a separate version, preserved as a
negative engineering result. The revised screen is `speed_profile_v2_20261105`;
both use the same prospective schedule seed and all four positions. Runtime
hashes identify each version. Do not pool their outcomes.

The third revision `speed_profile_v3_20261105` adds this geometry/timing
separation. It also initializes the numerical worker on a five-knot straight
case before declaring ready, within the existing five-second startup bound.
The live 40 ms SQP deadline is unchanged. The same eight-run paired schedule
is repeated; neither version's failures are deleted or rescored.

## Debugging evidence, separate from the frozen comparison

`speed_profile_debug_20261105_a`, position 50: initial combined candidate failed
35 s acceptance, with seven controller rejections (six actual dynamic conflicts),
one SQP deadline miss and no scored collision/boundary event. Minimum sampled
body clearance 0.2422 m. This version applied the new braking preview to returns;
it was revised before the paired screen. It must not be reported as a pass.

`speed_profile_right_debug_20261105_a`, same position with right-only passing:
completed at 27.65 s, no collision/boundary event; minimum body gap 0.6884 m.
This is a diagnostic side restriction on the initial candidate revision, not
the automatic-side comparison or evidence of general improvement.

The full offline suite checks jerk/lag, braking-distance dependence on positive
acceleration, sufficient curve-entry braking, body/map rejection and existing
real/Gym MPC previews. An initially insufficient braking allowance failed its
curve-speed assertion; that failed check was corrected, not weakened.

## Reading results

Use independent truth-confirmed pass/rejoin time, body/boundary scores, invalid
plans after the initial 2 seconds, cancellations, controller rejections and SQP/MPC
timeouts. Do not count the controller's completion counter alone as acceptance.
The evaluator's derivative of final speed command can spike during fail-stop;
it is not the optimized acceleration input or independently measured physical
acceleration. The audit separately reports `mpc_acceleration_command_mps2`.

Raw configs, source patches, trajectories, predictions, independent English
plots and graph audits are retained in each trial directory. Compare runtime
source hashes inside the frozen screen; never merge debugging versions into a
single method success rate.

## First paired screen (retained negative result)

Both arms completed 4/4 configured single-pass/rejoin trials with zero scored
collision or ego/lead boundary events. They were not uniformly clean: both
position-50 trials cancelled once and had five invalid-plan samples after 2 s;
position 25 had 25 legacy versus 20 candidate invalid samples. SQP deadline
misses are also counted by the stricter audit. No trial had an MPC controller
failure. The candidate did not improve completion time and was not promoted.

| Position | Legacy confirmation | First candidate confirmation |
| --- | --- | --- |
| 00 | 10.45 s | 11.95 s |
| 25 | 33.90 s | 33.90 s |
| 50 | 28.65 s | 30.15 s |
| 75 | 18.90 s | 20.15 s |

One trial per cell is an engineering comparison, not a statistical effect
estimate. The revised candidate removes its lateral-release ramp in favor of
complete speed-proposal admission, and fixes physical braking arc length.

Audit: `results/speed_profile_20261105/initial_screen_audit.json`.

## Second paired screen

| Position | Legacy | Speed-proposal candidate |
| --- | --- | --- |
| 00 | 10.95 s | 9.20 s |
| 25 | Failed 35 s acceptance | Failed 35 s acceptance |
| 50 | Failed 35 s acceptance | 29.15 s |
| 75 | 19.15 s | Failed 35 s acceptance |

Both arms passed 2/4 configured cases, with zero scored collision/boundary
events. The candidate had zero controller failures; legacy position 25 had
one. Position 00's minimum sampled body clearance was 0.4564 m legacy versus
0.5956 m candidate. Neither arm met uniform clean-motion acceptance; invalid
plans, SQP deadlines and incomplete passing remained. These data support a
local speed improvement, not an overall success claim. Legacy failures also
demonstrate the wall-clock repeatability limitation of these small screens.

Replaying the first three retained `linear_corridor_infeasible` requests from
the candidate's position-75 log with curvature bound 1.35 instead of 0.63
made the numerical solves feasible. This is a diagnostic of an artificial
geometric restriction, not proof that those trajectories pass Cartesian
collision, timing or MPC checks. The third revision retains those checks.

Audit: `results/speed_profile_20261105/second_screen_audit.json`.

## Third paired screen and prospective repeat

| Position | Legacy confirmation | Revised candidate confirmation |
| --- | --- | --- |
| 00 | 10.95 s | 8.70 s |
| 25 | Failed 35 s acceptance | 9.45 s |
| 50 | 27.90 s | 28.15 s |
| 75 | Failed 35 s acceptance | 21.40 s |

The candidate completed 4/4 configured cases versus 2/4 for legacy, with zero
scored collisions and ego/lead boundary violations in all eight. Each cell is
still just one run. Position 50 retains two cancellations, five invalid-plan
samples after 2 s and one cumulative controller failure; no corresponding
error sample was retained in its CSV, so its precise cause is not asserted.
Position 75 remains slower than successful legacy runs in the earlier screens.
Thus this is not a uniform speed or clean-motion acceptance claim.

Candidate minimum sampled body gaps were 0.4834/0.3501/0.4209/0.4258 m at
00/25/50/75. Optimized acceleration commands remained inside [-3,+3] m/s²;
their largest magnitude was 2.3443 m/s². The unchanged allowed bound is not
the acceleration the optimizer is forced to request. SQP deadline misses
persist despite moving cold numerical initialization before ready.

Freeze this runtime and repeat the four candidate cases in independent fresh
processes before deciding what to recommend. Keep scene and tracker seed 1
fixed (injected measurement noise is zero), shuffle case order with seed
20261105. These test scheduling/startup repeatability, not four new noise
distributions. Retain all outcomes. No runtime edit will be made during repeats.

```bash
TAKEOVER_PREDICTIVE_SPEED_PROFILE=true TAKEOVER_PROGRESSIVE_PASS_SPEED=false \
TAKEOVER_EARLY_VERIFIED_RETURN=false python3 tools/run_position_overtake_batch.py \
  --prefix speed_profile_v3_repeat_20261105 --repeats 1 --schedule-seed 20261105
```

Audit: `results/speed_profile_20261105/third_screen_audit.json`.

## Repeat outcome and handoff

The frozen candidate completed all four additional fresh-process repeats.
Runtime source hashes match all four candidate trials in the third paired
screen. Candidate totals are therefore 8/8 configured pass/rejoin completions
over two trials at each location, with zero scored collision and ego/lead
boundary events. This small screen does not establish a population success rate.

| Position | Candidate trial 1 | Candidate repeat | Legacy in the matched third screen |
| --- | --- | --- | --- |
| 00 | 8.70 s | 8.70 s | 10.95 s |
| 25 | 9.45 s | 10.20 s | Failed 35 s acceptance |
| 50 | 28.15 s | 28.40 s | 27.90 s |
| 75 | 21.40 s | 21.65 s | Failed 35 s acceptance |

The extra repeat had no controller failure. However, across the eight candidate
trials there was one cumulative controller failure, five invalid-plan samples
after 2 s, and persistent SQP deadline misses. Only 2/8 passed the additional
strict audit requiring zero such anomalies. Location 50 still cancels/retries;
locations 50 and 75 do not demonstrate uniform speed improvement. Location
00's 8.70 versus 10.95 s is a roughly 20.5% improvement in this matched screen,
not a generalized effect-size estimate.

All eight candidate graph audits passed. Opponent truth remains restricted to
opponent control/evaluation and final `/drive` remains arbitrated. Minimum
sampled body clearance across these eight was 0.3456 m. Optimized acceleration
commands stayed inside [-3,+3] m/s². Offline C++ geometry/timing/prediction,
non-actuating real/Gym MPC previews, socket recovery and 26 Python checks passed.
No GUI-load, new dropout/braking scenario or physical trial was run in this work.

The explicit experimental entry point is:

```bash
cd /data/projects/F1-Tenth-Duke/f1tenth-takeover/Code/sim_ws
bash run_predictive_speed_trial.sh fitzpatrick_position_00 '' 1 speed_experiment_gui_01 true
```

Use `false` for headless operation and a new run ID for each run. Legacy defaults
remain available and unchanged; the wrapper selects the candidate deliberately.
The remaining engineering priority is preview/live MPC agreement at changing
entry/recovery states and bounded recovery from interrupted/infeasible spline
updates, before increasing aggressiveness further.

There were 30 completed processes across all candidate revisions, controls and
debugging trials in this work: 23 configured passes, seven configured failures,
zero scored collisions and 30 passed live graph audits. These mixed versions
are an evidence inventory, not one method's pooled performance. Every run,
including failures, is losslessly exported with compressed CSV, config, source
patch, SQP failure trace, logs and separate English plots in
`results/speed_profile_20261105/trials/`. Its `MANIFEST.json` gives file hashes.
Raw ROS subdirectory logs and adapter snapshots remain at the original source.
Audits are indexed in [the results directory](../results/speed_profile_20261105/).

Jetson deployment remains separate: the retained physical v8 has a different
wheelbase/steering/longitudinal-response model and nominal command bounds
[-0.8,+1.0] m/s², versus this Gym profile's [-3,+3]. The latter are not measured
physical-car capabilities. Recheck the correct model, interfaces and whole
planner/SQP/MPC deadlines on the Jetson before any physical dynamic passing.
No real-vehicle files or running vehicle processes were changed, and no push
was performed in this work.

Method rationale and alternatives: [planning/control tradeoffs](PLANNING_CONTROL_TRADEOFFS.md).
