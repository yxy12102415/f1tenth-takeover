# Fitzpatrick native-Gym high-speed overtaking — 2026-10-23

## Final status of this implementation session

**Not fully accepted.** The constant-lead two-pass criterion passed in 2/3
fresh-process trials with identical runtime source hashes. All three had zero
scored collisions/boundaries, but all recorded MPC safety rejections; none
passed the stricter clean-motion audit. Weave two-pass acceptance remains unmet.
No overall completion, robust success-rate estimate or hardware approval is claimed.

| Current implementation run | Truth passes | Body / boundary events | MPC rejections | Minimum sampled body gap | Two-pass behavior/safety |
| --- | ---: | ---: | ---: | ---: | --- |
| `sqp_clock_pair_20261023_seed1` (constant) | 2 | 0 / 0 | 2 | 0.2216 m | Pass |
| `sqp_clock_repeat_20261023_seed1` (constant) | 2 | 0 / 0 | 4 | 0.2219 m | Pass |
| `sqp_clock_repeat_20261023_seed2` (constant) | 1 | 0 / 0 | 7 | 0.2255 m | Fail |
| `sqp_clock_pair_20261023_seed1` (weave) | 1 | 0 / 0 | 0 | 0.4755 m | Fail |

All four trials completed at least 180 simulation seconds. The three constant
runs reached approximately 3.0/1.8 m/s ego/lead maxima, with curve slowing.
See [the three-process audit](constant_three_process_audit.json) and
[the full clock pair audit](clock_pair_audit.json). Seed labels are repeats,
not independent random environments; the raw fixed-seed/noise settings remain
in each configuration. Failed development cohorts below are not pooled here.

The current empty-route native-Gym regression, `sqp_clock_clear_20261023`, also
completed its 40-second simulation duration and passed route-progress acceptance
with zero controller rejections/collisions/boundaries. This is **not** a new
complete static-obstacle/v8 regression or an endpoint-stop assertion.

Actual trajectories and model forecasts remain separate English PNGs in each
raw run directory, with equal spatial scales and truth-verified rejoin markers.
Figures were visually checked; they are engineering diagnostics, not claimed
publisher-compliant figures. Raw CSVs and failed trials remain intact.

Next priorities are whole-maneuver admission (not just a short safe entry),
executable cancellation/rejoin while alongside, and avoiding sudden fail-stop
command steps. MPC predicts motion and rejects dynamic conflicts, but obstacle
avoidance is not a hard dynamic-obstacle constraint inside its optimizer.
The retimed reference is tracked approximately, not guaranteed exactly.
Do not relax collision thresholds to eliminate these rejections. Complete
weave/acceleration/braking/dropout/noise and static-obstacle regression before
calling the overall simulation finished; Jetson scheduling/interfaces and
physical safety gates remain separate future work.

## Scope and acceptance

Simulation only: official Gym plant, Fitzpatrick map/reference route, ego/lead
maximum speeds 3.0/1.8 m/s with curvature-dependent slowing. The shared C++
controller jointly optimizes longitudinal acceleration and steering. A separate
generated Gym profile matches the 0.3302 m wheelbase and native 0.4189 rad
steering limit; the original real-v8 profile/defaults remain separate.

Acceptance requires two independently truth-scored pass-and-rejoin events in
one uninterrupted 180-second trial, with zero scored body collisions and map
boundary violations. The stricter clean-motion audit also exposes MPC
rejections, solver/SQP timeouts and invalid-planner evaluation samples after
two seconds. Failed and incomplete trials are retained, never overwritten.

This is an adaptation of the pinned Predictive Spliner numerical SQP routines,
not a reproduction of its full GP prediction/ROS 1 stack. See
[adapter scope and license caveat](../../docs/UPSTREAM_SQP_ADAPTER.md).

## Completed native-profile development pair

`sqp_native_pair_20261023_seed1`, two separate approximately 182-second trials:

| Opponent | Truth passes | Body collisions / ego boundary events | MPC rejections | Minimum body gap |
| --- | ---: | ---: | ---: | ---: |
| Constant-speed profile | 1 | 0 / 0 | 7 | 0.1685 m |
| Lateral weave, 0.22 m amplitude | 1 | 0 / 0 | 1 | 0.4891 m |

**Both failed two-pass acceptance.** The nominal speed maxima were reached;
native steering alone does not solve the second encounter. The constant case
repeatedly entered a corridor whose bend-dependent progress was insufficient,
then canceled when future map/curvature checks rejected continued overtaking.
Stopping while alongside also produced conservative predicted-body conflicts.

The next implementation revision adds a bounded static-map progress budget:
use the offset corridor's curvature speed and arc-length metric to update the
estimated passing distance, reserving ten percent progress for execution.
Reject insufficient/unsettled budgets rather than committing under a constant
3 m/s assumption. This is admission lookahead, not an extension of the target's
four-second trusted prediction window or a proof of safety.

The subsequent `sqp_budget_pair_20261023_seed1` pair also failed with one truth
pass in each case. Both retained zero scored body collisions/boundary events;
the constant case recorded seven controller rejections and the weave case
twelve. Rejection-reason tallies in the original summaries are 20 Hz sampled
statuses, not independent solver calls, and need not sum to those counters.
A maximum-speed/curvature budget alone is insufficient. See the
machine-readable `budget_pair_failed_audit.json` for the complete counters.

The next revision considers a gradual return after physical longitudinal
separation, without first demanding an extra axis-aligned inflated separation.
It still requires a valid full four-second inflated-body return, 0.10 m
predicted clearance reserve and a separate MPC preview. High-speed following
also gains lead-speed headway, closing-response and relative-braking allowance
in both current and future gap calculations. These changes are being tested
under `sqp_rejoin_pair_20261023`; no two-pass acceptance is claimed until the
full independent summaries support it.

That rejoin revision also failed: the constant case's internal controller
counter reached two, but independent scoring counted only one safe pass and
recorded one collision. The weave case recorded one truth pass without a
scored collision. Both remain failed evidence. In the constant CSV, physical
states held between approximately 98.6 and 100.1 wall seconds while prediction
age advanced, followed by valid tracker observations rejected as jumps. The
ego stopped and the lead later returned into it. This revealed that fixed-step
Gym physics and wall-clock ROS timestamps were not a coherent time base under
host scheduling stalls.

Native-profile trials now opt into physics-step ROS `/clock`, with 50 Hz fresh
sensor states instead of 250 Hz duplicate observations. All relevant nodes
use simulation time, including the C++ control timer; a steady wall driver
bootstraps zero-input physics. The external supervisor waits for the requested
simulation duration with a bounded wall budget. Solver timings/deadlines stay
on the wall clock. `sqp_clock_smoke_20261023` passed only the 35-second,
single-pass diagnostic criterion (minimum sampled body gap 0.4769 m, zero MPC
rejections/collision/boundary events). Full `sqp_clock_pair_20261023` trials
are required before two-pass acceptance can be claimed.

The completed clock pair produced two truth passes for the constant profile
(at 11.45 and 139.70 simulation seconds), zero scored collisions/boundaries,
minimum body gap 0.2216 m, and two MPC dynamic-conflict rejections. This meets
the configured behavior/safety criterion but **not** clean-motion acceptance.
The weave profile completed one pass, with zero scored collisions/boundaries
or controller rejections and minimum gap 0.4755 m; it failed two-pass acceptance.
Both clock runs had zero rejected tracker jumps and zero solver/SQP timeouts.
The constant CSV's maximum sample-time gap was 0.06 simulation seconds.

The fixed-point minimum-speed budget was then removed: it did not prevent
failed commitments and needlessly rejected feasible progress windows. The next
`sqp_exit_pair_20261023` revision retains map/steering checks and all same-time
collision thresholds, but considers a gradual diagonal return earlier when
the full return/clearance/MPC checks pass AND terminal predicted separation
exceeds 1.20 m. Results must still be independently scored; an internal pass
counter alone is never acceptance.

That exit revision failed both 180-second cases with one truth pass each.
The subsequent extended-window revision completed its constant case with zero
truth passes and 15 controller rejections (zero scored collisions/boundaries).
The batch was stopped after that full case; its weave case was **not run**.
Both experimental changes were rolled back. The planner source was restored
byte-for-byte to the completed clock pair's snapshot, SHA-256
`05fec99955e90d08bb46e0e27a71f40f770ab138231951dca61c54477a107785`.
Its existing static progress budget and physical-separation return trigger are
therefore retained, along with all clock/sensor/controller fixes. New constant
repeats use prefix `sqp_clock_repeat_20261023`; they are not mixed with the
failed exit/window revisions.

## Changed implementation areas

- Shared `joint_control.hpp` and solver generator: explicit simulation speed
  cap, separate native-Gym model/limits, measured yaw feedback and isolated
  non-actuating preview. Original real-v8 defaults/generated sources retained.
- `dynamic_overtaking.hpp`: continuous route interpolation, timed actuator/jerk
  retiming, committed-side SQP adapter, map/steering admission, full return
  validation and high-speed following/recovery.
- Tracker and opponent controller: state-aware side-panel sensing and opt-in
  opponent curvature speed governor. No opponent-truth input to ego.
- Gym bridge/launch: opt-in physics clock and fresh-state publication; ROS-clock
  C++ control timer. Existing wall-clock mode remains available.
- Trial/batch runners and audit/plot tools: immutable run directories, runtime
  source hashes, graph isolation audit, bounded simulation-time supervision,
  raw results and separate English figures.
- Offline checks: body scoring, prediction/retiming/recovery, preview state
  isolation, generated steering limits, synthetic side scans, physics-clock
  startup/rollover, SQP numerical/Jacobian and socket-timeout behavior.

## Reproduction and evidence

From `Code/sim_ws`, use a NEW run ID/prefix:

```bash
bash tools/test_takeover_offline.sh
TAKEOVER_PLANNING_BACKEND=upstream_sqp bash run_takeover_trial.sh fitzpatrick_double_3p0_1p8 '' 1 NEW_RUN false
TAKEOVER_PLANNING_BACKEND=upstream_sqp bash run_takeover_trial.sh fitzpatrick_double_3p0_1p8_lateral '' 1 NEW_WEAVE false
TAKEOVER_PLANNING_BACKEND=upstream_sqp bash run_double_overtake_batch.sh 3 NEW_BATCH 3p0_1p8
```

Set the final single-trial argument to `true` for GUI viewing. Do not run GUI
and batch processes concurrently. No SSH, real vehicle driver or actuator is
involved. The runner uses a localhost-only ROS domain and a single-trial lock.

Raw per-trial evidence is in
`Code/sim_ws/results/takeover/<scenario>/<run_id>/`: trajectory/prediction CSVs,
summary, configuration, source patch/hashes, live graph audit, solver failure
records and separate English trajectory/forecast figures. Opponent truth is
subscribed to only by its own controller and independent scoring, not ego
tracking/planning/MPC. The tracker uses LiDAR.

Repeated seed labels identify fresh process/wall-clock runs. Gym's default
random seed remains 12345 and injected measurement noise is zero in these two
profiles. These are repeatability checks, not independent randomized
environments or a statistically powered generalization study.

Body/map scoring is sampled at 20 Hz. Planner/MPC checks include dimensions,
uncertainty and inter-sample allowances, but zero sampled collisions is not a
continuous-time safety certificate. Gym contact flags are not independently
recorded in this version. Synchronous multiple SQP proposals can exceed one
50 ms control period; no Jetson real-time or hardware approval is implied.
