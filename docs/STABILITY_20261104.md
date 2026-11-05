# Dynamic-overtaking stability work — 2026-11-04

Simulation only. No vehicle connection, motion command, acceptance relaxation,
solver deadline increase or change to the real-v8 controller.

## Planned comparison (before running the batch)

Keep ego/lead limits 3.0/1.5 m/s, command acceleration [-3,+3] m/s²,
35 simulated seconds, native Gym dynamics, LiDAR-only opponent estimation,
and existing independent collision/return scoring. Compare the existing
0.68 m speed-release threshold with an opt-in smooth speed proposal.
Both arms include the recovery search fix; neither is the frozen old commit.

Use four existing start positions (00/25/50/75), one independent fresh-process
trial per arm per position initially. Block by position/seed, randomize arm order
and block order with seed 20261104. The default schedule is eight trials. This
is an engineering screen, not a success-rate estimate or powered experiment.
Preserve failed, incomplete and timed-out attempts. Assess configured success
AND the existing stricter clean-motion audit (invalid plans after 2 seconds,
controller failures and solver timeouts). Faster completion does not override
those checks. No policy promotion merely because one case is faster.

```bash
cd Code/sim_ws
python3 tools/run_stability_comparison.py --prefix stability_screen_20261104 --plan-only
python3 tools/run_stability_comparison.py --prefix stability_screen_20261104
```

Raw results remain in `Code/sim_ws/results/takeover/<scenario>/<run_id>/`;
the schedule and aggregated original summaries are under
`Code/sim_ws/results/takeover_stability_batches/<prefix>/`.

## Changes under test

- Search all existing recovery blend lengths when the shortest REJOIN/CANCEL
  path is blocked by map/body checks, not just when curvature fails.
- If a return is rejected while ego is ahead, validate a fresh same-side
  overtaking continuation with the existing clearance reserve. If infeasible,
  cancellation/fail-stop is retained. Retry return after 0.5 s, without stopping
  per-cycle validation of the passing trajectory or freshness checks.
- Optional `TAKEOVER_PROGRESSIVE_PASS_SPEED=true`: smoothly release the speed
  proposal between physical lateral separation and the existing inflation band.
  This is a geometric proposal, not a safety certificate. Full timed collision
  checks and MPC preview remain mandatory. The default stays false pending tests.
- Preserve original curve braking preview (0.6 m/s²) and real-v8 parameters.

## Exploratory reproduction before the comparison

`stability_before_20261104_a`, position 50, original 76d3398: failed 35.08 s
trial; no collisions/boundary events; minimum sampled body gap 0.2184 m.
Curvature cancellation was followed by return conflict and target loss.

`stability_recovery_20261104_a`, same position/settings with recovery changes:
passed 35.09 s trial; independent return confirmation 28.40 s; minimum body gap
0.4424 m; no collisions/boundary events. Still had a curvature cancellation
and an invalid plan after initialization, so NOT a clean-motion pass.

These are debugging trials, not independent validation of generalized behavior.
Detailed batch outcomes will be appended after completion.

## Final recovery revision and prospective repeat check

The eight-trial speed-policy screen completed: all eight met configured pass
criteria, but only four met the strict clean-motion criterion. Position 50
still had invalid plans; position 25 had MPC failures. Progressive speed was
not uniformly faster (position 75 was slower). It therefore remains opt-in.

An additional recovery change permits an earlier return behind the opponent
only when the full candidate, existing MPC preview and 0.10 m inflated-body
clearance reserve pass; the old larger-gap fallback remains. The targeted
position-50 run `stability_early_return_20261104_a` passed at 25.15 s with no
post-initialization invalid plans or controller failures/timeouts and no
collision/boundary events (minimum sampled body gap 0.4052 m).

Before further trials, freeze this runtime revision and retain the legacy
speed release (progressive=false). Run two fresh-process repeats at each of
the four positions with the existing shuffled-block scheduler, seed 20261024,
trial seeds 1 and 2. These eight runs test repeatability of the revised recovery,
not the earlier version or a general success rate. No further runtime edits
will be mixed into this batch.

```bash
TAKEOVER_PROGRESSIVE_PASS_SPEED=false TAKEOVER_EARLY_VERIFIED_RETURN=true \
  python3 tools/run_position_overtake_batch.py \
  --prefix stability_final_20261104 --repeats 2
```

## Decision after the repeated screen

**Not complete, not deployment-approved.** The batch named `stability_final`
was a candidate validation batch, not a declaration of final acceptance.
Only 4/8 met configured single-pass acceptance and 3/8 were clean. All eight
had zero scored collisions and ego/lead boundary events. One failed run had a
recorded SQP timeout; the other failures involved infeasible lateral corridors,
curvature rejection and retries that did not complete within 35 seconds.
Do not attribute all failures to compute or claim that earlier return caused
all of them. The simulator is not wall-clock deterministic; seed labels here
do not imply injected-noise variation (measurement noise is zero).

The early-return rule is now **opt-in**, `TAKEOVER_EARLY_VERIFIED_RETURN=true`.
It was unconditional in the repeated-screen runtime snapshot; the retained
`source.patch` and source hashes identify that version. The flag was added
after the experiment, so historical `config.json` files are NOT rewritten.
Both this and progressive speed default to false. Defaults retain the bounded
recovery-length search and checked same-side continuation. Those defaults
are also still a development version, not a stable release.

| Position | Initial recovery, legacy speed | Initial recovery, progressive | Early return, repeat 1 | Early return, repeat 2 |
|---|---|---|---|---|
| 00 | 10.95 s, clean | 10.20 s, clean | 10.70 s, clean | Fail; SQP timeout/cancel |
| 25 | 33.15 s; MPC failures | 32.65 s; MPC failures | Fail; recovery/retries | 33.90 s; invalid plans |
| 50 | 29.65 s; invalid plans | 28.65 s; invalid plans | 25.65 s, clean | Fail; infeasible corridors |
| 75 | 18.65 s, clean | 19.40 s, clean | 18.65 s, clean | Fail; infeasible corridors |

Times are the independent evaluator's pass-and-rejoin confirmation, not the
controller's internal completion counter. Each cell is one trial, not a
population estimate. Within each batch all runtime source hashes match;
the two batches have different code and must not be pooled as one method.
See [archived audits](../results/stability_20261104/).

## Tests and remaining work

The complete offline suite passes, including new recovery/body-conflict,
same-side continuation, early-return rejection, default-off behavior and
monotone speed-release checks. The suite also includes 25 Python tests and
the original C++ geometry/prediction/socket/non-actuating real/Gym MPC checks.
No new physical or full static-v8 ROS trial was performed.

Remaining priorities:

1. Whole-maneuver admission and repeated feasibility: an initially admitted
   passing corridor may later become infeasible before safe rejoin.
2. SQP interruption recovery: a bounded previously computed trajectory may be
   reused only after time-age, fresh-body/map and MPC revalidation. Never hold
   the last command or enlarge solver deadlines just to pass.
3. Recovery preview versus live MPC agreement, particularly position 25.
4. Repeat unchanged acceptance under GUI load and lead braking/lateral shifts,
   occlusion, noise, and insufficient space. Then reassess vehicle deployment.

The existing 0.6 m/s² curve braking preview was deliberately not retuned in
this work; ±3 m/s² remains a simulation command limit, not measured vehicle
performance. No MPPI, neural control or new opponent ground-truth input.

## Current default smoke test and final handoff

After making both experimental switches default-off, two fresh seed-2 trials
were run on the actual handoff source (`stability_default_20261104_a`):

- Position 25: failed to finish in 35 s; 15 invalid-planner evaluation samples
  after 2 s, zero controller failures. No collision or boundary events.
- Position 50: completed at 29.15 s; minimum sampled body gap 0.4233 m;
  five invalid-planner samples after 2 s, zero controller failures. No
  collision or boundary events, but not a clean-motion pass.

Thus even the conservative default is NOT yet repeatably accepted. Switching
off the experiments is not claimed to solve the underlying corridor issue.
The first 8/8 screen must not be used to hide these subsequent failures.

There were 22 completed trial processes in this work across the baseline,
debugging versions, two screens and final smoke checks. All had zero scored
collisions/ego/lead boundary events and all 22 live graph audits passed:
LiDAR measurement source; opponent truth subscribers only opponent controller
and evaluator; final `/drive` publisher only safety arbiter. Mixed-version
totals are an evidence inventory, not a pooled method success rate.
`all_trials_audit.json` retains every result; `default_smoke_audit.json` is
the latest source check. All simulation processes were stopped after tests.

Current GUI reproduction (choose a new run ID if it already exists):

```bash
cd /data/projects/F1-Tenth-Duke/f1tenth-takeover/Code/sim_ws
TAKEOVER_PLANNING_BACKEND=upstream_sqp \
  bash run_takeover_trial.sh fitzpatrick_position_50 '' 2 recovery_gui_01 true
```

No push or real-vehicle operation was performed in this work.
