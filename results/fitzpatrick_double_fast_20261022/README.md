# Fitzpatrick higher-speed double-overtake tests — 2026-10-22

## Verdict

**Two-pass task acceptance: 6/6. Strict uninterrupted-motion acceptance: 3/6.**
This is a simulation result, not real-vehicle approval or a claim of general
dynamic-obstacle robustness. The lateral-drift scenario still needs hardening.

Ego speed limit was increased from **1.5 to 1.8 m/s**, and opponent target speed
from **0.8 to 1.0 m/s**. Both measured maxima reached these settings. Old
scenarios were preserved. Ego still slows for curvature, entry and recovery;
these settings do not prescribe constant ego speed.

Each 110 s trial used one continuously moving opponent on the closed Fitzpatrick
route. Ego passed and rejoined, continued around the loop, and passed the same
opponent again. There was no reset/teleport between encounters. The independent
evaluator confirmed two distinct lap-relative pass-and-rejoin events per run.
Cancellation/recovery/retry is allowed within this task; completion is not a
claim that each overtaking attempt succeeds on its first try.

## Frozen-source repeated results

Run prefix: `fast_progress_20261022`; seeds 1, 2, 3 for each scenario. All six
completed trials used the same source-patch SHA-256:
`8570379cef8895a06ee3484f8e4b32ec7dd071d2f0dca470f155782854b7af77`.
Base commit: `cceddf01a2a12b58701437f5b179420a8595896c`.
The uncommitted source patch is preserved in each raw trial directory.

| Lead motion | Seed | Verified rejoin times (s) | Minimum body gap (m) | Post-rejoin route RMSE (m) | Invalid planner samples after 2 s | MPC failure events | Strict clean |
|---|---:|---|---:|---:|---:|---:|---|
| Constant 1.0 m/s | 1 | 12.70, 82.20 | 0.599 | 0.0697 | 0 | 0 | Yes |
| Constant 1.0 m/s | 2 | 12.44, 82.14 | 0.597 | 0.0716 | 0 | 0 | Yes |
| Constant 1.0 m/s | 3 | 12.39, 81.89 | 0.605 | 0.0697 | 0 | 0 | Yes |
| 1.0 m/s + lateral drift | 1 | 13.44, 95.99 | 0.326 | 0.0682 | 8 | 0 | No |
| 1.0 m/s + lateral drift | 2 | 13.34, 87.64 | 0.461 | 0.0713 | 0 | 1 | No |
| 1.0 m/s + lateral drift | 3 | 13.34, 98.09 | 0.324 | 0.0724 | 17 | 0 | No |

All six: zero measured body collisions, zero ego/lead boundary violation events,
zero solver timeouts. Across runs, planner P95 was **1.48–1.69 ms** and MPC P95
**1.35–1.60 ms** on this laptop. These are observed timing statistics, not
worst-case execution bounds or Jetson timings.

Lateral drift is a smooth ±0.22 m, two-period spatial sine applied ONLY to the
opponent's private reference. Its controller physically follows that path in
Gym. This is not a sudden/adversarial cut-in. No measurement noise or artificial
dropout was injected in this batch. Seeds fix scenario conditions, but ROS
wall-clock scheduling means repeats are not bit-identical trajectories.

The strict audit remains **failed**; its criteria were not weakened:

- Lateral seeds 1/3: 8/17 invalid 20 Hz evaluation samples after startup,
  predominantly near the outer map boundary during a cancelled maneuver.
  Protection stopped motion temporarily, then recovery completed the second pass.
  These samples are not 8/17 independent planning-failure events.
- Lateral seed 2: one live controller rejection for
  `mpc_actual_dynamic_conflict`, observed in two evaluator samples. This denotes
  an unsafe predicted motion that was rejected, not an actual collision.
- Return preview rejection, planner failure, live MPC rejection and actual
  collision are separate measurements. Successful completion does not erase any.

`final_audit.json` contains the full six-run strict audit. Post-rejoin RMSE uses
only CRUISE samples after the first completion, not the deliberate passing
offset. Finite-difference command acceleration can spike during fail-stop;
it must not be interpreted as measured physical acceleration.

## Implementation and pre-fix evidence

The first increased-speed probes used unchanged planning logic. Constant lead
completed only one pass; lateral drift completed two but had live controller
failures. Those runs remain under `fast_probe_20261022`; see
`pre_fix_audit.json`. No failed records were removed.

Two local changes were then made in adaptive mode only:

1. Use the same capped-clearance/progress/offset/curvature ranking at both offset
   search and left/right selection. Do not reward unlimited surplus wall space
   on an outer bend at the expense of route progress. Hard checks precede scoring.
2. During an aborted maneuver's return, cap the timed speed forecast at the
   observed lead speed (with the existing 0.70 m/s running floor). Do not resume
   gap-closing acceleration until aligned. The MPC receives this time-indexed
   reference; its optimized command is not overwritten afterward.

Constant-lead runs still sometimes cancel an outer-side attempt, return behind
the lead and retry on the other side after rejoining. Side switching during a
committed maneuver remains prohibited. This is successful recovery, not proof
of globally optimal initial side selection.

Data flow remains:

`LiDAR -> EKF track -> route-assisted motion prediction -> behavior + timed
Frenet (s(t), d(t)) -> dynamic MPC (acceleration + steering) -> safety arbiter`.

Adaptive offsets, C2 return blending and a separate non-actuating MPC return
preview are documented in `docs/ADAPTIVE_DOUBLE_OVERTAKE.md`. No tracker or MPC
model replacement, larger observation timeout, looser collision threshold,
ground-truth feed into ego tracking, or real-vehicle command was introduced.

The live graph audit passed during lateral seed 1: opponent truth subscribers
were only the opponent controller and evaluator; tracker source was `lidar`;
only the safety arbiter published `/drive`. See `graph_audit.json`.

## Reproduce and view

From the repository root:

```bash
cd Code/sim_ws
bash tools/test_takeover_offline.sh
# One headless, higher-speed continuous two-pass trial
./run_takeover_trial.sh fitzpatrick_double_fast '' 1 '' false
# GUI: the same isolated scenario, with RViz
./run_takeover_trial.sh fitzpatrick_double_fast '' 1 '' true
# Lateral-drift stress case (known brief protective interventions)
./run_takeover_trial.sh fitzpatrick_double_fast_lateral '' 1 '' true
# Six sequential trials; use a fresh prefix, never overwrite old runs
bash run_double_overtake_batch.sh 3 my_fast_repeat fast
```

The batch command exits nonzero when its stricter clean-motion audit fails,
even if all six task-level flags are true. Do not suppress this distinction.
Domain 73, localhost-only DDS and a process lock isolate the simulation and
prevent overlapping launches. Thread-limited headless trials were run serially;
no GUI, physical vehicle, drive device or system power setting was touched.

Raw data directory pattern:

`Code/sim_ws/results/takeover/<scenario>/fast_progress_20261022_seed<1|2|3>/`

Each directory includes `config.json`, `source.patch`, `summary.json`,
`trajectory.csv`, `predictions.csv`, build/launch logs, `trajectory_en.png`,
and separate `prediction*_en.png` figures. Predictions are explicitly labelled
model forecasts, not measured truth. OVERTAKE entry numbers may exceed the
number of completed passes because cancelled attempts remain visible.

Actual trajectories are plotted without smoothing; black squares show
independent truth-confirmed rejoin times. Figures use equal spatial scale,
English metre-labelled axes and redundant line styles. Rendered trajectory and
prediction outputs were inspected using the scientific-visualization workflow.
They are development figures, not a journal-format/accessibility certification.

## Remaining work

- Prevent outer-boundary dead ends before commitment, accounting for lateral
  drift and recoverable return space rather than only a constant-offset preview.
- Extend planner/control executable-footprint consistency checks to entry and
  sustained passing; the extra non-actuating preview currently checks returns.
- Repeat low-speed adaptive cases after these changes; historical 1.5/0.8 m/s
  successes/failures are not claimed as results for the current source.
- Add abrupt cut-ins, prediction errors, sensor loss and more initial conditions
  before calling dynamic-obstacle simulation generally complete.
- Recheck Jetson timing, localization error, actuator response and final-command
  fail-stop before considering any real-vehicle transfer.

## Existing-mode regression checks

Five additional fresh runs used run ID `fast_regression_20261022`; raw directories
follow the same pattern as above. See `regression_audit.json`.

| Scenario | Observed result | Strict clean-motion audit |
|---|---|---|
| `v8_clear` (12 s) | Travelled 15.34 m; no collision, boundary event or MPC failure | Pass |
| `fitzpatrick_clear` (40 s) | Completed approximately one 44.60 m loop; no collision or MPC failure | Pass |
| `fitzpatrick_1p5` (40 s) | Original fixed-offset case completed one pass; no collision or MPC failure | Pass |
| `no_space` (20 s) | No completed pass; cancellation/waiting with zero collisions; four controller rejections, 99 invalid planner samples after startup | Fail, protective interventions retained |
| `v8_static` (14 s) | Travelled 5.48 m and stopped at (11.73, 3.00), reason `all_candidates_blocked`; zero collisions but **did not bypass the obstacle** | Fail |

Important evaluator limitation: the legacy `v8_static` task-level `passed` flag
checks safety only and does **not** require progress beyond the obstacle.
Its `passed=true` is therefore NOT static-bypass acceptance. The stricter audit
and manual trajectory inspection expose this discrepancy. Original raw flags
were not rewritten. This regression does not establish whether the behavior
was already present before the adaptive changes; no successful static bypass
is claimed from this run. The live static planning/model code was not replaced.

Offline verification also passed: C++ candidate/footprint checks, prediction/
timing/recovery checks, separate-MPC-state isolation checks, and all 10 Python
geometry/evaluator tests. Shell syntax, Python compilation for the changed
report utility, and `git diff --check` passed. Tests did not access hardware.

Further static-regression work must add an explicit progress/bypass condition
and resolve the stopped case before calling the complete regression suite green.

The earlier development runs, including unsuccessful ones, remain in the local
results tree; earlier audit summaries are in `results/fitzpatrick_double_20261021`.
