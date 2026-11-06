# Checked MPC execution and control-feedback recovery — 2026-11-06

Simulation only. Physical-v8 source/model/bounds are not changed. P0/P1 work,
not asynchronous planning (P2) or full-maneuver admission (P3).

## Failure and prospective validation

User GUI run `fitzpatrick_position_50/gui_20261106_015712`: at 10.35 s ego was
1.145 m ahead by independent truth. A curvature-rejected passing corridor
triggered a newly checked REJOIN, but actual MPC dynamic-body validation
rejected it and zero/reset lost the passing speed. Completion was 25.40 s.
This truth gap is diagnostic/evaluation only, never planner input.

Before changes, fresh process `control_recovery_before_20261106` completed
at 25.15 s but stopped after 2 s on `mpc_center_map_clearance`. This is a
different planning failure; do not claim that all position-50 stops have the
same cause. Initial candidate `control_recovery_diag_20261106_a` completed
13.20 s, zero collision/boundary/control rejection, minimum sampled body gap
0.5684 m. The recovery branch was NOT exercised by that diagnostic.

Question: does checked execution reduce rejection-induced stops without
weakening the unchanged map/body, freshness, solver residual or compute checks?
Freeze runtime after diagnostics; compare flag OFF/ON with geometry, timed
prediction and verified exit ON in both arms. Four matched location blocks,
35 s native Gym, 3.0/1.5 m/s caps, +/-3 m/s² commands. Randomize block/arm
order, seed 20261106; fresh ROS/Gym process is the unit. Zero injected noise
means these are scheduling/process repeats, not independent noise draws.
Keep failures, report stop/retry events as well as final completion. This is
an exploratory engineering screen, not a population success-rate estimate.

```bash
cd Code/sim_ws
python3 tools/run_stability_comparison.py --control-recovery-comparison \
  --schedule-seed 20261106 --prefix control_recovery_screen_20261106
```

## Implementation contract

- Shared reference construction validates plan/control stamps and requires a
  remaining 1.2 s horizon. Timed preview/live execution both use the fixed
  50 ms shooting/command interval. This does not fix synchronous planning lag.
- Snapshot accepted physical command/servo/observer state before a live solve.
  On rejection roll it back. One cold re-solve is allowed for a dynamic-body
  veto, solver status 4 or nonlinear infeasibility, with unchanged full checks.
  No rejected command is published and no rejected actuator state is retained.
- If that fails, a fresh, longitudinally body-separated ego may request a NEW
  current-offset continuation, retaining the locked side. Require full timed
  body/map checks, 0.10 m inflated clearance reserve and MPC preview, then the
  SAME live validator. Hold/retry-return interval 0.5 s, revalidate every plan.
  Adjacent/behind vehicles, stale targets, map conflicts, deadline failures,
  absent/vetoing preview do not gain permission through this branch.
- A bounded solve/recovery attempt has a 45 ms execution budget (planning time
  is separately measured). If it fails, retain zero/reset fail-stop. Never hold
  the previous drive command or execute an unchecked cached path.
- Log initial rejections, cold recoveries and continuation attempts/successes
  separately from unrecovered controller failures. Event-only JSONL preserves
  rejected diagnostics. Independent scoring and opponent-truth isolation stay
  unchanged. Generic runner defaults OFF; explicit fast wrapper defaults ON.

Non-actuating regressions cover transactional rollback, cold retry validation,
clock/horizon expiry, preview veto/missing, adjacent/stale target, map block,
locked side and no speculative completed-pass counter. Both v8 and native Gym
solver variants are checked. Broader phase recovery and P2/P3 remain pending.

## Resumed validation — 2026-11-07

The 20261106 screen was interrupted after the position-25 pair (both 8.20 s).
Its position-50 candidate has only 20.03 s recorded, although its evaluator
reports a completed pass at 13.20 s. It is NOT a complete 35 s trial and is not
counted as a campaign pass. All partial files remain at their original paths.
No runtime-source changes were made before resuming. A new complete randomized
screen uses the same design/seed with prefix `control_recovery_screen_20261107`.
Do not pool the interrupted batch or diagnostic with this complete screen.

The user requested a visible GUI test between the position-25 and position-50
blocks. Only the batch supervisor was paused; the active trial finished before
the GUI acquired the exclusive simulation lock. The supervisor was resumed
after GUI teardown. GUI result `control_recovery_gui_20261107_a`: 35.20 s
recorded, pass/rejoin confirmed at 13.45 s, minimum sampled body gap 0.5688 m,
zero collision/ego or lead boundary events, zero post-2 s stop or invalid-plan
samples, zero CANCEL samples and zero controller failures. Two return-preview
dynamic vetoes were recorded; neither became a live control rejection. Cold
retry and same-side control recovery were NOT exercised. The RViz window was
confirmed viewable; this run is separate from the headless matched screen.

## Complete frozen screen outcome

All eight trials recorded at least 35 simulation seconds and passed the
unchanged independent complete-pass and collision/boundary checks. Runtime
source hashes match across all eight, graph audits passed (LiDAR-only tracker,
opponent truth consumers restricted to opponent controller/evaluator, final
drive publisher restricted to safety arbiter). No real vehicle was contacted.

| Start | Feedback OFF completion [s] | Feedback ON completion [s] | OFF stops after 2 s | ON stops after 2 s |
|---|---:|---:|---:|---:|
| 00 | 7.20 | 6.95 | 0 | 0 |
| 25 | 8.20 | 8.20 | 0 | 0 |
| 50 | 26.40 | 12.95 | 2 | 0 |
| 75 | 16.90 | 17.40 | 0 | 0 |

All four ON trials also had zero CANCEL entries, invalid-plan samples after
2 s, live controller failures and solver/SQP timeouts. Minimum sampled body
clearance across these four was 0.47845 m; MPC P95 was 1.77–1.97 ms and planner
P95 21.38–74.69 ms. This does not establish hard real-time bounds on Jetson.
OFF position 50 had a solver-4 fail-stop at 6.30 s, a curvature stop at 13.70 s,
one cancellation and 15 invalid-plan samples after 2 s. Retain its failed clean-
motion outcome even though final pass/rejoin succeeded. OFF other starts were
clean. Position 75 ON was 0.50 s slower, so no uniform speedup is claimed.

No ON nominal trial triggered a live control rejection, cold retry or same-side
feedback continuation. Consequently this screen supports clean nominal motion
of the combined fixed-step/checked-execution candidate, NOT a measured success
rate for recovery from live dynamic-body vetoes. Those guards are covered by
non-actuating synthetic tests; more end-to-end fault/perturbation validation is
still required. Scheduling variation and fixed-step changes are not isolated
causally by this small ON/OFF screen. The observed 50 improvement is encouraging,
not proof that its historical failure can never recur.

Machine-readable audit: [screen_audit.json](../results/control_recovery_20261107/screen_audit.json).
Raw trajectories, predictions, separate English figures, source patches and
all interrupted/diagnostic outcomes are retained in the corresponding archive.

Static regression follow-up: `v8_clear` is clean. `v8_static` is NOT CLEAN:
it stopped before the obstacle on `all_candidates_blocked`, despite the old
safety-only evaluator returning `passed=true`. This limitation is reported,
not hidden or fixed by relaxing map/footprint checks. See the
[complete report and strict audit](../results/control_recovery_20261107/REPORT.md).
The full offline suite (including both solver variants and 32 Python checks)
passed. Full static-regression acceptance remains pending.
