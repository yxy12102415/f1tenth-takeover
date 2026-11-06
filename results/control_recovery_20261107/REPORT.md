# Checked control / passing recovery validation

Date: 2026-11-07 (client timezone Asia/Shanghai). Simulation only.

## Result

The frozen four-position OFF/ON screen ran eight complete trials. ON completed
4/4 with no stop or invalid-plan samples after 2 s, cancellation entries, live
controller failures, collision or ego/lead boundary events. OFF completed 4/4
but only 3/4 were clean: position 50 stopped twice and cancelled once.
These small fixed-scenario process repeats are not population success rates.

| Position | OFF complete pass/rejoin [s] | ON complete pass/rejoin [s] |
|---|---:|---:|
| 00 | 7.20 | 6.95 |
| 25 | 8.20 | 8.20 |
| 50 | 26.40 | 12.95 |
| 75 | 16.90 | 17.40 |

The separate user-visible GUI position-50 test recorded 35.20 s and completed
at 13.45 s, zero stops/invalid plans after 2 s, cancellations, live controller
failures or scored collisions/boundary violations. Sampled minimum body gap
0.56880 m. Two candidate return previews were dynamically vetoed, correctly
not published as actual control. Live control recovery was not invoked.

## Scope and limitations

Physical-v8 source/model unchanged. No real vehicle connection/actuation.
The new opt-in simulator adapter unifies the 50 ms preview/command interval,
validates timed-reference age/horizon, rolls back rejected actuator runtime,
allows one fully checked cold re-solve, and proposes a separately checked
same-side continuation only for an already longitudinally separated ego.
Map/freshness/deadline checks and zero/reset fail-stop remain active.

No ON nominal run naturally triggered the cold/same-side recovery branch.
Synthetic offline tests verify rollback, veto and stale/adjacent/map guards;
end-to-end fault/perturbation coverage is pending. Position 75 was slower.
Planning is still synchronous (P95 up to 74.69 ms in the ON screen), not an
asynchronous P2 implementation or a full-maneuver P3 feasibility certificate.
This is not full simulation acceptance or Jetson deployment approval.

The 20261106 campaign was interrupted: only its 25 pair completed and its
50 trial recorded 20.03 s. The latter is excluded from complete-trial claims
even though the raw evaluator's `passed` field is true. Original files are
preserved; diagnostics and interrupted runs are not pooled with the new screen.

## Static regression warning and offline tests

`v8_clear` recorded 12.15 s, no collision/boundary/controller failure and no
post-2 s stop samples. `v8_static` recorded 14.09 s, no collision/boundary or
controller failure, BUT stopped at about 5.49 s on `all_candidates_blocked`
near (11.79, 3.00), with 172 post-2 s stop/invalid-plan samples. Its raw
`passed=true` is insufficient: the old evaluator falls through to safety-only
acceptance for `v8_static_avoidance_regression`, so stopping before an obstacle
can pass without actually avoiding it. Mark this regression NOT CLEAN/PENDING;
do not claim full static avoidance preservation from this test. The new flag
is OFF in these regressions; the unchanged physical planner is not retuned to
make this test pass. The independent cause/baseline needs further inspection.

Full offline suite passed: footprint/candidate and prediction/recovery C++
checks, both physical-v8/native-Gym non-actuating preview/rollback solvers,
socket late-result recovery, and 32 Python checks. These are not physical
tests or end-to-end natural recovery events.

[Validation summary](validation_summary.json) distinguishes raw evaluator
acceptance from strict clean checks. [Archive manifest](trials/MANIFEST.json)
contains 16 runs including diagnostics and the partial trial, with lossless
CSV compression and adapter-source snapshots. The [prior user GUI failure](prior_gui_failure/MANIFEST.json)
is retained separately. All archived file SHA256 hashes were verified.

## Reproduce

```bash
cd Code/sim_ws
# Explicit development GUI wrapper; use a fresh ID each time.
bash run_fast_overtake_trial.sh fitzpatrick_position_50 35 1 \
  "gui_$(date +%Y%m%d_%H%M%S)" true
# New complete eight-run headless screen; never overwrite prior trials.
python3 tools/run_stability_comparison.py --control-recovery-comparison \
  --schedule-seed 20261106 --prefix control_recovery_new_screen
```

See [implementation and design](../../docs/CONTROL_RECOVERY_20261106.md),
[machine-readable full audit](screen_audit.json), and the losslessly compressed
raw trial archive plus SHA256 manifest. Scoring and safety thresholds were not
relaxed. Matched position/seed blocks and randomized order follow the
experimental-design workflow; individual CSV rows are not independent runs.
