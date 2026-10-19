# Track-assisted timed MPC: progress snapshot

Prepared 2026-10-20; simulation trials below ran on 2026-10-18.
**Status: work in progress, not full simulation acceptance or vehicle approval.**

## Implemented

- Existing map-assisted LiDAR clustering and Cartesian CV Kalman tracking retained.
- Separate route-assisted forecast with Cartesian CV fallback for off-route motion.
- Side-committed overtaking, cancellation and feasible return trajectories.
- Timed Frenet position/speed references passed to the retained v8 dynamic MPC,
  with plan-age compensation and validation of its own predicted motion.
- Rejoin completion evaluated before target expiry; control-clock expiry requests
  immediate replanning instead of waiting for the next 5 Hz planning tick.
- Return entry based on body separation plus a feasible return candidate.
  Completion thresholds, collision inflation and target age limits were not relaxed.
- Optional candidate-side restriction for repeatable left/right regressions;
  default remains automatic. Safety arbiter and truth isolation remain in place.

## Actual evidence (do not pool code versions)

| Code stage / runs | Result | Limitation |
|---|---|---|
| `timed_verified_20261018`, 12 completed trials | Clear-route 3/3, 2.0 m/s overtaking 3/3, 1.5 m/s overtaking 3/3; all nine without MPC failure | Earlier code, before latest boundary fixes |
| Same campaign, dropout/noise seeds 1–3 | Configured behavioral/safety pass 2/3; seed 1 had one MPC expiry fail-stop, seed 3 had one collision and two MPC expiry failures | Failed evidence preserved; batch stopped before follow tests |
| `timed_boundaryfix_20261018_seed3` | Full automatic-side dropout/noise trial passed without collision or MPC failure | One trial, before adding the side-restriction test interface |
| `timed_rightfix_20261018_seed3` | Full right-side dropout/noise trial passed; no collision, boundary violation, controller failure, retry or timeout | Only one trial on the latest controller code |

The last trial ran 40.474 s and completed the pass-and-rejoin at 16.671 s.
Minimum sampled body clearance was 0.5955 m. Post-rejoin nearest-route-point
RMSE was 0.08255 m. Planning P95 was 0.344 ms; MPC P95 was 1.630 ms on the laptop.
Four startup planner rejections remained, but no invalid-planner samples after
2 s. One freshness-triggered replan was recorded. These are measured timings,
not Jetson deadline guarantees or continuous-time collision-safety proofs.

TRACKED-only position RMSE was 0.0753 m; whole-trial tracking position RMSE was
6.515 m because stale estimates remain after the lead exits the forward field
of view. Do not report the tracked-only value as whole-trial accuracy.

`RIGHT_SIDE_AUDIT.json` checks duration, configured outcome, controller failures,
timeouts and invalid-planner samples after startup. `PRE_BOUNDARY_FIX_AUDIT.json`
deliberately reports `all_clean=false`. The original coarse `passed` flag alone
is not a stability acceptance criterion.

## Evidence and reproduction

`pre_boundary_fix/`, `boundary_fix/`, and `right_side_fix/` contain all runs
matching those prefixes, including failures: configurations, summaries, source
patches, logs, separate English figures and losslessly compressed CSVs.
Each export has a SHA-256 manifest. Other intermediate Oct 16–17 trials remain
locally under `Code/sim_ws/results/takeover/`; they are not included in this
snapshot's acceptance denominator. The Oct 8 campaign is separately archived.

From `Code/sim_ws`:

```bash
bash tools/test_takeover_offline.sh
TAKEOVER_PASS_SIDE=-1 ./run_takeover_trial.sh fitzpatrick_occlusion '' 3 new_right_trial false
python3 tools/report_timed_consistency.py results/takeover new_right_trial \
  --expected-runs 1 --require-clean

# Pending full latest-code validation; use new, non-overwriting run IDs.
bash run_fitzpatrick_batch.sh 3 new_full_batch
python3 tools/report_timed_consistency.py results/takeover new_full_batch \
  --expected-runs 15 --require-clean
./run_takeover_trial.sh v8_clear '' 1 new_static_regression false
./run_takeover_trial.sh v8_static '' 1 new_static_regression false
```

The offline C++ and Python regression suites were also rerun when preparing
this push. All tests passed. No ROS simulation or real vehicle was started
during that push-time check.

Seeds control measurement perturbations; fixed layout and ROS wall-clock
scheduling mean these are limited repeatability checks, not a statistically
established success probability or bit-exact replay. The synthetic 0.6 s target
measurement interruption is not a general natural-occlusion benchmark.

## Remaining work

1. Rerun the full 15-trial Fitzpatrick matrix on the latest controller code.
2. Repeat both permitted passing sides; rerun static-v8 and lead acceleration,
   braking, stop/go and insufficient-space regressions.
3. Inspect failures and command smoothness independently of the coarse pass flag.
4. Validate actual Jetson timing, localization/actuator differences and safety
   interfaces before any physical deployment. No physical overtaking claim here.

## Technical route

Fixed map/reference route + ego localization; LiDAR -> existing KF estimate ->
gated route-assisted prediction -> cruise/follow/prepare/overtake/cancel/rejoin
behavior -> time-indexed Frenet `s(t), d(t)` with inflated-body checks -> shared
v8 dynamic MPC (longitudinal acceleration and steering) -> MPC future-motion
validation -> `/drive_nominal` -> safety arbiter -> `/drive`.

No acquired target: retain fixed-route/static-v8 planning. Opponent truth is
restricted to its own control and independent scoring. The predictor is a
single-target route prior with heuristic uncertainty inflation, not an
interactive intention model or a calibrated probabilistic safety guarantee.
