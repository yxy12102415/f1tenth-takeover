# Fitzpatrick upstream-SQP integration: new runs, not historical acceptance

Simulation only. This report concerns the optional `upstream_sqp` adapter,
not the default adaptive Frenet candidate planner and not a full reproduction
of Predictive Spliner's GP/ROS 1 stack. See
[adapter scope and setup](../../docs/UPSTREAM_SQP_ADAPTER.md).

## Fixed acceptance conditions

- Fitzpatrick map, ego maximum 1.8 m/s, lead reference speed 1.0 m/s.
- Two independently scored pass-and-rejoin events in one continuous 110 s trial.
- Constant-speed lead and lateral-weave lead (amplitude 0.22 m).
- Zero measured vehicle collisions and map-boundary violations.
- Separate stricter clean-motion audit: no controller failures, solver timeouts,
  SQP timeouts or planner-invalid samples after the first 2 s.
- Lead truth is for lead control and independent evaluation, not ego tracking,
  prediction, planning or MPC. Ego tracking uses LiDAR.
- All raw failures remain under `Code/sim_ws/results/takeover/`.
- Seeds 1–3 are independent wall-clock repeats. These scenarios configure zero
  injected measurement noise; they are not three distinct noise conditions.

## First frozen batch: `sqp_accept_20261022`

Six completed trials, each about 111 s. Constant lead: 2/3 achieved two passes.
Lateral weave: 1/3 achieved two passes. All six had zero measured collisions and
boundary violations. **0/6 passed the stricter clean-motion audit.**

| Scenario | Seed | Truth passes | MPC rejections | SQP timeouts |
|---|---:|---:|---:|---:|
| Constant lead | 1 | 1 | 4 | 5 |
| Constant lead | 2 | 2 | 5 | 7 |
| Constant lead | 3 | 2 | 0 | 12 |
| Lateral weave | 1 | 1 | 6 | 2 |
| Lateral weave | 2 | 2 | 1 | 6 |
| Lateral weave | 3 | 1 | 3 | 12 |

See `initial_audit.json` for all six rows, exact values, source hashes, and
rejection reasons.

Observed failure: the second encounter occurs near a bend; an infeasible SQP
proposal leads to cancellation. Some trials recover and retry successfully,
while others cannot find a safe recovery path and remain stopped. The collision
checker was not bypassed. Solver trace replay found infeasible linear corridor
constraints behind many expensive rejected SQP solves.

## Follow-up changes and evidence

`sqp_stage2_20261022` trials use a necessary-condition linear feasibility check
before the unchanged original SLSQP objective/constraints and final validation.
The check is an outer relaxation and cannot certify a trajectory. A failed
check rejects the proposal. Preparation also reaches approximately lead speed
on the reference route before admitting the lateral manoeuvre. Speeds are
changed through existing time parameterization and MPC, not post-solve overrides.

That follow-up **failed**: zero passes, zero measured collisions/boundary events,
three MPC dynamic-conflict rejections, zero SQP timeouts. Its preparation-speed
change was reverted. The separate `sqp_lp_20261022_seed1` test of only the linear
precheck also failed: zero passes, two MPC dynamic-conflict rejections, zero SQP
timeouts, zero measured collisions/boundary events. Both records are retained.

Faster rejection of infeasible entry candidates exposed an early commitment to
the opposite side during startup velocity estimation. The next revision adds an
estimate-only admission gate: distinct recent measurements, convergence of
velocity covariance, and a 0.6 s stable velocity interval. It affects admission only;
the tracker, committed-pass validation and stop conditions are unchanged.
Its tests/results must be assessed separately, not pooled with earlier revisions.

The first gate prototype (`sqp_gate_20261022_seed1`) used an inappropriate fixed
velocity-variance cutoff of 0.25, below this tracker's observed steady-state
value of about 0.51. It stayed in preparation/following and achieved zero passes.
That cutoff was removed; covariance convergence is now checked instead. This
does not change the collision inflation model or any collision threshold.

With convergence-based admission (`sqp_settled_20261022`), constant-lead seeds
1 and 3 achieved two passes; seed 2 stopped during the second encounter. All
three had zero measured collisions/boundary events and zero SQP timeouts.
The planned batch was **stopped after these three complete trials**, before its
lateral-weave cases, to address the recurring failure. This is an incomplete
batch, not six completed tests. See `settled_stopped_batch_audit.json`.

The `sqp_preview_20261022` revision adds 0.10 m of map execution reserve
and non-actuating MPC validation of SQP proposals, preserving live checks and
recording preview rejections separately. All six trials finished: constant lead
achieved two passes in 3/3 with zero controller failures and SQP timeouts.
Lateral weave achieved only one pass in each of 3/3 trials. Lateral seed 1 had
one scored collision; seed 3 had four controller rejections. Thus the lateral
cohort failed. The collision occurred after a cancelled manoeuvre: the ego
stopped on a bend because recovery curvature was infeasible, and the lead
subsequently hit the stationary ego. A zero command did not certify a safe
terminal configuration. All six records are retained.

The user's subsequent speed target is ego maximum 3.0 m/s and lead maximum
1.8 m/s. Those trials use separate scenario names and run IDs; see
`../fitzpatrick_3p0_1p8_20261022/README.md`. The lower-speed records above must
not be presented as high-speed acceptance.

## Offline regression checks, this session

`bash Code/sim_ws/tools/test_takeover_offline.sh` passed after the follow-up:
candidate/footprint checks; prediction/timing/lag/cancellation/stale-target checks;
non-actuating MPC preview state isolation; 10 Python geometry tests; socket
deadline/fragmented-response recovery; 7 optional upstream numerical tests.

## Reproduction

From `Code/sim_ws`:

```bash
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_double_fast '' 1 NEW_RUN_ID false
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_double_overtake_batch.sh 3 NEW_PREFIX fast
python3 tools/report_timed_consistency.py results/takeover NEW_PREFIX --expected-runs 6 --require-clean
```

Use a new run ID/prefix; existing evidence is never overwritten. Replace `false`
with `true` for the single-trial GUI, not concurrently with the batch. Independent
English `trajectory_en.png` and `prediction_en.png` are produced per trial.

## Limits

No Jetson or real actuator test. No GP opponent model. The upstream checkout's
license remains unresolved; no upstream source is vendored. The Python worker
is a local evaluation adapter, not a hard-real-time deployment. Default static
and candidate-planner implementations remain available, but the earlier short
static-obstacle trial stopped before completing its bypass and is not evidence
of a successful full static regression.
