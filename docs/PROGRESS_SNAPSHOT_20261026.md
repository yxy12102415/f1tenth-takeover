# Development snapshot — 2026-10-26

This commit saves the current laptop development state on
`codex/predictive-spliner-20261022`; it is not a release or real-vehicle approval.

## Included

- Selective upstream SQP adapter, track-assisted prediction, timed candidate
  checking, non-actuating MPC preview and recovery logic.
- Gym-specific model configuration, simulator clock checks, multi-position
  trial tools, RViz reference/planned-route visualization and plotting tools.
- Existing result summaries, including failed and incomplete trials.
- Chinese technical explanation with formulas, state-machine conditions and
  explicit real-vehicle versus simulation boundaries:
  [Word report](reports/F1TENTH项目技术说明_原理公式版_20261026.docx).

## Checks actually rerun for this snapshot

`bash Code/sim_ws/tools/test_takeover_offline.sh` passed on the laptop:

- Candidate scoring / footprint checks.
- Prediction, timing, longitudinal lag, cancellation and stale-target checks.
- Non-actuating MPC preview checks for both available model builds.
- 10 evaluator geometry tests, 2 tracker-side tests and 3 simulator-clock tests.
- Late / fragmented SQP socket response recovery check.
- 7 upstream SQP numerical tests.

Python AST parsing (18 changed/new Python files), shell syntax checks and
`git diff --check` also passed. No ROS driving graph, GUI simulation or physical
vehicle was started for this push. No new full-track success is claimed.

## Known limitations

Historical reports retain the settings used at the time; their numbers are not
fresh results for all current parameters. The four-position screen includes
failed trials and does not establish successful overtaking everywhere.

The current behavior code still limits speed near the opponent until sufficient
lateral separation is established. The requested more progressive acceleration
policy has not been validated in this snapshot. Dynamic overtaking has not been
accepted on the real vehicle. See `LAPTOP_TAKEOVER_SIM.md`,
`UPSTREAM_SQP_ADAPTER.md` and the dated result directories for details.

Generated builds, dependencies, large local raw logs and unrelated personal
documents are not included. This snapshot preserves the configured external
dependency workflow; it does not vendor upstream source or resolve its licensing.
