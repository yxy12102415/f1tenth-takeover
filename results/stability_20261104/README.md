# Actual laptop trials — 2026-11-04

See [the work report](../../docs/STABILITY_20261104.md) for version boundaries,
the decision not to promote experimental policies and outstanding failures.

- `screen_audit.json`: eight matched position/speed-policy trials; initial
  recovery fix, early verified return not yet implemented. Configured success
  8/8, strict clean motion 4/8.
- `early_return_repeat_audit.json`: eight trials of the subsequent early-return
  candidate (raw prefix `stability_final_20261104`). Configured success 4/8,
  clean motion 3/8. Failed trials are included, not deleted or rerated.
- `default_smoke_audit.json`: current source, both experiments off, seed 2 at
  positions 25 and 50. Position 25 failed completion; position 50 passed at
  29.15 s. Neither passed the strict clean-motion audit.
- `all_trials_audit.json`: all 22 completed trial processes including four
  exploratory debugging runs. Different revisions must not be pooled into
  a single method success rate. All 22 graph audits passed and all 22 had
  zero scored collision/boundary events.

Raw CSVs, per-trial configurations, `source.patch`, source hashes, live graph
audits, summaries, worker failure traces and separate English trajectory plots
remain under `Code/sim_ws/results/takeover/<scenario>/<run_id>/` on this laptop.
Schedules/aggregates are in `Code/sim_ws/results/takeover_stability_batches/`
and `Code/sim_ws/results/takeover_position_batches/`. Large raw logs are ignored
by Git as before; these audit JSONs preserve the selected numerical evidence.

The seed is recorded, but sensor injection noise is zero in these four scenes;
fresh-process scheduling variation remains. These are engineering screens,
not an estimated success probability or real-time/real-vehicle certification.
