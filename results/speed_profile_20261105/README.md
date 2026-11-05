# Predictive passing-speed experiments — 2026-11-05

The tests use official Gym native dynamics, the existing Fitzpatrick map and
route, LiDAR-only opponent estimation, ego/lead caps 3.0/1.5 m/s and optimized
command acceleration bounds [-3,+3] m/s². They are laptop simulation evidence,
not new physical-vehicle results.

- `initial_screen_audit.json`: eight matched trials of the first candidate;
  completion did not improve, despite all configured passes.
- `second_screen_audit.json`: eight trials after speed-proposal selection and
  physical braking arc length; both candidate and legacy completed only 2/4.
- `third_screen_audit.json`: eight trials after separating geometric steering
  from future speed feasibility and warming the numerical worker before ready.
  Candidate 4/4 and legacy 2/4 configured completions; not uniform clean motion.
- `repeat_audit.json`: four additional candidate repeats on the identical
  runtime source. All four completed; the eight candidate trials together
  completed 8/8, but only 2/8 passed the extra strict no-anomaly audit.
- `trials/MANIFEST.json`: checksummed lossless export of all 30 trial processes
  across versions, including all seven failed attempts. CSVs are gzip-compressed;
  configs, source patches, SQP failure traces, logs and separate English plots
  are included. Original ROS subdirectory logs remain in the source folders.

The JSON audits retain independent completion time, body/boundary outcomes,
invalid-plan samples, cancellations, control/runtime failures, source hashes and
live graph audits. Versions must not be pooled into a method success rate.

Complete configs, trajectories, predictions, source patches, English plots and
logs, including every failed attempt, remain under
`Code/sim_ws/results/takeover/<scenario>/<run_id>/`. Schedules and original
summaries remain under `takeover_stability_batches` / `takeover_position_batches`
inside the same results directory. The exact commands and interpretation are
in [the experiment report](../../docs/SPEED_PROFILE_20261105.md).

No collision threshold, boundary threshold, SQP deadline, or real-v8 file was
relaxed to obtain a pass. Faster proposed trajectories still require complete
timed inflated-body checks and a non-actuating MPC preview. The remaining
preview/live agreement and recovery problems are reported, not rescored away.

Selection rationale: [Frenet, splines, MPC and alternatives](../../docs/PLANNING_CONTROL_TRADEOFFS.md).
