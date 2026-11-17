# Control performance optimization — 2026-11-16

No physical motion was started. Tests use the original 381 2.0 map/route and the same stationary-opponent synthetic-plant fixture for PID and dynamic MPC.

## Change

The safe-speed preview used to scan the entire body grid for every swept pose of every tested stopping speed and steering perturbation. A paired replay of 12 captured states measured a 16.3 ms median preview call. It now first computes a conservative lower bound using the exact map Euclidean distance transform, the map boundary, a complete cell diagonal for query quantization, the body's circumscribed radius, the configured wall margin and the original body-sampling cover. Only a bound proving >=4 cm reserve can skip the dense body grid; ambiguous poses use the original grid. The speed-cap output, uncertainty model and clearance thresholds are unchanged. The final exact emergency-coast guard is unchanged.

The new preview matched the old executable in 60 cases, including map boundaries. Paired captured-state median time was 2.7 ms, approximately 6.1x faster. This benchmark includes identical local probe protocol overhead in both versions.

## Timing and guards

Added separate timings for pure acados calls, reference and tube setup, seed rollout, solver residual evaluation, independent dynamics residual evaluation, path speed preview, state coast preview, primary prediction check, exact emergency-coast check, and newest-observation recheck. Tube time is part of reference setup; path/state preview times are part of speed policy. These nested times must not be added twice.

The earlier control/guard timing is retained for compatible before/after comparison. `control_to_emit_ms` now also includes the newest-observation recheck. A final unchanged 100 ms hard check runs before issuing the command. `control_to_drive_publish_ms` measures callback entry through the ROS drive publish call; it is not sensor-to-physical-actuator latency. Repeated parsing of the same predicted path was removed; both full observation checks still execute.

Dynamic model, 24-step horizon, maximum 8 refinement calls, 0.001 nonlinear tolerance, 45 ms MPC soft budget/three-miss policy, 100 ms hard deadline, speed/acceleration limits, body gaps, wall margin, and physical coast model remain unchanged.

## Observed results

| dMPC timing, same legacy scope | Before | After |
| --- | ---: | ---: |
| Median | 25.5 ms | 11.2 ms |
| 95th percentile | 48.6 ms | 38.6 ms |
| Maximum | 99.9 ms | 48.1 ms |

New complete callback-to-ROS-publish timing: median 13.4 ms, p95 42.1 ms, maximum 50.5 ms. Pure acados calls summed per control cycle: median 3.2 ms, p95 6.6 ms, maximum 9.5 ms. These are same-scenario runtime samples, not identical recorded-input solver replays, so scheduler/trajectory differences can contribute to the aggregate comparison.

PID and dynamic MPC both completed the simulated route. Their minimum independent software body separations were 18.9 cm and 20.0 cm. These are not measured physical clearances. The revised C++ binaries have not been driven on the real car.

Evidence: `coast_preview_optimization_regression.json`, `both_controller_regression.json`, `control_performance_optimization.json`, and `reports/compact_fix/control_perf_both_ros.log`.

## Follow-up: captured field failure and reference preparation

The subsequent real run `20261116_153449` stopped without contact after 1.79 seconds of wheel motion. The first moving rejection was a 103.807 ms controller call, including 93.884 ms reference setup and 6.066 ms direct solver calls. Earlier cold-start rejections were not the driving stop.

The operator selected an exact path-versioned reference speed table and then requested a 200 ms control hard deadline. The planner now prepares numerical map speed limits for each accepted path. The subscriber validates the complete path, plan ID, immutable numerical map version, policy version, floor and wall margin. Both PID prediction and dMPC references reuse admitted limits; current-state coast and full observation guards remain live. A floor-infeasible point remains an explicit rejection when queried. The final 40 cm preserves the existing terminal exemption.

Captured-path tests compared 404 speed queries exactly and rejected seven metadata/path/parameter mismatches. A separate fixed-station 25-query cold-process microbenchmark measured a median 67.44 ms before versus 0.223 ms after. It excludes planner precomputation, which took about 105 ms cold, and is not a full controller replay. Timing at planner publication and closed-loop control must be inspected separately.

The 200 ms deadline covers internal PID/dMPC work and the full callback before output; refinement is explicitly capped at the previous 85 ms. The 45 ms sustained-overload policy, residual tolerance, sensor leases, body margins and vehicle settings are unchanged. This revision requires a new physical test after software validation.
