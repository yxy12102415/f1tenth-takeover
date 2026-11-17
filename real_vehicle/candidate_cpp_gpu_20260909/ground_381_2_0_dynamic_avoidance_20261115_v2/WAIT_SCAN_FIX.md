# Brief scan-alignment wait and MPC speed cap

The 2026-11-07 12:13:35 ground attempt stopped 0.997 seconds after its motion
window began. L1 remained held and health data were valid. The controller
reported `joint_mpc_solver_4`, followed by `planner_invalid` and `upstream_stop`.
The selected center path had no footprint collision. At the failing update,
the previous command was 1.24754 m/s with acceleration +0.19913 m/s²; a
`WAIT_SCAN_POSE` notification changed the hard command cap to 0.6 m/s.
The optimizer could not satisfy that step together with its acceleration and
jerk limits. Final recorded wheel speed was zero.

v2 retains the admitted plan's original driving state when a matching wait
notification arrives, and records `waiting_for_scan_pose=true` separately.
Neither the path nor the generation/receipt/input timestamps are refreshed by
that notification. The original 0.5 s plan/input lease still stops the vehicle
when data expire. Actual new blockage plans, collision checks, solver checks,
deadman behavior, acceleration/jerk limits and the 25 s motion window are unchanged.

`test_wait_scan_ros.py` exercises the actual C++ ROS controller on isolated
domain 83 without drivers. It reproduces the old failure, checks that a brief
wait preserves the cap, and verifies zero invalid output after the original
plan lease expires. These tests do not constitute a successful ground run.

The original v1 release, manifest and failed-run evidence remain intact.
