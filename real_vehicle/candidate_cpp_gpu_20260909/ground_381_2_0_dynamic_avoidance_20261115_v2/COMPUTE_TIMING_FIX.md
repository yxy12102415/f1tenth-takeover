# Bounded MPC compute jitter

The 2026-11-07 12:41 ground attempt stopped 4.463 s after motion began.
The optimizer succeeded (status 0); total computation took 46.271 ms, above
the old 45 ms cutoff. A later computation took 51.733 ms. The controller
published invalid zero and reset its state; the independent downstream guard
latched the attempt. Inputs were fresh and the planned footprint was clear.

This release retains 45 ms as a soft budget and accepts an occasional completed
MPC result up to 75 ms only after the existing solver status, finite-value,
nonlinear residual, actuator, speed-floor and swept-path checks pass. It rejects
the third consecutive soft-budget miss, and rejects any result over 75 ms.
A normal <=45 ms computation resets the consecutive-miss count. An accepted
soft miss preserves command, acceleration, steering and warm-start state.
There is no old-command replay or fallback path. PID keeps its old 45 ms rule.

These are admission deadlines checked after computation, not an operating-
system preemption guarantee. The independent downstream watchdogs remain active.
The ROS controller still rechecks input, plan and enable freshness after every
compute. The 0.5 s source/receipt leases, L1 behavior, 25 s motion limit,
collision distances and endpoint completion conditions are unchanged.

The nominal period remains 50 ms. With the existing catch-up suppression, an
occasional late cycle can reduce effective update rate; 3 consecutive misses
therefore stop instead of silently accepting sustained overload. Wall time and
thread CPU time are recorded separately to diagnose scheduling delays. One
redundant predicted-path JSON conversion was removed. This is a timing-policy
repair, not a claim that all Jetson scheduling jitter has been eliminated.

Route: original 381 2.0 start, endpoint extended 0.89 m; length 6.735 m.
Maximum command remains 1.25 m/s. No physical test is performed by validation.

Validation artifacts in run_logs:
- recorded_compute_replay.json: replay all 99 recorded input updates through
  the first failure using the original map/route assets and local paths.
- compute_timing_summary.json: actual C++ ROS node with test-only LD_PRELOAD
  delay injection. Old code rejects a brief delay; new code preserves control,
  rejects a hard overrun, and rejects the third consecutive soft overrun.
- wait_scan_regression.json: brief scan wait preserves cap; expired plan stops.
- joint_rollout_summary.json: independent plant rollout on the new route.
- terminal_ros_summary.json: terminal standstill, zero feedback and endpoint
  mismatch cases through the actual C++ ROS controller.
- timing_fix_integration.log: isolated non-actuating integration window.

The delay injector is a test artifact only: it is not linked to the controller
or selected by startup. Its synthetic ROS tests require localhost domain 83.
