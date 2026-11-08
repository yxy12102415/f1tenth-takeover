# Compact overtaking v5: input gate fix

Uses the same v4 C++ binaries, generated model, libraries, geometry and speed limits. Seven actual ROS-gate callback tests passed. Baseline v4 shared library dependencies are explicitly included in the v5 runtime manifest because copied binaries retain the v4 immutable RPATH. No C++ recompilation is required.

A measured L1 sequence contained a 0.14-second press, 0.24-second release, then a stable 5.94-second hold. Linux and ROS agreed. The old gate latched the initial prearm release; two field attempts received only about 0.6 seconds of effective L1 before their release latch and never rotated the F1 wheels.

Before ARM: release resets qualification and allows another hold. ARM requires continuous L1 for 1.0 second AND fresh MPC readiness stable for 0.5 second. After ARM: release or joystick timeout immediately latches deadman STOP, with no debounce or auto-rearm. Startup-held L1 still requires a release first.

Field sequence: F1 stays at original route start; lead center 1.80 m ahead (about 1.38 m LiDAR-to-rear), parallel headings. Keep L1 released until READY. Hold F1 L1 continuously; once F1 begins rolling, hold lead L1 for about 3 seconds then release it. Keep F1 L1 held until its endpoint stop. Both buttons stop their respective cars.

Stop remains zero-current coast, not commissioned active braking. This is an attended bounded moving-lead trial; physical v5 success is not claimed by these software tests.

Command: `cd ~/f1tenth_real_migration_ws_20260829` then `./381_2_0_compact_overtake.sh run`. Matching offline checks are cached; new fingerprint requires one 5-second real zero/localization check.
