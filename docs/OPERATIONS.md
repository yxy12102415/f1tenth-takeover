# Vehicle operations

These commands target the **already commissioned Jetson workspace**. They are not instructions to launch a fresh Git clone as an approved vehicle runtime.

```bash
cd ~/f1tenth_real_migration_ws_20260829
```

For the currently validated dynamic MPC branch:

```bash
./fitzpatrick_avoidance_dynamic_mpc.sh
```

For the separate PID branch:

```bash
./fitzpatrick_avoidance_pid.sh
```

The `run` entry point checks matching offline evidence and reuses eligible zero-output evidence. A new build requires matching evidence; the real zero-output/localization observation is five seconds. Once the actual vehicle session reports READY, hold L1 to move. Releasing L1 stops and latches the session; reset position/heading before a new attempt.

Fixed Fitzpatrick start: x = 0.6138574243 m, y = -0.3471186155 m, yaw = 0 rad. This is valid only in the saved Fitzpatrick map frame and after actual physical placement/localization alignment. Closed geometry is executed as one lap followed by a stop.

Cruise configuration is 0.70–2.00 m/s, longitudinal acceleration up to 1.0 m/s² and deceleration down to -0.8 m/s². Launch ramps from zero; after reaching the drivable floor, positive commands stay at or above 0.50 m/s. Terminal control may use 0.50–0.70 m/s before zero is latched.

For map creation and route recording, consult the included `real_vehicle/FITZPATRICK_TEST_GUIDE.md` as a historical operating guide. Its referenced auxiliary hardware tools live in the installed workspace and are not all part of this source export. Its dated earlier results are superseded by `docs/STATUS.md`.
