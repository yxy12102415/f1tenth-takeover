# Physical F1TENTH deployment

## Release status

The Gym implementation is ready for **sensor-only and shadow-mode deployment**.
It is not yet released for unattended physical driving. The physical vehicle
must first pass the command-loss watchdog, joystick deadman, LiDAR/odometry,
and frame checks below. The default launch intentionally publishes the final
command to `/drive_shadow`, which is not an input in the current vehicle mux.

The physical data path is:

```text
/scan + /odom
  -> LiDAR clustering and EKF belief
  -> Hybrid A* / LiDAR corridor candidates
  -> optional visibility and risk evaluation
  -> MPC
  -> /drive_nominal
  -> safety_arbiter
  -> /drive_shadow (default) or /drive (explicit live override)
  -> ackermann_mux -> VESC
```

There is no opponent ground-truth input in this launch. In particular, the
lead controller and `baseline_evaluator.py` are disabled, and
`measurement_source` is fixed to `lidar`.

## 1. Update the isolated checkout

Do not update or clean the legacy vehicle workspace. Use the isolated checkout
created during the vehicle audit:

```bash
cd /home/f1tenth/f1tenth-risk-aware-following-856c9e6
git status --short
git pull --ff-only origin main
git rev-parse HEAD
```

If `git status` is not empty, stop and preserve those changes before pulling.

## 2. Build the method in the migration workspace

The previous audit used
`/home/f1tenth/f1tenth_real_migration_ws_20260829`. Point that workspace at the
updated `mpc` package if it is not already linked:

```bash
mkdir -p /home/f1tenth/f1tenth_real_migration_ws_20260829/src
ln -s \
  /home/f1tenth/f1tenth-risk-aware-following-856c9e6/Code/sim_ws/src/f1tenth_mpc-main/mpc \
  /home/f1tenth/f1tenth_real_migration_ws_20260829/src/mpc
```

If the link already exists, verify it with `readlink -f` instead of replacing
it. Then build in a new terminal:

```bash
source /opt/ros/foxy/setup.bash
cd /home/f1tenth/f1tenth_real_migration_ws_20260829
python3 -c "import cvxpy, osqp, numpy, scipy; print('MPC Python dependencies OK')"
colcon build --symlink-install --packages-select mpc
source install/setup.bash
ros2 launch mpc real_follow_shadow_launch.py --show-args
```

If the Python import fails, install the missing packages into the same Python
environment used by ROS 2 and repeat the import test. Do not proceed merely
because `colcon build` succeeded: Python imports occur at node startup.

## 3. Hardware and command-safety preflight

Keep the vehicle raised with all four wheels clear. The test operator must be
able to disconnect traction power immediately. Before any live command:

1. Confirm the battery, connectors, steering linkage, wheels, and LiDAR mount.
2. Pair the PS4 controller and verify `/joy` changes at at least 20 Hz.
3. Verify the configured deadman button produces `/teleop` only while held and
   that joystick priority overrides `/drive` in `ackermann_mux`.
4. Prove that loss of `/drive` causes zero motor command within 0.2 s all the
   way downstream at the VESC. A mux input timeout alone is not proof if the
   VESC or an interpolator retains its last RPM command.
5. Confirm only one node is allowed to command the navigation mux input.

Useful read-only checks are:

```bash
ls -l /dev/sensors/vesc /dev/input/js* 2>/dev/null
ros2 topic list
ros2 topic info /drive --verbose
ros2 topic info /teleop --verbose
ros2 topic hz /joy
```

The repository's legacy `throttle_interpolator` retains its last setpoint if
its input stops. It is currently disabled in `f1tenth_stack/bringup_launch.py`;
do not enable it until it has a tested stale-input-to-zero watchdog.

## 4. Sensor-only verification

Start the existing vehicle bringup without any autonomous follower. Locate and
source the vehicle workspace used by the audited installation, then verify:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Required observations:

- `/scan` and `/odom` are continuous and timestamps advance;
- the tracker/follower SensorData QoS subscriber is compatible with `/scan`;
- `odom -> base_link -> laser` is connected, and the measured LiDAR extrinsic
  agrees with the physical mount;
- steering sign, speed units, and wheelbase agree with the vehicle config;
- a stationary lead-sized object is not accepted as a moving target during
  global reacquisition.

## 5. Shadow mode

With the vehicle still raised, start the ego stack in a second terminal. Begin
with `nominal`, not `full`:

```bash
source /opt/ros/foxy/setup.bash
source /home/f1tenth/f1tenth_real_migration_ws_20260829/install/setup.bash
ros2 launch mpc real_follow_shadow_launch.py \
  method:=nominal \
  ego_odom_topic:=/odom \
  scan_topic:=/scan
```

The physical-safe defaults are `max_speed=0.5 m/s`, `max_accel=0.5 m/s^2`,
`max_steer=0.30 rad`, `target_timeout=0.5 s`, and output
`/drive_shadow`. Verify them and record a bag:

```bash
ros2 topic hz /drive_shadow
ros2 topic echo /ego_tracker/status
ros2 topic echo /ego_mpc/control_mode
ros2 topic echo /ego_mpc/planner_valid
ros2 topic echo /ego_mpc/planner_decision
ros2 bag record /scan /odom /ego_racecar/opp_odom_ekf \
  /ego_tracker/status /drive_nominal /drive_shadow \
  /ego_mpc/control_mode /ego_mpc/planner_valid \
  /ego_mpc/planner_decision /ego_mpc/risk_score
```

Move the lead vehicle manually through straight, turning, braking, temporary
occlusion, and disappearance cases. The shadow command must become zero on
stale scan, stale odometry, target loss, invalid values, MPC failure, or lack
of a collision-checked path. Repeat with `method:=corridor`, then
`method:=full`; neither mode is permitted to actuate during this stage.

## 6. First low-speed actuation

Live actuation is allowed only after every preflight and shadow gate passes.
Use an empty, bounded test area, two people (operator and observer), a verified
deadman/override, and no bystanders. Start with the nominal method and an even
lower cap:

```bash
ros2 launch mpc real_follow_shadow_launch.py \
  method:=nominal \
  ego_odom_topic:=/odom \
  scan_topic:=/scan \
  drive_output_topic:=/drive \
  max_speed:=0.3 \
  max_accel:=0.3 \
  max_steer:=0.25 \
  target_timeout:=0.4
```

Test start, stop, target loss, process termination, joystick takeover, and
power removal before placing the car on the ground. Increase to `0.5 m/s` only
after those tests pass. Deploy `corridor` next. Deploy `full` last and only
after its physical shadow logs show bounded planning time and no unsafe
candidate switching.

## Known blockers before `full` physical driving

- The right-side narrow visibility benchmark still produces an occasional
  conservative fail-closed stop when the newly observed obstacle outline
  invalidates the frozen corridor. It never accepts the colliding path, but it
  is not yet a smooth physical-driving result.
- LiDAR target-center offset and the `base_link -> laser` extrinsic require
  physical measurement and bag-based validation.
- Simulation gains and the `0.20 m` planning inflation have not been identified
  from the physical vehicle footprint and steering response.
- A downstream VESC command-loss watchdog and the PS4 deadman behavior must be
  demonstrated on the raised car.

Until these items pass, the correct deployment state is **shadow mode**, not
closed-loop `full` driving.
