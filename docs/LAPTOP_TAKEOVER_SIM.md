# Laptop Dynamic-Overtaking Simulation

## Validated host

The implementation was built and exercised natively on Ubuntu 22.04 x86_64
with 16 CPU threads, 30.8 GiB RAM, ROS 2 Humble, GCC 11.4, CMake 3.22, Docker
29.1, and acados v0.4.0 commit
`43354449192aa51557abda37d1bc17d907077354`. Run the read-only audit with:

```bash
bash tools/check_laptop_environment.sh
```

The generated v8 solver is not ABI-compatible with the system's newer acados
v0.5.x. Install the pinned dependency inside the ignored `.deps/` directory:

```bash
bash tools/setup_laptop.sh
```

This host is Linux, so the native ROS path was selected. No Mac result is
claimed. On Intel or Apple-Silicon Macs, use an Ubuntu 22.04 ROS 2 Humble
container/VM matching the host architecture and build acados from source; do
not copy Jetson `aarch64` binaries. Headless trials are suitable for Docker.
RViz requires XQuartz/X11 forwarding or a Linux VM desktop and has not been
validated in this delivery.

## Architecture and truth isolation

```text
/scan + /ego_racecar/odom
        -> LiDAR clustering + EKF identity track
        -> constant-velocity prediction with covariance growth
        -> CRUISE/FOLLOW/PREPARE/OVERTAKE/REJOIN/WAIT/CANCEL
        -> time-indexed Frenet s(t), d(t) body-footprint validation
        -> v8 joint longitudinal/steering dynamic MPC
        -> /drive_nominal -> safety_arbiter -> /drive
```

`/opp_racecar/odom` has only two consumers: the opponent's own route controller
and `takeover_evaluator.py`. The ego controller subscribes to
`/ego_racecar/opp_odom_ekf`; the tracker is launched with
`measurement_source=lidar`. Opponent truth is never republished into the ego
tracking/planning/control chain. The evaluator's metadata and every summary
record this policy.

The behavior layer commits to one passing side. Each 5 Hz replan checks map
clearance, oriented vehicle footprints, time-aligned constant-velocity
prediction, covariance/process inflation, inter-sample closing distance, and
curvature. A temporarily unsafe full-speed trajectory is retried at lead speed
without switching sides. The 20 Hz v8 MPC remains responsible for acceleration
and steering together; no post-solve speed override was added.

## One-command operation

From `Code/sim_ws`:

```bash
# Headless deterministic trial
./run_takeover_trial.sh constant_speed "" 1 demo_seed1 false

# The same scenario with RViz
./run_takeover_trial.sh constant_speed "" 1 demo_gui true

# Five scenarios, three independent seeds each, then aggregate
./run_takeover_batch.sh 3 my_batch
```

Arguments are `scenario [duration_override] [seed] [run_id] [gui]`. A run ID
is immutable: the runner refuses to overwrite an existing directory. Outputs
are saved under `Code/sim_ws/results/takeover/<scenario>/<run_id>/`:

- `config.json`, `metadata.json`, and `summary.json`;
- `trajectory.csv` (truth, estimate, control, behavior, safety, and runtime);
- `predictions.csv` (all time-aware ego references and lead predictions);
- full launch/ROS logs;
- separate English `trajectory_en.png` and `prediction_en.png` figures.

The runner returns non-zero when the configured behavioral expectation is not
met, while preserving all logs and result files. In particular,
`constant_speed` must complete an overtake, whereas `no_space` must enter
FOLLOW/WAIT/CANCEL and must not complete an overtake.

Available scenarios are `v8_clear`, `v8_static`, `constant_speed`,
`accel_brake`, `stop_go`, `short_occlusion`, `no_space`, and
`fitzpatrick_constant_speed`. Definitions and seeds are in
`src/takeover_sim/config/scenarios.json`.

## Rebuilding maps and adapters

```bash
python3 generate_standardized_maps.py
python3 tools/import_fitzpatrick_sim.py
python3 tools/generate_takeover_assets.py benchmark_takeover
python3 tools/generate_takeover_assets.py Fitzpatrick_sim
```

`benchmark_takeover` is a 2.20 m-wide stadium with a 24 m passing straight and
4 m-radius turns. Existing standardized maps remain available for v8
regression. `Fitzpatrick_sim` is a format adapter generated from the real map
and speed-profile route; it is not a new map.

## Simulation versus the real vehicle

F1TENTH Gym uses a simplified bicycle/tire model and idealized Ackermann command
interface. The real car has calibrated VESC ERPM conversion, low-speed stiction,
servo lag/deadband, PF localization latency, scan occlusion, battery/load
variation, and command watchdogs. The shared C++ planner and v8 MPC reduce code
drift, but Jetson deployment still requires verification of:

1. ROS topic names, QoS, frames, timestamps, and LiDAR extrinsics;
2. target cluster gates and world-frame EKF velocity on recorded scans;
3. planning and MPC deadlines under the Jetson process load;
4. VESC speed conversion and the existing 0.5 m/s drivable floor;
5. final-command guard, deadman, physical E-stop, and stale-command zeroing;
6. pass-side clearance using the physical vehicle footprint.

Do not connect this simulation launch to the vehicle ROS domain. It publishes
`/drive`; run it only in an isolated laptop ROS domain/container.

## Current limitations

- The 15-trial standardized suite passes, but whole-trial tracking RMSE becomes
  large after the lead moves behind the forward LiDAR. The pass succeeds using
  a bounded committed CV prediction; this is not a robust 360-degree tracker.
- The current verified stop-and-go batch stayed above 0.406 m, but a preserved
  development seed reached only 0.170 m. This sensitivity still blocks direct
  physical deployment.
- A strict Fitzpatrick straight-window trial did not finish the pass before its
  horizon reached the following bend. Fitzpatrick dynamic overtaking is not
  accepted.
- The evaluator rotates Gym body-frame truth velocity into the map frame before
  comparing it with EKF velocity. Fitzpatrick still needs more independent
  trials before any publication claim.
