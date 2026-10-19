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
        -> gated track-assisted prediction (Cartesian CV fallback)
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

The laptop tracker additionally rejects returns explained by the known static
map and clusters more than 1.5 m from the reference route. This is a map-assisted
LiDAR tracker, not map-free perception or general multi-object tracking. Gym ego
odometry is used for localization; physical PF localization errors are not yet
injected. With no acquired target the adapter uses the original static v8
planner to follow the route. After passing, a stale track cannot re-trigger
following solely because route progress wraps around.

The behavior layer commits to one passing side. Among valid entry candidates,
it maximizes the minimum of remaining map clearance and inflated predicted
opponent-body clearance, rather than ranking wall clearance alone. Each 5 Hz replan checks map
clearance, oriented vehicle footprints, time-aligned track-assisted
prediction, covariance/process inflation, inter-sample closing distance, and
curvature. A temporarily unsafe full-speed trajectory is retried at lead speed
without switching sides. The 20 Hz v8 MPC remains responsible for acceleration
and steering together; no post-solve speed override was added.

The overtaking planner is a separate timed-Frenet extension, not the unchanged
static v8 planner. In the 2026-10-17 implementation it passes timed position and
speed references to the same v8 MPC, compensating for plan age. Longitudinal
references include acceleration, jerk, response lag and curvature preview. The
optimized first command is executed without an independent speed override, and
the MPC's own predicted motion is checked against the moving target before
publication. Exact schedule tracking/recursive feasibility is **not** guaranteed.
See [the integration contract](TRACK_AWARE_TIMED_CONTROL.md) for recovery,
prediction assumptions, optional legacy mode and diagnostic fields.

## One-command operation

From `Code/sim_ws`:

```bash
# Headless deterministic trial
./run_takeover_trial.sh constant_speed "" 1 demo_seed1 false

# Fitzpatrick map, fixed reference route, moving lead at 0.8 m/s
./run_takeover_trial.sh fitzpatrick_1p5 "" 1 fitz_demo false

# All five Fitzpatrick cases, three repetitions; preserves failures
bash run_fitzpatrick_batch.sh 3 fitz_batch

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
- timed-reference errors and MPC future motion in the same CSVs (new runs);
- full launch/ROS logs;
- separate English `trajectory_en.png` and `prediction_en.png` figures.

The runner returns non-zero when the configured behavioral expectation is not
met, while preserving all logs and result files. In particular,
`constant_speed` must complete an overtake, whereas `no_space` must enter
FOLLOW/WAIT/CANCEL and must not complete an overtake.

On this laptop, prefer headless runs and inspect the saved PNGs. Whole-desktop
freezes were reported with the previous interactive workflow; their root cause
has not been proved. The runner caps build/BLAS/Numba concurrency, allows only
one trial at a time, redirects verbose output to files, and uses localhost-only
Cyclone DDS on ROS domain 73 (override with `TAKEOVER_ROS_DOMAIN_ID`). Do not
start a second launch while a batch is running. RViz was not revalidated in this
headless test campaign.

New runs save the complete scenario configuration and a tracked-source patch
against the recorded Git commit. Seeds control injected tracker measurement
noise; Gym currently keeps its default LiDAR seed 12345. Separate trials are
repeatability runs, **not independent randomized maps/traffic layouts**. ROS
wall-clock scheduling means a fixed seed is not bit-for-bit replay.

Offline evaluator checks (no ROS nodes are started):

```bash
bash tools/test_takeover_offline.sh
```

Available scenarios are `v8_clear`, `v8_static`, `constant_speed`,
`accel_brake`, `stop_go`, `short_occlusion`, `no_space`, and
`fitzpatrick_constant_speed`, `fitzpatrick_clear`, `fitzpatrick_1p5`,
`fitzpatrick_occlusion`, and `fitzpatrick_follow`. Definitions and seeds are in
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
  horizon reached the following bend. The new whole-route cases do not erase
  that failure; they use a different initial position and a 40-second window.
  The revised whole-route batch passes cruise and nominal overtaking in all
  three repetitions, but dropout/noise passes only 1/3 and the 2.0 m/s runs
  still trigger MPC fail-stops. See the [2026-10-08 report](../results/fitzpatrick_sim_20261008/REPORT.md).
- The evaluator rotates Gym body-frame truth velocity into the map frame before
  comparing it with EKF velocity. Fitzpatrick still needs more independent
  trials before any publication claim.
- Evaluation checks the entire body against occupied map cells and verifies
  pass-and-rejoin using opponent truth. It samples at 20 Hz; zero sampled
  collisions is not a continuous-time safety proof. Clear-route acceptance is
  progress above 90% of route samples, not an endpoint-stop assertion.
- `maximum_abs_acceleration_command_mps2` is the finite difference of the final
  speed command, not measured vehicle acceleration. A fail-stop can produce a
  large command step; it must not be hidden by reporting MPC limits instead.
