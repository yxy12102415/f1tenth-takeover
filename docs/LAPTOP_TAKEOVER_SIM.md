# Laptop Dynamic-Overtaking Simulation

## Multi-position single-pass trials (2026-10-24)

The active test objective is now **one complete pass and rejoin**, with no
intentional launch delay. `run_takeover_trial.sh` defaults to
`fitzpatrick_position_00`. The historical `fitzpatrick_double_*` configurations
remain available for reproducing old results; they are not the current task.

Four configurations place ego at 0%, 25%, 50%, and 75% of the 44.7315 m route:
`fitzpatrick_position_00`, `_25`, `_50`, `_75`. Lead is always 2.25 m ahead in
route arc length; both start from rest. Each runs 35 simulation seconds, with
3.0/1.5 m/s ego/lead speed caps and the same curvature-dependent slowing,
tracker, SQP planner, MPC and safety checks. The spawn bodies were checked
against the occupied map and each other. These are different *initial
encounter locations*, not a command to force unsafe overtaking there. Logs
record actual OVERTAKE entry coordinates; a deferred maneuver may occur later.

The earlier `positions_20261024_screen` results used a 1.8 m/s lead cap;
they are not validation of the updated 1.5 m/s configuration. The acceleration
validation runs below used a 1.2 m/s lead and retain their original metadata.

The native-Gym controller now permits a maximum **command acceleration of
3.0 m/s²**. Planner retiming, generated MPC input bounds and the runtime command
check use this same limit. Normal minimum command acceleration is now
**-3.0 m/s²**, also shared by planner retiming, generated MPC and command checks.
Prediction jerk remains 3 m/s³ and the first control-step jerk bound
4 m/s³. Conservative bend-preview slowing is retained; permitting stronger
braking does not force every turn to use it. The real-v8 profile stays at
-0.8 to +1.0 m/s². This is a simulation change,
not a measured real-car acceleration capability or a real-vehicle deployment.
Each run records the bounds in `config.json` and the optimized acceleration in
`trajectory.csv` (`mpc_acceleration_command_mps2`), separate from the finite
difference of final speed commands. Emergency zero commands can still bypass
normal slew limits. Earlier location-screen results used +1.0 m/s² and do not
validate this new acceleration setting.

Earlier validation with a -0.8 lower bound:
`fitzpatrick_position_00/acceleration3_lead1p2_20261024` completed
35.19 simulation seconds and passed both single-pass and clean-motion checks:
truth-confirmed rejoin at 10.45 s, no body collision/ego or lead boundary events,
no MPC safety rejections, minimum sampled body gap 0.4276 m. Ego/lead observed
maxima were 2.9875/1.2000 m/s. Logged MPC command acceleration ranged from
-0.6773 to +1.3502 m/s²: the allowed upper bound is 3, not a forced constant
acceleration. This one run does not revalidate the other three start locations.
Actual generated MPC input bounds were checked at all 24 shooting stages for
both profiles at that revision: native Gym +3.0, real-v8 +1.0, both with lower
bound -0.8. That result is not validation of the subsequent -3.0 revision.

The subsequent symmetric-limit run
`fitzpatrick_position_00/accel_decel3_lead1p2_20261024` completed 35.04 simulation
seconds and passed single-pass and clean-motion checks: truth rejoin at
10.45 s, minimum sampled body gap 0.4315 m, zero scored collisions/boundaries
and zero MPC rejections. Logged command acceleration was -0.6198 to +1.3721
m/s², so this was a nominal overtaking regression, not a maximum-braking test.
The profile bounds are independently checked at every MPC shooting stage:
Gym [-3, +3], original real-v8 [-0.8, +1].

A later user GUI trial, `fitzpatrick_position_00/gui_20261024_003931`, used the
same runtime source hashes and 1.2 m/s lead as that successful headless run, but
did not complete a truth-confirmed pass in 35.21 s. At 7.25 s it canceled on
`upstream_sqp_rejected:sqp_worker_still_busy`, after one recorded SQP deadline
miss; recovery and retry followed, with three MPC dynamic-conflict rejections.
The socket worker uses a 40 ms wall-clock solve budget. This establishes a
planning-interruption cause for that slow/failed maneuver, not proof that
higher acceleration itself caused it or that RViz alone caused the deadline
miss. Neither deadline nor collision checks were relaxed. A larger acceleration
bound is not a demand to accelerate at that value; jerk, curved-path speed
limits and MPC tracking costs still apply.

With the updated 1.5 m/s lead cap, headless validation
`fitzpatrick_position_00/accel_decel3_lead1p5_20261024` completed 35.05 simulation
seconds, with truth-confirmed rejoin at 10.70 s, minimum sampled body gap
0.4173 m, no scored collisions/ego or lead boundary events, no MPC rejections
or SQP timeouts, and a passing clean-motion audit. The observed lead maximum
was 1.5 m/s. This is one position-00 run, not proof of GUI-load robustness or
success at all four positions; the failed GUI trial remains preserved above.

Acceptance uses the existing independent truth counter: at least one passage,
ending >1.20 m ahead, lateral error <0.10 m, heading error <0.15 rad and CRUISE
for 0.5 s. Zero scored collisions and zero ego/lead boundary events remain
required. MPC rejections are reported separately, not hidden by success.

```bash
cd Code/sim_ws
# One GUI run; change 00 to 25, 50 or 75 for another starting position.
TAKEOVER_PLANNING_BACKEND=upstream_sqp TAKEOVER_EGO_START_DELAY_S=0 \
  bash run_takeover_trial.sh fitzpatrick_position_00 '' 1 \
  "gui_$(date +%Y%m%d_%H%M%S)" true
# Four serial headless trials with a fixed shuffled order.
python3 tools/run_position_overtake_batch.py --prefix positions_MY_RUN
```

The batch defaults to one fresh process per location (an exploratory screen,
not a reliable success-rate estimate), shuffled with seed 20261024. All
locations occur once per repeat block; `--repeats 3` repeats the screen.
`results/takeover_position_batches/<prefix>/schedule.json` is saved before
execution; `results.json` records failures as well as actual entry locations.
Raw CSV, configuration, source hashes, summaries and separate English plots
remain under `results/takeover/<scenario>/<run_id>/`.

## RViz reference route and optional staggered start

The default RViz configuration displays a yellow `Track Centerline (Reference
Route)` Path on `/takeover/track_centerline`, in `map`. This is the loaded
fixed reference route, not a newly computed geometric medial axis. The full
route is transient-local, so RViz can subscribe after the trial starts.

The cyan `Overtaking Plan (Local Reference)` displays `/takeover/reference_path`:
the current local planner reference used during following, overtaking and
rejoining. It updates with planning and keeps only the latest path (no historical
trail). This is a planned geometric path, not the actual driven trajectory or a
guarantee that MPC/safety will execute it. Both displays use fixed frame `map`.

Takeover launch now defaults to zero delay. If a positive delay is requested, the lead
controller emits a single latched simulation timestamp when its own measured
speed first exceeds 0.05 m/s. Ego holds zero commands until the requested number
of simulation seconds after that event, then normal sensor/safety/planning checks apply.
The event contains no target position or velocity and does not update the
tracker, predictor or MPC target state. No-lead trials skip the gate.

```bash
cd Code/sim_ws
TAKEOVER_PLANNING_BACKEND=upstream_sqp bash run_takeover_trial.sh \
  fitzpatrick_position_00 '' 1 "gui_$(date +%Y%m%d_%H%M%S)" true
```

`TAKEOVER_EGO_START_DELAY_S=2` opts into the former delayed-start setup;
the default is `0.0`, recorded in every run's `config.json`. Existing historical
two-pass results used the earlier timing and must not be presented as tests of
the delayed-start scenario. Use Ctrl+C to stop a GUI trial.

Validation: `centerline_delay_20261024_check` (15 simulation seconds) and
`centerline_delay_20261024_final` (8 seconds) built and ran. A separate read-only
ROS subscriber received all 899 reference poses in `map`; both runs reported
lead start 0.10 s, first positive ego command 2.15 s, and zero ego commands
through the hold interval (2.05 s observed delay, one control tick quantization).
The graph audit confirmed opponent odometry subscribers remained the opponent
controller and evaluator only. RViz YAML/topic/QoS were checked; the desktop
window was not visually inspected. These short startup checks intentionally do
not satisfy the scenario's two-overtake acceptance: both retained `passed=false`,
zero overtakes, zero scored collision/boundary events. Full delayed-start
two-pass regression is still outstanding.

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

# Adaptive offsets: two passes in ONE continuous Fitzpatrick trial
./run_takeover_trial.sh fitzpatrick_double_overtake '' 1 double_demo false
# Same acceptance with a smoothly laterally drifting lead
./run_takeover_trial.sh fitzpatrick_double_lateral '' 1 drift_demo false
# Sequential repetitions of both new scenarios, followed by strict audit
bash run_double_overtake_batch.sh 3 double_batch

# The same scenario with RViz
./run_takeover_trial.sh constant_speed "" 1 demo_gui true

# Five scenarios, three independent seeds each, then aggregate
./run_takeover_batch.sh 3 my_batch
```

Arguments are `scenario [duration_override] [seed] [run_id] [gui]`. A run ID
is immutable: the runner refuses to overwrite an existing directory. Outputs
are saved under `Code/sim_ws/results/takeover/<scenario>/<run_id>/`:

The new double-pass scenarios are opt-in; existing scenarios retain their
fixed-offset defaults. See [adaptive two-pass implementation and acceptance](ADAPTIVE_DOUBLE_OVERTAKE.md).

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
# 3.0 / 1.8 m/s Gym model profile

The `fitzpatrick_double_3p0_1p8` and `_lateral` scenarios explicitly select
`controller_model=official_gym_native`. The original `takeover_controller_node` still
uses the real-v8 model; `takeover_gym_controller_node` uses the same C++ control
source, optimization dimensions, costs and constraint forms with separate
generated Gym vehicle parameters. The Gym profile uses its native 0.4189 rad
steering bound; the real-v8 default remains 0.269 rad. The hard lateral
acceleration bound remains 1.65 m/s² in both. Do not label the Gym model as a
real-v8 validation.

On the tested Ubuntu 22.04 x86-64 laptop, install the optional code-generation
dependencies from the repository root once:

```bash
python3 -m venv --system-site-packages .deps/mpc_codegen_env
.deps/mpc_codegen_env/bin/python -m pip install casadi==3.6.6 future-fstrings==1.2.0 'jinja2>=3,<4'
curl --fail --location --connect-timeout 10 --max-time 45 https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux --output .deps/acados-v0.4.0/bin/t_renderer
chmod +x .deps/acados-v0.4.0/bin/t_renderer
```

The tested renderer SHA-256 is
`390063f34a8e13620564b4a136012270168e1421dd7920a747048749e1d99718`.
This renderer binary is for Linux x86-64; it is not an Apple Silicon/Jetson
installation command. The existing laptop setup remains required for ROS/acados.

The trial runner generates the optional model under
`Code/sim_ws/build/takeover_sim/gym_model/` before compiling. Shipped real-v8
generated sources are not overwritten. Launch a single headless trial from
`Code/sim_ws`:

```bash
TAKEOVER_PLANNING_BACKEND=upstream_sqp ./run_takeover_trial.sh fitzpatrick_double_3p0_1p8 '' 1 NEW_RUN_ID false
```

Change `false` to `true` for the GUI. Batch profile `3p0_1p8` runs constant and
lateral leads serially. New trials also save a live `graph_audit.json` verifying
LiDAR tracking, opponent-truth isolation and final-command ownership.
Check the [latest native-profile results](../results/fitzpatrick_native_20261023/README.md)
before interpreting any scenario as accepted.

Gym uses a 0.3302 m wheelbase, while the real-v8 nominal model uses 0.25 m.
The Gym model also uses nominal static-load tire stiffness, a first-order
approximation of the steering delay, and separate acceleration/braking response
times derived from Gym's PID. Gym remains its original nonlinear simulator;
load transfer, bang-bang steering and its delay buffer are not exactly reproduced
by the MPC. Lateral odometry is a zero placeholder in this Gym version, so only
the measured yaw-rate input is enabled; lateral velocity remains model-estimated.

The new sensor profile preserves front-only initial acquisition. A recently
measured identity may use the existing approximately 270-degree scan and up to
300 side-panel beams. The extra rear mask expires after 0.3 s without a
measurement; ordinary tracking loss and command safety remain active.

The native Gym trial profile now opts into a physics-step `/clock`: 0.01 s per
actual Gym step, fresh scan/odometry publication at 50 Hz, and ROS-time timers
for tracking, planning/control, lead control, arbitration and evaluation. An OS
scheduling pause no longer masquerades as target motion. The physics driver
uses a steady wall timer to avoid circular clock startup, stepping zero inputs
before control timers start. Trials run for the configured **simulation**
duration, with a bounded wall supervision budget of `4 * duration + 60 s`.
Older/default profiles retain their wall-clock mode. Plant dynamics, body sizes,
freshness thresholds and collision checks are not changed by selecting a clock.
Solver/deadline computation measurements remain wall-clock measurements; slow
simulation is not evidence of hardware real-time performance.
