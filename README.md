# F1TENTH Takeover

Fixed-route navigation and static-obstacle avoidance on a real F1TENTH vehicle,
plus a laptop-tested dynamic-obstacle overtaking simulation.

**Latest physical checkpoint — 2026-11-08, 381 2.0:** both cars moved without reported contact; v5 startup gating worked, but MPC status 4 stopped the run before route completion. Physical dynamic overtaking is not yet accepted. [Current source, settings and evidence](docs/VEHICLE_PROGRESS_20261108.md).

**Historical completed physical result: Dynamic MPC v8, Fitzpatrick, 2026-09-27.** One complete lap with two static obstacles; the operator reported no contact, minimal oscillation, and a natural stop. This is one successful run, not a repeated-trial reliability claim.

**Laptop update, 2026-11-06:** checked MPC rollback/re-solve and ahead-only
same-side control-feedback recovery are implemented in the simulation adapter.
The current paired screen and limitations are recorded in
[checked execution/recovery](docs/CONTROL_RECOVERY_20261106.md).
This is not full dynamic-overtaking acceptance or physical deployment approval.

**Earlier laptop update, 2026-11-05:** an opt-in speed-proposal/retiming experiment
improves some passing trials but is not a uniformly faster, stable release.
[Speed experiment and preserved failures](docs/SPEED_PROFILE_20261105.md) ·
[Why Frenet + spline + MPC, and alternatives](docs/PLANNING_CONTROL_TRADEOFFS.md) ·
[Earlier recovery evidence](docs/STABILITY_20261104.md).

## Onboard obstacle-avoidance demo

[![Onboard MPC obstacle-avoidance demo](media/MPC_Avoidance_preview.gif)](media/MPC_Avoidance.mp4)

[Watch or download the full video](media/MPC_Avoidance.mp4) — 32.4 s, with original audio. The silent looping preview shows a 12-second excerpt; the MP4 preserves the complete recording.

## Simulation overtaking demo

[![Two-vehicle overtaking in simulation](media/Simulation_Overtaking_preview.gif)](media/Simulation_Overtaking.mp4)

[Watch or download the edited demo](media/Simulation_Overtaking.mp4) - approximately 18 s, original speed, no audio. The continuous RViz excerpt shows the two vehicles and the evolving path during an overtaking maneuver. Desktop/terminal footage, the battery notification, and RViz tool panels have been removed. The looping preview shows a 12-second excerpt at original speed.

This simulation recording is separate from the physical v8 result below; it does not establish real-vehicle overtaking performance. [Editing details](media/README.md#simulation-overtaking)

## Field result

| Measure | Result |
|---|---:|
| Route length | 44.73 m |
| Time to completion | 27.98 s |
| Final position error | 7.45 cm |
| RMS error relative to the selected local path | 3.43 cm |
| Maximum commanded speed | 2.0 m/s |
| Active invalid controller messages | 0 |

![Reference path, measured trajectory and obstacles](results/fitzpatrick_v8_20260927/trajectory_overview_en.png)

[Obstacle A](results/fitzpatrick_v8_20260927/obstacle_A_trajectory_en.png) · [Obstacle B](results/fitzpatrick_v8_20260927/obstacle_B_trajectory_en.png) · [Recorded result](results/fitzpatrick_v8_20260927/analysis_summary.json)

## How it works

1. **Localize:** GPU-assisted particle filtering matches LiDAR against a saved map and uses wheel odometry.
2. **Detect:** filter known wall returns, cluster additional returns, and confirm obstacles across observations.
3. **Plan:** a 5 Hz Frenet planner samples smooth quintic lateral detours within an approximately 3.2 m local window. Collision, curvature and continuity checks select the path.
4. **Control:** 20 Hz dynamic MPC jointly optimizes acceleration and steering over 24 steps / 1.2 s, including lateral vehicle dynamics and actuator lag.
5. **Execute:** independent command supervision, freshness checks and the L1 deadman gate vehicle motion.

v8 carries lateral velocity/yaw-rate estimates continuously through the dynamic model. The predicted lateral-acceleration bound is 1.65 m/s²; curvature speed planning still uses 1.20 m/s². These lateral states are model estimates, not independent IMU measurements. Dynamic-obstacle prediction and overtaking are implemented in the separate laptop simulation and experimental 381 v5 candidate; they are not part of the commissioned v8 release. The simulation demo does not establish physical overtaking acceptance.

## Where to start

- [Current status and version boundaries](docs/STATUS.md)
- [Vehicle commands](docs/OPERATIONS.md)
- [Source/build scope and dependencies](docs/BUILD.md)
- [Curated real-vehicle source snapshot](real_vehicle/)
- [Field evidence and English trajectory figures](results/fitzpatrick_v8_20260927/)
- [Risk-aware following simulation reference](Code/sim_ws/BASELINE_EVALUATION.md)
- [Laptop dynamic-overtaking setup and operation](docs/LAPTOP_TAKEOVER_SIM.md)
- [Current 15-trial overtaking report](results/takeover_sim_20261007/REPORT.md)
- [Historical materials and attribution](docs/HISTORY.md)

## Offline native checks

On Linux with a C++14 compiler, Eigen3, jsoncpp, Python dependencies in `tools/requirements-offline.txt`, and **acados v0.4.0** installed:

```bash
export ACADOS_INSTALL_DIR=/path/to/acados_install
bash tools/build_native.sh
bash tools/test_native.sh
```

These commands build a non-actuating probe and run model tests. They do not launch ROS drivers or move the car. The Git source export is not an already commissioned vehicle installation; see [BUILD.md](docs/BUILD.md).

## Repository scope

| Directory | Purpose |
|---|---|
| `real_vehicle/` | Historical v8 and current 381 v5 source snapshots, maps and routes |
| `deployment/381_2_0/` | Current compact overtaking sources, diagnostics and selected field evidence |
| `oppVehicle/` | ESP32 lead-car control; two independent PS4 controllers |
| `results/` | Selected successful v8 field evidence and English trajectory figures |
| `media/` | Onboard obstacle-avoidance and simulation-overtaking videos with animated README previews |
| `tools/` | Non-actuating native build and model tests |
| `docs/` | Status, operation, build, provenance and cleanup records |
| `Code/sim_ws/` | Independent risk-aware following simulation and shadow-mode reference |
| `Code/f1tenth_stack/` | Hardware stack reference used by the simulation deployment documentation |

The `Code/` reference implementation is separate from the validated v8 Frenet/dynamic-MPC vehicle release. Its simulation/shadow status does not establish physical acceptance.

Compiled binaries, build/install caches and raw ROS logs are excluded. Original field logs and working vehicle releases remain on the Jetson. Obsolete photos/videos, old maps/trajectories, vision/raceline experiments, duplicate workspaces and hardware-design attachments have been removed from the current tree. [HISTORY.md](docs/HISTORY.md) links to the preserved prior revision; Git history has not been rewritten, so historical large objects still contribute to a full clone's size.

Retained source/package notices and acados, BLASFEO and HPIPM notices remain intact. No new blanket license is applied to third-party or legacy material.
