# 381 2.0 physical progress — 2026-11-08

**Checkpoint, not overtaking acceptance.** The latest v5 trial moved both cars
without reported contact. The operator perceived passing and rejoining, but
the recorded controller stopped about 2.98 s after ARM with
`joint_mpc_solver_4`, still 4.78 m from the endpoint. `passed=false` and
`route_completed=false` remain the recorded outcome. Final wheel speed was
zero and the driver exited. Do not count this as a completed route.

## Current implementation

LiDAR/map localization → measured lead-body association and bounded forward
prediction → one fixed right-side quintic passing path → C++ dynamic MPC
joint acceleration/steering → supervised execution with an L1 deadman.
This compact vehicle candidate is separate from the general laptop SQP
overtaking planner; it is not a full port of that planner.

| Setting | Current v5 value |
|---|---|
| Map / route | 381 2.0 / 5.95 m open straight, original start retained |
| Start (map x, y, yaw) | −0.00477 m, 0.00983 m, 0.01885 rad |
| Right offset / entry / hold / return | 0.55 / 1.90 / 2.15 / 1.70 m |
| Reference speed cap / command cap | 2.0 / 1.85 m/s, with 0.15 m/s headroom |
| Acceleration / moving-command floor | 2.0 m/s² / 0.5 m/s; start/end zero |
| Lead model | 30 × 18 cm body; forward speed bounded to 0–0.55 m/s |
| Initial lead center distance | 1.80 ± 0.05 m; about 1.38 m LiDAR-to-rear |
| Lead firmware | Independent PS4 L1, 1150 µs throttle, 1000 µs coast |

The 1150 µs lead observation reached approximately 0.483 m/s using 0.5-second
LiDAR regression windows. Fixed PWM does not guarantee that speed for other
batteries or loads. Both vehicle dimensions include the operator-reported foam.

## Changes retained

- v3: bounded lateral association and full-body occupied-region checks.
- v4: smaller passing offset, longer return, shifted MPC controls rolled from
  the current state with the exact generated dynamics, independently checked
  dynamics defects, and terminal-zero handoff. Equality/inequality acceptance
  remains 0.001; refinement allows up to eight RTI iterations under 45 ms.
- v5: startup L1 qualification requires a continuous 1 s hold and 0.5 s MPC
  readiness. A release before ARM resets qualification and permits retry;
  a release or joystick timeout after ARM latches STOP immediately.

Seven non-actuating ROS gate callback checks passed. Matching v5 offline-window
and physical zero/localization checks passed before the latest motion attempt.
Those checks establish their own scope, not successful physical overtaking.
The latest failed solve returned status 4 in 6.74 ms; it was not a compute-time
deadline rejection. The earlier failure replay can pass offline on both old
and revised MPC code, so it does not establish a physical solver fix.

## Operation and limits

The commissioned Jetson entry point is
`~/f1tenth_real_migration_ws_20260829/381_2_0_compact_overtake.sh run`.
No new drive is requested by this checkpoint. The Git source export is not
an installed or commissioned replacement for that workspace.

Two independent controllers are used: F1 address ending B4 and lead ending
A3. Keep both L1 released until READY; hold F1 L1 continuously, then hold
lead L1 for about 3 s once F1 begins rolling. Release lead L1 and keep F1 L1
held until stopping. Each L1 controls only its own car.

STOP is zero-current coasting with centered steering, not calibrated active
braking. Target-loss plant tests confirm stop signaling, not collision-free
physical coasting. Continuous lead driving, general target motion, repeated
physical completion, and fault stopping distances are not accepted.

## Evidence and next work

- [Latest physical outcome, including separate operator observation](../deployment/381_2_0/reports/compact_v5_physical_20261108_121040.json)
- [Input-gate tests](../deployment/381_2_0/reports/input_gate_v5_contract.json)
- [Gate readiness and recorded button sequence](../deployment/381_2_0/reports/input_gate_v5_readiness.json)
- [Release source and original source hashes](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v5/EXPORT.md)
- [Current deployment index](../deployment/381_2_0/README.md)

The operator suspects that surrounding obstacles and limited track space make
this site unsuitable for early overtaking trials. That is a hypothesis; the
latest recorded stop was an MPC solve failure. Next: diagnose model-state and
constraint feasibility at that recorded input, then use a simpler, wider
straight test area before expanding the scenario. Do not resolve the failure
by merely disabling collision checks or counting an early stop as completion.
