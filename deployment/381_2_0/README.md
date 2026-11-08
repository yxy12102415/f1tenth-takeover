# 381 2.0 — current checkpoint (2026-11-08)

**Current: compact v5, experimental; physical route completion is NOT accepted.**
Both vehicles moved without reported contact. The latest log stopped with
`joint_mpc_solver_4` after about 2.98 s, 4.78 m before the endpoint. The v5 L1
startup fix worked; seven gate checks, an offline window and zero/localization
checks passed before motion. These are separate from route acceptance.

- [Current progress, geometry and limitations](../../docs/VEHICLE_PROGRESS_20261108.md)
- [Exact v5 source snapshot and non-actuating build](../../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v5/EXPORT.md)
- [Latest physical result](reports/compact_v5_physical_20261108_121040.json)
- [v5 gate/protocol](COMPACT_V5_TRIAL.md), [v4 planner/MPC changes](COMPACT_V4_TRIAL.md)
- [Selected report inventory](reports/INDEX.md)

Map: 381 2.0; 5.95 m straight reference, original start; fixed 0.55 m right
pass; command cap 1.85 m/s, acceleration 2 m/s², moving floor 0.5 m/s.
Lead uses its own PS4 at 1150 µs. F1 L1 does not stop the lead.
STOP remains zero-current coast, not active braking.

On the existing commissioned Jetson only: `cd ~/f1tenth_real_migration_ws_20260829`
then `./381_2_0_compact_overtake.sh run`. No new test is requested by this
checkpoint. `prepare_compact_release.py` below creates historical v1, not v5.

## Historical preparation notes — superseded configurations

The original dated notes below preserve the investigation history. Their
“no physical drive” statements and old dimensions/speeds apply to those older
configurations. Some large/local reports are excluded; see the selected index.

# 381 2.0 overtaking preparation

Status: **independent compact C++ candidate under software validation; physical
overtaking is not commissioned. No physical drive was started.**

**Latest decision:** v1 was withdrawn before motion. A target-loss plant test
showed that zero-current coasting from 1.25 m/s could leave the mapped free
space. Its physical-admission config was disabled; the old manifest is
intentionally invalid, not reissued. Do not run this candidate physically.
The subsequent 1160 us lead-car observation reached approximately 0.64 m/s
(0.5 s regression), moving 1.39 m forward and 0.26 m laterally. It does not
satisfy the slow straight-lead assumptions below. The 1140 us trial verified
the L1 command but did not start on the ground according to the operator.
The 1150 us intermediate firmware has now been reflashed, with flash hash,
boot version and stopped output verified. One 1150 us ground trial now
observed about 0.48 m/s (0.5 s fit), 1.43 m forward travel and about 0.02 m
lateral shift of visible returns; operator reports holding L1 about 3 seconds.
This exceeds the old 0.35 m/s lead bound and does not commission overtaking. See
`reports/lead_speed_1160_continuous_20261107/speed_summary.json` and
`reports/live_readiness_20261107.json` for the current blockers. Lower-speed
F1 rollouts are exploratory; none is commissioned for two-car motion.

## 1150 us lead replay (2026-11-07)

Native tracking rollouts at F1 caps 1.25, 1.5 and 1.85 m/s were replayed against
the measured 0.483 m/s lead and illustrative 0.50/0.55 m/s constant speeds. All
three violate buffered body clearance at the measured speed and all fail the
map-only emergency-coast screen. At 0.50 m/s lead and 1.25 m/s F1 cap, the model
shows unbuffered body overlap during rejoining. This evaluates the existing
path only, not every possible route. No physical F1 run was started. Evidence:
`reports/lead_1150_overtake_feasibility/assessment.json`; reproducible replay:
`python3 deployment/381_2_0/assess_1150_overtake.py` from repository root.

## Current compact straight candidate (2026-11-07)

The operator reports the ESP32 lead vehicle is approximately **30 × 18 cm**.
The candidate retains the original start and 5.6 m straight endpoint. Its fixed
quintic maneuver moves 0.60 m to the right, holds that offset, then rejoins the
straight. The commanded speed cap is 1.25 m/s (native cap 1.40 minus existing
0.15 m/s headroom); existing launch ramp, minimum drivable speed and terminal
coast remain in the native dynamic MPC. The nominal model stops within 1.4 cm
of the endpoint. This is model evidence, not a physical test result.

The required initial lead-center distance is **1.60 ± 0.05 m**, in the straight
route frame. The controlled scenario assumes forward-only lead motion at
0–0.35 m/s, approximately constant heading, and no more than 5 cm lateral
course deviation. The earlier short speed recording does **not** confirm that
1160 us maintains this speed range. Two independent PS4 controllers are used;
releasing the F1 L1 does not stop the ESP32 lead car.

`compact_overtake.hpp` bounds the lead center from observed point extents and
the reported size with allowance. It does not treat the visible 8–10 cm laser
cluster as the full vehicle footprint. Admission checks the full nominal MPC
rollout against the reachable lead region. Each controller tick also checks
the actual MPC prediction, interpolated full body/map clearance, and source
timestamps before publishing. Ambiguous/lost targets reject execution; failure
after enable is latched. The finite terminal distance is calculated from the
candidate path. This is an experimental **fixed single-candidate straight pass**,
not the general simulation SQP planner or a completed port of that planner.

The separate workspace candidate is
`candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261107_v1`.
The prior physical launchers/releases were not changed. The new C++ runtime
explicitly refuses physical ROS domain 85 until commissioning is complete;
it is not frozen and there is no new motion launcher.

Reproducible sources are `compact_node.patch`, `compact_overtake.hpp`,
`assets/compact_candidate.json`, and `prepare_compact_release.py`. The latter
checks the base node hash and refuses an existing destination. It does not
build, freeze or start anything.

Evidence and current acceptance status are in
`reports/live_readiness_20261107.json`. Geometry/plant checks and native
prediction replays use synthetic opponent observations. Physical LiDAR
association, actual lead speed, localization, loaded timing with all physical
nodes, and the complete motion-supervisor chain still require acceptance.

The older sections below preserve the initial generic-planner investigation;
their original 5.845 m route and 0.5 m/s lead assumption are not the compact
candidate's configuration.

## Selected scenario

Keep the original fixed start and the original 5.845 m open route. The longer
13.42 m map-derived candidate is an unselected backup. The user requested a
first straight/low-curvature overtake of another car moving in the same direction.

Screen with ego speed cap 2.0 m/s and opponent speed 0.5 m/s. The user chose
0.5 m/s as an assumption; the opponent's stable speed and dimensions remain
unmeasured. Use existing real-model acceleration +1.0 m/s² and deceleration
−0.8 m/s² for preparation. Initial reference-point separation is provisionally
1.5 m. These are not deployed actuator settings.

## Evidence and limits

* `reports/build.log`: native Foxy/aarch64 real-model controller build succeeded.
  The Gym model is not used. `measured_yaw_feedback=false` selects the dynamic
  model observer for the vehicle candidate; simulation defaults are preserved.
* `reports/sqp_runtime_warmed_checks.log`: all 10 upstream SQP numerical checks
  passed after representative startup warmup. Earlier cold runs had LP timeout
  failures, retained in `offline_checks_first_attempt.log` and
  `sqp_idle_recheck.log`. Runtime LP and socket deadlines were not increased.
  This does not establish Jetson loaded runtime deadline acceptance.
* `reports/shadow_smoke.json`: domain 187 isolated startup produced 222 zero
  proposals and no publishers on the checked physical command topics. No
  hardware input, vehicle motion, target tracking, or trajectory was tested.
* `reports/short_route_screen.json`: 18 optimistic native Frenet admission
  cases stayed in PREPARE_OVERTAKE. This screen has no SQP callback or MPC
  preview, so it is not an end-to-end test of the latest planner.
* `reports/short_route_longitudinal.json`: an ideal one-dimensional
  accelerate/cruise/brake profile at ego cap 2.0 m/s, opponent 0.5 m/s can
  stop 1.2 m ahead within 5.10 m. An initial 0.6 s stationary delay increases
  this to 5.50 m. Braking overlaps the pass. Neither result includes lateral
  geometry, steering lag, curvature limits, tracking margins or actual coast
  behavior. It is not a feasible collision-free trajectory certificate.

The original route is not ruled out by longitudinal distance alone. The current
planner, however, checks a nominal entry corridor of 2.6 + 2.8 +
2.0 × (1.5 + 1.2) / (2.0 − 0.5) = **9.0 m**. The old open route ends before
that corridor. Merely raising cruise speed does not fix this mismatch.

## Commands

Run from `/home/f1tenth/f1tenth-takeover-381-overtaking`:

```bash
# Reproduce the longitudinal screen. No ROS or hardware access.
python3 deployment/381_2_0/check_short_route.py

# Build the isolated real-model candidate. No vehicle drivers are launched.
bash deployment/381_2_0/build.sh

# Read-only shadow, if existing localization/odometry/LiDAR are already running.
bash deployment/381_2_0/shadow.sh --duration 30
```

Shadow publishes proposed commands only to `/overtake_381/proposed_drive`.
It does not connect that topic to VESC or the physical command supervisor.
Without valid sensor streams, zero output is expected and is not a successful
overtaking test. Existing PP/PID/DMPC physical launchers remain unchanged.

## Work required before motion

1. Evaluate a compact, time-indexed maneuver on this map with the actual car
   footprints, steering/acceleration limits, full SQP and MPC preview. Keep
   body clearance and live feasibility checks; do not remove entry constraints
   merely to force a short-route maneuver to start.
2. Integrate an open-route terminal profile and stop behavior. The simulation
   node currently uses `remaining = 100.0`; the existing simulated endpoint
   behavior is not accepted for this physical open route.
3. Connect through the physical deadman, command guard and fault supervisor;
   validate stale inputs, target loss and loaded Jetson timing without motion.
4. Measure opponent size/stable speed, check localization at the fixed start,
   and establish coordinated stopping so the following opponent does not
   catch the stopped ego at the route end.

## Endpoint extension screen (2026-11-07)

The user subsequently authorized checking both endpoint extensions. Run
`python3 deployment/381_2_0/extend_route_ends.py` to reproduce the map figure,
CSV and `reports/endpoint_extensions.json`. The original middle section is
retained exactly. The proposed extensions add 0.21 m before the start and
0.89 m after the end, making a 6.945 m candidate. These lengths retain an
extra 0.20 m from each sampled extension limit under the stated map/footprint
criteria. Gray unknown map cells are explicitly blocked.

The candidate is saved as `assets/381_2_0_extended_ends_candidate.csv` and shown
in `reports/381_2_0_endpoint_extensions.png`. It has no commissioned speed
profile and is not selected in `candidate_config.json` or any physical launcher.
Extending the route alone does not satisfy the current nominal 9 m admission
budget or certify a complete two-car maneuver.

Upstream source: `codex/predictive-spliner-20261022` at
`97bdd9cd0159534065c8439d5acc9284eec134ec`. Local preparation branch:
`vehicle/381-overtaking-20261107`. Frozen physical releases were not modified.

## Latest live-start audit (2026-11-07, after host reboot)

The lead PS4 fixed-throttle mode is flashed and button-verified at 1160 us;
the operator confirms reduced-throttle operation. The stationary LiDAR capture
was interrupted by a host reboot and does not establish sustained lead speed.
No vehicle motion controller was started during the latest start request.

`reports/straight_slow_lead_entry_screen.json` screens the actual 5.6 m straight
asset over 36 assumed speed/gap combinations (lead 0.1–0.5 m/s). None enters
OVERTAKE. This is the native Frenet screen without an SQP callback/MPC preview,
not a complete feasibility proof. Real actuation integration, compact maneuver
validation, and open-route terminal control remain incomplete. See
`reports/live_readiness_20261107.json`.
