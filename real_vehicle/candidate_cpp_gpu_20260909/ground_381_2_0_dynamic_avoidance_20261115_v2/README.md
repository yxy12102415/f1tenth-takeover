# 381 generic occupancy avoidance v1

New C++ local planner on the original 5.95m open 381 2.0 reference route.
PID/PI and joint dynamic MPC share the same perception and planner. Historic
compact successes do not qualify this release; see PREPARATION_STATUS.json.

## Behavior

1. Process every non-map LiDAR cluster, including single returns and separate
   wheel clusters. No designated opponent, vehicle-type recognition, fixed
   opponent-size acceptance gate, or mandatory initial target is required.
2. Join geometrically adjacent compatible components. Learn occupied body extent
   across partial views, propagate shape with estimated motion and associate new
   raw IDs geometrically. Current pose support always requires real new returns.
3. If a confirmed component is unobserved, retain its actual last detection timestamp and
   expand predicted occupancy. Remove it only with positive free-space evidence
   or associate its occupied shape into a fresh compatible component. Uncovered
   independent obstacles remain; loss of an ID never directly commands STOP.
4. Search both sides using Frenet quintic candidates, .45–.75m lateral offsets
   in .05m increments. Revalidate and hold a feasible selected path rather than
   reanchor the maneuver every scan. Preserve the original endpoint.
5. PID/PI or dynamic MPC controls steering and speed. Independently check predicted
   full-body sweeps and terminal coasting against map and obstacle occupancy.

Fresh components merge over gaps <=.35m with a combined span <=.65m; distinct
mature velocity intervals remain separate. New adjacent fragments inherit a
mature component's motion until independent velocity is observable. Sparse body
completion is primarily away from the source sensor, with a .36m minimum extent
and .02m measurement allowance. This is a geometric engineering model for this
trial, not a learned object classifier.

## Retained checks

- Full ego footprint .58x.31m, .05m map/body gap plus sampling coverage; MPC
  corridor and predicted sweep use the same footprint map reserve.
- .08m hard mutual body gap. Candidate selection additionally allows .30s
  transport age and .06m tracking reserve, including map reserve after .4m.
- Source timestamps are never refreshed solely by prediction. Missing occupancy
  grows within a bounded-speed reachable envelope; unknown space is not free.
- No stale or already colliding path fallback. Blocked geometry, invalid/stale
  sensor/localization data, controller rejection and permit loss still stop.
- Existing L1, .50s sensor/lease limits and zero/localization admission remain.

First commissioning: cap1.25m/s, acceleration<=2m/s², running floor .5m/s,
start/end0. First physical scope is a stationary obstacle. Software moving-target
cases use a straight .15m/s target; those are not moving-target field acceptance.

## Commands

From workspace, after qualification:

```bash
./381_2_0_generic_avoidance.sh pid run
./381_2_0_generic_avoidance.sh dynamic_mpc run
```

`run` reuses matching fresh evidence; otherwise it runs the targeted offline
window check and a fresh5s ground zero/localization check. Once READY, hold only
F1 L1 through automatic endpoint stopping; the lead stays stationary.

Non-actuating checks from this release directory:

```bash
python3 test_generic_avoidance.py
bash ../offline_env.sh python3 test_generic_controls.py
bash ../offline_env.sh env ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 python3 run_generic_ros_suite.py
```

ROS tests use actual C++ nodes, map raycasting, complete-time dynamic plant
integration, prompt command delivery and an independent full-body collision
check. Reports distinguish route completion from guarded STOP. Failed synthetic
iterations are preserved separately. Final binaries have source-hash build
provenance. No physical completion is claimed until a real attempt is recorded.

Legacy speed-floor entry points forward to the current2m/s² contract, including
running-floor, independent window and terminal zero-latch tests; old scripts are
archived under legacy_entrypoints. The saved synthetic MPC input replay is diagnostic:
historic low-reserve paths were rejected by final candidate selection.

Live zero calibration identified four self returns inside the declared lateral
body envelope. The self filter keeps the previous x range (-.27,.25)m and uses
strict |y|<.155m; boundary/exterior returns remain. Tentative detections (<4
actual observations) constrain current scans and a .50s hold; they cannot fill
persistent memory forever. Confirmed/associated components retain the lost-track
prediction policy. Captured real scans are replayed for bounded-memory testing
before another live zero check; replay pose is a PF estimate, not ground truth.

Motion state is independent of object category: >=4 supported observations with
no consistent motion form a static occupied region; three consistent measured
velocity updates latch moving history. Static regions keep their location while
unobserved; previously moving regions keep bounded dynamic prediction. Fresh
spatial evidence can associate static regions across raw-ID and time gaps.
Actual source timestamps remain unchanged without a real associated observation.
