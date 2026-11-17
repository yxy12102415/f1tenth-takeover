# 381 generic occupancy avoidance: adaptive corridor v2

New C++ local planner on the original 5.95m open 381 2.0 reference route.
PID/PI and joint dynamic MPC share the same perception and planner. Historic
compact successes do not qualify this release; see PREPARATION_STATUS.json.

## Behavior

1. Process every non-map LiDAR cluster supported by at least three adjacent
   returns, including separate wheel clusters. No designated opponent, vehicle-type recognition, fixed
   opponent-size acceptance gate, or mandatory initial target is required.
2. Join geometrically adjacent compatible components. Learn occupied body extent
   across partial views, propagate shape with estimated motion and associate new
   raw IDs geometrically. Current pose support always requires real new returns.
3. If a confirmed component is unobserved, retain its actual last detection timestamp and
   expand predicted occupancy. Remove it only with positive free-space evidence
   or associate its occupied shape into a fresh compatible component. Uncovered
   independent obstacles remain; loss of an ID never directly commands STOP.
4. Project measured/predicted occupied regions into Frenet free intervals.
   Compute the forbidden lateral interval using separating-axis inequalities
   for the complete oriented ego body. First try smooth curves whose offsets
   and trailing release come from these obstacle edges; choose the nearest
   feasible side and keep the other side available when blocked. If these seeds
   do not fit, solve a convex smoothing QP and clamped cubic spline, refining the
   intervals with the candidate heading. Passing offset is a continuous
   scene-dependent result, not a fixed .05m lattice. Revalidate and hold a feasible selected path; keep
   the original endpoint. Only a fresh full-body swept collision check admits a
   solution, including interpolation between stations and steering curvature.
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

Current compact commissioning: PID cap0.8m/s, dynamic MPC cap0.7m/s, reference-line lateral limit0.58m,
acceleration<=2m/s², running floor .5m/s, start/end0. The .58m reference-line
limit is not the body gap to the front vehicle. Conservative hidden extent,
measurement uncertainty, yaw and tracking margin still affect the achievable gap. First physical scope is a stationary obstacle. Software moving-target
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

## 2026-11-11 early-stop correction

Return position uses only objects intersecting the nominal full-body corridor. Local obstacle checking covers at least 3 m and the conservative complete-stop distance plus transport/replanning allowance. Map validation still covers the full candidate route. Every observed occupancy remains in controller prediction and complete-coast collision checks. Distant occupancies are replanned on approach, so this is not a promise that a currently uncertain endpoint will be traversable.

Candidates first require a preferred 0.14 m body gap. If none exists, a second search uses the existing 0.08 m hard body gap; controller/coast and sensor interlocks are unchanged. Recorded 16:17 early-stop occupancy is retained as a regression fixture. Software replay is not a completed physical test.

The exact recorded failure remains correctly BLOCKED with the close-start tentative return present; no claim of complete failure-replay resolution is made. Clear/reobserve the starting area before another physical attempt. The software regression explicitly preserves that rejection.

The planner runs on a 200 ms start-to-start schedule, without an additional 200 ms sleep after each calculation. Source timestamps and all 0.5 s stale-data guards are unchanged. This corrects a physical zero-check rejection (no motion) caused by a valid path aging while a replacement lattice search plus fixed sleep ran.

Stationary confirmed occupancy shapes no longer integrate small estimated velocities into their retained bounds. Shape transport and old-component reassociation use velocity displacement only after consistent observed motion has latched. Actual measured points continue to union into the shape, and moving-history prediction is unchanged. A 120-frame stationary jitter regression verifies bounded extent.

An incumbent path is revalidated immediately using current observations, the unchanged 0.08 m hard body gap and physical full-body map check. Preferred 0.14 m obstacle / 0.06 m extra map reserve applies to searching for a new path, not to repeatedly discarding an otherwise safe incumbent. This avoids a full lattice search on every minor preferred-margin fluctuation. All current occupancy, prediction and coast checks remain required; no stale-path fallback is permitted.

Raw-scan clustering again requires at least three adjacent, spatially connected returns, matching the original vehicle pipeline. Single/two-beam spikes remain in the untouched raw scan but are not confirmed obstacle regions. This trades isolated-return sensitivity for robustness; very sparse targets need sufficient spatial support and are outside the present vehicle/opaque-object commissioning scope. The ego mask and all hard collision/sensor leases are unchanged. Supervisor snapshots are retained at 5 Hz idle / 20 Hz moving, with every state transition retained; live safety status, health samples and source-age checks still process every callback. GC pauses are measured, not disabled.

The bounded supervisor defers cyclic GC until all command/hardware children have stopped; normal reference counting remains active and large trace snapshots are sampled. The window publishes STOP immediately on a newly latched permission withdrawal, health rejection or L1 release, while preserving the periodic heartbeat and existing 0.5 s leases / 550 ms offline fault budget.

2026-11-12: after a lattice search, the publisher processes the newest scan and revalidates the selected path against fresh occupancies and pose before attaching new source timestamps. This refresh never runs a second search and never extends a stale path lease without validation. A new obstacle invalidates the path immediately; all 0.5 s source-age guards remain unchanged.

Compact stationary commissioning after the 2026-11-12 wide-detour run: cap 0.8 m/s and lateral offset <=0.55 m, read from compact_config.json by planner and controller. Hard object gap 0.08 m, map footprint and source leases are unchanged. Every live control proposal additionally validates a conservative zero-current coast against full-body map clearance (100 ms delay, 0.4 m/s² deceleration, 250 ms steering centering). This additional map guard does not replace moving-occupancy prediction checks.

Return station correction: the last nominal full-body overlap already includes vehicle length. The generic wrapper compensates Planner.make() legacy center padding and retains only 0.10 m release distance, followed by complete candidate collision validation.

## 2026-11-13 adaptive corridor

The fixed-offset detour search was replaced by obstacle-relative free intervals,
fast smooth envelope-derived candidates, and a convex smoothing solver for
geometry that cannot use a simple parallel pass. Stop searching once a suitable
validated smooth seed exists, avoiding an unnecessary expensive global comparison. Smoothness minimizes lateral displacement,
slope, curvature and curvature change. Actual vehicle heading is used to refine
forbidden intervals; every iterate still needs independent swept-body validation.
Full vehicle rear/front extent constrains the return to the reference rather than
a fixed plateau or duplicated return padding. Feasible incumbents remain stable
until fresh observation invalidates them. No object-ID loss timer was reintroduced.

The model is receding-horizon: CV prediction is applied over the same1.2s time
horizon as existing checks; all current geometry is checked over the full local
stopping horizon. Moving-object software cases do not qualify real overtaking.
The first physical scope remains a stationary obstacle. Both homotopies choose
one passing side for nominal-route blockers; this is not a general optimal
weaving planner for arbitrary groups of moving objects.

Current evidence is in reports/adaptive_corridor_20261113/. Historical lattice
and compact-profile reports remain for comparison and do not qualify new binaries.
Physical reset and a matching5s zero/localization admission remain required after
software validation. The wrapper reuses matching checks; no repeated long suite
is required on each launch.

A very close lateral obstruction at the start may still be rejected: a nominally
clear centre position does not establish clearance for the rear swing during
steering. The strengthened short-scale curvature and swept-body checks can reject
a manoeuvre accepted by the old coarse average-curvature check. No unsafe held
path is reused to hide this condition.

Final profile reserves0.07m for newly revealed surfaces/tracking in envelope seeds, with a0.58m reference-offset limit. This replaces the unsuccessful0.55m integration profile; hard body/map gaps and source-age leases remain unchanged.

Controller-specific caps come from generic_tracker_caps_mps in the single root
config. freeze.py regenerates the derived MPC profile; both the real launcher
and ROS acceptance tests select it through controller_assets.py and reject stale
profile copies. MPC uses0.7m/s because0.8m/s failed the map coast check on this
tight route. The check is retained. First physical scope is stationary only;
the0.15m/s moving-obstacle diagnostic remains unqualified.

## 2026-11-15 non-map corrections

`speed_geometry.hpp` now owns the geometry speed-limit policy shared by the
planner and controller. Candidate arrival times start from measured speed and
integrate bounded acceleration (2m/s²), deceleration (0.55m/s²), curvature/map
speed limits, and the terminal target instead of dividing distance by the global
speed cap. This is a nominal reachable-speed schedule, not an exact MPC/actuator
rollout; independent live controller predictions remain mandatory.

Confirmed moving objects transport learned shape with the newest supported scan
translation. Their filtered velocity remains the prediction estimate, but no
longer leaves an ever-growing occupied trail solely because filtering lags the
actual translation. Static-object jitter suppression and unobserved-region
retention remain unchanged. Startup history before motion confirmation remains
conservative; this change does not erase all pre-confirmation extent.

Candidate search shares one200ms budget across preferred and hard-clearance
passes. The preferred pass reserves100ms for fallback. Checks occur between
bounded candidate computations, so this is a search-work budget, not a certified
hard real-time preemption deadline. Budget exhaustion is reported in score terms;
it never authorizes an invalid or stale path. Sensor/permit leases remain500ms.

Evidence: `reports/independent_fixes_20261115/`. Physical commissioning stays
closed; these corrections alone do not qualify complete moving overtaking.

### Stationary-scene commissioning (2026-11-15)

`generic_stationary_scene_v1=true` is restricted to `first_physical_scope=stationary_obstacle_only`. This profile assumes the opponent remains stationary with its ESC off and people outside the route. Measured and occluded occupied boxes remain in world coordinates; their velocity is zero for this explicit scene model. Clearance, map/coast checks, source leases and deadman conditions remain active. Historical wall-return velocity estimates do not extrapolate across the route indefinitely. This does not qualify moving-obstacle overtaking; set the scene option false and validate the moving model separately before a moving-opponent test.

The fixed-world scene flag is carried in each serialized occupancy. This scene retains measurement/observation-age padding and full collision checks, but applies no future obstacle process-motion growth. The constant-velocity scene retains its existing future motion uncertainty.
