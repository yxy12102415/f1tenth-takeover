# 381 2.0 v7: straight route from the original start

The route is a single 5.6 m straight. Start position AND heading are unchanged:
(-0.0047675326, 0.0098282107, 0.0188502322 rad). New endpoint:
(5.5942375694, 0.1153832598), with the same heading. Curvature is zero.

The previous 6.735 m curved route and its successful v6 physical run are kept.
This release changes route/assets only; C++ planner/controller source and all
control limits are unchanged from v6. The installed ROS binaries are rebuilt
with this release's library path. Joint MPC command cap remains 1.85 m/s,
acceleration upper limit 2 m/s², hard output limit 2 m/s, and overspeed trip
15750 ERPM. Launch synchronization and terminal stop behavior are retained.

Static-map screen: unknown blocked; assumed 0.58 x 0.31 m body. The route's
minimum lower-bound body clearance is 0.417 m; endpoint clearance 0.544 m.
Body-grid and between-pose coverage corrections are included. An additional
0.56 m forward corridor satisfies a 0.15 m body-clearance gate. This is a
map-space reserve, not independently measured braking distance. Body dimensions
have not been independently confirmed. Moving objects are outside this screen.

The speed-profile endpoints are zero. MPC computes its own curvature/terminal
speed and preserves the 0.5 m/s positive command floor after launch, then a
latched zero coast. Geometric simplicity does not establish two-car overtaking
feasibility; dynamic overtaking remains shadow-only.

Route: ../../routes/381_2_0_straight_original_start_20261107_v1.csv
Map plot: ../../maps/381_2_0_straight_original_start_20261107_v1.png
Construction script: ../../plan_381_2_0_straight_route.py
Validation and physical-test status: PREPARATION_STATUS.json
