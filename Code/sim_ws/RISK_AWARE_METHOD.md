# Risk-Aware Following Method

This is the paper-method layer built after the frozen pure-LiDAR nominal
baseline. It preserves the main architecture:

```text
/scan -> clustering/EKF belief -> candidate Hybrid A* / corridor paths
      -> risk-aware path selection -> risk-adaptive MPC -> /drive_nominal
      -> safety arbiter -> /drive
```

`/opp_racecar/odom` is used by the lead vehicle's own waypoint controller and
by `baseline_evaluator.py`; it never enters the ego tracker, planner, candidate
selector, or MPC.  The ego stack uses the LiDAR-derived EKF state, its
covariance, tracker status, ego odometry, and the current scan.

## Belief and adaptive following reference

Position uncertainty is projected into the estimated target tangent and normal
directions to obtain `sigma_long` and `sigma_lat`. For adaptive methods:

```text
d_ref = clip(d0 + T_h*v_ego
             + k_sigma*sigma_long
             + k_age*measurement_age
             + k_vis*(1 - predicted_visibility))
```

The visibility term is added after the selected corridor has been evaluated;
the preliminary planning anchor uses the uncertainty and age terms. The
nominal method retains `d0 + T_h*v_ego`. All methods predict the moving rear
anchor over the one-second MPC preview.

## Candidate generation and selection

In free space, `corridor` follows the same center rear-anchor candidate as the
nominal controller; it does not weave between arbitrary lateral goals before a
blockage is confirmed. Hybrid A* uses one target-filtered occupancy grid per
control cycle, and every accepted path is collision checked against the
inflated grid.

For a compact object blocking the following corridor, consecutive LiDAR beams
are clustered. Long left/right wall clusters are robustly fitted with local
polynomials (including MAD outlier rejection) to estimate a curved LiDAR-only
corridor centerline. With two visible walls the fit estimates both center
offset and curvature; with one wall it uses only the relative wall shape, so it
does not assume a fixed lane width. A blockage candidate is compared with this
curved centerline rather than the ego-frame x-axis.

Only compact clusters behind the estimated lead vehicle, no farther than
`4.6 m`, with longitudinal span at most `0.40 m` and lateral span at most
`0.55 m`, can trigger a pass. A candidate must appear at a consistent position
in three consecutive scans. This range and persistence gate prevents a short
visible fragment of a curved track wall from being treated as an obstacle.
Smooth left/right corridor seeds follow the fitted centerline. Feasible offsets
from `0.58 m` to `0.88 m` are collision checked and ranked, instead of assuming
one fixed `0.58 m` offset. The occupancy grid uses Euclidean inflation with a
half-cell raster allowance; the old enclosing-square rasterization inflated a
requested `0.20 m` radius to about `0.34 m` at diagonal cells. Hybrid A* remains
the general-search fallback. For non-nominal methods only, the collision grid
clears a `0.24 m` disk already occupied by the ego footprint so an inflated
wall cell cannot make it impossible to plan out of the current pose. The raw
visibility grid and the independent scan-based safety filter are not cleared.

The curved geometry is refreshed at `0.20 s` intervals before commitment. Once
a side is selected, that world-frame corridor is frozen until the obstacle is
behind the ego vehicle, while its remaining path is still checked against the
newest scan every control cycle. This prevents viewpoint-dependent LiDAR fits
from replacing the reference midway through a pass. A same-side Hybrid A*
fallback is used if the analytic seed becomes invalid. Committed paths carry an
explicit side label and are trimmed only to a forward, heading-consistent
suffix. This prevents a nearby reverse-facing branch from entering the MPC
reference. After the obstacle is behind the ego vehicle, the center rear-anchor
candidate performs rejoin. A previous path is reused only while its remaining
suffix is still collision free and its label matches the attempted corridor.

The dynamic lead is removed from static occupancy only when its LiDAR
measurement is fresh. The planner matches the compact scan cluster to the EKF
target and removes that cluster plus the configured vehicle footprint, avoiding
both leftover vehicle-side returns and deletion of a nearby fixed obstacle.

Candidate score terms are normalized path length, curvature, clearance, rejoin
offset, predicted target visibility, and switching cost. The side lock prevents
left/right oscillation and discontinuous cross-obstacle replanning.

Global LiDAR reacquisition is motion confirmed. Rejected compact clusters build
short world-frame tracks with an existence score; reinitialization requires a
minimum observation time, speed, and bounded constant-velocity fit residual.
A persistent static obstacle therefore cannot become the tracked lead merely
because it is the nearest forward cluster.

## Visibility and risk

Predicted visibility is evaluated at five points over a three-second obstacle-
pass horizon. The target mean follows the EKF constant-velocity model; the
short MPC rear anchor retains its separate one-second constant-turn-rate
prediction. At each point, a covariance-preserving five-point unscented rule
(mean weight `0.5`, four `+/-2 sigma` weights of `0.125`) is checked for sensor
range, field of view, and raw (non-footprint-inflated) LiDAR occupancy-grid
line of sight. Adjacent returns on one observed surface are connected in the
visibility grid so a ray cannot pass through scan/grid sampling gaps:

```text
risk = w_sigma*normalized_sigma
     + w_age*normalized_measurement_age
     + w_vis*(1 - predicted_visibility)
```

In the `visibility`-only ablation this calculation runs only when two or more
feasible corridors are available, because a one-candidate visibility score
cannot affect selection and needlessly consumes the 20 Hz planning budget.
The `full` method continues to evaluate it for a single candidate because the
result also drives risk-adaptive speed and gap control.

A candidate must improve predicted visibility by at least `0.075` over the
geometry-preferred corridor before visibility is allowed to change the passing
side. With the configured unscented/time rule this requires three minimum
`0.025` probability increments, rejecting the earlier `0.04` single-ray
quantization changes.

Risk increases take effect immediately, while decreases use a slow exponential
release filter. This avoids repeated acceleration and braking when visibility
changes by one occupancy-grid sample. The MPC uses the filtered risk to scale
only positive closing speed above the estimated lead speed, including the
positive part of the longitudinal safe-speed cap. It does not scale the lead
speed itself: doing so makes the ego permanently slower than a moving lead and
can push the target out of LiDAR range. Uncertainty, measurement age, and low
predicted visibility enlarge the dynamic following reference. Steering and
acceleration remain constrained by MPC, the longitudinal safety filter, and
the final command arbiter.

## Ablation modes

The fifth argument to `run_baseline_trial.sh` selects the method:

| Method | Candidate corridors | Visibility score | Adaptive gap/speed |
| --- | --- | --- | --- |
| `nominal` | no | no | no |
| `corridor` | yes | no | no |
| `visibility` | yes | yes | no |
| `adaptive` | no | no | yes |
| `full` | yes | yes | yes |

All non-nominal modes are fail-closed: an unchecked straight-line fallback is
never sent. A planning failure produces zero speed through the existing safety
chain.

```bash
./run_baseline_trial.sh turning 30 full_center_01 \
  benchmark_straight_obstacle_center full

./run_baseline_trial.sh turning 30 nominal_center_01 \
  benchmark_straight_obstacle_center nominal
```

Each result directory adds sampled `decision.csv`, event-complete
`decision_events.csv`, and a `risk_aware_planning` section
to `summary.json`, including risk, longitudinal/lateral uncertainty, predicted
visibility, selected-candidate counts, switching, checked-path failures, the
number of decisions with multiple feasible corridors, and whether the selected
corridor had the best predicted visibility. The logs also preserve raw risk,
filtered risk, applied speed scale, dynamic following distance, and same-side
corridor refresh requests. A refresh is permitted only after the committed
path and a same-side Hybrid A* recovery both fail collision checking; it never
unlocks the passing side or bypasses the fail-closed stop. Fixed-distance RMSE
is retained for comparison with the nominal baseline, while
`dynamic_distance_rmse_m` is the primary gap-tracking metric for `adaptive`
and `full`.

For a balanced matched batch, rotate all five modes across repetitions with:

```bash
./run_method_ablation.sh turning 30 \
  benchmark_occlusion_a_obstacle_center 5 experiment_01
```

Each whole trial is one replicate; the 20 Hz samples within a trial are not
treated as independent replicates.

For the focused mirrored corridor-versus-visibility comparison, run:

```bash
./run_visibility_comparison.sh 12 5 visibility_confirmatory_01
```

This alternates method order within each repetition and mirrored lead-detour
condition, then writes the design and run-level aggregate to
`results/visibility_comparison/<batch_id>/`.

## Current validation status

The corridor implementation is now a tested simulation V1 ablation, not final
research evidence. Three independent 15-second trials on the former failure
case `benchmark_occlusion_a_obstacle_center` all completed center-to-left pass
and center rejoin with zero target losses, zero unchecked-path stops, and
`100%` planner success. Tracking position RMSE was `0.0266--0.0272 m`, following
distance RMSE was `0.634--0.693 m`, minimum lead distance was
`1.342--1.367 m`, mean planning time was `10.3--11.1 ms`, and planning-time p95
was `11.0--13.9 ms`.

The 12-second obstacle-free Map A regressions retained zero target losses and
`100%` planner success for both nominal and corridor; corridor selected only
the center candidate. The 12-second straight-center-obstacle regression had
zero target losses, `99.52%` planner success, and one transient fail-closed
cycle. Dynamic clear-map regressions for acceleration/braking, turning, and
stop-and-go all had zero target losses, `100%` planner success, and tracking
position RMSE of `0.026--0.032 m`.

The September 5 visibility V1 hardening separates raw line-of-sight occupancy
from collision-inflated occupancy, connects adjacent surface returns, skips
closed-side Hybrid A* searches, freezes the selected world-frame corridor,
reserves at least `0.67 m` lateral offset, uses an EKF-consistent CV visibility
mean, and applies the `0.075` deadband. The deterministic mirrored maps
`benchmark_straight_visibility_left/right` use identical geometry: one
`0.32 x 0.60 m` center obstacle and an observable, early `+/-0.70 m` lead
detour. This isolates visibility selection from a fixed left/right preference.

The frozen-parameter `visibility_confirmatory_v2` integration batch completed
all 20 planned trials: five repetitions of corridor and visibility on each
mirrored condition. Visibility selected the expected passing side in `10/10`
trials and had zero target losses. Its mean tracking position RMSE was
`0.0204 m` on the left condition and `0.0186 m` on the right, versus
`0.4179 m` and `1.1283 m` for corridor. Mean following-distance RMSE was
`0.6932 m` and `0.9508 m`, versus `0.9021 m` and `1.7574 m` for corridor.
Visibility planner success was `100%` on left and `98.78%` on right; the right
condition retained `2.6` transient fail-closed/refresh cycles per trial on
average. In the right condition, visibility cleared the decision margin and
changed the geometry-preferred left pass to the expected right pass in all
`5/5` trials. Planning-time p95 averaged about `12.0 ms` in both conditions.
The design, trial paths, and full run-level values are stored in
`results/visibility_comparison/visibility_confirmatory_v2/summary.json`.

This is repeated integration evidence, not yet a statistical or generalization
claim. The mirrored maps deliberately create the visibility contrast used to
test the implementation, and the right condition still exhibits a small
number of transient fail-closed cycles.

The original compact obstacle maps remain integration tests rather than strong
visibility contrasts. The held-out Map B probe also remains failed and must not
be reported as successful generalization.

The September 6 adaptive-control development regression corrected an unsafe
first formulation that multiplied the complete speed command by the risk
scale. Risk now limits only positive closing speed. A preliminary acceleration/
braking `adaptive` run completed with zero target losses, `100%` planner
success, and `0.3783 m` dynamic following-distance RMSE.

The current `full` development set uses a `0.58 m` minimum corridor offset and
the non-nominal `0.24 m` start-footprint clearing described above. When a
frozen corridor and same-side Hybrid A* both fail, it now attempts one same-
cycle LiDAR geometry refresh while preserving the committed side. Motion
continues only if the refreshed path passes the same inflated-grid collision
check; otherwise the existing fail-closed stop remains active.

Three left and three right mirrored 12-second runs all selected the expected
side and had zero target losses. Mean planner success was `99.84%` left and
`99.38%` right; mean position RMSE was `0.0198 m` and `0.0186 m`. Mean dynamic
following-distance RMSE was `0.5062 m` left and `0.5367 m` right. Two of three
left runs and one of three right runs reached `100%` planner success; the other
runs retained one or two sampled fail-closed cycles. Therefore the full method
has passed a functional smoke gate but has not passed a stability or
confirmatory gate. The earlier `visibility_confirmatory_v2` results used the
old `0.67 m` offset and must not be presented as validation of this current
parameter set.

These are deterministic integration and regression trials. They establish that
nominal and corridor are ready for controlled shadow-mode preparation and that
visibility V1 passes the focused mirrored simulation integration gate. They do
not establish that the full visibility/adaptive method is paper-ready or safe
for autonomous physical driving. The remaining gates are held-out-map testing,
sensor-only real-vehicle validation, and low-speed shadow-mode checks, followed
by a predeclared confirmatory experiment for any research claim.

A final five-mode, eight-second clear-map launch regression after adding the
physical shadow entry point completed with zero target losses and `100%`
planner success for `nominal`, `corridor`, `visibility`, `adaptive`, and
`full`. Full had `0.219 m` dynamic-gap RMSE and `13.47 ms` planning-time p95 in
that short run. This verifies mode wiring and prevents the real launch changes
from altering the simulation defaults; it is not a performance comparison.

The final 15-second held-out Map B full-method probe did **not** pass: it had
one target loss, `98.29%` planner success, `0.429 m` position RMSE, and
`1.017 m` dynamic-gap RMSE. The method stopped rather than accepting unchecked
paths. Map B therefore remains an explicit generalization failure and blocks
any claim that `full` is ready for autonomous physical deployment. The safe
next step is the staged sensor/shadow procedure in
[REAL_VEHICLE_DEPLOYMENT.md](REAL_VEHICLE_DEPLOYMENT.md).
