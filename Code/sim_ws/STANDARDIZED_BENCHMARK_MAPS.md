# Standardized F1TENTH Following Benchmark Maps

These maps create reproducible simulation environments for pure-LiDAR following.
They are not online inputs to the ego tracker, planner, or MPC. The ego stack must
continue to use `/scan`, ego odometry, and the estimated target belief only.

## Map set

All maps use `0.025 m/pixel`, a `2.20 m` driveable width, a `0.10 m` boundary,
and a closed centerline sampled every `0.05 m`.

Lead-vehicle waypoints contain a curvature-aware speed profile rather than one
constant speed. The nominal limit is `2.4 m/s` on straights, with
`v_ref = sqrt(0.65 / |curvature|)` in bends and a lower clamp of `0.8 m/s`.
Use the `turning` scenario to follow this profile; `constant_speed` remains the
low-speed control condition.

| Course | Purpose | Map names |
|---|---|---|
| `benchmark_straight` | Calibration, nominal following, and mirrored visibility selection | base, three `_obstacle_*` variants, plus `_visibility_left` and `_visibility_right` |
| `benchmark_occlusion_a` | Primary large rounded-square turning benchmark | base plus `_obstacle_center`, `_obstacle_left`, `_obstacle_right` |
| `benchmark_occlusion_b` | Held-out asymmetric-curvature generalization map | base plus `_obstacle_center`, `_obstacle_left`, `_obstacle_right` |

The obstacle footprint is `0.32 x 0.35 m`, with a nominal physical height of
`0.30 m`. `left` and `right` describe the obstacle location relative to the
direction of travel; they do not prescribe which side the vehicle must pass.

The straight-course stadium is `24 x 14 m`, with `12 m` straight sections and
`4.0 m`-radius end turns. Only the middle `9 m` of the first straight is the
nominal straight-evaluation zone; the end turns merely close the simulated
course. Map A is a `14 x 14 m` image containing a `10 x 10 m`
rounded-square centerline envelope with `3 m`-radius corners. The base map names
contain no obstacles. Each course also has deterministic center/left/right
obstacle variants; the center-obstacle lead waypoint performs a smooth
repeatable detour, while left/right variants retain the course centerline.

The visibility pair has identical map geometry: one `0.32 x 0.60 m` center
foam-board obstacle. Its lead trajectory starts an observable `+/-0.70 m`
detour over a `4.5 m` half-span before reaching the obstacle. The suffix names
the side on which the lead is expected to remain visible; it is not supplied to
the online ego stack. Mirroring the lead motion exposes a controller that
always chooses one fixed side.

Map A is the development and parameter-freezing environment. Map B must remain
held out until the method and success criteria are frozen.

## Generated artifacts

Run from `Code/sim_ws`:

```bash
./generate_standardized_maps.py
```

The deterministic generator creates:

- Gym occupancy images and ROS map YAML files in `src/f1tenth_gym_ros/maps/`;
- centerlines with `s`, pose, curvature, and reference speed;
- course, evaluation-zone, spawn, and obstacle specifications;
- lead-vehicle waypoint files in `src/f1tenth_mpc-main/mpc/waypoints/`;
- `benchmark_manifest.json`, including SHA-256 hashes for generated artifacts.

The same generator input can be used as a dimensioned layout for a physical
course. A narrower physical course requires a separately validated vehicle
inflation value. The nominal method retains its original `0.35 m` setting; the
paper method uses a vehicle-scale `0.20 m` inflation and must be revalidated if
the physical footprint or course width changes.

## Run examples

Straight calibration:

```bash
./run_baseline_trial.sh constant_speed 30 straight_01 benchmark_straight
```

Curvature-aware higher-speed trial:

```bash
./run_baseline_trial.sh turning 35 straight_profile_01 benchmark_straight
```

To open any base benchmark directly without starting a timed trial, use
`gym.sh`. It automatically selects the correct spawn poses and waypoint file:

```bash
./gym.sh map_name:=benchmark_occlusion_b
```

Primary center-obstacle trial:

```bash
./run_baseline_trial.sh turning 30 map_a_center_01 \
  benchmark_occlusion_a_obstacle_center full
```

Primary left/right obstacle trials:

```bash
./run_baseline_trial.sh turning 30 map_a_left_01 \
  benchmark_occlusion_a_obstacle_left full
./run_baseline_trial.sh turning 30 map_a_right_01 \
  benchmark_occlusion_a_obstacle_right full
```

Held-out generalization trial:

```bash
./run_baseline_trial.sh turning 30 map_b_center_01 \
  benchmark_occlusion_b_obstacle_center full
```

Focused visibility integration trials:

```bash
./run_baseline_trial.sh constant_speed 12 visibility_left_01 \
  benchmark_straight_visibility_left visibility
./run_baseline_trial.sh constant_speed 12 visibility_right_01 \
  benchmark_straight_visibility_right visibility
```

Balanced run-level comparison (recommended minimum integration batch):

```bash
./run_visibility_comparison.sh 12 5 visibility_confirmatory_01
```

The completed frozen-parameter reference batch is
`results/visibility_comparison/visibility_confirmatory_v2/`. All 20 planned
trials completed; visibility selected the expected side in all ten visibility
trials with zero target losses. These purpose-built mirrored maps validate the
visibility decision mechanism, while Map B remains the held-out generalization
gate.

`run_baseline_trial.sh` reads the benchmark manifest and supplies the matching
map, lead waypoints, ego spawn, and opponent spawn. The evaluator records the
selected `map_name` in `metadata.json`.

## Experimental use

- Use the straight map to calibrate tracking bias and nominal control only.
- Develop and tune on Map A.
- Compare methods with the same map, scenario, spawn, lead profile, and seed.
- Randomize method order within each matched block.
- Run Map B only after parameters are frozen.
- Treat a complete trial as one replicate; samples within a trial are not
  independent replicates.
