# Fitzpatrick multi-position single-pass screen — 2026-10-24

The requested objective is now one complete overtake and rejoin per trial,
without an intentional two-second startup delay. Four actual headless trials
were completed with the same runtime source hashes. Two passed both the
behavior/safety criterion and the stricter clean-motion audit. Two failed the
35-second single-pass criterion. This is one exploratory trial per location,
not a repeatability or generalized success-rate claim.

## Fixed design and scoring

- Four ego starts at 0%, 25%, 50%, 75% of the 44.7315 m reference route.
- Lead starts 2.25 m ahead along the route; both start from rest.
- Ego/lead caps 3.0/1.8 m/s, same curvature-dependent slowing.
- Same 35-second simulation duration, native Gym model, LiDAR tracker,
  track-assisted prediction, upstream numerical SQP adapter, dynamic MPC.
- `TAKEOVER_EGO_START_DELAY_S=0`; normal sensor initialization remains.
- Fixed shuffled run order with schedule seed 20261024: 75%, 0%, 50%, 25%.
- Independent truth counter requires actual passage, >1.20 m longitudinal
  lead, <0.10 m lateral error, <0.15 rad heading error, and CRUISE for 0.5 s.
  Ego/lead boundary events and body collisions must all be zero. The existing
  counter and collision geometry were not relaxed; required pass count is one.
- Unit of observation is a fresh-process trial, not each 20 Hz CSV row.

The experimental-design workflow informed matched initial gaps, fixed caps and
duration, a saved pre-run randomized schedule, and separation of spawn location
from actual planner commitment location. No statistical significance is claimed.

## Results

| Start | Truth-confirmed passes | Completion time | Minimum sampled body gap | MPC rejections | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| 0% | 1 | 11.70 s | 0.4183 m | 0 | Pass, clean |
| 25% | 0 | Not confirmed by 35 s | 0.3519 m | 0 | Fail |
| 50% | 0 | Not completed | 1.3385 m | 0 | Fail |
| 75% | 1 | 22.40 s | 0.3494 m | 0 | Pass, clean |

All four completed 35.03–35.09 simulation seconds with zero scored body
collisions, zero ego/lead boundary events and zero MPC/SQP timeouts. The
25%/50% trials had 15/5 planner-invalid evaluation samples after two seconds;
these are sampled statuses, not independent failure calls. Planning P95 was
18.31–36.98 ms and MPC P95 1.65–1.82 ms. These values do not certify hard
real-time operation or physical safety.

## Where overtaking actually started

- **0%:** committed at 0.95 s near (0.614, -0.347) m; completed normally.
- **25%:** committed at 0.95 s near (8.341, 6.387) m. At 7.50 s,
  `map_or_static_footprint` rejected the future committed corridor. It canceled,
  rejoined behind, then tried again near (0.411, -0.423) m at 22.90 s.
  The controller's internal counter became one at 33.90 s, but independent
  longitudinal separation was then only about 1.045 m. At the last sample
  it was about 1.242 m; the required stable 0.5-second confirmation had not
  completed. The trial remains failed, rather than trusting the internal count.
- **50%:** committed at 5.95 s near (1.266, 15.668) m. At 12.00 s,
  `curvature_limit` invalidated continuation. It canceled and safely rejoined
  behind at 15.95 s, then continued following/preparing; no completed pass.
- **75%:** initially followed; committed at 10.70 s near (0.412, -0.431) m,
  close to the route origin, then completed at 22.40 s.

Thus different spawn positions do not imply successful overtaking at all those
locations. Actual commitments occurred at several locations, but the completed
passes still rely on the lower entry corridor. Whole-maneuver bend/map
feasibility remains the next planner improvement; do not force an offset or
bypass collision checks to make a local attempt succeed.

## Reproduce and inspect

From `Code/sim_ws`:

```bash
# GUI: choose 00, 25, 50 or 75.
TAKEOVER_PLANNING_BACKEND=upstream_sqp TAKEOVER_EGO_START_DELAY_S=0 \
  bash run_takeover_trial.sh fitzpatrick_position_00 '' 1 \
  "gui_$(date +%Y%m%d_%H%M%S)" true

# Serial, shuffled four-location batch; use a new prefix each time.
python3 tools/run_position_overtake_batch.py --prefix positions_NEW_RUN
```

The RViz yellow reference route remains available. No GUI window was opened
during this headless screen. The legacy `fitzpatrick_double_*` configurations
and historical results are preserved for reproduction, but are not the active
acceptance requirement.

Recorded batch: `Code/sim_ws/results/takeover_position_batches/positions_20261024_screen/`
contains `schedule.json`, `results.json` (including actual entry coordinates),
and `audit.json`. Each raw trial is under
`Code/sim_ws/results/takeover/fitzpatrick_position_XX/positions_20261024_screen_repeat1/`
with config, source hashes, graph audit, CSVs, summary, logs and separate
`trajectory_en.png` / prediction plots. Failed results remain intact.

Graph audits passed in all four runs: opponent odometry is consumed only by
the opponent controller and evaluator; tracker remains LiDAR-only. Final drive
publisher remains the safety arbiter. No real vehicle was contacted.

## Changes and validation

- Runner and launch: default ego start delay zero; runner default scenario 00.
- Scenario JSON: four matched position configurations.
- Evaluator: new single-pass outcome using the existing strict truth counter.
- `tools/run_position_overtake_batch.py`: bounded serial runner with saved
  seeded schedule, retained failures and entry-location reporting.
- Laptop guide: GUI and batch instructions, current vs. historical scope.

All 10 existing evaluator geometry/pass-counter tests passed. Spawn positions
and yaw were checked against route interpolation, and both complete bodies
checked against occupied map cells and one another. Shell/Python syntax and
`git diff --check` passed. Four complete current-code trials provide the runtime
validation; they are not substituted with earlier historical runs.
