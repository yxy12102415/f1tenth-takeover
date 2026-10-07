#!/usr/bin/env bash
# Deterministic ROS 2 / F1TENTH Gym dynamic-overtaking trial.
set -euo pipefail

cd "$(dirname "$0")"
SCENARIO="${1:-constant_speed}"
DURATION_OVERRIDE="${2:-}"
SEED="${3:-0}"
RUN_ID="${4:-$(date -u +%Y%m%dT%H%M%SZ)_seed${SEED}}"
GUI="${5:-false}"

case "$GUI" in true|false) ;; *) echo "GUI must be true or false" >&2; exit 2 ;; esac
if ! [[ "$SEED" =~ ^[0-9]+$ ]]; then echo "Seed must be a non-negative integer" >&2; exit 2; fi

mapfile -t CONFIG < <(python3 - "$SCENARIO" <<'PY'
import json
import sys
from pathlib import Path
data = json.loads(Path('src/takeover_sim/config/scenarios.json').read_text())
if sys.argv[1] not in data:
    raise SystemExit('Unknown scenario: ' + sys.argv[1])
c = data[sys.argv[1]]
base = c['map']
for suffix in ('_obstacle_center', '_obstacle_left', '_obstacle_right', '_visibility_left', '_visibility_right'):
    if base.endswith(suffix):
        base = base[:-len(suffix)]
values = [
    c['map'], c['duration_s'], c.get('mode', 'takeover'), c['lead_scenario'],
    c['lead_speed_mps'], c['ego_max_speed_mps'], *c['ego_spawn'], *c['lead_spawn'],
    c.get('dropout_start_s', -1.0), c.get('dropout_duration_s', 0.0),
    c.get('measurement_noise_std_m', 0.0), c['expected'], base,
]
print('\n'.join(map(str, values)))
PY
)
if [ "${#CONFIG[@]}" -ne 17 ]; then echo "Invalid scenario configuration" >&2; exit 2; fi
MAP_NAME="${CONFIG[0]}"; DURATION="${DURATION_OVERRIDE:-${CONFIG[1]}}"; MODE="${CONFIG[2]}"
LEAD_SCENARIO="${CONFIG[3]}"; LEAD_SPEED="${CONFIG[4]}"; EGO_SPEED="${CONFIG[5]}"
EGO_X="${CONFIG[6]}"; EGO_Y="${CONFIG[7]}"; EGO_YAW="${CONFIG[8]}"
OPP_X="${CONFIG[9]}"; OPP_Y="${CONFIG[10]}"; OPP_YAW="${CONFIG[11]}"
DROPOUT_START="${CONFIG[12]}"; DROPOUT_DURATION="${CONFIG[13]}"; NOISE="${CONFIG[14]}"
EXPECTED="${CONFIG[15]}"; ROUTE_NAME="${CONFIG[16]}"
if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then echo "Duration must be numeric" >&2; exit 2; fi

OUTPUT_DIR="$PWD/results/takeover/$SCENARIO/$RUN_ID"
if [ -e "$OUTPUT_DIR" ]; then echo "Refusing to overwrite $OUTPUT_DIR" >&2; exit 2; fi
mkdir -p "$OUTPUT_DIR"

REPO_ROOT=$(cd ../.. && pwd)
export ACADOS_INSTALL_DIR="${ACADOS_INSTALL_DIR:-$REPO_ROOT/.deps/acados-v0.4.0}"
if [ ! -f "$ACADOS_INSTALL_DIR/lib/libacados.so" ]; then
  echo "acados v0.4.0 is missing; run ../../tools/setup_laptop.sh" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
export LD_LIBRARY_PATH="$ACADOS_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$OUTPUT_DIR/ros_log"

python3 tools/generate_takeover_assets.py "$MAP_NAME" >/dev/null
colcon build --packages-select f1tenth_gym_ros mpc takeover_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source install/setup.bash
set -u

ASSETS_PATH="$PWD/takeover_assets/$MAP_NAME.json"
WAYPOINTS_PATH="$PWD/install/mpc/share/mpc/waypoints/${ROUTE_NAME}_mpc.csv"
cat > "$OUTPUT_DIR/config.json" <<EOF
{"scenario":"$SCENARIO","run_id":"$RUN_ID","seed":$SEED,"duration_s":$DURATION,"map":"$MAP_NAME","mode":"$MODE","lead_scenario":"$LEAD_SCENARIO","lead_speed_mps":$LEAD_SPEED,"ego_max_speed_mps":$EGO_SPEED,"expected":"$EXPECTED","ground_truth_input_to_ego":false}
EOF

setsid ros2 launch takeover_sim gym_takeover_launch.py \
  scenario:="$LEAD_SCENARIO" trial_scenario:="$SCENARIO" expected_outcome:="$EXPECTED" \
  map_name:="$MAP_NAME" gui:="$GUI" mode:="$MODE" \
  assets_path:="$ASSETS_PATH" waypoints_path:="$WAYPOINTS_PATH" \
  results_dir:="$OUTPUT_DIR" run_id:="$RUN_ID" seed:="$SEED" \
  lead_speed:="$LEAD_SPEED" ego_max_speed:="$EGO_SPEED" \
  measurement_noise_std:="$NOISE" dropout_start_s:="$DROPOUT_START" \
  dropout_duration_s:="$DROPOUT_DURATION" \
  ego_start_x:="$EGO_X" ego_start_y:="$EGO_Y" ego_start_yaw:="$EGO_YAW" \
  opp_start_x:="$OPP_X" opp_start_y:="$OPP_Y" opp_start_yaw:="$OPP_YAW" \
  >"$OUTPUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

READY=0
for _ in $(seq 1 300); do
  if [ -f "$OUTPUT_DIR/metadata.json" ]; then READY=1; break; fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  sleep 0.1
done
if [ "$READY" -ne 1 ]; then
  echo "Trial failed to start; see $OUTPUT_DIR/launch.log" >&2
  cleanup; wait "$LAUNCH_PID" || true; exit 1
fi

set +e
timeout "${DURATION}s" tail --pid="$LAUNCH_PID" -f /dev/null
TIMER_STATUS=$?
set -e
if [ "$TIMER_STATUS" -ne 0 ]; then cleanup; fi
for _ in $(seq 1 100); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  sleep 0.1
done
if kill -0 "$LAUNCH_PID" 2>/dev/null; then kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true; fi
set +e
wait "$LAUNCH_PID"
LAUNCH_STATUS=$?
set -e
trap - EXIT INT TERM

if [ ! -s "$OUTPUT_DIR/summary.json" ] || [ ! -s "$OUTPUT_DIR/trajectory.csv" ]; then
  echo "Trial has no complete result; see $OUTPUT_DIR/launch.log" >&2
  exit 1
fi
python3 tools/plot_takeover_trial.py "$OUTPUT_DIR" "$WAYPOINTS_PATH"
python3 - "$OUTPUT_DIR/summary.json" "$DURATION" <<'PY'
import json
import sys
s = json.load(open(sys.argv[1]))
if s.get('samples', 0) <= 0:
    raise SystemExit('No evaluation samples')
if s.get('duration_s', 0) + 0.75 < float(sys.argv[2]):
    raise SystemExit('Incomplete trial duration')
print(json.dumps({
    'passed': s['passed'],
    'collision_events': s['safety']['collision_events'],
    'boundary_events': s['safety']['boundary_violation_events'],
    'overtakes_completed': s['overtaking']['completed'],
    'minimum_body_clearance_m': s['safety']['minimum_body_clearance_m'],
    'planning_p95_ms': s['runtime']['planning_p95_ms'],
    'mpc_p95_ms': s['runtime']['mpc_p95_ms'],
}, indent=2))
if not s['passed']:
    raise SystemExit('Trial failed configured acceptance criteria; results were preserved')
PY
if [ "$LAUNCH_STATUS" -ne 0 ] && [ "$LAUNCH_STATUS" -ne 130 ]; then
  echo "ROS teardown status: $LAUNCH_STATUS (result files are preserved)" >&2
fi
echo "Trial complete: $OUTPUT_DIR"
