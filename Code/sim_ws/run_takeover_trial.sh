#!/usr/bin/env bash
# Deterministic ROS 2 / F1TENTH Gym dynamic-overtaking trial.
set -euo pipefail

cd "$(dirname "$0")"
SCENARIO="${1:-fitzpatrick_position_00}"
DURATION_OVERRIDE="${2:-}"
SEED="${3:-0}"
RUN_ID="${4:-$(date -u +%Y%m%dT%H%M%SZ)_seed${SEED}}"
GUI="${5:-false}"
export TAKEOVER_EGO_START_DELAY_S="${TAKEOVER_EGO_START_DELAY_S:-0.0}"
export TAKEOVER_PLANNING_BACKEND="${TAKEOVER_PLANNING_BACKEND:-frenet_candidates}"
case "$TAKEOVER_PLANNING_BACKEND" in frenet_candidates|upstream_sqp) ;; *) echo 'Unknown planning backend' >&2; exit 2 ;; esac
export TAKEOVER_TRACK_PREDICTION="${TAKEOVER_TRACK_PREDICTION:-true}"
export TAKEOVER_TIMED_CONTROL="${TAKEOVER_TIMED_CONTROL:-true}"
export TAKEOVER_PASS_SIDE="${TAKEOVER_PASS_SIDE:-0}"
case "$TAKEOVER_PASS_SIDE" in -1|0|1) ;; *) echo 'Pass side must be -1, 0 or 1' >&2; exit 2 ;; esac
for value in "$TAKEOVER_TRACK_PREDICTION" "$TAKEOVER_TIMED_CONTROL"; do
  case "$value" in true|false) ;; *) echo 'Feature flags must be true or false' >&2; exit 2 ;; esac
done

# Keep laptop trials local and prevent accidental overlapping launches.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${TAKEOVER_ROS_DOMAIN_ID:-73}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=2
export CMAKE_BUILD_PARALLEL_LEVEL=2
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/takeover-sim-${UID}.lock"
flock -n 9 || { echo 'Another takeover trial is running.' >&2; exit 2; }

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
    str(c.get('lead_enabled', True)).lower(),
    c.get('pass_offset_m', 0.82),
    str(c.get('adaptive_offset', False)).lower(),
    c.get('lateral_amplitude_m', 0.0),
    c.get('lead_lateral_accel_mps2', 0.0),
    c.get('controller_model', 'v8'),
    c.get('tracker_fov_deg', 190.0), c.get('tracker_min_forward_x_m', -0.5),
    c.get('tracker_tracking_fov_deg', 0.0),
]
print('\n'.join(map(str, values)))
PY
)
if [ "${#CONFIG[@]}" -ne 26 ]; then echo "Invalid scenario configuration" >&2; exit 2; fi
MAP_NAME="${CONFIG[0]}"; DURATION="${DURATION_OVERRIDE:-${CONFIG[1]}}"; MODE="${CONFIG[2]}"
LEAD_SCENARIO="${CONFIG[3]}"; LEAD_SPEED="${CONFIG[4]}"; EGO_SPEED="${CONFIG[5]}"
EGO_X="${CONFIG[6]}"; EGO_Y="${CONFIG[7]}"; EGO_YAW="${CONFIG[8]}"
OPP_X="${CONFIG[9]}"; OPP_Y="${CONFIG[10]}"; OPP_YAW="${CONFIG[11]}"
DROPOUT_START="${CONFIG[12]}"; DROPOUT_DURATION="${CONFIG[13]}"; NOISE="${CONFIG[14]}"
EXPECTED="${CONFIG[15]}"; ROUTE_NAME="${CONFIG[16]}"
LEAD_ENABLED="${CONFIG[17]}"
PASS_OFFSET="${CONFIG[18]}"
ADAPTIVE_OFFSET="${CONFIG[19]}"
LATERAL_AMPLITUDE="${CONFIG[20]}"
LEAD_LATERAL_ACCEL="${CONFIG[21]}"
CONTROLLER_MODEL="${CONFIG[22]}"
TRACKER_FOV="${CONFIG[23]}"; TRACKER_MIN_X="${CONFIG[24]}"
TRACKER_TRACKING_FOV="${CONFIG[25]}"
case "$CONTROLLER_MODEL" in v8|official_gym_native) ;; *) echo 'Unknown controller model' >&2; exit 2 ;; esac
USE_SIM_TIME=false
if [ "$CONTROLLER_MODEL" = official_gym_native ]; then USE_SIM_TIME=true; fi
NUM_AGENTS=2
if [ "$LEAD_ENABLED" = false ]; then NUM_AGENTS=1; fi
if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then echo "Duration must be numeric" >&2; exit 2; fi

OUTPUT_DIR="$PWD/results/takeover/$SCENARIO/$RUN_ID"
if [ -e "$OUTPUT_DIR" ]; then echo "Refusing to overwrite $OUTPUT_DIR" >&2; exit 2; fi
mkdir -p "$OUTPUT_DIR"
export TAKEOVER_SQP_TRACE="$OUTPUT_DIR/sqp_solver_failures.jsonl"

REPO_ROOT=$(cd ../.. && pwd)
SQP_SOURCE="$REPO_ROOT/.deps/predictive-spliner-upstream/src/sqp_avoidance_node.py"
if [ "$TAKEOVER_PLANNING_BACKEND" = upstream_sqp ] && [ ! -f "$SQP_SOURCE" ]; then
  echo 'Missing optional upstream checkout; see docs/UPSTREAM_SQP_ADAPTER.md' >&2; exit 2
fi
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
if [ "$CONTROLLER_MODEL" = official_gym_native ]; then
  bash tools/generate_gym_mpc.sh >"$OUTPUT_DIR/codegen.log" 2>&1 || {
    tail -n 20 "$OUTPUT_DIR/codegen.log" >&2; exit 1;
  }
fi
if ! colcon build --executor sequential --packages-select f1tenth_gym_ros mpc takeover_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release >"$OUTPUT_DIR/build.log" 2>&1; then
  tail -n 25 "$OUTPUT_DIR/build.log" >&2
  exit 1
fi
set +u
source install/setup.bash
set -u

ASSETS_PATH="$PWD/takeover_assets/$MAP_NAME.json"
WAYPOINTS_PATH="$PWD/install/mpc/share/mpc/waypoints/${ROUTE_NAME}_mpc.csv"
python3 - "$SCENARIO" "$RUN_ID" "$SEED" "$DURATION" "$OUTPUT_DIR" <<'PY'
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
scenario, run_id, seed, duration, output = sys.argv[1:]
config = json.loads(Path('src/takeover_sim/config/scenarios.json').read_text())[scenario]
config.update(scenario=scenario, run_id=run_id, seed=int(seed),
              duration_s=float(duration), ground_truth_input_to_ego=False)
config.setdefault('lead_enabled', True)
config['ego_start_delay_s'] = float(os.environ['TAKEOVER_EGO_START_DELAY_S'])
config.setdefault('pass_offset_m', 0.82)
config.setdefault('adaptive_offset', False)
config.setdefault('lateral_amplitude_m', 0.0)
config['git_commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
root=Path('../..').resolve()
runtime_sources=[
    'Code/sim_ws/src/takeover_sim/src/takeover_controller_node.cpp',
    'Code/sim_ws/src/takeover_sim/include/takeover_sim/dynamic_overtaking.hpp',
    'Code/sim_ws/src/takeover_sim/include/takeover_sim/sqp_worker.hpp',
    'Code/sim_ws/src/takeover_sim/launch/gym_takeover_launch.py',
    'Code/sim_ws/src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py',
    'Code/sim_ws/src/f1tenth_gym_ros/launch/gym_bridge_launch.py',
    'Code/sim_ws/src/f1tenth_mpc-main/mpc/scripts/opponent_ekf_tracker.py',
    'Code/sim_ws/src/f1tenth_mpc-main/mpc/scripts/opp_pid_node.py',
    'Code/sim_ws/src/f1tenth_mpc-main/mpc/scripts/takeover_evaluator.py',
    'Code/sim_ws/tools/upstream_sqp_worker.py',
    'real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/src/cpp_frenet/joint_control.hpp',
    'real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/generate_joint_solver.py',
]
config['runtime_source_sha256']={s:hashlib.sha256((root/s).read_bytes()).hexdigest()
                                 for s in runtime_sources if (root/s).exists()}
config['gym_default_seed']=12345
config['use_sim_time']=config.get('controller_model')=='official_gym_native'
config['sensor_publish_hz']=50 if config['use_sim_time'] else 250
config['physics_step_s']=.01
config['repeatability_scope']='Independent process/wall-clock repeats; Gym default seed is fixed, tracker injected noise is configured separately.'
config['controller_steering_limit_rad']=.4189 if config.get('controller_model')=='official_gym_native' else .269
config['ego_acceleration_max_mps2']=3.0 if config.get('controller_model')=='official_gym_native' else 1.0
config['ego_acceleration_min_mps2']=-3.0 if config.get('controller_model')=='official_gym_native' else -.8
if config.get('controller_model')=='official_gym_native':
    generated=Path('build/takeover_sim/gym_model/generated_joint/joint_vehicle_model/joint_vehicle_dyn_disc_phi_fun.c')
    config['generated_gym_dynamics_sha256']=hashlib.sha256(generated.read_bytes()).hexdigest()
config['track_assisted_prediction'] = os.environ['TAKEOVER_TRACK_PREDICTION'] == 'true'
config['timed_control'] = os.environ['TAKEOVER_TIMED_CONTROL'] == 'true'
config['planning_backend'] = os.environ['TAKEOVER_PLANNING_BACKEND']
if config['planning_backend'] == 'upstream_sqp':
    import numpy
    import scipy
    config['numpy_version'] = numpy.__version__
    config['scipy_version'] = scipy.__version__
    config['upstream_sqp_commit'] = 'ddbfbb2312eccda7bb5d84469ec0980ddb07a5e9'
    config['upstream_sqp_scope'] = 'numerical lateral optimizer only; not full GP Predictive Spliner'
config['allowed_pass_side'] = int(os.environ['TAKEOVER_PASS_SIDE'])
diff = subprocess.check_output(['git', 'diff', 'HEAD'])
config['working_diff_sha256'] = hashlib.sha256(diff).hexdigest()
Path(output, 'source.patch').write_bytes(diff)
# A dirty diff excludes new files. Preserve the new adapter sources explicitly
# so pre-commit experiments still have a replayable implementation snapshot.
if config['planning_backend'] == 'upstream_sqp':
    import shutil
    snapshot = Path(output, 'adapter_sources')
    snapshot.mkdir()
    config['adapter_source_sha256'] = {}
    for relative in ['tools/upstream_sqp_worker.py', 'src/takeover_sim/include/takeover_sim/sqp_worker.hpp',
                     'tools/generate_gym_mpc.sh']:
        source = Path(relative)
        shutil.copy2(source, snapshot / source.name)
        config['adapter_source_sha256'][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
Path(output, 'config.json').write_text(json.dumps(config, indent=2) + '\n')
PY

setsid ros2 launch takeover_sim gym_takeover_launch.py \
  scenario:="$LEAD_SCENARIO" trial_scenario:="$SCENARIO" expected_outcome:="$EXPECTED" \
  map_name:="$MAP_NAME" gui:="$GUI" mode:="$MODE" controller_model:="$CONTROLLER_MODEL" \
  use_sim_time:="$USE_SIM_TIME" \
  ego_start_delay_s:="$TAKEOVER_EGO_START_DELAY_S" \
  num_agents:="$NUM_AGENTS" lead_enabled:="$LEAD_ENABLED" \
  pass_offset:="$PASS_OFFSET" \
  adaptive_offset:="$ADAPTIVE_OFFSET" lateral_amplitude:="$LATERAL_AMPLITUDE" \
  planning_backend:="$TAKEOVER_PLANNING_BACKEND" \
  sqp_worker_script:="$PWD/tools/upstream_sqp_worker.py" sqp_upstream_source:="$SQP_SOURCE" \
  track_assisted_prediction:="$TAKEOVER_TRACK_PREDICTION" timed_control:="$TAKEOVER_TIMED_CONTROL" \
  allowed_pass_side:="$TAKEOVER_PASS_SIDE" \
  assets_path:="$ASSETS_PATH" waypoints_path:="$WAYPOINTS_PATH" \
  results_dir:="$OUTPUT_DIR" run_id:="$RUN_ID" seed:="$SEED" \
  lead_speed:="$LEAD_SPEED" lead_lateral_accel:="$LEAD_LATERAL_ACCEL" ego_max_speed:="$EGO_SPEED" \
  measurement_noise_std:="$NOISE" dropout_start_s:="$DROPOUT_START" \
  tracker_fov_deg:="$TRACKER_FOV" tracker_min_forward_x:="$TRACKER_MIN_X" \
  tracker_tracking_fov_deg:="$TRACKER_TRACKING_FOV" \
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

# Observe the live isolated graph without delaying the fixed trial timer.
timeout 12 python3 tools/audit_takeover_graph.py \
  >"$OUTPUT_DIR/graph_audit.json" 2>"$OUTPUT_DIR/graph_audit.log" &
AUDIT_PID=$!

set +e
if [ "$USE_SIM_TIME" = true ]; then
  python3 tools/wait_sim_completion.py "$OUTPUT_DIR/summary.json" "$DURATION" "$LAUNCH_PID"
else
  timeout "${DURATION}s" tail --pid="$LAUNCH_PID" -f /dev/null
fi
TIMER_STATUS=$?
set -e
if [ "$TIMER_STATUS" -ne 0 ] || [ "$USE_SIM_TIME" = true ]; then cleanup; fi
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

if ! wait "$AUDIT_PID"; then
  echo "Live graph audit failed; see $OUTPUT_DIR/graph_audit.log" >&2
  exit 1
fi

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
