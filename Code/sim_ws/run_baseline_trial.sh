#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

SCENARIO="${1:-constant_speed}"
DURATION="${2:-60}"
RUN_ID="${3:-$(date -u +%Y%m%dT%H%M%SZ)}"
MAP_NAME="${4:-Melbourne_map}"
METHOD="${5:-nominal}"
WAYPOINTS_FILE="Melbourne_map_mpc.csv"
EGO_START_X="-23.1983"
EGO_START_Y="22.1783"
EGO_START_YAW="-0.787580"
OPP_START_X="-21.9370"
OPP_START_Y="20.9170"
OPP_START_YAW="-0.782731"

case "$SCENARIO" in
  constant_speed|accel_brake|turning|stop_go) ;;
  *)
    echo "Unknown scenario: $SCENARIO" >&2
    echo "Use constant_speed, accel_brake, turning, or stop_go." >&2
    exit 2
    ;;
esac

case "$METHOD" in
  nominal|corridor|visibility|adaptive|full) ;;
  *)
    echo "Unknown method: $METHOD" >&2
    echo "Use nominal, corridor, visibility, adaptive, or full." >&2
    exit 2
    ;;
esac

if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Duration must be a positive number of seconds." >&2
  exit 2
fi

if [[ "$MAP_NAME" == benchmark_* ]]; then
  if ! mapfile -t BENCHMARK_CONFIG < <(python3 ./generate_standardized_maps.py --lookup "$MAP_NAME"); then
    exit 2
  fi
  if [ "${#BENCHMARK_CONFIG[@]}" -ne 7 ]; then
    echo "Invalid benchmark configuration for map: $MAP_NAME" >&2
    exit 2
  fi
  WAYPOINTS_FILE="${BENCHMARK_CONFIG[0]}"
  EGO_START_X="${BENCHMARK_CONFIG[1]}"
  EGO_START_Y="${BENCHMARK_CONFIG[2]}"
  EGO_START_YAW="${BENCHMARK_CONFIG[3]}"
  OPP_START_X="${BENCHMARK_CONFIG[4]}"
  OPP_START_Y="${BENCHMARK_CONFIG[5]}"
  OPP_START_YAW="${BENCHMARK_CONFIG[6]}"
fi

OUTPUT_DIR="$PWD/results/$SCENARIO/$RUN_ID"
if [ -e "$OUTPUT_DIR" ]; then
  echo "Refusing to overwrite existing trial: $OUTPUT_DIR" >&2
  exit 2
fi
mkdir -p results
BUILD_ONLY=1 ./gym.sh
setsid env SKIP_BUILD=1 ./gym.sh \
  scenario:="$SCENARIO" \
  run_id:="$RUN_ID" \
  results_root:="$PWD/results" \
  map_name:="$MAP_NAME" \
  method:="$METHOD" \
  waypoints_file:="$WAYPOINTS_FILE" \
  ego_start_x:="$EGO_START_X" \
  ego_start_y:="$EGO_START_Y" \
  ego_start_yaw:="$EGO_START_YAW" \
  opp_start_x:="$OPP_START_X" \
  opp_start_y:="$OPP_START_Y" \
  opp_start_yaw:="$OPP_START_YAW" &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

READY=0
for _ in $(seq 1 200); do
  if [ -f "$OUTPUT_DIR/metadata.json" ]; then
    READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [ "$READY" -ne 1 ]; then
  echo "Trial did not become ready within 20 seconds: $OUTPUT_DIR" >&2
  cleanup
  wait "$LAUNCH_PID" || true
  exit 1
fi

# Measure the requested duration from evaluator readiness, not from the build or
# ROS launch startup. The monitor exits early if ros2 launch fails.
set +e
timeout "${DURATION}s" tail --pid="$LAUNCH_PID" -f /dev/null
TIMER_STATUS=$?
set -e
if [ "$TIMER_STATUS" -eq 0 ]; then
  set +e
  wait "$LAUNCH_PID"
  STATUS=$?
  set -e
else
  cleanup
  for _ in $(seq 1 100); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
  set +e
  wait "$LAUNCH_PID"
  STATUS=$?
  set -e
fi
trap - EXIT INT TERM

SUMMARY="$OUTPUT_DIR/summary.json"
if [ ! -f "$SUMMARY" ]; then
  echo "Trial ended without a summary: $SUMMARY" >&2
  if [ "$STATUS" -ne 0 ]; then
    exit "$STATUS"
  fi
  exit 1
fi
if ! python3 - "$SUMMARY" "$DURATION" <<'PY'
import json
import sys

with open(sys.argv[1], encoding='utf-8') as handle:
    summary = json.load(handle)
requested_duration = float(sys.argv[2])
actual_duration = float(summary.get('duration_s') or 0.0)
attempts = int(summary.get('runtime', {}).get('planner_attempts') or 0)
if attempts <= 0:
    raise SystemExit('Trial is invalid: the planner produced no attempts')
if actual_duration + 0.5 < requested_duration:
    raise SystemExit(
        f'Trial is incomplete: requested {requested_duration:.3f}s, '
        f'recorded {actual_duration:.3f}s'
    )
PY
then
  exit 1
fi
if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 130 ]; then
  echo "Trial data are complete; accepting abnormal ROS teardown status $STATUS." >&2
fi
echo "Trial complete: $SUMMARY"
