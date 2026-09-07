#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  CLEAN_LD_LIBRARY_PATH=""
  OLD_IFS="$IFS"
  IFS=":"
  for path in $LD_LIBRARY_PATH; do
    case "$path" in
      /snap/*) ;;
      *) CLEAN_LD_LIBRARY_PATH="${CLEAN_LD_LIBRARY_PATH:+$CLEAN_LD_LIBRARY_PATH:}$path" ;;
    esac
  done
  IFS="$OLD_IFS"
  export LD_LIBRARY_PATH="$CLEAN_LD_LIBRARY_PATH"
fi

if [ -n "${ROS_DISTRO:-}" ]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
  ROS_SETUP="/opt/ros/humble/setup.bash"
else
  ROS_SETUP="/opt/ros/foxy/setup.bash"
fi
if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 2 setup file not found: $ROS_SETUP" >&2
  echo "Install ROS 2 first, or set ROS_DISTRO to an installed distro such as humble." >&2
  exit 1
fi
source "$ROS_SETUP"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  colcon build --packages-select f1tenth_gym_ros mpc
fi
source install/setup.bash
if [ "${BUILD_ONLY:-0}" = "1" ]; then
  exit 0
fi

# A direct `./gym.sh map_name:=benchmark_*` invocation must not retain the
# Melbourne defaults: those poses are outside the generated benchmark maps.
# Add deterministic per-map waypoints and spawn poses unless the caller has
# already supplied an explicit override (as run_baseline_trial.sh does).
BENCHMARK_MAP=""
for argument in "$@"; do
  case "$argument" in
    map_name:=benchmark_*) BENCHMARK_MAP="${argument#map_name:=}" ;;
  esac
done
if [ -n "$BENCHMARK_MAP" ]; then
  mapfile -t BENCHMARK_CONFIG < <(python3 ./generate_standardized_maps.py --lookup "$BENCHMARK_MAP")
  if [ "${#BENCHMARK_CONFIG[@]}" -ne 7 ]; then
    echo "Invalid benchmark configuration for map: $BENCHMARK_MAP" >&2
    exit 2
  fi
  BENCHMARK_ARGUMENTS=(
    "waypoints_file:=${BENCHMARK_CONFIG[0]}"
    "ego_start_x:=${BENCHMARK_CONFIG[1]}"
    "ego_start_y:=${BENCHMARK_CONFIG[2]}"
    "ego_start_yaw:=${BENCHMARK_CONFIG[3]}"
    "opp_start_x:=${BENCHMARK_CONFIG[4]}"
    "opp_start_y:=${BENCHMARK_CONFIG[5]}"
    "opp_start_yaw:=${BENCHMARK_CONFIG[6]}"
  )
  for benchmark_argument in "${BENCHMARK_ARGUMENTS[@]}"; do
    argument_name="${benchmark_argument%%:=*}:="
    has_override=0
    for argument in "$@"; do
      case "$argument" in
        "$argument_name"*) has_override=1 ;;
      esac
    done
    if [ "$has_override" -eq 0 ]; then
      set -- "$@" "$benchmark_argument"
    fi
  done
fi
exec ros2 launch mpc gym_ego_follow_launch.py "$@"
