#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/foxy/setup.bash
source /home/f1tenth/f12026_ws/install/setup.bash
source "$HERE/install/setup.bash"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export LD_LIBRARY_PATH="/home/f1tenth/f1tenth_real_migration_ws_20260829/reference_sources/joint_mpc_20260926/acados_install/lib:${LD_LIBRARY_PATH:-}"
exec python3 "$HERE/shadow.py" "$@"
