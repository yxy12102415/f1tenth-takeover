#!/usr/bin/env bash
# Build only; no drivers or ROS nodes are launched.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
source /opt/ros/foxy/setup.bash
source /home/f1tenth/f12026_ws/install/setup.bash
export ACADOS_INSTALL_DIR=/home/f1tenth/f1tenth_real_migration_ws_20260829/reference_sources/joint_mpc_20260926/acados_install
cd "$REPO"
colcon --log-base "$HERE/log" build --base-paths Code/sim_ws/src/takeover_sim --build-base "$HERE/build" --install-base "$HERE/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
