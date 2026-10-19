#!/usr/bin/env bash
# Unit/geometry checks only. Starts no ROS graph and opens no actuator devices.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p build/takeover_offline_tests
g++ -O1 -std=c++14 -I/usr/include/eigen3 -I/usr/include/jsoncpp \
  -Isrc/takeover_sim/include \
  -I../../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/src/cpp_frenet \
  tools/test_takeover_candidates.cpp -ljsoncpp \
  -o build/takeover_offline_tests/test_candidates
build/takeover_offline_tests/test_candidates
V8_ROOT=../../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8
ACADOS_PREFIX="${ACADOS_INSTALL_DIR:-../../.deps/acados-v0.4.0}"
g++ -O1 -std=c++14 -I/usr/include/eigen3 -I/usr/include/jsoncpp \
  -Isrc/takeover_sim/include -I"$V8_ROOT/src/cpp_frenet" -I"$V8_ROOT/generated_joint" \
  -I"$ACADOS_PREFIX/include" -I"$ACADOS_PREFIX/include/blasfeo/include" \
  -I"$ACADOS_PREFIX/include/hpipm/include" tools/test_takeover_prediction.cpp -ljsoncpp \
  -o build/takeover_offline_tests/test_prediction
build/takeover_offline_tests/test_prediction takeover_assets/Fitzpatrick_sim.json
python3 tools/test_takeover_geometry.py
