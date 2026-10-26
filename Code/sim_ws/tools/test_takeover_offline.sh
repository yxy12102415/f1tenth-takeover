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
g++ -O1 -std=c++14 -I/usr/include/eigen3 -I/usr/include/jsoncpp \
  -I"$V8_ROOT/src/cpp_frenet" -I"$V8_ROOT/generated_joint" \
  -I"$ACADOS_PREFIX/include" -I"$ACADOS_PREFIX/include/blasfeo/include" \
  -I"$ACADOS_PREFIX/include/hpipm/include" tools/test_takeover_preview.cpp \
  "$V8_ROOT/src/dynamic_mpc.cpp" "$V8_ROOT/src/mpc_iteration.cpp" \
  -Lbuild/takeover_sim -L"$ACADOS_PREFIX/lib" -Wl,-rpath-link,"$ACADOS_PREFIX/lib" \
  -ltakeover_joint_solver -lacados -lhpipm -lblasfeo -ljsoncpp \
  -o build/takeover_offline_tests/test_preview
LD_LIBRARY_PATH="build/takeover_sim:$ACADOS_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  build/takeover_offline_tests/test_preview takeover_assets/Fitzpatrick_sim.json
if [ -f build/takeover_sim/libtakeover_gym_joint_solver.so ]; then
  g++ -O1 -std=c++14 -DTAKEOVER_OFFICIAL_GYM_MODEL=1 \
    -I/usr/include/eigen3 -I/usr/include/jsoncpp -I"$V8_ROOT/src/cpp_frenet" \
    -Ibuild/takeover_sim/gym_model/generated_joint -I"$ACADOS_PREFIX/include" \
    -I"$ACADOS_PREFIX/include/blasfeo/include" -I"$ACADOS_PREFIX/include/hpipm/include" \
    tools/test_takeover_preview.cpp "$V8_ROOT/src/dynamic_mpc.cpp" "$V8_ROOT/src/mpc_iteration.cpp" \
    -Lbuild/takeover_sim -L"$ACADOS_PREFIX/lib" -Wl,-rpath-link,"$ACADOS_PREFIX/lib" \
    -ltakeover_gym_joint_solver -lacados -lhpipm -lblasfeo -ljsoncpp \
    -o build/takeover_offline_tests/test_gym_preview
  LD_LIBRARY_PATH="build/takeover_sim:$ACADOS_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
    build/takeover_offline_tests/test_gym_preview takeover_assets/Fitzpatrick_sim.json
fi
python3 tools/test_takeover_geometry.py
python3 tools/test_tracker_side_geometry.py
python3 tools/test_sim_clock.py
g++ -O1 -std=c++14 -I/usr/include/jsoncpp -Isrc/takeover_sim/include \
  tools/test_sqp_socket.cpp -ljsoncpp -o build/takeover_offline_tests/test_sqp_socket
build/takeover_offline_tests/test_sqp_socket tools/sqp_socket_test_fixture.py
if [ -f ../../.deps/predictive-spliner-upstream/src/sqp_avoidance_node.py ]; then
  python3 tools/test_upstream_sqp.py
else
  echo 'Optional upstream SQP numerical tests skipped: external checkout absent'
fi
