#!/usr/bin/env bash
set -euo pipefail
cpp_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$cpp_root"
python3 export_assets.py
bash build_mpc_native.sh
mkdir -p bin run_logs
g++ -std=c++14 -O3 -ffp-contract=off -I/usr/include/eigen3 \
  src/cpp_frenet/probe.cpp -Llib -lmpc_iteration -Wl,--disable-new-dtags,-rpath,'$ORIGIN/../lib' \
  -Igenerated_joint -I../../reference_sources/joint_mpc_20260926/acados_install/include \
  -I../../reference_sources/joint_mpc_20260926/acados_install/include/blasfeo/include \
  -I../../reference_sources/joint_mpc_20260926/acados_install/include/hpipm/include \
  -lacados_ocp_solver_joint_vehicle -lacados -lhpipm -lblasfeo -ljsoncpp -o bin/core_probe
CMAKE_BUILD_PARALLEL_LEVEL=2 bash ../offline_env.sh colcon --log-base log_cpp build \
  --base-paths src/cpp_frenet --build-base build_cpp --install-base install_cpp \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
