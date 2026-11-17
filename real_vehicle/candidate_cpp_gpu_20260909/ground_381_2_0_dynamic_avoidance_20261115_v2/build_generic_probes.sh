#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p bin
g++ -std=c++14 -O3 -ffp-contract=off -I/usr/include/eigen3 src/cpp_frenet/generic_probe.cpp -ljsoncpp -o bin/generic_probe
g++ -std=c++14 -O3 -ffp-contract=off -I/usr/include/eigen3 \
  src/cpp_frenet/probe.cpp -Llib -lmpc_iteration -Wl,--disable-new-dtags,-rpath,'$ORIGIN/../lib' \
  -Igenerated_joint -I../../reference_sources/joint_mpc_20260926/acados_install/include \
  -I../../reference_sources/joint_mpc_20260926/acados_install/include/blasfeo/include \
  -I../../reference_sources/joint_mpc_20260926/acados_install/include/hpipm/include \
  -lacados_ocp_solver_joint_vehicle -lacados -lhpipm -lblasfeo -ljsoncpp -o bin/core_probe
