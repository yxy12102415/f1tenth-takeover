#!/usr/bin/env bash
# Build source-export probes only; never launch ROS or a vehicle driver.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
release="$repo_root/real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v5"
: "${ACADOS_INSTALL_DIR:?Set ACADOS_INSTALL_DIR to the acados v0.4.0 installation prefix}"
acados_prefix=$(realpath "$ACADOS_INSTALL_DIR")
[[ -f "$acados_prefix/include/acados_c/ocp_nlp_interface.h" && -f "$acados_prefix/lib/libacados.so" ]]
mkdir -p "$release/bin" "$release/lib"
python3 "$release/export_assets.py"
include_flags=(-I"$acados_prefix/include" -I"$acados_prefix/include/blasfeo/include" -I"$acados_prefix/include/hpipm/include" -I"$release/generated_joint")
solver_sources=("$release/generated_joint/acados_solver_joint_vehicle.c" "$release"/generated_joint/joint_vehicle_model/*.c "$release"/generated_joint/joint_vehicle_cost/*.c "$release"/generated_joint/joint_vehicle_constraints/*.c)
cc -O2 -fPIC -shared "${include_flags[@]}" "${solver_sources[@]}" \
  -L"$acados_prefix/lib" -lacados -lhpipm -lblasfeo -lm \
  -Wl,--disable-new-dtags,-rpath,"$acados_prefix/lib" \
  -o "$release/lib/libacados_ocp_solver_joint_vehicle.so"
bash "$release/build_mpc_native.sh"
for probe in core_probe compact_probe; do
  source_file="$release/src/cpp_frenet/probe.cpp"
  if [[ "$probe" == compact_probe ]]; then source_file="$repo_root/deployment/381_2_0/compact_probe.cpp"; fi
  g++ -std=c++14 -O3 -ffp-contract=off -I/usr/include/eigen3 \
    -I"$release/src/cpp_frenet" "${include_flags[@]}" "$source_file" \
    -L"$release/lib" -L"$acados_prefix/lib" \
    -lmpc_iteration -lacados_ocp_solver_joint_vehicle -lacados -lhpipm -lblasfeo -ljsoncpp \
    -Wl,--disable-new-dtags,-rpath,'$ORIGIN/../lib' -Wl,-rpath,"$acados_prefix/lib" \
    -o "$release/bin/$probe"
done
printf '%s\n' "Built non-actuating v5 probes in $release/bin"
