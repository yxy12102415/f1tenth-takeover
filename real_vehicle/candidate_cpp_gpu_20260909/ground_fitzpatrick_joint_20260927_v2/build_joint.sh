#!/usr/bin/env bash
set -euo pipefail
joint_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
joint_tools="$joint_root/../../reference_sources/joint_mpc_20260926"
export ACADOS_SOURCE_DIR="$joint_tools/acados_install"
export ACADOS_PYTHON_INTERFACE_PATH="$joint_tools/acados/interfaces/acados_template/acados_template"
export PYTHONPATH="$joint_tools/generator_packages:$joint_tools/acados/interfaces/acados_template:${PYTHONPATH:-}"
export TERA_PATH="$joint_tools/acados/interfaces/acados_template/tera_renderer/target/release/t_renderer"
export LD_LIBRARY_PATH="$joint_tools/acados_install/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 "$joint_root/generate_joint_solver.py"
cp "$joint_tools"/acados_install/lib/lib*.so "$joint_root/lib/"
cp "$joint_root/generated_joint/libacados_ocp_solver_joint_vehicle.so" "$joint_root/lib/"
