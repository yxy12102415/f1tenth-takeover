#!/usr/bin/env bash
# Separate build-time model generation. Does not touch shipped real-v8 codegen.
set -euo pipefail
cd "$(dirname "$0")/.."
repo_root=$(cd ../.. && pwd)
generator="$repo_root/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/generate_joint_solver.py"
output="$PWD/build/takeover_sim/gym_model"
python="$repo_root/.deps/mpc_codegen_env/bin/python"
if [ ! -x "$python" ]; then
  echo 'Create .deps/mpc_codegen_env with casadi==3.6.6, future-fstrings, jinja2; see docs/LAPTOP_TAKEOVER_SIM.md' >&2
  exit 2
fi
if [ -f "$output/generated_joint/acados_solver_joint_vehicle.c" ] &&
   [ "$output/generated_joint/acados_solver_joint_vehicle.c" -nt "$generator" ]; then exit 0; fi
export TAKEOVER_MODEL_PROFILE=official_gym_native TAKEOVER_GENERATE_ONLY=1 TAKEOVER_CODEGEN_DIR="$output"
export ACADOS_SOURCE_DIR="$repo_root/.deps/acados-v0.4.0"
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$python" "$generator"
