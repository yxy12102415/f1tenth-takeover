#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export FRENET_TRACKER=dynamic_mpc
exec bash candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/launch.sh "${1:-run}"
