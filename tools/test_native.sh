#!/usr/bin/env bash
# Model checks only. No ROS nodes, publishers, serial devices or hardware access.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 test_two_obstacles.py
python3 validate_servo_response.py
