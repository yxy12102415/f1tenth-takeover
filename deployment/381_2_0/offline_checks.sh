#!/usr/bin/env bash
# Native numerical regressions, no ROS nodes or devices.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SIM="$REPO/Code/sim_ws"
V8="$REPO/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8"
ACADOS=/home/f1tenth/f1tenth_real_migration_ws_20260829/reference_sources/joint_mpc_20260926/acados_install
FIXTURE=/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/runtime_assets.json
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$HERE/build/checks"
cd "$SIM"
FLAGS=(-O1 -std=c++14 -I/usr/include/eigen3 -I/usr/include/jsoncpp -Isrc/takeover_sim/include -I"$V8/src/cpp_frenet" -I"$V8/generated_joint" -I"$ACADOS/include" -I"$ACADOS/include/blasfeo/include" -I"$ACADOS/include/hpipm/include")
g++ "${FLAGS[@]}" tools/test_takeover_candidates.cpp -ljsoncpp -o "$HERE/build/checks/candidates"
"$HERE/build/checks/candidates"
g++ "${FLAGS[@]}" tools/test_takeover_prediction.cpp -ljsoncpp -o "$HERE/build/checks/prediction"
"$HERE/build/checks/prediction" "$FIXTURE"
g++ "${FLAGS[@]}" tools/test_takeover_preview.cpp "$V8/src/dynamic_mpc.cpp" "$V8/src/mpc_iteration.cpp" -L"$HERE/build/takeover_sim" -L"$ACADOS/lib" -Wl,-rpath-link,"$ACADOS/lib" -ltakeover_joint_solver -lacados -lhpipm -lblasfeo -ljsoncpp -o "$HERE/build/checks/preview"
LD_LIBRARY_PATH="$HERE/build/takeover_sim:$ACADOS/lib:${LD_LIBRARY_PATH:-}" "$HERE/build/checks/preview" "$FIXTURE"
python3 tools/test_takeover_geometry.py
python3 tools/test_tracker_side_geometry.py
python3 tools/test_sim_clock.py
python3 tools/test_stability_schedule.py
python3 tools/test_upstream_sqp.py
printf '%s\n' 'Native real-model regressions passed; these are not 381 ground acceptance.'
