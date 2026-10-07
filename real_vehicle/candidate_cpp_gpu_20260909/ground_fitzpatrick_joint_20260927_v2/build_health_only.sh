#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
bash ../offline_env.sh cmake -S src/ground_pp -B build_health_v13 -DCMAKE_BUILD_TYPE=Release
bash ../offline_env.sh cmake --build build_health_v13 --target health_interlock_node -j1
cp build_health_v13/health_interlock_node install_v1/ground_pp/lib/ground_pp/health_interlock_node
