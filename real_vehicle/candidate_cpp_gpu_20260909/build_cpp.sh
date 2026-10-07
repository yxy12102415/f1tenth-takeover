#!/usr/bin/env bash
set -eo pipefail
CANDIDATE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$CANDIDATE_ROOT"
exec bash ../baseline_pf_pp_20260908/clean_env.sh colcon --log-base log_v1 build \
  --base-paths src --build-base build_v1 --install-base install_v1 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
