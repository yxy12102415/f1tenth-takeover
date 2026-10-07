#!/usr/bin/env bash
# Build only the independent clone; never install globally or replace native_v1.
set -eo pipefail
CANDIDATE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$CANDIDATE_ROOT/upstream_range_libc/pywrapper"
exec bash "$CANDIDATE_ROOT/../baseline_pf_pp_20260908/clean_env.sh" env WITH_CUDA=ON \
  python3 setup.py build_ext --build-lib "$CANDIDATE_ROOT/gpu_native_v1" \
  --build-temp "$CANDIDATE_ROOT/gpu_build_v1"
