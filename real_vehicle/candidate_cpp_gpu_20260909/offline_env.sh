#!/usr/bin/env bash
set -eo pipefail
CANDIDATE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$CANDIDATE_ROOT/../baseline_pf_pp_20260908/clean_env.sh" \
  bash --noprofile --norc -c '
    source "$BASELINE_ROOT/install_groundprep_v3/local_setup.bash"
    exec "$@"
  ' candidate-offline "$@"
