#!/usr/bin/env bash
# Offline only: clean domain 83, synthetic sensors, virtual vehicle sink.
set -eo pipefail
PREP_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$PREP_ROOT"
bash clean_env.sh bash --noprofile --norc -c '
  set -e
  source "$BASELINE_ROOT/install_groundprep_v3/local_setup.bash"
  python3 -m pytest test -q
'
# Sequential: every integration test deliberately owns the same session lock.
for fault in window release guard_kill window_kill supervisor_permit_loss supervisor_main_stall; do
  bash start_pf_pp_stand.sh --offline-self-test --pose-timeout 0.30 --fault "$fault"
done
