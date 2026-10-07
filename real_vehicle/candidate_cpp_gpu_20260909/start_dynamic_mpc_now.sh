#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$ROOT/dynamic_mpc_test_workflow_20260911"
exec env DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  python3 workflow.py drive-cached \
  --ground-confirmed --cutoff-confirmed \
  --route-clear-confirmed --route-start-confirmed
