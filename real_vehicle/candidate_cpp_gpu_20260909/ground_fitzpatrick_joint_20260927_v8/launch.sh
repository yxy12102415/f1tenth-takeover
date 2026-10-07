#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ${1:-run} == offline ]]; then
  exec bash start.sh --offline-self-test --fault window --duration 90
fi
exec python3 vehicle_session.py "${FRENET_TRACKER:-pid}" "${1:-run}"
