#!/usr/bin/env bash
# User-invoked physical entrypoints. Offline mode never opens hardware drivers.
set -euo pipefail
cpp_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd "$cpp_root/../.." && pwd)
cd "$cpp_root"
mode=${1:-run}
case "${FRENET_TRACKER:-pid}" in pp|pid|dynamic_mpc) ;; *) echo 'Invalid FRENET_TRACKER' >&2; exit 2;; esac
case "$mode" in run|start|zero|offline) ;; *) echo 'Use run, start, zero, or offline' >&2; exit 2;; esac
if [[ $mode == offline ]]; then
  exec bash start.sh --offline-self-test --fault window --duration 90
fi
exec python3 "$workspace/cpp_vehicle_session.py" "${FRENET_TRACKER:-pid}" "$mode"
