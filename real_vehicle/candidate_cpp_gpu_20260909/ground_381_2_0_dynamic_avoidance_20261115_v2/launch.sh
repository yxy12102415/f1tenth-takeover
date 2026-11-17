#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd "$root/../.." && pwd)
exec python3 "$workspace/generic_avoidance_vehicle_session.py" "$@"
