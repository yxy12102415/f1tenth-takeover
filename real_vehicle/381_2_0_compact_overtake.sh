#!/usr/bin/env bash
set -euo pipefail
ws_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$ws_root/compact_overtake_vehicle_session.py" "$@"
