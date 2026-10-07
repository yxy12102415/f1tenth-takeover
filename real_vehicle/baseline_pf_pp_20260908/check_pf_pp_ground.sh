#!/usr/bin/env bash
set -eo pipefail
GROUND_CHECK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 "$GROUND_CHECK_ROOT/ground_preflight.py" "$@"
