#!/usr/bin/env bash
set -eo pipefail
BASELINE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /bin/bash "$BASELINE_ROOT/clean_env.sh" /usr/bin/python3 "$BASELINE_ROOT/sensor_only.py" "$@"
