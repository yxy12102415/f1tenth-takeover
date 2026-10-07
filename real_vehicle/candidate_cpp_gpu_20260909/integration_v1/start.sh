#!/usr/bin/env bash
set -eo pipefail
INTEGRATION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
INTEGRATION_DOMAIN=85
for arg in "$@"; do
  if [[ $arg == --offline-self-test ]]; then INTEGRATION_DOMAIN=83; fi
done
exec bash "$INTEGRATION_ROOT/../offline_env.sh" env ROS_DOMAIN_ID="$INTEGRATION_DOMAIN" \
  python3 "$INTEGRATION_ROOT/stand_follow.py" "$@"
