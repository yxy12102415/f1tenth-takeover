#!/usr/bin/env bash
set -eo pipefail
GROUND_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
GROUND_DOMAIN=85
for arg in "$@"; do
  if [[ $arg == --offline-self-test ]]; then GROUND_DOMAIN=83; fi
done
exec bash "$GROUND_ROOT/../offline_env.sh" env ROS_DOMAIN_ID="$GROUND_DOMAIN" FRENET_TRACKER="${FRENET_TRACKER:-dynamic_mpc}" \
  python3 "$GROUND_ROOT/stand_follow.py" "$@"
