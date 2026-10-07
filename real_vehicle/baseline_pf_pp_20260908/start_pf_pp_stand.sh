#!/usr/bin/env bash
set -eo pipefail
BASELINE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Domain 85 is dedicated to hardware stand tests; domain 83 remains offline.
STAND_DOMAIN=85
for arg in "$@"; do
  if [[ $arg == --offline-self-test ]]; then STAND_DOMAIN=83; fi
done
exec /bin/bash "$BASELINE_ROOT/clean_env.sh" /bin/bash --noprofile --norc -c '
  set -e
  source "$BASELINE_ROOT/install_groundprep_v3/local_setup.bash"
  export ROS_DOMAIN_ID="$1"
  shift
  exec /usr/bin/python3 "$BASELINE_ROOT/stand_follow.py" "$@"
' stand "$STAND_DOMAIN" "$@"
