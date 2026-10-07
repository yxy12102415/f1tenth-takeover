#!/usr/bin/env bash
# Executed wrapper, not a script to source. Never imports user shell/old overlays.
set -eo pipefail
BASELINE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# -eq 0 ]]; then
  printf '%s\n' 'Usage: bash clean_env.sh COMMAND [ARGS...]' >&2
  exit 2
fi
exec /usr/bin/env -i \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin LANG=C.UTF-8 \
  BASELINE_ROOT="$BASELINE_ROOT" \
  ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR="$BASELINE_ROOT/runtime_logs" \
  PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
  /bin/bash --noprofile --norc -c '
    set -e
    source /opt/ros/foxy/local_setup.bash
    source "$BASELINE_ROOT/../route_install_v12/local_setup.bash"
    source "$BASELINE_ROOT/install_v3/local_setup.bash"
    export PYTHONPATH="$BASELINE_ROOT/dependencies/native_v1:$BASELINE_ROOT/dependencies/python:$PYTHONPATH"
    exec "$@"
  ' baseline-clean "$@"
