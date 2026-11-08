#!/usr/bin/env bash
set -eo pipefail
INTEGRATION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$INTEGRATION_ROOT"
bash "$INTEGRATION_ROOT/build_mpc_native.sh"
exec bash ../offline_env.sh colcon --log-base log_v1 build --base-paths src \
  --build-base build_v1 --install-base install_v1 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
