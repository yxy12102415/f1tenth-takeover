#!/usr/bin/env bash
set -euo pipefail
mpc_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$mpc_root/lib"
mpc_output=$(mktemp "$mpc_root/lib/.mpc_iteration.XXXXXX")
trap 'rm -f "$mpc_output"' EXIT
g++ -std=c++14 -O3 -fPIC -shared -ffp-contract=off -Wall -Wextra -Werror \
  -I/usr/include/eigen3 "$mpc_root/src/mpc_iteration.cpp" \
  "$mpc_root/src/dynamic_mpc.cpp" -o "$mpc_output"
mv "$mpc_output" "$mpc_root/lib/libmpc_iteration.so"
