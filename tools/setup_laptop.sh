#!/usr/bin/env bash
# Build the exact acados ABI used by the checked-in v8 generated solver.
# ROS 2 Humble and ordinary build/Python dependencies must already be present.
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
prefix="$repo_root/.deps/acados-v0.4.0"
commit=43354449192aa51557abda37d1bc17d907077354

for command in git cmake make g++ python3; do
  command -v "$command" >/dev/null || { echo "Missing required command: $command" >&2; exit 2; }
done
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo 'ROS 2 Humble is required for the native Ubuntu path.' >&2
  exit 2
fi
if [ -f "$prefix/lib/libacados.so" ] &&
   [ "$(git -C "$prefix" rev-parse HEAD 2>/dev/null || true)" = "$commit" ]; then
  echo "Exact acados v0.4.0 already installed at $prefix"
  exit 0
fi
if [ -e "$prefix" ]; then
  echo "Refusing to replace existing non-matching dependency directory: $prefix" >&2
  exit 2
fi
mkdir -p "$repo_root/.deps"
git clone --recursive https://github.com/acados/acados.git "$prefix"
git -C "$prefix" checkout "$commit"
git -C "$prefix" submodule update --init --recursive
cmake -S "$prefix" -B "$prefix/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_SHARED_LIBS=ON \
  -DACADOS_WITH_QPOASES=ON
cmake --build "$prefix/build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$prefix/build"
echo "Installed acados v0.4.0 at $prefix"
