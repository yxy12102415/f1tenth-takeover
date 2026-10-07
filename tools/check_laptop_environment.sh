#!/usr/bin/env bash
# Read-only host audit for the takeover simulation.
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
acados_prefix="${ACADOS_INSTALL_DIR:-$repo_root/.deps/acados-v0.4.0}"

printf 'os='; . /etc/os-release && printf '%s %s\n' "$ID" "$VERSION_ID"
printf 'architecture=%s\n' "$(uname -m)"
printf 'cpu_threads=%s\n' "$(nproc)"
printf 'memory_gib='; awk '/MemTotal/ {printf "%.1f\n", $2/1024/1024}' /proc/meminfo
printf 'compiler=%s\n' "$(g++ -dumpfullversion -dumpversion 2>/dev/null || echo missing)"
printf 'cmake=%s\n' "$(cmake --version 2>/dev/null | awk 'NR==1 {print $3}' || echo missing)"
if command -v docker >/dev/null 2>&1; then
  printf 'docker=%s\n' "$(docker --version | sed 's/,.*//')"
else
  printf 'docker=missing (not required for native Ubuntu)\n'
fi
if [ -f /opt/ros/humble/setup.bash ]; then
  printf 'ros=humble\n'
else
  printf 'ros=missing\n'
fi
if [ -f "$acados_prefix/lib/libacados.so" ]; then
  printf 'acados_prefix=%s\n' "$acados_prefix"
  if [ -d "$acados_prefix/.git" ]; then
    printf 'acados_commit=%s\n' "$(git -C "$acados_prefix" rev-parse HEAD)"
  fi
else
  printf 'acados=missing (run tools/setup_laptop.sh)\n'
fi
python3 - <<'PY'
import importlib.util
for name in ('numpy', 'scipy', 'PIL', 'yaml', 'matplotlib', 'f110_gym'):
    print(f'python_{name}=' + ('ok' if importlib.util.find_spec(name) else 'missing'))
PY
