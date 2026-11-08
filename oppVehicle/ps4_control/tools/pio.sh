#!/bin/bash
set -euo pipefail
source_dir="$(cd "$(dirname "$0")/.." && pwd)"
"$source_dir/tools/bootstrap.sh"
build_dir="$HOME/.cache/hosim-ps4-build"
mkdir -p "$build_dir"
# ESP-IDF 4.4 rejects source paths containing spaces. Mirror our dedicated
# project into a dedicated disposable directory, preserving the build cache.
rsync -a --delete --exclude='.git' --exclude='.pio' --exclude='sdkconfig.esp32dev' \
    "$source_dir/" "$build_dir/"
pio_bin="${PIO_BIN:-$HOME/.local/share/opp-link-tools/bin/pio}"
if [[ ! -x "$pio_bin" ]]; then
    pio_bin="$HOME/.platformio/penv/bin/pio"
fi
exec "$pio_bin" "$@" -d "$build_dir"
