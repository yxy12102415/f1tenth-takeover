#!/bin/bash
# Restore unchanged upstream components; keep project-specific source in Git.
set -euo pipefail
source_dir="$(cd "$(dirname "$0")/.." && pwd)"
template_commit=fa03ce64f9aea9d750601aa92ce9893f72066b0a
arduino_commit=5e19e086c43d0fa5e5a596497ff8f11a0a43f6c2
if [[ -f "$source_dir/components/arduino/CMakeLists.txt" &&
      -f "$source_dir/components/bluepad32/CMakeLists.txt" &&
      -f "$source_dir/components/btstack/CMakeLists.txt" ]]; then
    exit 0
fi
# Never overwrite partially restored or locally modified dependencies.
if [[ -e "$source_dir/components" ]]; then
    echo "Incomplete components directory: move it aside before restoring dependencies." >&2
    exit 1
fi
cache_dir="$HOME/.cache/hosim-bluepad32-$template_commit"
mkdir -p "$(dirname "$cache_dir")"
if [[ ! -d "$cache_dir/.git" ]]; then
    git clone --depth 1 --branch 4.1.0 \
        https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git "$cache_dir"
fi
[[ "$(git -C "$cache_dir" rev-parse HEAD)" == "$template_commit" ]] || {
    echo "Unexpected template revision in $cache_dir" >&2; exit 1;
}
git -C "$cache_dir" submodule update --init --depth 1 components/arduino
[[ "$(git -C "$cache_dir/components/arduino" rev-parse HEAD)" == "$arduino_commit" ]] || {
    echo "Unexpected Arduino revision" >&2; exit 1;
}
if [[ -n "$(git -C "$cache_dir" status --porcelain -- components)" ]]; then
    echo "Dependency cache has local changes; refusing to copy it." >&2
    exit 1
fi
# Stage beside the destination so an interrupted copy cannot look complete.
stage_dir="$(mktemp -d "$source_dir/.components.XXXXXX")"
trap 'rm -rf "$stage_dir"' EXIT
rsync -a --exclude='.git' "$cache_dir/components/" "$stage_dir/"
mv "$stage_dir" "$source_dir/components"
echo "Restored pinned Bluepad32 / Arduino / BTstack components."
