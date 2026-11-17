#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python3 review/test_tight_profile.py
bash review/test_delayed_departure_ros.sh
python3 review/accept_offset_wall_revision.py
python3 freeze.py
sha256sum --check --quiet runtime_manifest.sha256
