#!/usr/bin/env bash
# Explicit simulation experiment; same arguments as run_takeover_trial.sh.
set -euo pipefail
cd "$(dirname "$0")"
export TAKEOVER_PLANNING_BACKEND=upstream_sqp
export TAKEOVER_PREDICTIVE_SPEED_PROFILE=true
export TAKEOVER_PROGRESSIVE_PASS_SPEED=false
export TAKEOVER_EARLY_VERIFIED_RETURN=false
exec bash run_takeover_trial.sh "$@"
