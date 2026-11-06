#!/usr/bin/env bash
# Explicit laptop-only candidate. No physical-v8 deployment permission.
# Same arguments as run_takeover_trial.sh; validated scope is recorded in
# docs/FAST_PASS_20261106.md, not a guarantee for every accepted scenario.
set -euo pipefail
cd "$(dirname "$0")"
export TAKEOVER_PLANNING_BACKEND=upstream_sqp
export TAKEOVER_TRACK_PREDICTION=true
export TAKEOVER_TIMED_CONTROL=true
export TAKEOVER_PREDICTIVE_SPEED_PROFILE=true
export TAKEOVER_FAST_PASS_GEOMETRY=true
export TAKEOVER_FAST_PASS_RESPONSE=false
export TAKEOVER_VERIFIED_EXIT_RECOVERY=true
export TAKEOVER_CHECKED_CONTROL_RECOVERY="${TAKEOVER_CHECKED_CONTROL_RECOVERY:-true}"
export TAKEOVER_PROGRESSIVE_PASS_SPEED=false
export TAKEOVER_EARLY_VERIFIED_RETURN=false
exec bash run_takeover_trial.sh "$@"
