#!/usr/bin/env bash
set -u
cd "$(dirname "$0")"
REPETITIONS="${1:-3}"
TAG="${2:-batch_$(date -u +%Y%m%dT%H%M%SZ)}"
STATUS=0
for scenario in constant_speed accel_brake stop_go short_occlusion no_space; do
  for seed in $(seq 1 "$REPETITIONS"); do
    ./run_takeover_trial.sh "$scenario" "" "$seed" "${TAG}_seed${seed}" false || STATUS=1
  done
done
python3 tools/summarize_takeover_batch.py "results/takeover" "$TAG"
exit "$STATUS"
