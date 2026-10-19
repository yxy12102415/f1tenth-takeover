#!/usr/bin/env bash
# Serial, headless Fitzpatrick suite; failures remain in the output directory.
set -u
cd "$(dirname "$0")"
REPETITIONS="${1:-3}"
TAG="${2:-fitz_$(date -u +%Y%m%dT%H%M%SZ)}"
[[ "$REPETITIONS" =~ ^[1-9][0-9]*$ ]] || exit 2
STATUS=0
for scenario in fitzpatrick_clear fitzpatrick_constant_speed fitzpatrick_1p5 fitzpatrick_occlusion fitzpatrick_follow; do
  for seed in $(seq 1 "$REPETITIONS"); do
    ./run_takeover_trial.sh "$scenario" "" "$seed" "${TAG}_seed${seed}" false || STATUS=1
  done
done
python3 tools/summarize_takeover_batch.py results/takeover "$TAG" || STATUS=1
exit "$STATUS"
