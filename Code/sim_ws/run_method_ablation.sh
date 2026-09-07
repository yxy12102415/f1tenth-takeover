#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

SCENARIO="${1:-turning}"
DURATION="${2:-30}"
MAP_NAME="${3:-benchmark_occlusion_a_obstacle_center}"
REPETITIONS="${4:-5}"
BATCH_ID="${5:-$(date -u +%Y%m%dT%H%M%SZ)}"
METHODS=(nominal corridor visibility adaptive full)

if ! [[ "$REPETITIONS" =~ ^[1-9][0-9]*$ ]]; then
  echo "Repetitions must be a positive integer." >&2
  exit 2
fi

# Rotate method order inside each matched repetition. This balances first/last
# run effects without introducing a hidden random seed into a reproducibility
# script. A trial is the replicate; 20 Hz samples inside it are not replicates.
for repetition in $(seq 0 $((REPETITIONS - 1))); do
  for method_index in $(seq 0 $((${#METHODS[@]} - 1))); do
    rotated_index=$(((method_index + repetition) % ${#METHODS[@]}))
    method_name="${METHODS[$rotated_index]}"
    printf -v repetition_label '%02d' "$((repetition + 1))"
    run_id="${BATCH_ID}_r${repetition_label}_${method_name}"
    ./run_baseline_trial.sh \
      "$SCENARIO" "$DURATION" "$run_id" "$MAP_NAME" "$method_name"
  done
done

echo "Ablation batch complete: $BATCH_ID"
