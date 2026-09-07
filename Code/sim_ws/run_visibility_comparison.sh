#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

DURATION="${1:-12}"
REPETITIONS="${2:-5}"
BATCH_ID="${3:-$(date -u +%Y%m%dT%H%M%SZ)}"
SCENARIO="constant_speed"
BATCH_DIR="$PWD/results/visibility_comparison/$BATCH_ID"
DESIGN_FILE="$BATCH_DIR/design.csv"

if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Duration must be a positive number of seconds." >&2
  exit 2
fi
if ! [[ "$REPETITIONS" =~ ^[1-9][0-9]*$ ]]; then
  echo "Repetitions must be a positive integer." >&2
  exit 2
fi
if [ -e "$BATCH_DIR" ]; then
  echo "Refusing to overwrite comparison batch: $BATCH_DIR" >&2
  exit 2
fi

mkdir -p "$BATCH_DIR"
printf '%s\n' \
  'block,expected_side,map_name,method,run_id,summary_path' > "$DESIGN_FILE"

conditions=(left right)
methods=(corridor visibility)
for repetition in $(seq 0 $((REPETITIONS - 1))); do
  printf -v repetition_label '%02d' "$((repetition + 1))"
  for condition_index in "${!conditions[@]}"; do
    expected_side="${conditions[$condition_index]}"
    map_name="benchmark_straight_visibility_${expected_side}"
    # Rotate order within each repetition and mirrored condition.  A complete
    # trial is one replicate; its 20 Hz samples are repeated measurements.
    order_offset=$(((repetition + condition_index) % ${#methods[@]}))
    for method_index in "${!methods[@]}"; do
      rotated_index=$(((method_index + order_offset) % ${#methods[@]}))
      method="${methods[$rotated_index]}"
      run_id="${BATCH_ID}_${expected_side}_r${repetition_label}_${method}"
      summary_path="$PWD/results/$SCENARIO/$run_id/summary.json"
      printf '%s,%s,%s,%s,%s,%s\n' \
        "$repetition_label" "$expected_side" "$map_name" "$method" \
        "$run_id" "$summary_path" >> "$DESIGN_FILE"
      ./run_baseline_trial.sh \
        "$SCENARIO" "$DURATION" "$run_id" "$map_name" "$method"
    done
  done
done

python3 ./summarize_visibility_comparison.py \
  "$DESIGN_FILE" "$BATCH_DIR/summary.json"
echo "Visibility comparison complete: $BATCH_DIR/summary.json"
