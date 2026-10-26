#!/usr/bin/env bash
# Sequential, localhost-only trials; every failed result is retained.
set -uo pipefail
cd "$(dirname "$0")"
REPEATS="${1:-3}"
PREFIX="${2:-double_$(date -u +%Y%m%dT%H%M%SZ)}"
PROFILE="${3:-baseline}"
[[ "$REPEATS" =~ ^[1-9][0-9]*$ ]] || { echo 'Repeats must be positive' >&2; exit 2; }
[[ "$PREFIX" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Invalid run prefix' >&2; exit 2; }
case "$PROFILE" in
  baseline) scenarios=(fitzpatrick_double_overtake fitzpatrick_double_lateral) ;;
  fast) scenarios=(fitzpatrick_double_fast fitzpatrick_double_fast_lateral) ;;
  3p0_1p8) scenarios=(fitzpatrick_double_3p0_1p8 fitzpatrick_double_3p0_1p8_lateral) ;;
  *) echo 'Profile must be baseline, fast or 3p0_1p8' >&2; exit 2 ;;
esac
failed=0
for scenario in "${scenarios[@]}"; do
  for ((seed=1; seed<=REPEATS; ++seed)); do
    if ! bash run_takeover_trial.sh "$scenario" '' "$seed" "${PREFIX}_seed${seed}" false; then
      failed=1
    fi
  done
done
python3 tools/report_timed_consistency.py results/takeover "$PREFIX" \
  --expected-runs "$((2*REPEATS))" --require-clean || failed=1
exit "$failed"
