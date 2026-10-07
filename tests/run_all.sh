#!/usr/bin/env bash
# tests/run_all.sh quick|standard|full
#   quick     T1 T2 T3a                      ~8 min   -> is it the drone, the timing, or SPH?
#   standard  quick + T3b T4 T8              ~35 min
#   full      standard + T5 T7 T9 T10 T11    ~2 h
# Afterwards: one comparison table of all runs (src/logs/runs/compare_<batch>/compare.md)
set -u
cd "$(dirname "$0")/.." || exit 1
LEVEL="${1:-quick}"
case "$LEVEL" in
  quick)    SEQ="T1 T2 T3a" ;;
  standard) SEQ="T1 T2 T3a T3b T4 T8" ;;
  full)     SEQ="T1 T2 T3a T3b T4 T8 T5 T7 T9 T10 T11" ;;
  *) echo "usage: $0 quick|standard|full"; exit 1 ;;
esac
BATCH="$(date +%Y%m%d_%H%M%S)_${LEVEL}"
mkdir -p src/logs/runs 2>/dev/null
export IDS_FILE="$PWD/src/logs/runs/${BATCH}_ids.txt"
: > "$IDS_FILE"
for t in $SEQ; do tests/run_test.sh "$t"; done
mapfile -t IDS < "$IDS_FILE"
if [ "${#IDS[@]}" -gt 0 ]; then
  args=(); for i in "${IDS[@]}"; do args+=("src/logs/runs/$i"); done
  python3 src/analyze_run.py --compare "${args[@]}" --out "src/logs/runs/compare_${BATCH}"
fi
echo "batch done: $IDS_FILE"
