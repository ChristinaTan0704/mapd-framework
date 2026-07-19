#!/bin/bash
# Screen the remaining FAST (non-LNS) methods at agents 10 & 50.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen
mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-180}"
METHODS=(
"Hungarian+PBS-MLSIPP" "Hungarian+wPBS-MLSIPP"
"TP-SIPP" "TPTS-SIPP" "Hungarian+PP-SIPP"
)
for M in "${METHODS[@]}"; do
  SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ SCREENING: $M ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "10,50" "$OUT/$SAFE.csv"
done
echo "FAST DONE" >&2
