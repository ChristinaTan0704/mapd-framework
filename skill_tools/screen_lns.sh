#!/bin/bash
# Screen the 5 slow LNS methods at agents 10 & 50, all freqs, with a 600s/cell timeout.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen
mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-600}"
METHODS=(
"LNS(1s)+PBS-MLA*" "LNS(1s)+wPBS-MLA*"
"LNS(1s)+PBS-MLSIPP" "LNS(1s)+wPBS-MLSIPP"
"LNS(1s)+PP-SIPP"
)
for M in "${METHODS[@]}"; do
  SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ SCREENING: $M ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "10,50" "$OUT/$SAFE.csv"
done
echo "LNS DONE" >&2
