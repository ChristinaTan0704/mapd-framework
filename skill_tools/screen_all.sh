#!/bin/bash
# Screen ALL methods at agents 10 & 50 across all freqs. Produces /tmp/screen/<method>.csv
# and a combined /tmp/screen_all.csv (cols: method,agents,freq,makespan,swt,runtime_s,status)
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen
mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-180}"

METHODS=(
"TP-STA*" "TPTS-STA*" "CENTRAL-ECBS" "CENTRAL-ECBS-SIPP"
"TA-Hybrid-STA*" "TA-Prioritized-STA*"
"Hungarian+PBS-MLA*" "Hungarian+wPBS-MLA*"
"LNS(1s)+PBS-MLA*" "LNS(1s)+wPBS-MLA*"
"Hungarian+PBS-MLSIPP" "Hungarian+wPBS-MLSIPP"
"LNS(1s)+PBS-MLSIPP" "LNS(1s)+wPBS-MLSIPP"
"TP-SIPP" "TPTS-SIPP"
"Hungarian+PP-SIPP" "LNS(1s)+PP-SIPP"
)

COMBINED=/tmp/screen_all.csv
echo "method,agents,freq,makespan,swt,runtime_s,status" > "$COMBINED"
for M in "${METHODS[@]}"; do
  SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ SCREENING: $M ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "10,50" "$OUT/$SAFE.csv"
  tail -n +2 "$OUT/$SAFE.csv" >> "$COMBINED"
done
echo "ALL DONE -> $COMBINED" >&2
