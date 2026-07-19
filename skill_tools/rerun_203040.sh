#!/bin/bash
# Sequential (ONE mapd at a time -> clean runtime) re-run of agents 20,30,40 for ALL methods.
# Fast methods first, slow LNS last. Per-method CSVs -> /tmp/rr/<m>.csv ; combined -> /tmp/rr_all.csv (rebuilt each method).
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/rr; mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-2700}"
METHODS=(
"TP-STA*" "TPTS-STA*" "CENTRAL-ECBS" "CENTRAL-ECBS-SIPP"
"TA-Hybrid-STA*" "TA-Prioritized-STA*"
"Hungarian+PBS-MLA*" "Hungarian+wPBS-MLA*"
"Hungarian+PBS-MLSIPP" "Hungarian+wPBS-MLSIPP"
"TP-SIPP" "TPTS-SIPP" "Hungarian+PP-SIPP"
"LNS(1s)+PBS-MLSIPP" "LNS(1s)+wPBS-MLSIPP" "LNS(1s)+PP-SIPP"
"LNS(1s)+PBS-MLA*" "LNS(1s)+wPBS-MLA*"
)
for M in "${METHODS[@]}"; do
  SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ $M (20,30,40) ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "20,30,40" "$OUT/$SAFE.csv"
  { echo "method,agents,freq,makespan,swt,runtime_s,status"; for f in "$OUT"/*.csv; do tail -n +2 "$f"; done; } > /tmp/rr_all.csv
  echo "[PERSIST-READY] $M done" >&2
done
echo "RERUN 203040 DONE -> /tmp/rr_all.csv" >&2
