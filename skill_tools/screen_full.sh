#!/bin/bash
# FULL re-run of ALL methods at agents 10,20,30,40,50 (TA auto freq=500).
# Sequential (no concurrency -> clean runtime). Fast methods first, slow LNS last.
# Per-method CSVs -> /tmp/screen_full/<m>.csv ; combined -> /tmp/screen_full_all.csv (rebuilt each method).
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen_full; mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-1000}"

METHODS=(
"TP-STA*" "TPTS-STA*" "CENTRAL-ECBS" "CENTRAL-ECBS-SIPP"
"TA-Hybrid-STA*" "TA-Prioritized-STA*"
"Hungarian+PBS-MLA*" "Hungarian+wPBS-MLA*"
"Hungarian+PBS-MLSIPP" "Hungarian+wPBS-MLSIPP"
"TP-SIPP" "TPTS-SIPP" "Hungarian+PP-SIPP"
"LNS(1s)+PBS-MLA*" "LNS(1s)+wPBS-MLA*"
"LNS(1s)+PBS-MLSIPP" "LNS(1s)+wPBS-MLSIPP" "LNS(1s)+PP-SIPP"
)

for M in "${METHODS[@]}"; do
  SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ FULL: $M ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "10,20,30,40,50" "$OUT/$SAFE.csv"
  # rebuild combined from all per-method csvs done so far
  { echo "method,agents,freq,makespan,swt,runtime_s,status";
    for f in "$OUT"/*.csv; do tail -n +2 "$f"; done; } > /tmp/screen_full_all.csv
  echo "[PERSIST-READY] $M done -> /tmp/screen_full_all.csv" >&2
done
echo "FULL DONE -> /tmp/screen_full_all.csv" >&2
