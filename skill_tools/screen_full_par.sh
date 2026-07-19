#!/bin/bash
# FULL re-run of ALL methods at agents 10..50, parallelized at the (method,agent) level
# with a hard cap of MAXPAR concurrent workers (each worker runs ONE mapd at a time ->
# at most MAXPAR concurrent mapd processes).  TA methods auto-restrict to freq=500.
#
# NOTE: runtimes measured under parallel load are inflated (cores saturated -> no turbo +
# memory contention).  makespan/SWT/collision are deterministic and unaffected.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen_full; mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-1000}"
MAXPAR="${MAXPAR:-10}"

METHODS=(
"TP-STA*" "TPTS-STA*" "CENTRAL-ECBS" "CENTRAL-ECBS-SIPP"
"TA-Hybrid-STA*" "TA-Prioritized-STA*"
"Hungarian+PBS-MLA*" "Hungarian+wPBS-MLA*"
"Hungarian+PBS-MLSIPP" "Hungarian+wPBS-MLSIPP"
"TP-SIPP" "TPTS-SIPP" "Hungarian+PP-SIPP"
"LNS(1s)+PBS-MLA*" "LNS(1s)+wPBS-MLA*"
"LNS(1s)+PBS-MLSIPP" "LNS(1s)+wPBS-MLSIPP" "LNS(1s)+PP-SIPP"
)
AGENTS=(10 20 30 40 50)

# Build the job list (method,agent). Run LNS-heavy + high-agent jobs by natural order;
# the pool keeps MAXPAR busy regardless.
worker() {  # $1=method  $2=agent
  local M="$1" AG="$2" SAFE
  SAFE="${M//[^A-Za-z0-9]/_}"
  "$ROOT/skill_tools/run_method.sh" "$M" "$AG" "$OUT/${SAFE}__${AG}.csv" >/dev/null 2>&1
  echo "[CELLGROUP DONE] $M ag=$AG" >&2
}

for M in "${METHODS[@]}"; do
  for AG in "${AGENTS[@]}"; do
    # portable pool (bash 3.2): block until fewer than MAXPAR workers are running
    while [ "$(jobs -r 2>/dev/null | wc -l | tr -d ' ')" -ge "$MAXPAR" ]; do
      sleep 1
    done
    worker "$M" "$AG" &
  done
done
wait

# Combine all per-(method,agent) CSVs
{ echo "method,agents,freq,makespan,swt,runtime_s,status";
  for f in "$OUT"/*__*.csv; do tail -n +2 "$f" 2>/dev/null; done; } > /tmp/screen_full_all.csv
echo "PAR FULL DONE -> /tmp/screen_full_all.csv" >&2
