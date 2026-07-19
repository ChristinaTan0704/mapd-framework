#!/bin/bash
# (b)+(c): sequential (no concurrency -> clean runtime) LNS screen after the collision fix.
#  - LNS(1s)+wPBS-MLSIPP : FULL grid (10,20,30,40,50) — shares the fixed windowed-SIPP path
#  - other 4 LNS methods  : 10,50 screen (resume broader LNS screen)
# Writes per-method CSVs to /tmp/screen2/ and a combined /tmp/screen2_all.csv.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen2; mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-1000}"
COMBINED=/tmp/screen2_all.csv
echo "method,agents,freq,makespan,swt,runtime_s,status" > "$COMBINED"

run() {  # method  agents
  local M="$1" AG="$2"; local SAFE="${M//[^A-Za-z0-9]/_}"
  echo "############ $M  agents=$AG ############" >&2
  "$ROOT/skill_tools/run_method.sh" "$M" "$AG" "$OUT/$SAFE.csv"
  tail -n +2 "$OUT/$SAFE.csv" >> "$COMBINED"
}

# (b) full-grid verify of the method whose code path was changed
run "LNS(1s)+wPBS-MLSIPP" "10,20,30,40,50"
# (c) broader LNS screen at 10,50
run "LNS(1s)+PBS-MLA*"    "10,50"
run "LNS(1s)+wPBS-MLA*"   "10,50"
run "LNS(1s)+PBS-MLSIPP"  "10,50"
run "LNS(1s)+PP-SIPP"     "10,50"
echo "LNS VERIFY DONE -> $COMBINED" >&2
