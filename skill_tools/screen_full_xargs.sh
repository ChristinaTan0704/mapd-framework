#!/bin/bash
# FULL re-run of ALL methods, parallelized per-CELL with a hard cap of MAXPAR via xargs -P.
# Each xargs job runs exactly ONE mapd -> at most MAXPAR concurrent mapd processes.
# NOTE: runtimes under parallel load are inflated; makespan/SWT/collision are unaffected.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
OUT=/tmp/screen_full; mkdir -p "$OUT"
export TIMEOUT="${TIMEOUT:-1000}"
export CELLOUT="$OUT/cells.csv"
MAXPAR="${MAXPAR:-10}"
: > "$CELLOUT"

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
FREQS=(0.2 0.5 1 2 5 10 500)

JOBS="$OUT/jobs.txt"; : > "$JOBS"
for M in "${METHODS[@]}"; do
  for AG in "${AGENTS[@]}"; do
    if [[ "$M" == TA-* ]]; then FL=(500); else FL=("${FREQS[@]}"); fi
    for F in "${FL[@]}"; do
      echo "${M}|${AG}|${F}" >> "$JOBS"
    done
  done
done
echo "total cells: $(wc -l < "$JOBS" | tr -d ' ')  | MAXPAR=$MAXPAR TIMEOUT=$TIMEOUT" >&2

# xargs -I reads one line per job (newline-delimited); method labels contain no spaces.
cat "$JOBS" | xargs -P "$MAXPAR" -I CELL "$ROOT/skill_tools/run_cell.sh" "CELL"

# Build combined csv with header
{ echo "method,agents,freq,makespan,swt,runtime_s,status"; cat "$CELLOUT"; } > /tmp/screen_full_all.csv
echo "XARGS FULL DONE -> /tmp/screen_full_all.csv ($(wc -l < "$CELLOUT" | tr -d ' ') cells)" >&2
