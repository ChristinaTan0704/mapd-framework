#!/bin/bash
# Watch the full run: persist live results into all_results.csv every 5 min,
# do a final persist when the dispatcher prints its DONE flag, then exit.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
LOG=/tmp/screen_full_run.log
CELLS=/tmp/screen_full/cells.csv
snap_and_persist() {
  { echo "method,agents,freq,makespan,swt,runtime_s,status"; cat "$CELLS" 2>/dev/null; } > /tmp/screen_live.csv
  python3 "$ROOT/skill_tools/persist_results.py" /tmp/screen_live.csv 2>/dev/null
}
while ! grep -q "XARGS FULL DONE" "$LOG" 2>/dev/null; do
  snap_and_persist
  echo "[watch] $(wc -l < "$CELLS" 2>/dev/null | tr -d ' ')/570 cells persisted"
  sleep 300
done
snap_and_persist
echo "[watch] FINAL persist done — $(wc -l < "$CELLS" | tr -d ' ')/570 cells"
echo "ALL PERSISTED"
