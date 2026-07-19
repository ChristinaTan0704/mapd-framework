#!/bin/bash
# Persist /tmp/rr_all.csv into all_results.csv every 5 min; final persist on DONE; then exit.
set -u
ROOT="/Users/jiaqit/Desktop/paper/MAPD_framework_imp"
while ! grep -q "RERUN 203040 DONE" /tmp/rr_run.log 2>/dev/null; do
  [ -s /tmp/rr_all.csv ] && python3 "$ROOT/skill_tools/persist_results.py" /tmp/rr_all.csv >/dev/null 2>&1
  echo "[watch] $(($(wc -l < /tmp/rr_all.csv 2>/dev/null)-1)) cells persisted; methods done: $(grep -c PERSIST-READY /tmp/rr_run.log)"
  sleep 300
done
[ -s /tmp/rr_all.csv ] && python3 "$ROOT/skill_tools/persist_results.py" /tmp/rr_all.csv >/dev/null 2>&1
echo "[watch] FINAL persist done — $(($(wc -l < /tmp/rr_all.csv)-1)) cells"
echo "ALL PERSISTED"
