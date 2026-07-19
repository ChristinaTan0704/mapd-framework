#!/bin/bash
# Re-run the runtime-unmatched LNS cells, ONE program at a time (strictly sequential, no parallelism).
# For each, measure BOTH the same-machine reference baseline AND the reimpl, so the comparison is fair.
set -u
ROOT="/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
D="/Users/jiaqit/Desktop/2026 meta docs/paper/reference_code/data/Instances/small"
PBSBIN="/Users/jiaqit/Desktop/2026 meta docs/paper/reference_code/MGMAPD/LNS-PBS/lifelong_simple"
WPBSBIN="/Users/jiaqit/Desktop/2026 meta docs/paper/reference_code/MGMAPD/LNS-wPBS/lifelong_simple"
TO=2700
REFOUT=/tmp/unmatch_ref.csv
REOUT=/tmp/unmatch_re.csv
echo "method,agents,freq,makespan,swt,runtime_s,status" > "$REOUT"
echo "method,agents,freq,makespan,swt,runtime_s,status" > "$REFOUT"

# unique reference baselines to refresh (compare-against | bin | plan_window)
#   LNS+PBS-MLA*  -> LNS-PBS  plan_window=5000   : 30/1 30/500 40/1 40/2 40/500
#   LNS+wPBS-MLA* -> LNS-wPBS plan_window=10     : 30/1 30/2 30/5 30/10 30/500 40/1 40/2 40/5 40/10 40/500
ref_cell(){ # base ag fr bin pw
  local base="$1" ag="$2" fr="$3" bin="$4" pw="$5"
  local o ms ft rt swt tot
  o=$(gtimeout $TO "$bin" "$D/kiva-$ag-500-5.map" "$ag" "$D/kiva-$fr.task" 1 10 "$pw" 5000 0 2>&1 | grep -i Makespan)
  ms=$(echo "$o"|sed -nE 's/.*Makespan:[[:space:]]*([0-9]+).*/\1/p')
  ft=$(echo "$o"|sed -nE 's/.*Flowtime:[[:space:]]*([0-9.]+).*/\1/p')
  rt=$(echo "$o"|sed -nE 's/.*Runtime:[[:space:]]*([0-9.]+).*/\1/p')
  if [ -n "$ms" ]; then swt=$(python3 -c "print(int(float('$ft')*500))"); tot=$(python3 -c "print(round(float('$rt')*float('$ms')/1000,3))"); echo "$base,$ag,$fr,$ms,$swt,$tot,ok" >> "$REFOUT"; fi
  echo "[ref] $base $ag/$fr -> $o" >&2
}
for c in "30 1" "30 500" "40 1" "40 2" "40 500"; do ref_cell "LNS(1s)+PBS-MLA*" $c "$PBSBIN" 5000; done
for c in "30 1" "30 2" "30 5" "30 10" "30 500" "40 1" "40 2" "40 5" "40 10" "40 500"; do ref_cell "LNS(1s)+wPBS-MLA*" $c "$WPBSBIN" 10; done
echo "[stage] references done; now reimpl" >&2

# reimpl: every unmatched cell (sequential, via run_cell.sh)
export CELLOUT="$REOUT" TIMEOUT=$TO
while IFS= read -r line; do
  [ -z "$line" ] && continue
  "$ROOT/skill_tools/run_cell.sh" "$line"
done < /tmp/unmatch_jobs.txt
echo "RERUN UNMATCH DONE" >&2
