#!/bin/bash
# Sequentially re-run the LNS cells whose comparison-CSV runtime is stale (pre runtime-guard fix).
# MUST be sequential: the LNS 1s budget is clock()-based and corrupts under CPU contention.
set -u
D="/Users/jiaqit/Desktop/meta/paper/reference_code/data/Instances/small"
M="/Users/jiaqit/Desktop/meta/paper/MAPD_framework_imp/mapd"
OUT="/tmp/lns_seq_results.csv"
echo "method,agents,freq,MS,SWT,RT_s" > "$OUT"

cell() { # label ag freq "flags"
  local label="$1" ag="$2" fr="$3" flags="$4"
  local t="/tmp/seq_${ag}_${fr}_$(echo "$flags"|tr -cd 'A-Za-z').txt"
  $M -m "$D/kiva-${ag}-500-5.map" -t "$D/kiva-${fr}.task" $flags -s 1 > "$t" 2>&1
  local ms=$(grep "Finishing Timestep:" "$t"|awk -F'\t' '{print $2}')
  local swt=$(grep "Sum of Task Waiting Time:" "$t"|awk -F'\t' '{print $2}')
  local rtms=$(grep "Total runtime:" "$t"|awk '{print $(NF-1)}')
  local rts=$(awk -v x="$rtms" 'BEGIN{printf "%.1f", x/1000.0}')
  echo "${label},${ag},${fr},${ms},${swt},${rts}" >> "$OUT"
  echo "[done] ${label} ${ag}/${fr}: MS=${ms} SWT=${swt} RT=${rts}s" >&2
}

cell "LNS(1s)+wPBS-MLA*"   30 500 "-a LNS_wPBS --lns_time 1"
cell "LNS(1s)+wPBS-MLA*"   50 0.2 "-a LNS_wPBS --lns_time 1"
cell "LNS(1s)+PBS-MLSIPP"  50 0.2 "-a LNS_PBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 40 10  "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 40 500 "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 50 2   "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 50 5   "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 50 10  "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+wPBS-MLSIPP" 50 500 "-a LNS_wPBS --lns_time 1 --sipp"
cell "LNS(1s)+PP-SIPP"     50 0.2 "-a LNS_PBS --mapf PP --lns_time 1 --sipp"
echo "LNS_SEQ_DONE" >&2
