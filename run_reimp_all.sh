#!/bin/bash
# Run all reimplementation experiments to match reference results
# Methods: TP, TPTS, HBH+MLSIPP, TA-Prioritized, TA-Hybrid,
#          Hungarian-PBS/wPBS/PP+MLSIPP, LNS versions

MAPD="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/mapd"
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
TOURDIR="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/tour"
OUTDIR="/tmp/mapd_reimp_results"
TIMEOUT_SEC=1800
MAX_PARALLEL=6

mkdir -p "$OUTDIR"

SUMMARY="$OUTDIR/summary.csv"
echo "algorithm,agents,frequency,makespan,swt,avg_service,tasks_completed,runtime_ms,status" > "$SUMMARY"

wait_for_slot() {
    while [ "$(jobs -rp | wc -l)" -ge "$MAX_PARALLEL" ]; do
        sleep 0.5
    done
}

run_one() {
    local ALGO_LABEL="$1"; shift
    local NAGS="$1"; shift
    local FREQ="$1"; shift

    local MAP_FILE="$DATA/kiva-${NAGS}-500-5.map"
    local TASK_FILE="$DATA/kiva-${FREQ}.task"
    local OUT_FILE="$OUTDIR/${ALGO_LABEL}_${NAGS}_${FREQ}.txt"

    [ ! -f "$MAP_FILE" ] && { echo "[SKIP] $ALGO_LABEL ${NAGS}ag f=$FREQ: no map"; return; }
    [ ! -f "$TASK_FILE" ] && { echo "[SKIP] $ALGO_LABEL ${NAGS}ag f=$FREQ: no task"; return; }

    local T0=$(python3 -c "import time; print(time.time())")
    gtimeout "$TIMEOUT_SEC" "$MAPD" -m "$MAP_FILE" -t "$TASK_FILE" "$@" -s 1 > "$OUT_FILE" 2>&1
    local EC=$?
    local T1=$(python3 -c "import time; print(time.time())")
    local WALLTIME=$(python3 -c "print(round($T1 - $T0, 3))")

    local STATUS="ok"
    [ $EC -eq 124 ] && STATUS="timeout"
    [ $EC -ne 0 ] && [ $EC -ne 124 ] && STATUS="error($EC)"
    grep -q "COLLISION DETECTED" "$OUT_FILE" 2>/dev/null && STATUS="collision"

    local MAKESPAN=$(grep "Finishing Timestep:" "$OUT_FILE" 2>/dev/null | awk '{print $NF}')
    local SWT=$(grep "Sum of Task Waiting Time:" "$OUT_FILE" 2>/dev/null | awk '{print $NF}')
    local RUNTIME_MS=$(grep "Total runtime:" "$OUT_FILE" 2>/dev/null | awk '{print $(NF-1)}')
    local NTASKS=500
    local AVG=$(python3 -c "print(round(float('${SWT:-0}')/$NTASKS,1))" 2>/dev/null)

    [ -z "$MAKESPAN" ] && MAKESPAN="N/A" && AVG="N/A"
    [ -z "$SWT" ] && SWT="N/A"
    [ -z "$RUNTIME_MS" ] && RUNTIME_MS="N/A"

    echo "${ALGO_LABEL},${NAGS},${FREQ},${MAKESPAN},${SWT},${AVG},500,${RUNTIME_MS},${STATUS}" >> "$SUMMARY"
    echo "[DONE] ${ALGO_LABEL} ${NAGS}ag f=$FREQ: ms=${MAKESPAN} swt=${SWT} rt=${RUNTIME_MS}ms wall=${WALLTIME}s [${STATUS}]"
}

export -f run_one wait_for_slot
export MAPD DATA TOURDIR OUTDIR TIMEOUT_SEC MAX_PARALLEL SUMMARY

echo "=== Starting reimplementation experiments at $(date) ==="
echo "=== Output: $OUTDIR ==="
echo ""

AGENTS=(10 50)
FREQS=(0.2 0.5 1 2 5 10 500)

###############################################################################
# Group 1: TP variants (all frequencies)
###############################################################################
echo "--- Group 1: TP variants ---"
for AG in "${AGENTS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        wait_for_slot
        run_one "TP+2SIPP" "$AG" "$FREQ" -a TP &
        wait_for_slot
        run_one "TP+MLSIPP" "$AG" "$FREQ" -a TP --single_agent MLSIPP &
    done
done
wait
echo "--- Group 1 complete ---"

###############################################################################
# Group 2: TPTS variants (all frequencies)
###############################################################################
echo "--- Group 2: TPTS variants ---"
for AG in "${AGENTS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        wait_for_slot
        run_one "TPTS+2SIPP" "$AG" "$FREQ" -a TPTS &
        wait_for_slot
        run_one "TPTS+MLSIPP" "$AG" "$FREQ" -a TPTS --single_agent MLSIPP &
    done
done
wait
echo "--- Group 2 complete ---"

###############################################################################
# Group 3: HBH+MLSIPP (all frequencies)
###############################################################################
echo "--- Group 3: HBH+MLSIPP ---"
for AG in "${AGENTS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        wait_for_slot
        run_one "HBH+MLSIPP" "$AG" "$FREQ" -a HBH_MLA &
    done
done
wait
echo "--- Group 3 complete ---"

###############################################################################
# Group 4: TA-Prioritized variants (offline, freq=500 only)
###############################################################################
echo "--- Group 4: TA-Prioritized variants ---"
for AG in "${AGENTS[@]}"; do
    TF="$TOURDIR/${AG}-500.tour"
    if [ -f "$TF" ]; then
        wait_for_slot
        run_one "TA-Prio+2SIPP" "$AG" "500" -a TA_PRIORITIZED --tour "$TF" &
        wait_for_slot
        run_one "TA-Prio+MLSIPP" "$AG" "500" -a TA_PRIORITIZED --tour "$TF" --single_agent MLA &
    else
        echo "[WARN] Tour file not found: $TF"
    fi
done
wait
echo "--- Group 4 complete ---"

###############################################################################
# Group 5: TA-Hybrid (offline, freq=500 only)
###############################################################################
echo "--- Group 5: TA-Hybrid ---"
for AG in "${AGENTS[@]}"; do
    TF="$TOURDIR/${AG}-500.tour"
    if [ -f "$TF" ]; then
        wait_for_slot
        run_one "TA-Hybrid" "$AG" "500" -a TA_HYBRID --tour "$TF" &
    fi
done
wait
echo "--- Group 5 complete ---"

###############################################################################
# Group 6: Hungarian variants (all frequencies)
###############################################################################
echo "--- Group 6: Hungarian variants ---"
for AG in "${AGENTS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        wait_for_slot
        run_one "Hungarian-PBS" "$AG" "$FREQ" -a HUNGARIAN_PBS --lns_time 0 &
        wait_for_slot
        run_one "Hungarian-wPBS" "$AG" "$FREQ" -a HUNGARIAN_wPBS --lns_time 0 &
        wait_for_slot
        run_one "Hungarian-PP+MLSIPP" "$AG" "$FREQ" -a HUNGARIAN_PBS --mapf PP --single_agent MLA --lns_time 0 &
    done
done
wait
echo "--- Group 6 complete ---"

###############################################################################
# Group 7: LNS(1s)-Hungarian variants (all frequencies)
###############################################################################
echo "--- Group 7: LNS(1s) variants ---"
for AG in "${AGENTS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        wait_for_slot
        run_one "LNS1s-Hungarian-PBS" "$AG" "$FREQ" -a LNS_PBS --lns_time 1 &
        wait_for_slot
        run_one "LNS1s-Hungarian-wPBS" "$AG" "$FREQ" -a LNS_wPBS --lns_time 1 &
        wait_for_slot
        run_one "LNS1s-Hungarian-PP+MLSIPP" "$AG" "$FREQ" -a LNS_PBS --mapf PP --single_agent MLA --lns_time 1 &
    done
done
wait
echo "--- Group 7 complete ---"

echo ""
echo "=== All experiments complete at $(date) ==="

# Sort and display summary
SORTED="$OUTDIR/summary_sorted.csv"
head -1 "$SUMMARY" > "$SORTED"
tail -n +2 "$SUMMARY" | sort -t',' -k1,1 -k2,2n -k3,3g >> "$SORTED"
mv "$SORTED" "$SUMMARY"

echo "=== Summary: $SUMMARY ==="
cat "$SUMMARY"
