#!/bin/bash
# Rerun all 18 methods for agents=10,50. Max 4 parallel. Outputs CSV.
set -u

MAPD="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/mapd"
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
TOUR="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/tour"
OUTCSV="${1:-/tmp/mapd_rerun_results.csv}"
TIMEOUT=600
MAX_PAR=4

echo "method,agents,freq,makespan,swt,runtime_ms,status" > "$OUTCSV"

RUNNING=0

run_one() {
    local METHOD="$1" ALGO="$2" AG="$3" FREQ="$4"
    shift 4
    local EXTRA="$*"

    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local TMP="/tmp/mapd_run_${METHOD}_${AG}_${FREQ}.txt"

    if [ ! -f "$MAP" ] || [ ! -f "$TASK" ]; then
        echo "${METHOD},${AG},${FREQ},N/A,N/A,N/A,skip" >> "$OUTCSV"
        echo "[SKIP] ${METHOD} ${AG}ag f=${FREQ}: missing files"
        return
    fi

    # Check tour file for TA methods
    if echo "$EXTRA" | grep -q -- "--tour"; then
        local TOURFILE=$(echo "$EXTRA" | grep -oE '/[^ ]*\.tour')
        if [ -n "$TOURFILE" ] && [ ! -f "$TOURFILE" ]; then
            echo "${METHOD},${AG},${FREQ},N/A,N/A,N/A,skip" >> "$OUTCSV"
            echo "[SKIP] ${METHOD} ${AG}ag f=${FREQ}: tour missing"
            return
        fi
    fi

    gtimeout "$TIMEOUT" "$MAPD" -m "$MAP" -t "$TASK" -a "$ALGO" $EXTRA -s 1 > "$TMP" 2>&1
    local EC=$?

    local STATUS="ok"
    [ $EC -eq 124 ] && STATUS="timeout"
    [ $EC -ne 0 ] && [ $EC -ne 124 ] && STATUS="error"
    grep -q "COLLISION DETECTED" "$TMP" 2>/dev/null && STATUS="collision"

    local MS=$(grep "Finishing Timestep:" "$TMP" 2>/dev/null | awk '{print $NF}')
    local SWT=$(grep "Sum of Task Waiting Time:" "$TMP" 2>/dev/null | awk '{print $NF}')
    local RT=$(grep "Total runtime:" "$TMP" 2>/dev/null | awk '{print $(NF-1)}')

    [ -z "$MS" ] && MS="N/A"
    [ -z "$SWT" ] && SWT="N/A"
    [ -z "$RT" ] && RT="N/A"

    echo "${METHOD},${AG},${FREQ},${MS},${SWT},${RT},${STATUS}" >> "$OUTCSV"
    echo "[DONE] ${METHOD} ${AG}ag f=${FREQ}: ms=${MS} swt=${SWT} rt=${RT} [${STATUS}]"
}

wait_slot() {
    while [ "$(jobs -rp | wc -l)" -ge "$MAX_PAR" ]; do
        sleep 1
    done
}

# Method definitions: key algo extra_args
# For TA methods with tour, AGENTS gets replaced with actual count
METHODS=(
    "TP-STA*|TP|"
    "TPTS-STA*|TPTS|"
    "CENTRAL-ECBS|CENTRAL|"
    "TA-Hybrid-STA*|TA_HYBRID|--tour TOUR/AGENTS-500.tour"
    "TA-Prioritized-STA*|TA_PRIORITIZED|--tour TOUR/AGENTS-500.tour"
    "Hungarian+PBS-MLA*|HUNGARIAN_PBS|"
    "Hungarian+wPBS-MLA*|HUNGARIAN_wPBS|"
    "LNS(1s)+PBS-MLA*|LNS_PBS|--lns_time 1"
    "LNS(1s)+wPBS-MLA*|LNS_wPBS|--lns_time 1"
    "Hungarian+PBS-MLSIPP|HUNGARIAN_PBS|--sipp"
    "Hungarian+wPBS-MLSIPP|HUNGARIAN_wPBS|--sipp"
    "LNS(1s)+PBS-MLSIPP|LNS_PBS|--lns_time 1 --sipp"
    "LNS(1s)+wPBS-MLSIPP|LNS_wPBS|--lns_time 1 --sipp"
    "TP-SIPP|TP|--single_agent MLA --sipp"
    "TPTS-SIPP|TPTS|--single_agent MLA --sipp"
    "CENTRAL-ECBS-SIPP|CENTRAL|--sipp"
    "Hungarian+PP-SIPP|HUNGARIAN_PBS|--mapf PP --sipp"
    "LNS(1s)+PP-SIPP|LNS_PBS|--mapf PP --lns_time 1 --sipp"
)

FREQS=(0.2 0.5 1 2 5 10 500)
AGENTS=(10 50)

TOTAL=0
for M in "${METHODS[@]}"; do
    IFS='|' read -r KEY ALGO EXTRA <<< "$M"
    for AG in "${AGENTS[@]}"; do
        # TA methods: only freq=500
        if [[ "$KEY" == TA-* ]]; then
            FLIST=(500)
        else
            FLIST=("${FREQS[@]}")
        fi
        for FREQ in "${FLIST[@]}"; do
            TOTAL=$((TOTAL + 1))
        done
    done
done

echo "=== Running $TOTAL experiments (max $MAX_PAR parallel, ${TIMEOUT}s timeout) ==="
echo "=== Output: $OUTCSV ==="

COUNT=0
for M in "${METHODS[@]}"; do
    IFS='|' read -r KEY ALGO EXTRA <<< "$M"
    for AG in "${AGENTS[@]}"; do
        if [[ "$KEY" == TA-* ]]; then
            FLIST=(500)
        else
            FLIST=("${FREQS[@]}")
        fi
        for FREQ in "${FLIST[@]}"; do
            COUNT=$((COUNT + 1))
            # Replace TOUR and AGENTS placeholders
            ACTUAL_EXTRA=$(echo "$EXTRA" | sed "s|TOUR|$TOUR|g" | sed "s|AGENTS|$AG|g")
            echo "[$COUNT/$TOTAL] ${KEY} ${AG}ag f=${FREQ}"
            wait_slot
            run_one "$KEY" "$ALGO" "$AG" "$FREQ" $ACTUAL_EXTRA &
        done
    done
done

wait
echo ""
echo "=== All $TOTAL experiments complete ==="
echo "=== Results: $OUTCSV ==="
