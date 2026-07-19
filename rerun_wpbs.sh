#!/bin/bash

MAPD="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/mapd"
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
TOUR="/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid-build/tour"
OUTDIR="/Users/jiaqit/Desktop/paper/output"
TIMEOUT_SEC=300
MAX_PARALLEL=1

SUMMARY="$OUTDIR/summary.csv"

mkdir -p "$OUTDIR"

# Initialize summary if needed
if [ ! -f "$SUMMARY" ]; then
    echo "algorithm,agents,frequency,makespan,swt,completed,total,runtime_ms,status" > "$SUMMARY"
fi

run_one() {
    local ALGO_NAME="$1"; shift
    local NAGS="$1"; shift
    local FREQ="$1"; shift

    local MAP_FILE="$DATA/kiva-${NAGS}-500-5.map"
    local TASK_FILE="$DATA/kiva-${FREQ}.task"
    local OUT_NAME="${ALGO_NAME}-${NAGS}ag-freq${FREQ}"
    local OUT_FILE="$OUTDIR/${OUT_NAME}.txt"

    [ ! -f "$MAP_FILE" ] || [ ! -f "$TASK_FILE" ] && return

    gtimeout "$TIMEOUT_SEC" "$MAPD" -m "$MAP_FILE" -t "$TASK_FILE" \
        "$@" --save_output --output_dir "$OUTDIR" -s 1 \
        > "$OUT_FILE" 2>&1
    local EC=$?

    local STATUS="ok"
    [ $EC -eq 124 ] && STATUS="timeout"
    [ $EC -ne 0 ] && [ $EC -ne 124 ] && STATUS="error"

    local MAKESPAN=$(grep "Finishing Timestep:" "$OUT_FILE" 2>/dev/null | awk '{print $NF}')
    local SWT=$(grep "Sum of Task Waiting Time:" "$OUT_FILE" 2>/dev/null | awk '{print $NF}')
    local TASKS=$(grep "Tasks completed:" "$OUT_FILE" 2>/dev/null | awk '{print $NF}')
    local RUNTIME=$(grep "Total runtime:" "$OUT_FILE" 2>/dev/null | awk '{print $(NF-1)}')
    local COMPLETED=$(echo "$TASKS" | cut -d'/' -f1)
    local TOTAL=$(echo "$TASKS" | cut -d'/' -f2)

    # Check for collision failures
    if grep -q "COLLISION CHECK FAILED" "$OUT_FILE" 2>/dev/null; then
        STATUS="collision"
    fi

    [ -z "$MAKESPAN" ] && MAKESPAN="N/A"
    [ -z "$SWT" ] && SWT="N/A"
    [ -z "$COMPLETED" ] && COMPLETED="N/A"
    [ -z "$TOTAL" ] && TOTAL="N/A"
    [ -z "$RUNTIME" ] && RUNTIME="N/A"

    echo "${ALGO_NAME},${NAGS},${FREQ},${MAKESPAN},${SWT},${COMPLETED},${TOTAL},${RUNTIME},${STATUS}" >> "$SUMMARY"
    echo "[DONE] ${OUT_NAME}: makespan=${MAKESPAN} swt=${SWT} runtime=${RUNTIME}ms [${STATUS}]"
}

if ! command -v gtimeout &>/dev/null; then
    if command -v timeout &>/dev/null; then
        gtimeout() { timeout "$@"; }
    else
        echo "ERROR: Need gtimeout or timeout"; exit 1
    fi
fi

MAPS=(10 50)
FREQS=(0.2 0.5 1 2 5 10 500)

PIDS=()
COUNT=0

count_total() {
    local t=0
    for NAGS in "${MAPS[@]}"; do
        for FREQ in "${FREQS[@]}"; do
            # Online algorithms: all freqs
            t=$((t + 16))  # 16 online algorithms
            # Offline algorithms: only freq=500
            if [ "$FREQ" = "500" ]; then
                t=$((t + 4))  # 4 offline algorithms (TA-*)
            fi
        done
    done
    echo $t
}
TOTAL=$(count_total)

for NAGS in "${MAPS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        # ======== ONLINE ALGORITHMS (all frequencies) ========

        # 1. TP + 2SIPP*
        for ARGS in \
            "TP-2SIPP $NAGS $FREQ -a TP --sipp" \
            "TP-MLSIPP $NAGS $FREQ -a TP --single_agent MLA --sipp" \
            "TPTS-2SIPP $NAGS $FREQ -a TPTS --sipp" \
            "TPTS-MLSIPP $NAGS $FREQ -a TPTS --single_agent MLA --sipp" \
            "CENTRAL-CBS $NAGS $FREQ -a CENTRAL" \
            "CENTRAL-PBS $NAGS $FREQ -a CENTRAL --mapf PBS" \
            "CENTRAL_FIXED-CBS $NAGS $FREQ -a CENTRAL_FIXED" \
            "CENTRAL_FIXED-PBS $NAGS $FREQ -a CENTRAL_FIXED --mapf PBS" \
            "HBH-MLSIPP $NAGS $FREQ -a HBH_MLA --sipp" \
            "HUNGARIAN-PBS $NAGS $FREQ -a HUNGARIAN_PBS" \
            "HUNGARIAN-wPBS $NAGS $FREQ -a HUNGARIAN_wPBS" \
            "HUNGARIAN-PP-MLSIPP $NAGS $FREQ -a HUNGARIAN_PBS --mapf PP --sipp" \
            "LNS1s-HUNGARIAN-PBS $NAGS $FREQ -a LNS_PBS --lns_time 1" \
            "LNS1s-HUNGARIAN-wPBS $NAGS $FREQ -a LNS_wPBS --lns_time 1" \
            "LNS1s-HUNGARIAN-PP-MLSIPP $NAGS $FREQ -a LNS_PBS --mapf PP --sipp --lns_time 1"
        do
            COUNT=$((COUNT + 1))
            ALGO=$(echo "$ARGS" | awk '{print $1}')
            echo "[$COUNT/$TOTAL] Starting: $ALGO ${NAGS}ag freq${FREQ}"
            eval "run_one $ARGS" &
            PIDS+=($!)
            if [ ${#PIDS[@]} -ge $MAX_PARALLEL ]; then
                wait "${PIDS[0]}"
                PIDS=("${PIDS[@]:1}")
            fi
        done

        # ======== OFFLINE ALGORITHMS (only freq=500) ========
        if [ "$FREQ" = "500" ]; then
            TOUR_FILE="$TOUR/${NAGS}-500.tour"
            if [ -f "$TOUR_FILE" ]; then
                for ARGS in \
                    "TA-PRIORITIZED-2SIPP $NAGS $FREQ -a TA_PRIORITIZED --sipp --tour $TOUR_FILE" \
                    "TA-PRIORITIZED-MLSIPP $NAGS $FREQ -a TA_PRIORITIZED --sipp --single_agent MLA --tour $TOUR_FILE" \
                    "TA-HYBRID $NAGS $FREQ -a TA_HYBRID --tour $TOUR_FILE"
                do
                    COUNT=$((COUNT + 1))
                    ALGO=$(echo "$ARGS" | awk '{print $1}')
                    echo "[$COUNT/$TOTAL] Starting: $ALGO ${NAGS}ag freq${FREQ}"
                    eval "run_one $ARGS" &
                    PIDS+=($!)
                    if [ ${#PIDS[@]} -ge $MAX_PARALLEL ]; then
                        wait "${PIDS[0]}"
                        PIDS=("${PIDS[@]:1}")
                    fi
                done
            fi
        fi
    done
done

for PID in "${PIDS[@]}"; do wait "$PID"; done

echo ""
echo "=== Experiment complete ($COUNT runs) ==="

# Sort summary
if [ -f "$SUMMARY" ]; then
    SORTED="$OUTDIR/summary_sorted.csv"
    head -1 "$SUMMARY" > "$SORTED"
    tail -n +2 "$SUMMARY" | sort -t',' -k1,1 -k2,2n -k3,3g >> "$SORTED"
    mv "$SORTED" "$SUMMARY"
    echo "Results saved to: $SUMMARY"
fi
