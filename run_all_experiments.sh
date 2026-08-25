#!/bin/bash

MAPD="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/mapd"
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
TOURDIR="/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid-build/tour"
OUTDIR="/Users/jiaqit/Desktop/paper/output"
TIMEOUT_SEC=600
MAX_PARALLEL=8

mkdir -p "$OUTDIR"

SUMMARY="$OUTDIR/summary.csv"
echo "algorithm,agents,frequency,makespan,swt,tasks_completed,total_tasks,runtime_ms,status" > "$SUMMARY"

run_one() {
    local ALGO_NAME="$1"; shift
    local NAGS="$1"; shift
    local FREQ="$1"; shift

    local MAP_FILE="$DATA/kiva-${NAGS}-500-5.map"
    local TASK_FILE="$DATA/kiva-${FREQ}.task"
    local OUT_NAME="${ALGO_NAME}-${NAGS}ag-freq${FREQ}"
    local OUT_FILE="$OUTDIR/${OUT_NAME}.txt"

    if [ ! -f "$MAP_FILE" ] || [ ! -f "$TASK_FILE" ]; then
        echo "[SKIP] $OUT_NAME: missing files"
        return
    fi

    # Run with timeout
    gtimeout "$TIMEOUT_SEC" "$MAPD" -m "$MAP_FILE" -t "$TASK_FILE" \
        "$@" \
        --save_output --output_dir "$OUTDIR" -s 1 \
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

    if grep -q "COLLISION DETECTED" "$OUT_FILE" 2>/dev/null; then
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

# Check if gtimeout exists (macOS coreutils)
if ! command -v gtimeout &>/dev/null; then
    if command -v timeout &>/dev/null; then
        gtimeout() { timeout "$@"; }
    else
        echo "ERROR: Need 'gtimeout' (brew install coreutils) or 'timeout'"
        exit 1
    fi
fi

MAPS=(10 20 30 40 50)
FREQS=(0.2 0.5 1 2 5 10 500)
TOUR_FREQS=(1 2 5 10 500)

# Build job list
declare -a JOBS
add_job() { JOBS+=("$*"); }

for NAGS in "${MAPS[@]}"; do
    for FREQ in "${FREQS[@]}"; do
        add_job "TP-STA $NAGS $FREQ -a TP"
        add_job "TP-MLA $NAGS $FREQ -a TP --single_agent MLA"
        add_job "TPTS-STA $NAGS $FREQ -a TPTS"
        add_job "TPTS-MLA $NAGS $FREQ -a TPTS --single_agent MLA"
        add_job "CENTRAL-CBS $NAGS $FREQ -a CENTRAL-CBS"
        add_job "CENTRAL-fixed-CBS $NAGS $FREQ -a CENTRAL-fixed"
        add_job "HBH-MLA $NAGS $FREQ -a HBH_MLA"
        add_job "HUNGARIAN-PBS $NAGS $FREQ -a HUNGARIAN_PBS"
        add_job "HUNGARIAN-wPBS $NAGS $FREQ -a HUNGARIAN_wPBS"
        add_job "HUNGARIAN-PP $NAGS $FREQ -a HUNGARIAN_PBS --mapf PP"
        add_job "LNS1s-PBS $NAGS $FREQ -a LNS_PBS --lns_time 1"
        add_job "LNS1s-wPBS $NAGS $FREQ -a LNS_wPBS --lns_time 1"
        add_job "LNS1s-PP $NAGS $FREQ -a LNS_PBS --mapf PP --lns_time 1"
    done
    # TA algorithms: offline only (kiva-500)
    TF="$TOURDIR/${NAGS}-500.tour"
    if [ -f "$TF" ]; then
        add_job "TA_PRIORITIZED-STA $NAGS 500 -a TA_PRIORITIZED --tour $TF"
        add_job "TA_PRIORITIZED-MLA $NAGS 500 -a TA_PRIORITIZED --single_agent MLA --tour $TF"
        add_job "TA_HYBRID $NAGS 500 -a TA_HYBRID --tour $TF"
    fi
done

TOTAL=${#JOBS[@]}
echo "=== Running $TOTAL experiments (max $MAX_PARALLEL parallel, ${TIMEOUT_SEC}s timeout) ==="
echo "=== Output directory: $OUTDIR ==="
echo ""

COUNTER=0
PIDS=()

for JOB in "${JOBS[@]}"; do
    COUNTER=$((COUNTER + 1))
    ALGO=$(echo "$JOB" | awk '{print $1}')
    AG=$(echo "$JOB" | awk '{print $2}')
    FR=$(echo "$JOB" | awk '{print $3}')
    echo "[$COUNTER/$TOTAL] Starting: $ALGO ${AG}ag freq${FR}"

    eval "run_one $JOB" &
    PIDS+=($!)

    # Limit parallelism
    if [ ${#PIDS[@]} -ge $MAX_PARALLEL ]; then
        wait "${PIDS[0]}"
        PIDS=("${PIDS[@]:1}")
    fi
done

# Wait for remaining
for PID in "${PIDS[@]}"; do
    wait "$PID"
done

echo ""
echo "=== All $TOTAL experiments complete ==="

# Sort summary
SORTED="$OUTDIR/summary_sorted.csv"
head -1 "$SUMMARY" > "$SORTED"
tail -n +2 "$SUMMARY" | sort -t',' -k1,1 -k2,2n -k3,3g >> "$SORTED"
mv "$SORTED" "$SUMMARY"

echo "=== Summary saved to: $SUMMARY ==="
