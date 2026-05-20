#!/bin/bash
set -e

BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
COBRA="$BASE/CENTRAL-TP-TPTS/COBRA/cobra"
CENTRAL_BIN="$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central"
CENTRAL_FIXED="$BASE/CENTRAL-fixed/driver"
MGMAPD_PBS="$BASE/MGMAPD/LNS-PBS/lifelong"
MGMAPD_wPBS="$BASE/MGMAPD/LNS-wPBS/lifelong"
CONVERTER="$BASE/convert_tasks.py"
OUTFILE="$BASE/experiment_results_new.txt"
TIMEOUT=600

> "$OUTFILE"

echo "========================================================================" | tee -a "$OUTFILE"
echo "MAPD Reference Code Experiment Results" | tee -a "$OUTFILE"
echo "Date: $(date)" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "" | tee -a "$OUTFILE"
printf "%-20s %6s %6s %10s %10s %12s %10s\n" "Algorithm" "Agents" "Freq" "Makespan" "SWT" "AvgService" "Runtime" | tee -a "$OUTFILE"
printf "%-20s %6s %6s %10s %10s %12s %10s\n" "---------" "------" "----" "--------" "---" "----------" "-------" | tee -a "$OUTFILE"

parse_cobra() {
    local OUTPUT="$1" AG="$2" FREQ="$3"
    local TP_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | head -1 | awk -F'\t' '{print $NF}')
    local TP_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | head -1 | awk -F'\t' '{print $NF}')
    local TP_DONE=$(echo "$OUTPUT" | grep "Tasks completed:" | head -1 | awk -F'\t' '{print $NF}')
    local TPTS_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | tail -1 | awk -F'\t' '{print $NF}')
    local TPTS_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | tail -1 | awk -F'\t' '{print $NF}')

    local NTASKS=$(echo "$TP_DONE" | cut -d'/' -f2)
    [ -z "$NTASKS" ] && NTASKS=500

    if [ -n "$TP_MS" ] && [ "$TP_MS" != "" ]; then
        local TP_AVG=$(python3 -c "print(round(float('$TP_SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "$TP_MS" "$TP_SWT" "$TP_AVG" "N/A" | tee -a "$OUTFILE"
    else
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
    if [ -n "$TPTS_MS" ] && [ "$TPTS_MS" != "" ]; then
        local TPTS_AVG=$(python3 -c "print(round(float('$TPTS_SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "$TPTS_MS" "$TPTS_SWT" "$TPTS_AVG" "N/A" | tee -a "$OUTFILE"
    else
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

parse_central() {
    local OUTPUT="$1" ALGO="$2" AG="$3" FREQ="$4"
    local MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | awk -F'\t' '{print $NF}')
    local SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | awk -F'\t' '{print $NF}')
    local DONE=$(echo "$OUTPUT" | grep "Tasks completed:" | awk -F'\t' '{print $NF}')

    local NTASKS=$(echo "$DONE" | cut -d'/' -f2)
    [ -z "$NTASKS" ] && NTASKS=500

    if [ -n "$MS" ] && [ "$MS" != "" ]; then
        local AVG=$(python3 -c "print(round(float('$SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        local RT=$(echo "$OUTPUT" | grep -i "runtime\|Time:" | tail -1 | awk '{print $NF}')
        [ -z "$RT" ] && RT="N/A"
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "$RT" | tee -a "$OUTFILE"
    else
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

parse_ta() {
    local OUTPUT="$1" ALGO="$2" AG="$3" FREQ="$4"
    local MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | tail -1 | awk -F'\t' '{print $NF}')
    local SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | tail -1 | awk -F'\t' '{print $NF}')
    local DONE=$(echo "$OUTPUT" | grep "Tasks completed:" | tail -1 | awk -F'\t' '{print $NF}')
    local RT=$(echo "$OUTPUT" | grep "Wall time:" | awk '{print $(NF-1)}')

    local NTASKS=$(echo "$DONE" | cut -d'/' -f2)
    [ -z "$NTASKS" ] && NTASKS=500

    if [ -n "$MS" ] && [ "$MS" != "" ]; then
        local AVG=$(python3 -c "print(round(float('$SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "${RT}s" | tee -a "$OUTFILE"
    else
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

parse_mgmapd() {
    local OUTDIR="$1" ALGO="$2" AG="$3" FREQ="$4"
    local SOLVER_CSV="$OUTDIR/solver.csv"
    if [ ! -f "$SOLVER_CSV" ] || [ ! -s "$SOLVER_CSV" ]; then
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
        return
    fi
    # Extract results from solver.csv
    # Columns: runtime,nodes,?,cost,?,soc,bound,length,conflicts,timestep,drives,window
    local LAST_LINE=$(tail -1 "$SOLVER_CSV")
    local LAST_TS=$(echo "$LAST_LINE" | cut -d',' -f10)
    local TOTAL_COST=$(echo "$LAST_LINE" | cut -d',' -f6)
    local TOTAL_RT=$(awk -F',' '{sum+=$1} END {printf "%.3f", sum}' "$SOLVER_CSV")

    if [ -n "$LAST_TS" ] && [ "$LAST_TS" != "" ]; then
        printf "%-20s %6s %6s %10s %10s %12s %10ss\n" "$ALGO" "$AG" "$FREQ" "$LAST_TS" "$TOTAL_COST" "N/A" "$TOTAL_RT" | tee -a "$OUTFILE"
    else
        printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

for AG in 10 50; do
    echo "" | tee -a "$OUTFILE"
    echo "=== $AG Agents ===" | tee -a "$OUTFILE"

    for FREQ in 0.2 0.5 1 2 5 10 500; do
        MAP="$DATA/kiva-${AG}-500-5.map"
        TASK="$DATA/kiva-${FREQ}.task"
        [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && continue

        echo "--- ${AG}ag freq=${FREQ} ---" >&2

        # TP + TPTS (COBRA runs both)
        echo -n "  [COBRA] " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$COBRA" "$MAP" "$TASK" 2>&1) || true
        if [ -z "$OUTPUT" ]; then
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
        else
            parse_cobra "$OUTPUT" "$AG" "$FREQ"
        fi
        echo "done" >&2

        # CENTRAL (from CENTRAL-TP-TPTS/Centralized - ECBS)
        echo -n "  [CENTRAL] " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$CENTRAL_BIN" "$MAP" "$TASK" 1.0 2>&1) || true
        if [ -z "$OUTPUT" ]; then
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
        else
            parse_central "$OUTPUT" "CENTRAL" "$AG" "$FREQ"
        fi
        echo "done" >&2

        # CENTRAL-FIXED
        echo -n "  [CENTRAL-FIXED] " >&2
        OUTPUT=$(cd "$BASE/CENTRAL-fixed" && gtimeout $TIMEOUT ./driver "$MAP" "$TASK" 2>&1) || true
        if [ -z "$OUTPUT" ]; then
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL-FIXED" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
        else
            parse_central "$OUTPUT" "CENTRAL-FIXED" "$AG" "$FREQ"
        fi
        echo "done" >&2

        # LNS-PBS (MGMAPD)
        echo -n "  [LNS-PBS] " >&2
        MG_TASK="/tmp/mgmapd_task_${AG}_${FREQ}.task"
        python3 "$CONVERTER" "$TASK" "$AG" "$MG_TASK" 2>/dev/null
        MG_OUTDIR="/tmp/mgmapd_lnspbs_${AG}_${FREQ}"
        rm -rf "$MG_OUTDIR"
        OUTPUT=$(cd "$BASE/MGMAPD/LNS-PBS" && gtimeout $TIMEOUT ./lifelong \
            -m "$MAP" -k $AG --scenario=KIVA --solver=PBS \
            --task="$MG_TASK" --simulation_time=5000 -t 120 \
            --simulation_window=5 --planning_window=1073741823 \
            --lns_time=1 --seed=0 --dummy_paths=1 \
            -o "$MG_OUTDIR" -s 0 2>&1) || true
        parse_mgmapd "$MG_OUTDIR" "LNS-PBS" "$AG" "$FREQ"
        echo "done" >&2

        # LNS-wPBS (MGMAPD)
        echo -n "  [LNS-wPBS] " >&2
        MG_OUTDIR="/tmp/mgmapd_lnswpbs_${AG}_${FREQ}"
        rm -rf "$MG_OUTDIR"
        OUTPUT=$(cd "$BASE/MGMAPD/LNS-wPBS" && gtimeout $TIMEOUT ./lifelong \
            -m "$MAP" -k $AG --scenario=KIVA --solver=PBS \
            --task="$MG_TASK" --simulation_time=5000 -t 120 \
            --simulation_window=15 --planning_window=15 \
            --lns_time=1 --seed=0 --dummy_paths=1 \
            -o "$MG_OUTDIR" -s 0 2>&1) || true
        parse_mgmapd "$MG_OUTDIR" "LNS-wPBS" "$AG" "$FREQ"
        echo "done" >&2
    done

    # --- Offline algorithms: only kiva-500 ---
    FREQ=500
    TASK="$DATA/kiva-500.task"
    MAP="$DATA/kiva-${AG}-500-5.map"

    # TA-Prioritized
    echo -n "  [TA-Prioritized freq=500] " >&2
    TOUR="$BASE/TA-Prioritized/tour/${AG}-500.tour"
    OUTPUT=$(cd "$BASE/TA-Prioritized" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" 2>&1) || true
    parse_ta "$OUTPUT" "TA-Prioritized" "$AG" "500"
    echo "done" >&2

    # TA-Hybrid
    echo -n "  [TA-Hybrid freq=500] " >&2
    if [ "$AG" = "10" ]; then
        TOUR="$BASE/TA-Hybrid/tour/small-500-10-.tour"
    else
        TOUR="$BASE/TA-Hybrid/tour/${AG}-500.tour"
    fi
    OUTPUT=$(cd "$BASE/TA-Hybrid" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" 2>&1) || true
    parse_ta "$OUTPUT" "TA-Hybrid" "$AG" "500"
    echo "done" >&2
done

echo "" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "Done." | tee -a "$OUTFILE"
echo "" >&2
echo "Results saved to: $OUTFILE" >&2
