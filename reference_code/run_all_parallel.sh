#!/bin/bash
# Run all experiments in parallel by algorithm class
BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
OUTDIR="/tmp/mapd_results"
mkdir -p "$OUTDIR"
TIMEOUT=600

###############################################################################
# 1) COBRA (TP + TPTS) - run for all agent/freq combos
###############################################################################
run_cobra() {
    local AG=$1 FREQ=$2
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local OUT="$OUTDIR/cobra_${AG}_${FREQ}.txt"
    [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && return
    gtimeout $TIMEOUT "$BASE/CENTRAL-TP-TPTS/COBRA/cobra" "$MAP" "$TASK" > "$OUT" 2>&1
    echo "$AG $FREQ COBRA_DONE" >&2
}

###############################################################################
# 2) CENTRAL (from CENTRAL-TP-TPTS/Centralized - ECBS)
###############################################################################
run_central() {
    local AG=$1 FREQ=$2
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local OUT="$OUTDIR/central_${AG}_${FREQ}.txt"
    [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && return
    gtimeout $TIMEOUT "$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central" "$MAP" "$TASK" 1.0 > "$OUT" 2>&1
    echo "$AG $FREQ CENTRAL_DONE" >&2
}

###############################################################################
# 3) CENTRAL-FIXED
###############################################################################
run_central_fixed() {
    local AG=$1 FREQ=$2
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local OUT="$OUTDIR/centralfixed_${AG}_${FREQ}.txt"
    [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && return
    cd "$BASE/CENTRAL-fixed"
    gtimeout $TIMEOUT ./driver "$MAP" "$TASK" > "$OUT" 2>&1
    echo "$AG $FREQ CENTRALFIXED_DONE" >&2
}

###############################################################################
# 4) LNS-PBS (MGMAPD)
###############################################################################
run_lns_pbs() {
    local AG=$1 FREQ=$2
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local MG_TASK="/tmp/mgmapd_task_${AG}_${FREQ}.task"
    local MG_OUTDIR="/tmp/mgmapd_lnspbs_${AG}_${FREQ}"
    python3 "$BASE/convert_tasks.py" "$TASK" "$AG" "$MG_TASK" 2>/dev/null
    rm -rf "$MG_OUTDIR"
    cd "$BASE/MGMAPD/LNS-PBS"
    gtimeout $TIMEOUT ./lifelong \
        -m "$MAP" -k $AG --scenario=KIVA --solver=PBS \
        --task="$MG_TASK" --simulation_time=5000 -t 120 \
        --simulation_window=5 --planning_window=1073741823 \
        --lns_time=1 --seed=0 --dummy_paths=1 \
        -o "$MG_OUTDIR" -s 0 > "$OUTDIR/lnspbs_${AG}_${FREQ}.txt" 2>&1
    echo "$AG $FREQ LNSPBS_DONE" >&2
}

###############################################################################
# 5) LNS-wPBS (MGMAPD)
###############################################################################
run_lns_wpbs() {
    local AG=$1 FREQ=$2
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-${FREQ}.task"
    local MG_TASK="/tmp/mgmapd_task_${AG}_${FREQ}.task"
    local MG_OUTDIR="/tmp/mgmapd_lnswpbs_${AG}_${FREQ}"
    python3 "$BASE/convert_tasks.py" "$TASK" "$AG" "$MG_TASK" 2>/dev/null
    rm -rf "$MG_OUTDIR"
    cd "$BASE/MGMAPD/LNS-wPBS"
    gtimeout $TIMEOUT ./lifelong \
        -m "$MAP" -k $AG --scenario=KIVA --solver=PBS \
        --task="$MG_TASK" --simulation_time=5000 -t 120 \
        --simulation_window=15 --planning_window=15 \
        --lns_time=1 --seed=0 --dummy_paths=1 \
        -o "$MG_OUTDIR" -s 0 > "$OUTDIR/lnswpbs_${AG}_${FREQ}.txt" 2>&1
    echo "$AG $FREQ LNSWPBS_DONE" >&2
}

###############################################################################
# 6) TA-Prioritized (freq=500 only)
###############################################################################
run_ta_prio() {
    local AG=$1
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-500.task"
    local TOUR="$BASE/TA-Prioritized/tour/${AG}-500.tour"
    local OUT="$OUTDIR/taprio_${AG}_500.txt"
    cd "$BASE/TA-Prioritized"
    gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" > "$OUT" 2>&1
    echo "$AG 500 TAPRIO_DONE" >&2
}

###############################################################################
# 7) TA-Hybrid (freq=500 only)
###############################################################################
run_ta_hybrid() {
    local AG=$1
    local MAP="$DATA/kiva-${AG}-500-5.map"
    local TASK="$DATA/kiva-500.task"
    local TOUR
    if [ "$AG" = "10" ]; then
        TOUR="$BASE/TA-Hybrid/tour/small-500-10-.tour"
    else
        TOUR="$BASE/TA-Hybrid/tour/${AG}-500.tour"
    fi
    local OUT="$OUTDIR/tahybrid_${AG}_500.txt"
    cd "$BASE/TA-Hybrid"
    gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" > "$OUT" 2>&1
    echo "$AG 500 TAHYBRID_DONE" >&2
}

export -f run_cobra run_central run_central_fixed run_lns_pbs run_lns_wpbs run_ta_prio run_ta_hybrid
export BASE DATA OUTDIR TIMEOUT

echo "Starting all experiments at $(date)..." >&2

# Launch all experiments in parallel
PIDS=""
for AG in 10 50; do
    for FREQ in 0.2 0.5 1 2 5 10 500; do
        run_cobra $AG $FREQ &
        PIDS="$PIDS $!"
        run_central $AG $FREQ &
        PIDS="$PIDS $!"
        run_central_fixed $AG $FREQ &
        PIDS="$PIDS $!"
        run_lns_pbs $AG $FREQ &
        PIDS="$PIDS $!"
        run_lns_wpbs $AG $FREQ &
        PIDS="$PIDS $!"
    done
    # TA algorithms (freq=500 only)
    run_ta_prio $AG &
    PIDS="$PIDS $!"
    run_ta_hybrid $AG &
    PIDS="$PIDS $!"
done

echo "Launched $(echo $PIDS | wc -w) processes, waiting..." >&2

# Wait for all
for PID in $PIDS; do
    wait $PID 2>/dev/null
done

echo "All experiments complete at $(date)!" >&2
echo "" >&2

###############################################################################
# Collect results
###############################################################################
RESULTS="$BASE/experiment_results_new.txt"
> "$RESULTS"

echo "========================================================================" >> "$RESULTS"
echo "MAPD Reference Code Experiment Results" >> "$RESULTS"
echo "Date: $(date)" >> "$RESULTS"
echo "========================================================================" >> "$RESULTS"
echo "" >> "$RESULTS"
printf "%-20s %6s %6s %10s %10s %12s %10s\n" "Algorithm" "Agents" "Freq" "Makespan" "SWT" "AvgService" "Runtime" >> "$RESULTS"
printf "%-20s %6s %6s %10s %10s %12s %10s\n" "---------" "------" "----" "--------" "---" "----------" "-------" >> "$RESULTS"

for AG in 10 50; do
    echo "" >> "$RESULTS"
    echo "=== $AG Agents ===" >> "$RESULTS"

    for FREQ in 0.2 0.5 1 2 5 10 500; do
        # Parse COBRA (TP + TPTS)
        F="$OUTDIR/cobra_${AG}_${FREQ}.txt"
        if [ -f "$F" ] && [ -s "$F" ]; then
            TP_MS=$(grep "Finishing Timestep:" "$F" | head -1 | awk -F'\t' '{print $NF}')
            TP_SWT=$(grep "Sum of Task Waiting Time:" "$F" | head -1 | awk -F'\t' '{print $NF}')
            TPTS_MS=$(grep "Finishing Timestep:" "$F" | tail -1 | awk -F'\t' '{print $NF}')
            TPTS_SWT=$(grep "Sum of Task Waiting Time:" "$F" | tail -1 | awk -F'\t' '{print $NF}')
            DONE=$(grep "Tasks completed:" "$F" | head -1 | awk -F'\t' '{print $NF}')
            NT=$(echo "$DONE" | cut -d'/' -f2); [ -z "$NT" ] && NT=500
            if [ -n "$TP_MS" ]; then
                TP_AVG=$(python3 -c "print(round(float('$TP_SWT')/$NT,1))" 2>/dev/null)
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "$TP_MS" "$TP_SWT" "$TP_AVG" "N/A" >> "$RESULTS"
            else
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
            fi
            if [ -n "$TPTS_MS" ]; then
                TPTS_AVG=$(python3 -c "print(round(float('$TPTS_SWT')/$NT,1))" 2>/dev/null)
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "$TPTS_MS" "$TPTS_SWT" "$TPTS_AVG" "N/A" >> "$RESULTS"
            else
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
            fi
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TP" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "TPTS" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
        fi

        # Parse CENTRAL
        F="$OUTDIR/central_${AG}_${FREQ}.txt"
        if [ -f "$F" ] && [ -s "$F" ]; then
            MS=$(grep "Finishing Timestep:" "$F" | awk -F'\t' '{print $NF}')
            SWT=$(grep "Sum of Task Waiting Time:" "$F" | awk -F'\t' '{print $NF}')
            RT=$(grep "runtime" "$F" | tail -1 | awk '{print $NF}')
            DONE=$(grep "Tasks completed:" "$F" | awk -F'\t' '{print $NF}')
            NT=$(echo "$DONE" | cut -d'/' -f2); [ -z "$NT" ] && NT=500
            if [ -n "$MS" ]; then
                AVG=$(python3 -c "print(round(float('$SWT')/$NT,1))" 2>/dev/null)
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "$RT" >> "$RESULTS"
            else
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
            fi
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
        fi

        # Parse CENTRAL-FIXED
        F="$OUTDIR/centralfixed_${AG}_${FREQ}.txt"
        if [ -f "$F" ] && [ -s "$F" ]; then
            MS=$(grep "Finishing Timestep:" "$F" | awk -F'\t' '{print $NF}')
            SWT=$(grep "Sum of Task Waiting Time:" "$F" | awk -F'\t' '{print $NF}')
            DONE=$(grep "Tasks completed:" "$F" | awk -F'\t' '{print $NF}')
            NT=$(echo "$DONE" | cut -d'/' -f2); [ -z "$NT" ] && NT=500
            if [ -n "$MS" ]; then
                AVG=$(python3 -c "print(round(float('$SWT')/$NT,1))" 2>/dev/null)
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL-FIXED" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "N/A" >> "$RESULTS"
            else
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL-FIXED" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
            fi
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "CENTRAL-FIXED" "$AG" "$FREQ" "TO" "TO" "TO" "TO" >> "$RESULTS"
        fi

        # Parse LNS-PBS
        MG_OUTDIR="/tmp/mgmapd_lnspbs_${AG}_${FREQ}"
        SOLVER_CSV="$MG_OUTDIR/solver.csv"
        if [ -f "$SOLVER_CSV" ] && [ -s "$SOLVER_CSV" ]; then
            LAST_TS=$(tail -1 "$SOLVER_CSV" | cut -d',' -f10)
            TOTAL_COST=$(tail -1 "$SOLVER_CSV" | cut -d',' -f6)
            TOTAL_RT=$(awk -F',' '{sum+=$1} END {printf "%.3f", sum}' "$SOLVER_CSV")
            printf "%-20s %6s %6s %10s %10s %12s %10ss\n" "LNS-PBS" "$AG" "$FREQ" "$LAST_TS" "$TOTAL_COST" "N/A" "$TOTAL_RT" >> "$RESULTS"
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "LNS-PBS" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" >> "$RESULTS"
        fi

        # Parse LNS-wPBS
        MG_OUTDIR="/tmp/mgmapd_lnswpbs_${AG}_${FREQ}"
        SOLVER_CSV="$MG_OUTDIR/solver.csv"
        if [ -f "$SOLVER_CSV" ] && [ -s "$SOLVER_CSV" ]; then
            LAST_TS=$(tail -1 "$SOLVER_CSV" | cut -d',' -f10)
            TOTAL_COST=$(tail -1 "$SOLVER_CSV" | cut -d',' -f6)
            TOTAL_RT=$(awk -F',' '{sum+=$1} END {printf "%.3f", sum}' "$SOLVER_CSV")
            printf "%-20s %6s %6s %10s %10s %12s %10ss\n" "LNS-wPBS" "$AG" "$FREQ" "$LAST_TS" "$TOTAL_COST" "N/A" "$TOTAL_RT" >> "$RESULTS"
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "LNS-wPBS" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" >> "$RESULTS"
        fi
    done

    # TA algorithms (freq=500 only)
    for ALGO_TAG in taprio tahybrid; do
        if [ "$ALGO_TAG" = "taprio" ]; then ALGO_NAME="TA-Prioritized"; else ALGO_NAME="TA-Hybrid"; fi
        F="$OUTDIR/${ALGO_TAG}_${AG}_500.txt"
        if [ -f "$F" ] && [ -s "$F" ]; then
            MS=$(grep "Finishing Timestep:" "$F" | tail -1 | awk -F'\t' '{print $NF}')
            SWT=$(grep "Sum of Task Waiting Time:" "$F" | tail -1 | awk -F'\t' '{print $NF}')
            RT=$(grep "Wall time:" "$F" | awk '{print $(NF-1)}')
            DONE=$(grep "Tasks completed:" "$F" | tail -1 | awk -F'\t' '{print $NF}')
            NT=$(echo "$DONE" | cut -d'/' -f2); [ -z "$NT" ] && NT=500
            if [ -n "$MS" ]; then
                AVG=$(python3 -c "print(round(float('$SWT')/$NT,1))" 2>/dev/null)
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO_NAME" "$AG" "500" "$MS" "$SWT" "$AVG" "${RT}s" >> "$RESULTS"
            else
                printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO_NAME" "$AG" "500" "FAIL" "FAIL" "FAIL" "FAIL" >> "$RESULTS"
            fi
        else
            printf "%-20s %6s %6s %10s %10s %12s %10s\n" "$ALGO_NAME" "$AG" "500" "FAIL" "FAIL" "FAIL" "FAIL" >> "$RESULTS"
        fi
    done
done

echo "" >> "$RESULTS"
echo "========================================================================" >> "$RESULTS"
echo "Done." >> "$RESULTS"
cat "$RESULTS"
echo "" >&2
echo "Results saved to: $RESULTS" >&2
