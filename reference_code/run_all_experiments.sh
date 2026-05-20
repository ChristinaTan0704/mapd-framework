#!/bin/bash
set -e

BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
COBRA="$BASE/CENTRAL-TP-TPTS/COBRA/cobra"
CENTRAL="$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central"
TA_PRIO="$BASE/TA-Prioritized/driver_scale"
TA_HYBRID="$BASE/TA-Hybrid/driver_scale"
MGMAPD_PBS="$BASE/MGMAPD/LNS-PBS/lifelong"
MGMAPD_wPBS="$BASE/MGMAPD/LNS-wPBS/lifelong"
OUTFILE="$BASE/experiment_results.txt"
TIMEOUT=600

> "$OUTFILE"

echo "========================================================================" | tee -a "$OUTFILE"
echo "MAPD Reference Code Experiment Results" | tee -a "$OUTFILE"
echo "Date: $(date)" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "" | tee -a "$OUTFILE"
printf "%-18s %5s %6s %10s %10s %12s %10s\n" "Algorithm" "Agents" "Freq" "Makespan" "SWT" "AvgService" "Runtime" | tee -a "$OUTFILE"
printf "%-18s %5s %6s %10s %10s %12s %10s\n" "---------" "------" "----" "--------" "---" "----------" "-------" | tee -a "$OUTFILE"

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
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TP-REF" "$AG" "$FREQ" "$TP_MS" "$TP_SWT" "$TP_AVG" "N/A" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TP-REF" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
    if [ -n "$TPTS_MS" ] && [ "$TPTS_MS" != "" ]; then
        local TPTS_AVG=$(python3 -c "print(round(float('$TPTS_SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TPTS-REF" "$AG" "$FREQ" "$TPTS_MS" "$TPTS_SWT" "$TPTS_AVG" "N/A" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TPTS-REF" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

parse_central() {
    local OUTPUT="$1" AG="$2" FREQ="$3"
    local MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | awk -F'\t' '{print $NF}')
    local SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | awk -F'\t' '{print $NF}')
    local DONE=$(echo "$OUTPUT" | grep "Tasks completed:" | awk -F'\t' '{print $NF}')
    local RT=$(echo "$OUTPUT" | grep "runtime:" | awk '{print $NF}')

    local NTASKS=$(echo "$DONE" | cut -d'/' -f2)
    [ -z "$NTASKS" ] && NTASKS=500

    if [ -n "$MS" ] && [ "$MS" != "" ]; then
        local AVG=$(python3 -c "print(round(float('$SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "CENTRAL-REF" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "$RT" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "CENTRAL-REF" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
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
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "${RT}s" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

parse_mgmapd() {
    local OUTPUT="$1" ALGO="$2" AG="$3" FREQ="$4"
    local MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | awk -F'\t' '{print $NF}')
    local SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | awk -F'\t' '{print $NF}')
    local DONE=$(echo "$OUTPUT" | grep "Tasks completed:" | awk -F'\t' '{print $NF}')

    local NTASKS=$(echo "$DONE" | cut -d'/' -f2)
    [ -z "$NTASKS" ] && NTASKS=500

    local TOTAL_TIME=$(echo "$OUTPUT" | grep "CPU time (seconds)" | awk '{print $NF}')
    [ -z "$TOTAL_TIME" ] && TOTAL_TIME=$(echo "$OUTPUT" | grep "total time" | awk '{print $NF}')

    if [ -n "$MS" ] && [ "$MS" != "" ]; then
        local AVG=$(python3 -c "print(round(float('$SWT')/$NTASKS,1))" 2>/dev/null || echo "N/A")
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "$MS" "$SWT" "$AVG" "${TOTAL_TIME}s" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %10s\n" "$ALGO" "$AG" "$FREQ" "FAIL" "FAIL" "FAIL" "FAIL" | tee -a "$OUTFILE"
    fi
}

for AG in 10 50; do
    MAP="$DATA/kiva-${AG}-500-5.map"
    [ ! -f "$MAP" ] && { echo "SKIP: $MAP not found" >&2; continue; }

    echo "" | tee -a "$OUTFILE"
    echo "=== ${AG} Agents ===" | tee -a "$OUTFILE"

    # --- Online algorithms: all frequencies ---
    for FREQ in 0.2 0.5 1 2 5 10 500; do
        TASK="$DATA/kiva-${FREQ}.task"
        [ ! -f "$TASK" ] && { echo "SKIP: $TASK not found" >&2; continue; }

        # TP + TPTS (COBRA)
        echo -n "[${AG}ag freq=${FREQ}] COBRA (TP+TPTS)... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$COBRA" "$MAP" "$TASK" 2>&1) || true
        if [ -z "$OUTPUT" ]; then
            printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TP-REF" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
            printf "%-18s %5s %6s %10s %10s %12s %10s\n" "TPTS-REF" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
        else
            parse_cobra "$OUTPUT" "$AG" "$FREQ"
        fi
        echo "done" >&2

        # CENTRAL
        echo -n "[${AG}ag freq=${FREQ}] CENTRAL... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$CENTRAL" "$MAP" "$TASK" 1.0 2>&1) || true
        if [ -z "$OUTPUT" ]; then
            printf "%-18s %5s %6s %10s %10s %12s %10s\n" "CENTRAL-REF" "$AG" "$FREQ" "TO" "TO" "TO" "TO" | tee -a "$OUTFILE"
        else
            parse_central "$OUTPUT" "$AG" "$FREQ"
        fi
        echo "done" >&2

        # HUNGARIAN_PBS (MGMAPD LNS-PBS with lns_time=0)
        echo -n "[${AG}ag freq=${FREQ}] HUNGARIAN_PBS... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_PBS" -m "$DATA/kiva-${AG}-500-5" -k $AG --scenario=KIVA --solver=PBS --task="$TASK" --simulation_time=5000 -t 120 --simulation_window=1073741823 --planning_window=1073741823 --lns_time=0 --seed=0 -o /tmp/mgmapd_hpbs 2>&1) || true
        parse_mgmapd "$OUTPUT" "HUNGARIAN_PBS" "$AG" "$FREQ"
        echo "done" >&2

        # HUNGARIAN_wPBS (MGMAPD LNS-wPBS with lns_time=0)
        echo -n "[${AG}ag freq=${FREQ}] HUNGARIAN_wPBS... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_wPBS" -m "$DATA/kiva-${AG}-500-5" -k $AG --scenario=KIVA --solver=PBS --task="$TASK" --simulation_time=5000 -t 120 --simulation_window=15 --planning_window=15 --lns_time=0 --seed=0 -o /tmp/mgmapd_hwpbs 2>&1) || true
        parse_mgmapd "$OUTPUT" "HUNGARIAN_wPBS" "$AG" "$FREQ"
        echo "done" >&2

        # LNS-PBS (MGMAPD LNS-PBS with lns_time=1)
        echo -n "[${AG}ag freq=${FREQ}] LNS-PBS... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_PBS" -m "$DATA/kiva-${AG}-500-5" -k $AG --scenario=KIVA --solver=PBS --task="$TASK" --simulation_time=5000 -t 120 --simulation_window=1073741823 --planning_window=1073741823 --lns_time=1 --seed=0 -o /tmp/mgmapd_lpbs 2>&1) || true
        parse_mgmapd "$OUTPUT" "LNS-PBS" "$AG" "$FREQ"
        echo "done" >&2

        # LNS-wPBS (MGMAPD LNS-wPBS with lns_time=1)
        echo -n "[${AG}ag freq=${FREQ}] LNS-wPBS... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_wPBS" -m "$DATA/kiva-${AG}-500-5" -k $AG --scenario=KIVA --solver=PBS --task="$TASK" --simulation_time=5000 -t 120 --simulation_window=15 --planning_window=15 --lns_time=1 --seed=0 -o /tmp/mgmapd_lwpbs 2>&1) || true
        parse_mgmapd "$OUTPUT" "LNS-wPBS" "$AG" "$FREQ"
        echo "done" >&2
    done

    # --- Offline algorithms: only kiva-500 ---
    TASK="$DATA/kiva-500.task"

    # TA-Prioritized
    echo -n "[${AG}ag freq=500] TA-Prioritized... " >&2
    TOUR="$BASE/TA-Prioritized/tour/${AG}-500.tour"
    OUTPUT=$(cd "$BASE/TA-Prioritized" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" 2>&1) || true
    parse_ta "$OUTPUT" "TA-PRIO-REF" "$AG" "500"
    echo "done" >&2

    # TA-Hybrid
    echo -n "[${AG}ag freq=500] TA-Hybrid... " >&2
    if [ "$AG" = "10" ]; then
        TOUR="$BASE/TA-Hybrid/tour/small-500-10-.tour"
    else
        TOUR="$BASE/TA-Hybrid/tour/${AG}-500.tour"
    fi
    OUTPUT=$(cd "$BASE/TA-Hybrid" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" 2>&1) || true
    parse_ta "$OUTPUT" "TA-HYBRID-REF" "$AG" "500"
    echo "done" >&2
done

echo "" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "Done." | tee -a "$OUTFILE"
echo "" >&2
echo "Results saved to: $OUTFILE" >&2
