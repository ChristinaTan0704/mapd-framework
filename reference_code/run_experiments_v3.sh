#!/bin/bash
BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
COBRA="$BASE/CENTRAL-TP-TPTS/COBRA/cobra"
CENTRAL="$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central"
TA_PRIO="$BASE/TA-Prioritized/driver_scale"
MGMAPD_PBS="$BASE/MGMAPD/LNS-PBS/lifelong_simple"
MGMAPD_wPBS="$BASE/MGMAPD/LNS-wPBS/lifelong_simple"
OUTFILE="$BASE/experiment_results_v3.txt"
TIMEOUT=600

> "$OUTFILE"

echo "========================================================================" | tee -a "$OUTFILE"
echo "MAPD Reference Code Experiment Results" | tee -a "$OUTFILE"
echo "Date: $(date)" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "" | tee -a "$OUTFILE"
printf "%-18s %5s %6s %10s %10s %12s %12s\n" "Algorithm" "Ag" "Freq" "Makespan" "SWT" "AvgService" "Runtime" | tee -a "$OUTFILE"
printf "%-18s %5s %6s %10s %10s %12s %12s\n" "---------" "--" "----" "--------" "---" "----------" "-------" | tee -a "$OUTFILE"

parse_mgmapd() {
    local OUTPUT="$1" ALGO="$2" AG="$3" FREQ="$4" RT="$5" NTASKS="$6"
    # Output line: "Makespan:  2512 Flowtime:  27.414 Runtime: 0.25405 ms"
    local CLEANLINE=$(echo "$OUTPUT" | grep "Makespan:" | sed 's/  */ /g')
    local M_MS=$(echo "$CLEANLINE" | awk '{for(i=1;i<=NF;i++) if($i=="Makespan:") print $(i+1)}')
    local M_AVG=$(echo "$CLEANLINE" | awk '{for(i=1;i<=NF;i++) if($i=="Flowtime:") print $(i+1)}')

    if [ -n "$M_MS" ] && [ "$M_MS" != "" ]; then
        local M_SWT=$(python3 -c "print(int(round(float('${M_AVG}') * $NTASKS)))")
        local M_AVGR=$(python3 -c "print(round(float('${M_AVG}'),1))")
        printf "%-18s %5s %6s %10s %10s %12s %12s\n" "$ALGO" "$AG" "$FREQ" "$M_MS" "$M_SWT" "$M_AVGR" "$RT" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %12s\n" "$ALGO" "$AG" "$FREQ" "TO" "-" "-" "$RT" | tee -a "$OUTFILE"
    fi
}

for AG in 10 50; do
    MAP="$DATA/kiva-${AG}-500-5.map"
    [ ! -f "$MAP" ] && { echo "SKIP: $MAP not found" >&2; continue; }

    echo "" | tee -a "$OUTFILE"
    echo "=== ${AG} Agents ===" | tee -a "$OUTFILE"

    for FREQ in 0.2 0.5 1 2 5 10 500; do
        TASK="$DATA/kiva-${FREQ}.task"
        [ ! -f "$TASK" ] && { echo "SKIP: $TASK not found" >&2; continue; }
        NTASKS=$(head -1 "$TASK")

        # --- TP + TPTS ---
        echo -n "[${AG}ag f=${FREQ}] TP+TPTS... " >&2
        T0=$(python3 -c "import time; print(time.time())")
        OUTPUT=$(gtimeout $TIMEOUT "$COBRA" "$MAP" "$TASK" 2>&1) || true
        T1=$(python3 -c "import time; print(time.time())")
        RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")

        TP_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | head -1 | awk -F'\t' '{print $NF}')
        TP_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | head -1 | awk -F'\t' '{print $NF}')
        TPTS_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | tail -1 | awk -F'\t' '{print $NF}')
        TPTS_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | tail -1 | awk -F'\t' '{print $NF}')

        if [ -n "$TP_MS" ] && [ "$TP_MS" != "" ]; then
            TP_AVG=$(python3 -c "print(round(float('$TP_SWT')/$NTASKS,1))")
            TPTS_AVG=$(python3 -c "print(round(float('$TPTS_SWT')/$NTASKS,1))")
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TP" "$AG" "$FREQ" "$TP_MS" "$TP_SWT" "$TP_AVG" "$RT" | tee -a "$OUTFILE"
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TPTS" "$AG" "$FREQ" "$TPTS_MS" "$TPTS_SWT" "$TPTS_AVG" "$RT" | tee -a "$OUTFILE"
        else
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TP" "$AG" "$FREQ" "TO" "-" "-" "$RT" | tee -a "$OUTFILE"
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TPTS" "$AG" "$FREQ" "TO" "-" "-" "$RT" | tee -a "$OUTFILE"
        fi
        echo "done" >&2

        # --- CENTRAL ---
        echo -n "[${AG}ag f=${FREQ}] CENTRAL... " >&2
        T0=$(python3 -c "import time; print(time.time())")
        OUTPUT=$(gtimeout $TIMEOUT "$CENTRAL" "$MAP" "$TASK" 1.0 2>&1) || true
        T1=$(python3 -c "import time; print(time.time())")
        RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")

        C_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | awk -F'\t' '{print $NF}')
        C_AVG_RAW=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | awk -F'\t' '{print $NF}')

        if [ -n "$C_MS" ] && [ "$C_MS" != "" ]; then
            C_SWT=$(python3 -c "print(int(round(float('$C_AVG_RAW') * $NTASKS)))")
            C_AVG=$(python3 -c "print(round(float('$C_AVG_RAW'),1))")
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "CENTRAL" "$AG" "$FREQ" "$C_MS" "$C_SWT" "$C_AVG" "$RT" | tee -a "$OUTFILE"
        else
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "CENTRAL" "$AG" "$FREQ" "TO" "-" "-" "$RT" | tee -a "$OUTFILE"
        fi
        echo "done" >&2

        # --- HUNGARIAN_PBS (lns_time=0, full window) --- fast, all freqs
        echo -n "[${AG}ag f=${FREQ}] H_PBS... " >&2
        T0=$(python3 -c "import time; print(time.time())")
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_PBS" "$MAP" $AG "$TASK" 0 1073741823 1073741823 5000 0 2>&1) || true
        T1=$(python3 -c "import time; print(time.time())")
        RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")
        parse_mgmapd "$OUTPUT" "HUNGARIAN_PBS" "$AG" "$FREQ" "$RT" "$NTASKS"
        echo "done" >&2

        # --- HUNGARIAN_wPBS (lns_time=0, window=15) --- fast, all freqs
        echo -n "[${AG}ag f=${FREQ}] H_wPBS... " >&2
        T0=$(python3 -c "import time; print(time.time())")
        OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_wPBS" "$MAP" $AG "$TASK" 0 15 15 5000 0 2>&1) || true
        T1=$(python3 -c "import time; print(time.time())")
        RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")
        parse_mgmapd "$OUTPUT" "HUNGARIAN_wPBS" "$AG" "$FREQ" "$RT" "$NTASKS"
        echo "done" >&2
    done

    # --- LNS-PBS and LNS-wPBS: only on freq=500 (batch) to avoid multi-minute LNS runs ---
    TASK="$DATA/kiva-500.task"
    NTASKS=$(head -1 "$TASK")

    echo -n "[${AG}ag f=500] LNS-PBS... " >&2
    T0=$(python3 -c "import time; print(time.time())")
    OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_PBS" "$MAP" $AG "$TASK" 1 1073741823 1073741823 5000 0 2>&1) || true
    T1=$(python3 -c "import time; print(time.time())")
    RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")
    parse_mgmapd "$OUTPUT" "LNS-PBS" "$AG" "500" "$RT" "$NTASKS"
    echo "done" >&2

    echo -n "[${AG}ag f=500] LNS-wPBS... " >&2
    T0=$(python3 -c "import time; print(time.time())")
    OUTPUT=$(gtimeout $TIMEOUT "$MGMAPD_wPBS" "$MAP" $AG "$TASK" 1 15 15 5000 0 2>&1) || true
    T1=$(python3 -c "import time; print(time.time())")
    RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")
    parse_mgmapd "$OUTPUT" "LNS-wPBS" "$AG" "500" "$RT" "$NTASKS"
    echo "done" >&2

    # --- Offline: TA-Prioritized (500 only) ---
    TOUR="$BASE/TA-Prioritized/tour/${AG}-500.tour"
    echo -n "[${AG}ag f=500] TA-Prio... " >&2
    T0=$(python3 -c "import time; print(time.time())")
    OUTPUT=$(cd "$BASE/TA-Prioritized" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" 2>&1) || true
    T1=$(python3 -c "import time; print(time.time())")
    RT=$(python3 -c "print(f'{${T1}-${T0}:.1f}s')")

    LAST=$(echo "$OUTPUT" | grep "^[0-9]" | tail -1)
    P_MS=$(echo "$LAST" | awk '{print $2}')
    P_FT=$(echo "$LAST" | awk '{print $1}')
    if [ -n "$P_MS" ] && [ "$P_MS" != "" ]; then
        P_AVG=$(python3 -c "print(round(float('$P_FT')/$NTASKS,1))")
        printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TA-PRIORITIZED" "$AG" "500" "$P_MS" "$P_FT" "$P_AVG" "$RT" | tee -a "$OUTFILE"
    else
        printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TA-PRIORITIZED" "$AG" "500" "FAIL" "-" "-" "$RT" | tee -a "$OUTFILE"
    fi
    echo "done" >&2

    # --- Offline: TA-Hybrid (from pre-computed results) ---
    if [ "$AG" = "10" ]; then
        HYBRID_OUT="$BASE/TA-Hybrid-build/result-500.out"
        if [ -f "$HYBRID_OUT" ]; then
            H_MS=$(awk 'NR>1 {if($3>max) max=$3} END {print max}' "$HYBRID_OUT")
            H_FT=$(awk 'NR>1 {sum+=$3} END {print sum}' "$HYBRID_OUT")
            H_RT=$(grep "runtime:" "$HYBRID_OUT" | awk '{print $2}')
            H_AVG=$(python3 -c "print(round(float('$H_FT')/$NTASKS,1))")
            printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TA-HYBRID" "$AG" "500" "$H_MS" "$H_FT" "$H_AVG" "${H_RT}s" | tee -a "$OUTFILE"
        fi
    else
        printf "%-18s %5s %6s %10s %10s %12s %12s\n" "TA-HYBRID" "$AG" "500" "N/A(hang)" "-" "-" "-" | tee -a "$OUTFILE"
    fi
done

echo "" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "Notes:" | tee -a "$OUTFILE"
echo "  SWT = Sum of (completion_time - release_time) across all tasks" | tee -a "$OUTFILE"
echo "  AvgService = SWT / num_tasks" | tee -a "$OUTFILE"
echo "  CENTRAL ref prints AvgService directly (code divides by 500)" | tee -a "$OUTFILE"
echo "  MGMAPD ref prints AvgService as 'Flowtime'" | tee -a "$OUTFILE"
echo "  TA-PRIORITIZED SWT = total flowtime from ref (sum of completion times)" | tee -a "$OUTFILE"
echo "  LNS-PBS/wPBS only run on freq=500 (LNS 1s/call makes online too slow)" | tee -a "$OUTFILE"
echo "  TA-HYBRID 50ag: reference binary hangs, no results available" | tee -a "$OUTFILE"
echo "========================================================================" | tee -a "$OUTFILE"
echo "" >&2
echo "Results saved to: $OUTFILE" >&2
