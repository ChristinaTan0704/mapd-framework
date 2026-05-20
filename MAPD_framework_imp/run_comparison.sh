#!/bin/bash
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
COBRA="/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/COBRA/cobra"
CENTRAL="/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/Centralized - ECBS/central"
PBS_REF="/Users/jiaqit/Desktop/paper/reference_code/MGMAPD/LNS-PBS/lifelong_simple"
WPBS_REF="/Users/jiaqit/Desktop/paper/reference_code/MGMAPD/LNS-wPBS/lifelong_simple"
TA_PRIO="/Users/jiaqit/Desktop/paper/reference_code/TA-Prioritized"
TA_HYB="/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid"
TA_HYB_BUILD="/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid-build"
MAPD="/Users/jiaqit/Desktop/paper/MAPD_framework_imp/mapd"
TOUR="/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid-build/tour"
OUT="/Users/jiaqit/Desktop/paper/output/comparison.csv"
TMP="/tmp/comparison_tmp.txt"
TIMEOUT=600

echo "method,source,agents,frequency,makespan,swt" > "$OUT"

run_ref_cobra() {
    local AG=$1 FREQ=$2 MAP=$3 TASK=$4
    gtimeout $TIMEOUT "$COBRA" "$MAP" "$TASK" > "$TMP" 2>&1
    local TP_MS=$(grep "Finishing Timestep:" "$TMP" | head -1 | awk '{print $NF}')
    local TP_SWT=$(grep "Sum of Task Waiting Time:" "$TMP" | head -1 | awk '{print $NF}')
    local TPTS_MS=$(grep "Finishing Timestep:" "$TMP" | tail -1 | awk '{print $NF}')
    local TPTS_SWT=$(grep "Sum of Task Waiting Time:" "$TMP" | tail -1 | awk '{print $NF}')
    [ -n "$TP_MS" ] && echo "TP-STA,REF,$AG,$FREQ,$TP_MS,$TP_SWT" >> "$OUT"
    [ -n "$TPTS_MS" ] && echo "TPTS-STA,REF,$AG,$FREQ,$TPTS_MS,$TPTS_SWT" >> "$OUT"
}

run_ref_central() {
    local AG=$1 FREQ=$2 MAP=$3 TASK=$4
    gtimeout $TIMEOUT "$CENTRAL" "$MAP" "$TASK" 1.0 > "$TMP" 2>&1
    local MS=$(grep "Finishing Timestep:" "$TMP" | awk '{print $NF}')
    local SWT_AVG=$(grep "Sum of Task Waiting Time:" "$TMP" | awk '{print $NF}')
    if [ -n "$MS" ]; then
        local SWT=$(python3 -c "print(int(float('$SWT_AVG')*500))" 2>/dev/null)
        echo "CENTRAL-CBS,REF,$AG,$FREQ,$MS,$SWT" >> "$OUT"
    fi
}

run_ref_mgmapd() {
    local METHOD=$1 BIN=$2 AG=$3 FREQ=$4 MAP=$5 TASK=$6 LNS=$7
    # PBS driver: map task agents task_trunc lns_time simulation_time seed outdir
    gtimeout $TIMEOUT "$BIN" "$MAP" "$TASK" $AG 2 $LNS 5000 0 "/tmp/mgmapd_${METHOD}_${AG}_${FREQ}" > "$TMP" 2>&1
    local MS=$(grep "Makespan:" "$TMP" | awk '{print $2}')
    local FT=$(grep "Flowtime:" "$TMP" | awk '{print $4}')
    if [ -n "$MS" ]; then
        local SWT=$(python3 -c "print(int(float('$FT')*500))" 2>/dev/null)
        echo "$METHOD,REF,$AG,$FREQ,$MS,$SWT" >> "$OUT"
    fi
}

run_ref_mgmapd_wpbs() {
    local METHOD=$1 BIN=$2 AG=$3 FREQ=$4 MAP=$5 TASK=$6 LNS=$7 WIN=$8
    # wPBS driver: map task agents task_trunc lns_time planning_window simulation_time seed outdir
    gtimeout $TIMEOUT "$BIN" "$MAP" "$TASK" $AG 2 $LNS $WIN 5000 0 "/tmp/mgmapd_${METHOD}_${AG}_${FREQ}" > "$TMP" 2>&1
    local MS=$(grep "Makespan:" "$TMP" | awk '{print $2}')
    local FT=$(grep "Flowtime:" "$TMP" | awk '{print $4}')
    if [ -n "$MS" ]; then
        local SWT=$(python3 -c "print(int(float('$FT')*500))" 2>/dev/null)
        echo "$METHOD,REF,$AG,$FREQ,$MS,$SWT" >> "$OUT"
    fi
}

run_ours() {
    local METHOD=$1 AG=$2 FREQ=$3; shift 3
    gtimeout $TIMEOUT "$MAPD" "$@" > "$TMP" 2>&1
    local MS=$(grep "Finishing Timestep:" "$TMP" | awk '{print $NF}')
    local SWT=$(grep "Sum of Task Waiting Time:" "$TMP" | awk '{print $NF}')
    [ -n "$MS" ] && echo "$METHOD,OURS,$AG,$FREQ,$MS,$SWT" >> "$OUT"
}

for AG in 10 50; do
    for FREQ in 0.2 0.5 1 2 5 10 500; do
        MAP="$DATA/kiva-${AG}-500-5.map"
        TASK="$DATA/kiva-${FREQ}.task"
        echo "--- ${AG}ag freq${FREQ} ---" >&2

        echo -n "  COBRA..." >&2
        run_ref_cobra $AG $FREQ "$MAP" "$TASK"
        echo " done" >&2

        echo -n "  CENTRAL..." >&2
        run_ref_central $AG $FREQ "$MAP" "$TASK"
        echo " done" >&2

        echo -n "  HUNG-PBS..." >&2
        run_ref_mgmapd "HUNGARIAN-PBS" "$PBS_REF" $AG $FREQ "$MAP" "$TASK" 0
        echo " done" >&2

        echo -n "  HUNG-wPBS..." >&2
        run_ref_mgmapd_wpbs "HUNGARIAN-wPBS" "$WPBS_REF" $AG $FREQ "$MAP" "$TASK" 0 10
        echo " done" >&2

        if [ "$FREQ" = "500" ] || [ "$FREQ" = "10" ] || [ "$FREQ" = "5" ]; then
            echo -n "  LNS-PBS..." >&2
            run_ref_mgmapd "LNS-PBS" "$PBS_REF" $AG $FREQ "$MAP" "$TASK" 1
            echo " done" >&2

            echo -n "  LNS-wPBS..." >&2
            run_ref_mgmapd_wpbs "LNS-wPBS" "$WPBS_REF" $AG $FREQ "$MAP" "$TASK" 1 10
            echo " done" >&2
        fi

        echo -n "  ours..." >&2
        run_ours "TP-STA" $AG $FREQ -m "$MAP" -t "$TASK" -a TP
        run_ours "TPTS-STA" $AG $FREQ -m "$MAP" -t "$TASK" -a TPTS
        run_ours "CENTRAL-CBS" $AG $FREQ -m "$MAP" -t "$TASK" -a CENTRAL
        run_ours "HUNGARIAN-PBS" $AG $FREQ -m "$MAP" -t "$TASK" -a HUNGARIAN_PBS
        run_ours "HUNGARIAN-wPBS" $AG $FREQ -m "$MAP" -t "$TASK" -a HUNGARIAN_wPBS

        if [ "$FREQ" = "500" ] || [ "$FREQ" = "10" ] || [ "$FREQ" = "5" ]; then
            run_ours "LNS-PBS" $AG $FREQ -m "$MAP" -t "$TASK" -a LNS_PBS --lns_time 1
            run_ours "LNS-wPBS" $AG $FREQ -m "$MAP" -t "$TASK" -a LNS_wPBS --lns_time 1
        fi
        echo " done" >&2
    done

    # TA: offline only (freq500)
    MAP="$DATA/kiva-${AG}-500-5.map"
    TASK="$DATA/kiva-500.task"

    # REF: TA-Prioritized (only 10ag has driver_exp)
    if [ "$AG" = "10" ]; then
        echo -n "  TA-Prioritized REF..." >&2
        cd "$TA_PRIO"
        gtimeout $TIMEOUT ./driver_exp --500-only > /dev/null 2>&1
        if [ -f "output/offline/10-500.out" ]; then
            python3 -c "
lines=open('output/offline/10-500.out').readlines()
n=int(lines[0])
tl=open('$TASK').readlines()
rel=[int(tl[i+1].split()[0]) for i in range(n)]
ms=max(int(lines[i+1]) for i in range(n))
swt=sum(int(lines[i+1])-rel[i] for i in range(n))
print(f'TA-Prioritized,REF,$AG,500,{ms},{swt}')
" >> "$OUT"
        fi
        cd /Users/jiaqit/Desktop/paper/MAPD_framework_imp
        echo " done" >&2
    fi

    # REF: TA-Hybrid
    TA_HYB_TOUR="$TA_HYB/tour/${AG}-500.tour"
    if [ -f "$TA_HYB_TOUR" ]; then
        echo -n "  TA-Hybrid REF..." >&2
        gtimeout $TIMEOUT "$TA_HYB_BUILD/driver_new" "$MAP" "$TASK" "$TA_HYB_TOUR" /tmp/tah_${AG}.txt > "$TMP" 2>&1
        MS=$(grep "Makespan:" "$TMP" | awk '{print $NF}')
        [ -n "$MS" ] && echo "TA-Hybrid,REF,$AG,500,$MS,N/A" >> "$OUT"
        echo " done" >&2
    fi

    # OURS: TA
    OUR_TOUR="$TOUR/${AG}-500.tour"
    if [ -f "$OUR_TOUR" ]; then
        run_ours "TA-Hybrid" $AG 500 -m "$MAP" -t "$TASK" -a TA_HYBRID --tour "$OUR_TOUR"
        run_ours "TA-Prioritized" $AG 500 -m "$MAP" -t "$TASK" -a TA_PRIORITIZED --tour "$OUR_TOUR"
    fi
done

echo "=== Done! ===" >&2
