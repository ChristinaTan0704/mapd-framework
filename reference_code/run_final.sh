#!/bin/bash
BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
OUTDIR="/tmp/mapd_final"
mkdir -p "$OUTDIR"
TIMEOUT=600
CONVERTER="$BASE/convert_tasks.py"

PIDS=""
for AG in 10 50; do
    for FREQ in 0.2 0.5 1 2 5 10 500; do
        MAP="$DATA/kiva-${AG}-500-5.map"
        TASK="$DATA/kiva-${FREQ}.task"
        [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && continue

        # COBRA (TP + TPTS)
        (gtimeout $TIMEOUT "$BASE/CENTRAL-TP-TPTS/COBRA/cobra" "$MAP" "$TASK" > "$OUTDIR/cobra_${AG}_${FREQ}.txt" 2>&1; echo "$AG $FREQ COBRA" >&2) &
        PIDS="$PIDS $!"

        # CENTRAL
        (gtimeout $TIMEOUT "$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central" "$MAP" "$TASK" 1.0 > "$OUTDIR/central_${AG}_${FREQ}.txt" 2>&1; echo "$AG $FREQ CENTRAL" >&2) &
        PIDS="$PIDS $!"

        # CENTRAL-FIXED
        (cd "$BASE/CENTRAL-fixed" && gtimeout $TIMEOUT ./driver "$MAP" "$TASK" > "$OUTDIR/centralfixed_${AG}_${FREQ}.txt" 2>&1; echo "$AG $FREQ CFIXED" >&2) &
        PIDS="$PIDS $!"

        # LNS-PBS
        MG_TASK="/tmp/mgmapd_task_${AG}_${FREQ}.task"
        python3 "$CONVERTER" "$TASK" "$AG" "$MG_TASK" 2>/dev/null
        MG_OUT="/tmp/mgmapd_final_lnspbs_${AG}_${FREQ}"
        rm -rf "$MG_OUT"
        (cd "$BASE/MGMAPD/LNS-PBS" && gtimeout $TIMEOUT ./lifelong -m "$MAP" -k $AG --scenario=KIVA --solver=PBS --task="$MG_TASK" --simulation_time=5000 -t 120 --simulation_window=5 --planning_window=1073741823 --lns_time=1 --seed=0 --dummy_paths=1 -o "$MG_OUT" -s 0 > "$OUTDIR/lnspbs_${AG}_${FREQ}.txt" 2>&1; echo "$AG $FREQ LNSPBS" >&2) &
        PIDS="$PIDS $!"

        # LNS-wPBS
        MG_OUT="/tmp/mgmapd_final_lnswpbs_${AG}_${FREQ}"
        rm -rf "$MG_OUT"
        (cd "$BASE/MGMAPD/LNS-wPBS" && gtimeout $TIMEOUT ./lifelong -m "$MAP" -k $AG --scenario=KIVA --solver=PBS --task="$MG_TASK" --simulation_time=5000 -t 120 --simulation_window=15 --planning_window=15 --lns_time=1 --seed=0 --dummy_paths=1 -o "$MG_OUT" -s 0 > "$OUTDIR/lnswpbs_${AG}_${FREQ}.txt" 2>&1; echo "$AG $FREQ LNSWPBS" >&2) &
        PIDS="$PIDS $!"
    done

    # TA-Prioritized
    TASK="$DATA/kiva-500.task"
    MAP="$DATA/kiva-${AG}-500-5.map"
    TOUR="$BASE/TA-Prioritized/tour/${AG}-500.tour"
    (cd "$BASE/TA-Prioritized" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" > "$OUTDIR/taprio_${AG}_500.txt" 2>&1; echo "$AG TAPRIO" >&2) &
    PIDS="$PIDS $!"

    # TA-Hybrid (use fixed build)
    if [ "$AG" = "10" ]; then
        TOUR="$BASE/TA-Hybrid-build/tour/small-500-10-.tour"
    else
        TOUR="$BASE/TA-Hybrid-build/tour/${AG}-500.tour"
    fi
    (cd "$BASE/TA-Hybrid-build" && gtimeout $TIMEOUT ./driver_scale "$MAP" "$TASK" "$TOUR" > "$OUTDIR/tahybrid_${AG}_500.txt" 2>&1; echo "$AG TAHYBRID" >&2) &
    PIDS="$PIDS $!"
done

echo "Launched $(echo $PIDS | wc -w) jobs, waiting..." >&2
for PID in $PIDS; do wait $PID 2>/dev/null; done
echo "All done at $(date)!" >&2
