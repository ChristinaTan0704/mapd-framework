#!/bin/bash
BASE="/Users/jiaqit/Desktop/paper/reference_code"
DATA="$BASE/data/Instances/small"
OUTDIR="/tmp/mapd_final"
TIMEOUT=1200
CONVERTER="$BASE/convert_tasks.py"

PIDS=""

# CENTRAL-FIXED 10ag/0.2 (was TO at 600s)
(cd "$BASE/CENTRAL-fixed" && gtimeout $TIMEOUT ./driver "$DATA/kiva-10-500-5.map" "$DATA/kiva-0.2.task" > "$OUTDIR/centralfixed_10_0.2.txt" 2>&1; echo "CFIXED 10 0.2 done" >&2) &
PIDS="$PIDS $!"

# CENTRAL-FIXED 50ag/0.2 (was TO)
(cd "$BASE/CENTRAL-fixed" && gtimeout $TIMEOUT ./driver "$DATA/kiva-50-500-5.map" "$DATA/kiva-0.2.task" > "$OUTDIR/centralfixed_50_0.2.txt" 2>&1; echo "CFIXED 50 0.2 done" >&2) &
PIDS="$PIDS $!"

# CENTRAL-FIXED 50ag/0.5 (was TO)
(cd "$BASE/CENTRAL-fixed" && gtimeout $TIMEOUT ./driver "$DATA/kiva-50-500-5.map" "$DATA/kiva-0.5.task" > "$OUTDIR/centralfixed_50_0.5.txt" 2>&1; echo "CFIXED 50 0.5 done" >&2) &
PIDS="$PIDS $!"

# CENTRAL 50ag/500 (was TO)
(gtimeout $TIMEOUT "$BASE/CENTRAL-TP-TPTS/Centralized - ECBS/central" "$DATA/kiva-50-500-5.map" "$DATA/kiva-500.task" 1.0 > "$OUTDIR/central_50_500.txt" 2>&1; echo "CENTRAL 50 500 done" >&2) &
PIDS="$PIDS $!"

# LNS-PBS 50ag/5 (was CRASH)
MG_TASK="/tmp/mgmapd_task_50_5.task"
python3 "$CONVERTER" "$DATA/kiva-5.task" 50 "$MG_TASK" 2>/dev/null
MG_OUT="/tmp/mgmapd_final_lnspbs_50_5"
rm -rf "$MG_OUT"
(cd "$BASE/MGMAPD/LNS-PBS" && gtimeout $TIMEOUT ./lifelong -m "$DATA/kiva-50-500-5.map" -k 50 --scenario=KIVA --solver=PBS --task="$MG_TASK" --simulation_time=5000 -t 120 --simulation_window=5 --planning_window=1073741823 --lns_time=1 --seed=0 --dummy_paths=1 -o "$MG_OUT" -s 0 > "$OUTDIR/lnspbs_50_5.txt" 2>&1; echo "LNSPBS 50 5 done" >&2) &
PIDS="$PIDS $!"

# TA-Hybrid (rerun with SWT output)
(cd "$BASE/TA-Hybrid-build" && gtimeout $TIMEOUT ./driver_scale "$DATA/kiva-10-500-5.map" "$DATA/kiva-500.task" tour/small-500-10-.tour > "$OUTDIR/tahybrid_10_500.txt" 2>&1; echo "TAHYBRID 10 done" >&2) &
PIDS="$PIDS $!"
(cd "$BASE/TA-Hybrid-build" && gtimeout $TIMEOUT ./driver_scale "$DATA/kiva-50-500-5.map" "$DATA/kiva-500.task" tour/50-500.tour > "$OUTDIR/tahybrid_50_500.txt" 2>&1; echo "TAHYBRID 50 done" >&2) &
PIDS="$PIDS $!"

echo "Launched $(echo $PIDS | wc -w) fix jobs, waiting..." >&2
for PID in $PIDS; do wait $PID 2>/dev/null; done
echo "All fix runs done at $(date)!" >&2
