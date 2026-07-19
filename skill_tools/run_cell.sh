#!/bin/bash
# run_cell.sh — run ONE (method,agent,freq) cell and append a CSV line to a shared file.
# Arg: "METHOD|AGENT|FREQ"   (pipe-delimited; method label may contain spaces/+/*/parens)
# Output line: method,agents,freq,makespan,swt,runtime_s,status  -> $CELLOUT (default /tmp/screen_full/cells.csv)
set -u
ROOT="/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
MAPD="$ROOT/mapd"
DATA="/Users/jiaqit/Desktop/2026 meta docs/paper/reference_code/data/Instances/small"
TOUR="$ROOT/tour"
TIMEOUT="${TIMEOUT:-1000}"
CELLOUT="${CELLOUT:-/tmp/screen_full/cells.csv}"

IFS='|' read -r METHOD AG FREQ <<< "$1"

get_flags() {
  case "$1" in
    "TP-STA*")               echo "TP|" ;;
    "TPTS-STA*")             echo "TPTS|" ;;
    "CENTRAL-ECBS")          echo "CENTRAL|" ;;
    "CENTRAL-ECBS-SIPP")     echo "CENTRAL|--sipp" ;;
    "TA-Hybrid-STA*")        echo "TA_HYBRID|--tour TOUR/AGENTS-500.tour" ;;
    "TA-Prioritized-STA*")   echo "TA_PRIORITIZED|--tour TOUR/AGENTS-500.tour" ;;
    "Hungarian+PBS-MLA*")    echo "HUNGARIAN_PBS|" ;;
    "Hungarian+wPBS-MLA*")   echo "HUNGARIAN_wPBS|" ;;
    "LNS(1s)+PBS-MLA*")      echo "LNS_PBS|--lns_time 1" ;;
    "LNS(1s)+wPBS-MLA*")     echo "LNS_wPBS|--lns_time 1" ;;
    "Hungarian+PBS-MLSIPP")  echo "HUNGARIAN_PBS|--sipp" ;;
    "Hungarian+wPBS-MLSIPP") echo "HUNGARIAN_wPBS|--sipp" ;;
    "LNS(1s)+PBS-MLSIPP")    echo "LNS_PBS|--lns_time 1 --sipp" ;;
    "LNS(1s)+wPBS-MLSIPP")   echo "LNS_wPBS|--lns_time 1 --sipp" ;;
    "TP-SIPP")               echo "TP|--single_agent MLA --sipp" ;;
    "TPTS-SIPP")             echo "TPTS|--single_agent MLA --sipp" ;;
    "Hungarian+PP-SIPP")     echo "HUNGARIAN_PBS|--mapf PP --sipp" ;;
    "LNS(1s)+PP-SIPP")       echo "LNS_PBS|--mapf PP --lns_time 1 --sipp" ;;
    *) echo "" ;;
  esac
}

SPEC="$(get_flags "$METHOD")"
[ -z "$SPEC" ] && { echo "${METHOD},${AG},${FREQ},N/A,N/A,N/A,bad_method" >> "$CELLOUT"; exit 0; }
ALGO="${SPEC%%|*}"; EXTRA_TMPL="${SPEC#*|}"
EXTRA="${EXTRA_TMPL//TOUR/$TOUR}"; EXTRA="${EXTRA//AGENTS/$AG}"

MAP="$DATA/kiva-${AG}-500-5.map"; TASK="$DATA/kiva-${FREQ}.task"
if [ ! -f "$MAP" ] || [ ! -f "$TASK" ]; then
  echo "${METHOD},${AG},${FREQ},N/A,N/A,N/A,missing_input" >> "$CELLOUT"; exit 0
fi
TMP="/tmp/screen_full/out_${METHOD//[^A-Za-z0-9]/_}_${AG}_${FREQ}.txt"
gtimeout "$TIMEOUT" "$MAPD" -m "$MAP" -t "$TASK" -a "$ALGO" $EXTRA -s 1 > "$TMP" 2>&1
EC=$?
STATUS="ok"
[ $EC -eq 124 ] && STATUS="timeout"
[ $EC -ne 0 ] && [ $EC -ne 124 ] && STATUS="error"
grep -qi "COLLISION DETECTED\|COLLISION CHECK FAILED" "$TMP" 2>/dev/null && STATUS="collision"
MS=$(grep "Finishing Timestep:" "$TMP" 2>/dev/null | awk '{print $NF}')
SWT=$(grep "Sum of Task Waiting Time:" "$TMP" 2>/dev/null | awk '{print $NF}')
RT_MS=$(grep "Total runtime:" "$TMP" 2>/dev/null | awk '{print $(NF-1)}')
RT_S=$(awk -v x="$RT_MS" 'BEGIN{ if(x=="")print"N/A"; else printf"%.3f", x/1000.0 }')
[ -z "$MS" ] && MS="N/A"; [ -z "$SWT" ] && SWT="N/A"
echo "${METHOD},${AG},${FREQ},${MS},${SWT},${RT_S},${STATUS}" >> "$CELLOUT"
echo "[CELL] ${METHOD} ${AG}ag f=${FREQ}: ms=${MS} swt=${SWT} rt=${RT_S}s [${STATUS}]" >&2
