#!/bin/bash
# run_method.sh — run ONE reimplementation method across agent counts x all freqs,
# parse makespan / SWT / runtime, and emit a CSV (runtime in SECONDS).
#
# Usage:
#   run_method.sh "<METHOD_LABEL>" "<AGENTS_CSV>" [OUT_CSV]
# Examples:
#   run_method.sh "Hungarian+PBS-MLA*" "10,50"            # screen
#   run_method.sh "Hungarian+PBS-MLA*" "10,20,30,40,50"   # full grid after a fix
#
# Output CSV columns: method,agents,freq,makespan,swt,runtime_s,status
set -u

ROOT="/Users/jiaqit/Desktop/meta/paper/MAPD_framework_imp"
MAPD="$ROOT/mapd"
DATA="/Users/jiaqit/Desktop/meta/paper/reference_code/data/Instances/small"
TOUR="$ROOT/tour"
TIMEOUT="${TIMEOUT:-600}"   # per-cell wall-clock cap (seconds); override via env for fast screens

METHOD="${1:?method label required}"
AGENTS_CSV="${2:?agents csv required, e.g. 10,50}"
OUTCSV="${3:-/tmp/run_${METHOD//[^A-Za-z0-9]/_}.csv}"

# Map method label -> "ALGO|EXTRA_ARGS"  (EXTRA may contain TOUR/AGENTS placeholders)
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
if [ -z "$SPEC" ]; then
  echo "ERROR: unknown method label '$METHOD'" >&2
  exit 2
fi
ALGO="${SPEC%%|*}"
EXTRA_TMPL="${SPEC#*|}"

FREQS=(0.2 0.5 1 2 5 10 500)
echo "method,agents,freq,makespan,swt,runtime_s,status" > "$OUTCSV"

IFS=',' read -ra AGENTS <<< "$AGENTS_CSV"
for AG in "${AGENTS[@]}"; do
  if [[ "$METHOD" == TA-* ]]; then FLIST=(500); else FLIST=("${FREQS[@]}"); fi
  EXTRA="${EXTRA_TMPL//TOUR/$TOUR}"; EXTRA="${EXTRA//AGENTS/$AG}"
  for FREQ in "${FLIST[@]}"; do
    MAP="$DATA/kiva-${AG}-500-5.map"
    TASK="$DATA/kiva-${FREQ}.task"
    if [ ! -f "$MAP" ] || [ ! -f "$TASK" ]; then
      echo "${METHOD},${AG},${FREQ},N/A,N/A,N/A,missing_input" >> "$OUTCSV"; continue
    fi
    TMP="/tmp/run_${METHOD//[^A-Za-z0-9]/_}_${AG}_${FREQ}.txt"
    gtimeout "$TIMEOUT" "$MAPD" -m "$MAP" -t "$TASK" -a "$ALGO" $EXTRA -s 1 > "$TMP" 2>&1
    EC=$?
    STATUS="ok"
    [ $EC -eq 124 ] && STATUS="timeout"
    [ $EC -ne 0 ] && [ $EC -ne 124 ] && STATUS="error"
    grep -qi "COLLISION DETECTED\|COLLISION CHECK FAILED" "$TMP" 2>/dev/null && STATUS="collision"
    MS=$(grep "Finishing Timestep:" "$TMP" 2>/dev/null | awk '{print $NF}')
    SWT=$(grep "Sum of Task Waiting Time:" "$TMP" 2>/dev/null | awk '{print $NF}')
    RT_MS=$(grep "Total runtime:" "$TMP" 2>/dev/null | awk '{print $(NF-1)}')
    # convert runtime ms -> seconds
    RT_S=$(awk -v x="$RT_MS" 'BEGIN{ if(x=="")print"N/A"; else printf"%.3f", x/1000.0 }')
    [ -z "$MS" ]  && MS="N/A"
    [ -z "$SWT" ] && SWT="N/A"
    echo "${METHOD},${AG},${FREQ},${MS},${SWT},${RT_S},${STATUS}" >> "$OUTCSV"
    echo "[DONE] ${METHOD} ${AG}ag f=${FREQ}: ms=${MS} swt=${SWT} rt=${RT_S}s [${STATUS}]" >&2
  done
done
echo "$OUTCSV"
