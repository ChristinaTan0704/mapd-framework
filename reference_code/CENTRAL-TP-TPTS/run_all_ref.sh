#!/bin/bash
DATA="/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
COBRA="/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/COBRA/cobra"
CENTRAL="/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/Centralized - ECBS/central"
CSV="/Users/jiaqit/Desktop/paper/output/reference_results.csv"
TIMEOUT=600

echo "algorithm,agents,frequency,makespan,swt,avg_service,runtime_ms" > "$CSV"

parse_result() {
    local OUTPUT="$1"
    local ALGO="$2"
    local AG="$3"
    local FREQ="$4"

    local MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | tail -1 | awk '{print $NF}')
    local SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | tail -1 | awk '{print $NF}')

    if [ -z "$MS" ] || [ "$MS" = "" ]; then
        echo "$ALGO,$AG,$FREQ,TO,TO,TO,TO" >> "$CSV"
        echo "TIMEOUT" >&2
        return
    fi

    local AVG=$(python3 -c "print(round(float('$SWT')/500,1))" 2>/dev/null || echo "N/A")
    local RT=$(echo "$OUTPUT" | grep "runtime" | tail -1 | awk '{print $NF}')
    [ -z "$RT" ] && RT="N/A"

    echo "$ALGO,$AG,$FREQ,$MS,$SWT,$AVG,$RT" >> "$CSV"
    echo "makespan=$MS swt=$SWT" >&2
}

for AG in 10 20 30 40 50; do
    for FREQ in 0.2 0.5 1 2 5 10 500; do
        MAP="$DATA/kiva-${AG}-500-5.map"
        TASK="$DATA/kiva-${FREQ}.task"
        [ ! -f "$MAP" ] || [ ! -f "$TASK" ] && continue

        # TP + TPTS (COBRA runs both)
        echo -n "[${AG}ag freq${FREQ}] COBRA (TP+TPTS)... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$COBRA" "$MAP" "$TASK" 2>&1)
        EC=$?
        if [ $EC -eq 124 ] || [ -z "$OUTPUT" ]; then
            echo "TP-REF,$AG,$FREQ,TO,TO,TO,TO" >> "$CSV"
            echo "TPTS-REF,$AG,$FREQ,TO,TO,TO,TO" >> "$CSV"
            echo "TIMEOUT" >&2
        else
            # First ShowTask = TP, Second ShowTask = TPTS
            TP_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | head -1 | awk '{print $NF}')
            TP_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | head -1 | awk '{print $NF}')
            TPTS_MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | tail -1 | awk '{print $NF}')
            TPTS_SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | tail -1 | awk '{print $NF}')

            TP_AVG=$(python3 -c "print(round(float('$TP_SWT')/500,1))" 2>/dev/null || echo "N/A")
            TPTS_AVG=$(python3 -c "print(round(float('$TPTS_SWT')/500,1))" 2>/dev/null || echo "N/A")

            echo "TP-REF,$AG,$FREQ,$TP_MS,$TP_SWT,$TP_AVG,N/A" >> "$CSV"
            echo "TPTS-REF,$AG,$FREQ,$TPTS_MS,$TPTS_SWT,$TPTS_AVG,N/A" >> "$CSV"
            echo "TP=$TP_MS/$TP_SWT TPTS=$TPTS_MS/$TPTS_SWT" >&2
        fi

        # CENTRAL
        echo -n "[${AG}ag freq${FREQ}] CENTRAL... " >&2
        OUTPUT=$(gtimeout $TIMEOUT "$CENTRAL" "$MAP" "$TASK" 1.0 2>&1)
        EC=$?
        if [ $EC -eq 124 ] || [ -z "$OUTPUT" ]; then
            echo "CENTRAL-REF,$AG,$FREQ,TO,TO,TO,TO" >> "$CSV"
            echo "TIMEOUT" >&2
        else
            MS=$(echo "$OUTPUT" | grep "Finishing Timestep:" | awk '{print $NF}')
            SWT=$(echo "$OUTPUT" | grep "Sum of Task Waiting Time:" | awk '{print $NF}')
            RT=$(echo "$OUTPUT" | grep "runtime" | awk '{print $NF}')
            AVG=$(python3 -c "print(round(float('$SWT')/500,1))" 2>/dev/null || echo "N/A")
            echo "CENTRAL-REF,$AG,$FREQ,$MS,$SWT,$AVG,$RT" >> "$CSV"
            echo "makespan=$MS swt=$SWT" >&2
        fi
    done
done
echo "Done! Results: $CSV" >&2
