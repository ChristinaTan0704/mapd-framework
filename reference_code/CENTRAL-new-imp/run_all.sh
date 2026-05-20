#!/bin/bash
cd /Users/jiaqit/Desktop/paper/reference_code/CENTRAL-new-imp
CSV="/Users/jiaqit/Desktop/paper/output/central_ref_results.csv"
echo "algorithm,agents,frequency,makespan,swt,avg_service,tasks,runtime_ms" > "$CSV"

for AG in 10 20 30 40 50; do
    for FREQ in 500 10 5 2 1 0.5 0.2; do
        echo -n "CENTRAL-REF ${AG}ag freq${FREQ}... " >&2
        LINE=$(gtimeout 600 ./central_single "$AG" "$FREQ" </dev/null 2>&1 >/dev/null | grep "^RESULT,")
        EC=$?
        if [ -z "$LINE" ] || [ $EC -eq 124 ]; then
            echo "TIMEOUT" >&2
            echo "CENTRAL-REF,$AG,$FREQ,TO,TO,TO,500,>600000" >> "$CSV"
        else
            MS=$(echo "$LINE" | cut -d',' -f4)
            SWT=$(echo "$LINE" | cut -d',' -f5)
            RT=$(echo "$LINE" | cut -d',' -f6)
            AVG=$(python3 -c "print(round($SWT/500,1))")
            echo "makespan=$MS swt=$SWT (${RT}ms)" >&2
            echo "CENTRAL-REF,$AG,$FREQ,$MS,$SWT,$AVG,500,$RT" >> "$CSV"
        fi
    done
done
echo "Done! Results: $CSV" >&2
