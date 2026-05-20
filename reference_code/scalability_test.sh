#!/bin/bash
# Scalability test: TA-Hybrid and TA-Prioritized on kiva-500.task with increasing agents
# Timeout: 300 seconds (5 minutes) per run

TIMEOUT=300
DATA=data/Instances/small
BASE=$(cd "$(dirname "$0")" && pwd)

echo "============================================="
echo "Scalability Test — kiva-500.task (batch)"
echo "Timeout: ${TIMEOUT}s per run"
echo "============================================="
echo ""

# TA-Hybrid
echo "--- TA-Hybrid ---"
for N in 10 20 30 40 50; do
    MAP=$BASE/$DATA/kiva-${N}-500-5.map
    TASK=$BASE/$DATA/kiva-500.task
    TOUR=$BASE/TA-Hybrid-build/tour/${N}-500.tour
    OUT=$BASE/TA-Hybrid-build/scale-${N}-500.out
    if [ ! -f "$TOUR" ]; then
        echo "  ${N} agents: SKIP (no tour file)"
        continue
    fi
    echo -n "  ${N} agents: "
    RESULT=$(cd $BASE/TA-Hybrid-build && timeout ${TIMEOUT} ./driver_new "$MAP" "$TASK" "$TOUR" "$OUT" 2>&1)
    EXIT=$?
    if [ $EXIT -eq 124 ]; then
        echo "TIMEOUT (>${TIMEOUT}s)"
    elif [ $EXIT -ne 0 ]; then
        echo "ERROR (exit $EXIT)"
    else
        WALL=$(echo "$RESULT" | grep "Wall time" | awk '{print $3}')
        MAKESPAN=$(echo "$RESULT" | grep "makespan" | awk '{print $2}')
        echo "OK — wall=${WALL}s, makespan=${MAKESPAN}"
    fi
done

echo ""

# TA-Prioritized (need to run from its directory, hardcoded to 10 agents)
# We need to create a flexible driver. For now, test by symlinking maps.
echo "--- TA-Prioritized ---"
for N in 10 20 30 40 50; do
    MAP=Instances/small/kiva-${N}-500-5.map
    TASK=Instances/small/kiva-500.task
    TOUR=tour/${N}-500.tour
    OUT=output/offline/${N}-500.out
    if [ ! -f "$BASE/TA-Prioritized/$TOUR" ]; then
        echo "  ${N} agents: SKIP (no tour file)"
        continue
    fi
    echo -n "  ${N} agents: "
    # TA-Prioritized driver is hardcoded; we need to rebuild. Skip for now.
    echo "SKIP (driver hardcoded to 10 agents, needs rebuild)"
    break
done

echo ""
echo "Done."
