---
name: review-implementation
description: Build, test all 9 MAPD algorithms, verify collision-free results match references, and iteratively fix any failures.
---

# Review MAPD Framework Implementation

You are reviewing the MAPD unified framework at `/Users/jiaqit/Desktop/paper/MAPD_framework_imp/`.

## Step 1: Build

```bash
cd /Users/jiaqit/Desktop/paper/MAPD_framework_imp
make clean && make
```

If the build fails, read the compiler errors, fix them in `src/simulation.cpp`, `inc/simulation.h`, or other files, and rebuild. Do not proceed until the build succeeds.

## Step 2: Run all 9 algorithms

Use these paths:
```
MAP=/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/COBRA/kiva-10-500-5.map
TASK500=/Users/jiaqit/Desktop/paper/reference_code/TA-ICBS/Instances/small/kiva-500.task
TASK02=/Users/jiaqit/Desktop/paper/reference_code/CENTRAL-TP-TPTS/COBRA/kiva-0.2.task
TOUR_TA=/Users/jiaqit/Desktop/paper/reference_code/TA-Prioritized/tour/10-500.tour
TOUR_HY=/Users/jiaqit/Desktop/paper/reference_code/TA-Hybrid/tour/10-500.tour
```

Run each algorithm with appropriate timeouts. Capture the collision check result, makespan, and tasks completed:

```bash
# Online algorithms — test on BOTH kiva-500 and kiva-0.2
./mapd -m $MAP -t $TASK500 -a TP
./mapd -m $MAP -t $TASK02  -a TP
./mapd -m $MAP -t $TASK500 -a TPTS
./mapd -m $MAP -t $TASK02  -a TPTS
./mapd -m $MAP -t $TASK500 -a HBH_MLA
./mapd -m $MAP -t $TASK02  -a HBH_MLA
timeout 60 ./mapd -m $MAP -t $TASK500 -a CENTRAL
timeout 60 ./mapd -m $MAP -t $TASK02  -a CENTRAL
timeout 60 ./mapd -m $MAP -t $TASK500 -a CENTRAL_FIXED
timeout 60 ./mapd -m $MAP -t $TASK02  -a CENTRAL_FIXED
timeout 120 ./mapd -m $MAP -t $TASK500 -a HUNGARIAN_PBS
timeout 120 ./mapd -m $MAP -t $TASK02  -a HUNGARIAN_PBS
timeout 120 ./mapd -m $MAP -t $TASK500 -a HUNGARIAN_wPBS
timeout 120 ./mapd -m $MAP -t $TASK02  -a HUNGARIAN_wPBS

# Offline algorithms — kiva-500 only (require tour files)
timeout 120 ./mapd -m $MAP -t $TASK500 -a TA_PRIORITIZED --tour $TOUR_TA
timeout 30  ./mapd -m $MAP -t $TASK500 -a TA_HYBRID --tour $TOUR_HY
```

You may run multiple independent tests in parallel to save time.

## Step 3: Verify each result

For EVERY test, check these three criteria:

1. **Collision check**: output must contain `COLLISION CHECK PASSED`
2. **Tasks completed**: must be `500/500`
3. **Makespan**: must be within the acceptable range of reference values

### Reference values and acceptable ranges

| Algorithm | kiva-500 ref | kiva-500 range | kiva-0.2 ref | kiva-0.2 range |
|-----------|-------------|----------------|-------------|----------------|
| TP | 1136 | 1050–1200 | 2532 | 2480–2600 |
| TPTS | 1105 | 1050–1200 | 2532 | 2480–2600 |
| HBH_MLA | ~1135 | 1050–1200 | ~2532 | 2480–2600 |
| CENTRAL | 1101 | 1050–1200 | 2516 | 2480–2600 |
| CENTRAL_FIXED | ~1133 | 1050–1200 | ~2514 | 2480–2600 |
| TA_PRIORITIZED | 1053 | 1000–1150 | — | — |
| TA_HYBRID | 1037 | 1000–1150 | — | — |
| HUNGARIAN_PBS | 1138 | 1050–1300 | 2512 | 2480–2600 |
| HUNGARIAN_wPBS | 1153 | 1050–1600 | 2513 | 2480–2600 |

### Failure modes to check

- **COLLISION CHECK FAILED**: path planning bug — read the collision details (agent IDs, location, timestep) and trace through the relevant path planning function
- **Tasks completed < 500**: loop termination bug — check `end()`, task removal logic, or assignment logic
- **Makespan way too high** (e.g., >3000 for kiva-500): usually means agents are getting stuck or path planning fails silently and agents wait
- **Timeout**: algorithm hangs — check for infinite loops in the main loop, A*/MLA* search, or PBS DFS
- **Crash (exit 138/139)**: memory issue — check for stack overflow in recursive functions (TPTS swaps), dangling pointers, or out-of-bounds access

## Step 4: Fix failures

If ANY algorithm fails:

1. **Identify** which algorithm and benchmark failed, and the failure mode
2. **Read** the relevant source code:
   - `src/simulation.cpp` — main implementation (see ALGORITHM_TRACES.md for function locations)
   - `inc/simulation.h` — class declarations
   - `src/cbs.cpp` / `inc/cbs.h` — CBS/ECBS (for CENTRAL/CENTRAL_FIXED)
3. **Diagnose** the root cause. Add debug prints if needed (cerr, not cout)
4. **Fix** the issue
5. **Rebuild**: `make`
6. **Re-test** the failing algorithm ONLY to verify the fix
7. **Re-test ALL algorithms** to check for regressions (a fix for one algorithm must not break others)

Repeat steps 4–7 until all 9 algorithms pass on all applicable benchmarks.

## Step 5: Report

When all tests pass, output a summary table:

```
| Algorithm       | kiva-500 | kiva-0.2 | Collision-Free |
|-----------------|----------|----------|----------------|
| TP              | XXXX     | XXXX     | PASS/FAIL      |
| TPTS            | XXXX     | XXXX     | PASS/FAIL      |
| HBH_MLA         | XXXX     | XXXX     | PASS/FAIL      |
| CENTRAL         | XXXX     | XXXX     | PASS/FAIL      |
| CENTRAL_FIXED   | XXXX     | XXXX     | PASS/FAIL      |
| TA_PRIORITIZED  | XXXX     | —        | PASS/FAIL      |
| TA_HYBRID       | XXXX     | —        | PASS/FAIL      |
| HUNGARIAN_PBS   | XXXX     | XXXX     | PASS/FAIL      |
| HUNGARIAN_wPBS  | XXXX     | XXXX     | PASS/FAIL      |
```

State whether any fixes were needed and what was changed.
