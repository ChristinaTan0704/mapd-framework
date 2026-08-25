## Unified Framework Results

All on **kiva-10-500-5.map** (10 agents, 302 endpoints, 21x35 grid).
All paths verified **collision-free**. All 500/500 tasks completed.

### kiva-500.task (batch, all 500 tasks at t=0)

| Algorithm | Ours | Reference | Gap |
|-----------|------|-----------|-----|
| TA_HYBRID | **1050** | 1037 | +1.3% |
| TA_PRIORITIZED | **1052** | 1053 | -0.1% |
| HBH_MLA | **1095** | — | — |
| TP | 1133 | 1136 | -0.3% |
| TPTS | 1145 | 1105 | +3.6% |
| HUNGARIAN_PBS | 1190 | 1138 | +4.6% |
| LNS_PBS | 1222 | — | — |
| LNS_wPBS | 1418 | — | — |
| HUNGARIAN_wPBS | 1468 | 1153 | +27% |

### kiva-0.2.task (online, 1 task per 5 steps)

| Algorithm | Ours | Reference | Gap |
|-----------|------|-----------|-----|
| HUNGARIAN_PBS | **2513** | 2512 | +0.04% |
| HUNGARIAN_wPBS | **2513** | 2513 | exact |
| HBH_MLA | **2532** | — | — |
| TP | **2532** | 2532 | exact |
| TPTS | **2532** | 2532 | exact |

### Algorithms Implemented (11 + post-processing)

| # | Algorithm | Type | Assignment | MAPF | Dummy path? |
|---|-----------|------|------------|------|----------|
| 1 | TP | Online/IA | Decoupled Greedy | PP (2x A*) | Yes (may be zero-length) |
| 2 | TPTS | Online/IA | Greedy+Swaps | PP (2x A*) | Yes (may be zero-length) |
| 3 | CENTRAL-CBS | Online/IA | Hungarian every timestep | CBS, two-stage | No |
| 3b | CENTRAL-fixed-CBS | Online/IA | Hungarian on new-task/free-agent events | CBS, event-driven two-stage | No |
| 4 | HBH_MLA | Online or semi-online/IA | H-value centralized greedy | PP (MLA*) | Yes (may be zero-length) |
| 5 | TA_PRIORITIZED | Offline/TA | LKH3 TSP tour | Paper-order PP (SEQ_STA), arbitrary ordered goals | Yes |
| 6 | TA_HYBRID | Offline/TA | LKH3 TSP | Two-Group (Flow+CBS) | Yes |
| 7 | HUNGARIAN_PBS | Online/TA | Repeated Hungarian | PBS (MLA*) | Yes |
| 8 | HUNGARIAN_wPBS | Online/TA | Repeated Hungarian | wPBS (MLA*) | Yes |
| 9 | LNS_PBS | Online/TA | Repeated Hungarian + LNS | PBS (MLA*) | Yes |
| 10 | LNS_wPBS | Online/TA | Repeated Hungarian + LNS | wPBS (MLA*) | Yes |
| + | REALPATH_LNS_IMP | Post-processing | Hungarian reassign | PP (A*) | — |

### Build & Run

```bash
make clean && make
./mapd --help

# Online algorithms
./mapd -m <map> -t <tasks> -a TP
./mapd -m <map> -t <tasks> -a TPTS
./mapd -m <map> -t <tasks> -a CENTRAL-CBS
./mapd -m <map> -t <tasks> -a CENTRAL-fixed
./mapd -m <map> -t <tasks> -a HBH_MLA
./mapd -m <map> -t <tasks> -a HUNGARIAN_PBS
./mapd -m <map> -t <tasks> -a HUNGARIAN_wPBS
./mapd -m <map> -t <tasks> -a LNS_PBS [--lns_time <seconds>]
./mapd -m <map> -t <tasks> -a LNS_wPBS [--lns_time <seconds>]

# Offline algorithms (require LKH3 tour file)
./mapd -m <map> -t <tasks> -a TA_PRIORITIZED --tour <tour_file>
./mapd -m <map> -t <tasks> -a TA_HYBRID --tour <tour_file>

# Post-processing improvement (can follow any algorithm)
./mapd -m <map> -t <tasks> -a HBH_MLA --lns_imp 100 --lns_imp_group 8

# Multi-goal tasks (auto-detected varying format)
./mapd -m <map> -t <varying_tasks> -a LNS_PBS
```
