# Conversation Summary — MAPD Unified Framework Project

**Date range:** May 10–11, 2026
**Papers folder:** `/Users/jiaqit/Desktop/paper/`
**Pseudocode folder:** `/Users/jiaqit/Desktop/paper/pesudo/`

---

## 1. Papers Read

All papers are in `/Users/jiaqit/Desktop/paper/`:

| File | Paper | Role |
|------|-------|------|
| `MAPD framework.pdf` | "Multi-Agent Pickup and Delivery: Formulations and Algorithms" | **Main framework paper** — Table 1 defines all 10 algorithms |
| `Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks copy.pdf` | Ma et al., AAMAS 2017 | Original TP, TPTS, CENTRAL paper |
| `Target assignment and path planning for navigation tasks with teams of agents.pdf` | Ma, Ph.D. thesis, USC 2020 | CENTRAL-fixed (Section 6.5.3) |
| `Task and path planning for multi-agent pickup and delivery.pdf` | Liu et al., AAMAS 2019 | TA-Prioritized, TA-Hybrid |
| `A Multi-Label A_ Algorithm for Multi-Agent Pathfinding copy.pdf` | Grenouilleau et al., ICAPS 2019 | HBH-MLA* |
| `Integrated task assignment and path planning for capacitated multi-agent pickup and delivery.pdf` | Chen et al., IEEE RA-L 2021 | RMCA |
| `Multi-Goal Multi-Agent Pickup and Delivery copy.pdf` | Xu et al., IROS 2022 | LNS-PBS, LNS-wPBS |

---

## 2. Files Created/Updated

### In `/Users/jiaqit/Desktop/paper/pesudo/`:

1. **`unified_mapd_pseudocode.md`** — Complete pseudocode for the unified MAPD framework
   - Section 0: Configuration enums and algorithm presets
   - Section 0.1: All 10 algorithm configs from Table 1
   - Section 1: Data structures + Constraint Table API (add/delete/update/copy/copy_without/is_valid_for/can_hold)
   - Section 2: Main loop (unified for online/offline/semi-online — no separate pre-loop offline block)
   - Section 3: GetReleasedTasks (OFFLINE returns all tasks at t=0)
   - Section 4: Update_System
   - Section 5: ShouldAssign (ONCE triggers at t=0)
   - Section 6: Task_Assignment dispatcher
   - Section 7: GetReplanAgents
   - Section 8: GetAvailTasks
   - Section 9: Assignment methods (Decoupled Greedy, TPTS swaps, Centralized Greedy, Hungarian)
   - Section 10: TA methods (TSP, TA-Hybrid, RMCA, TA-LNS)
   - Section 11: Path_Planning dispatcher
   - Section 12: Two-layer planning architecture:
     - `Find_Path` (pure, no side effects)
     - `Plan_Agent_Path` (stateful wrapper: delete/plan/add)
     - CBS, PBS, wPBS with proper ct management
   - Section 13: Single-agent search (STA*, MLA*, SIPP)
   - Section 14: Helper functions
   - Section 15: ShouldReplan + GetAgentsToReplan
   - Section 16: LNS subroutines
   - Section 17: Constraint table lifecycle summary
   - Section 18: Algorithm instantiation summary table

2. **`algorithm_call_traces.md`** — Call traces for all 10 algorithms showing exact pseudocode sections hit, with constraint table lifecycle

3. **`algorithm_steps_with_quotes.md`** — Step-by-step summaries with quotes from ORIGINAL papers (not the framework paper)

---

## 3. Key Design Decisions

### 3.1 Constraint Table Architecture
- **Two-layer planning:** `Find_Path` (pure) vs `Plan_Agent_Path` (stateful wrapper)
- **ct operations:** `add`, `delete`, `update`, `copy`, `copy_without`
- **CBS/PBS pattern:** `base_ct = ct.copy_without(agents_to_plan)` → search → batch `ct.update` on solution
- **Decoupled PP pattern:** `ct.delete` → `Find_Path` → `ct.add` per agent
- **TPTS swap pattern:** Save → `ct.delete(agent_j)` → plan → rollback with `ct.add` on failure

### 3.2 Endpoint Holding (Deadlock Avoidance)
From the framework paper (Section 4.4):
- **One mechanism:** Endpoint Holding — every path ends at endpoint ℓ_i
- **Three choices of ℓ_i:**
  1. Task endpoint (ℓ_i = g[K]) → "Holding Endpoint" in Table 1 → zero-length dummy path
  2. Fixed non-task endpoint → "Dummy Path" in Table 1
  3. Flexible endpoint
- Paper quote: *"Set ℓ_i = g[K], which corresponds to 'holding a task endpoint' adopted in [1] and can be viewed as a zero-length dummy path."*

### 3.3 Unified Main Loop (Online/Offline)
- Removed separate pre-loop offline initialization
- `GetReleasedTasks(OFFLINE)` returns all tasks at t=0
- `ShouldAssign(ONCE)` triggers at t=0 only
- Offline algorithms (TA-Prioritized, TA-Hybrid) flow through the same while loop

### 3.4 STA*/MLA* take agent_i as parameter
- So they call `ct.is_valid_for(agent_i, ...)` which ignores the agent's own reservations

---

## 4. Key Questions & Answers

### Q: What's the difference between endpoint holding and dummy path?
**A:** They're the same mechanism. The paper says endpoint holding with ℓ_i = g[K] (task endpoint) "can be viewed as a zero-length dummy path." Dummy path is just the extra path needed when ℓ_i ≠ g[K].

### Q: Can TP use STA_NONTASK_EP instead of STA_TASK_EP?
**A:** Yes. We traced through the pseudocode and verified all functions compose correctly. `FilterEndpointConflicts` becomes essentially a no-op (non-task EP ≠ task goals), `Plan_Dummy_Path(HOLDING_ENDPOINT)` stays at non-task EP, and `Update_System` correctly handles mid-path task completion.

### Q: How does Path_Planning_CBS choose endpoints?
**A:** It didn't — this was a gap in the original pseudocode. `Find_Single_Agent_Path` was undefined. We fixed it by creating `Find_Path_CBS` which builds `merged_ct = base_ct + CBS constraints` and calls `Find_Path` (which handles endpoint choice via `config.single_agent`).

### Q: What's the difference between CENTRAL and CENTRAL-fixed?
**A:** Two differences from Table 1:
1. **Trigger:** CENTRAL = every timestep; CENTRAL-fixed = event-driven (new task or agent becomes free)
2. **Completeness:** CENTRAL = no; CENTRAL-fixed = yes

### Q: Why is CENTRAL incomplete?
**A:** No paper explicitly states the reason. The original Ma 2017 paper just says *"We want CENTRAL to be reasonably efficient and effective but do not require that it is optimally effective or even solves all well-formed MAPD instances."*

Our analysis: CENTRAL reassigns every timestep via Hungarian → assignments can oscillate → agents never complete tasks. TP avoids this because assignments persist. CENTRAL-fixed avoids this because it only triggers on events.

**Important correction:** CBS itself always finds paths (Properties 6.4 and 6.5 in Ma's thesis prove this). The incompleteness is about task completion progress, not CBS feasibility.

### Q: How does ICBS work in TA-Hybrid?
**A:** ICBS is used only for Group 1 (newly occupied agents going to delivery). It modifies the low-level A* goal test to include dummy path reservation. Group 2 (free agents) uses min-cost max-flow instead because free agents can swap pickup locations.

### Q: Does TA-Hybrid call assignment only once?
**A:** TSP assignment is called once. But min-cost max-flow can swap pickup locations among free agents at any timestep when the set of free agents changes.

---

## 5. Algorithm Summary

| Algorithm | Source Paper | Type | Trigger | Assignment | Path Planning | Deadlock | Complete? |
|-----------|-------------|------|---------|------------|---------------|----------|-----------|
| TP | Ma 2017 | IA | free waits | decoupled greedy | PP (STA*) | hold EP | yes |
| TPTS | Ma 2017 | IA | free waits | greedy + swaps | PP (STA*) | hold EP | yes |
| CENTRAL | Ma 2017 | IA | every step | Hungarian | CBS | hold EP | no |
| CENTRAL-fixed | Ma thesis 2020 | IA | event-driven | Hungarian | CBS | hold EP | yes |
| HBH-MLA* | Grenouilleau 2019 | IA | free waits | centralized greedy | PP (MLA*) | hold EP | yes |
| TA-Prioritized | Liu 2019 | TA | once | LKH3 TSP | PP (seq STA*) | dummy path | yes |
| TA-Hybrid | Liu 2019 | TA | once + on free | LKH3 + flow | ICBS + flow | dummy path | yes |
| RMCA | Chen 2021 | TA | once/new task | greedy insert + LNS | coupled | none | no |
| LNS-PBS | Xu 2022 | TA | unassigned/free | rep. Hungarian + LNS | PBS (MLA*) | dummy path | yes |
| LNS-wPBS | Xu 2022 | TA | unassigned/free | rep. Hungarian + LNS | wPBS (MLA*) | dummy path | no |

---

## 6. Attempted But Not Completed

- **Building MGMAPD reference code** (`/Users/jiaqit/Desktop/paper/reference_code/MGMAPD/`): The pre-compiled binary is Linux ELF. Attempted to install cmake + boost via Homebrew but hit permissions issues. Code is C++11, uses Boost program_options. Need `sudo` access to fix Homebrew permissions, then `cmake . && make` in the `LNS-wPBS/` or `LNS-PBS/` directory.
- Test data locations:
  - `/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small/kiva-0.2.task`
  - `/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small/kiva-500.task`
  - Map: `/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small/kiva-10-500-5.map`

---

## 7. File Locations

```
/Users/jiaqit/Desktop/paper/
├── pesudo/
│   ├── unified_mapd_pseudocode.md      ← Main pseudocode
│   ├── algorithm_call_traces.md        ← Call traces for all 10 algorithms
│   └── algorithm_steps_with_quotes.md  ← Steps with original paper quotes
├── MAPD framework.pdf                  ← Main framework paper
├── Lifelong Multi-Agent Path Finding... ← Ma 2017 (TP, TPTS, CENTRAL)
├── Target assignment and path planning... ← Ma thesis 2020 (CENTRAL-fixed)
├── Task and path planning...           ← Liu 2019 (TA-Prioritized, TA-Hybrid)
├── A Multi-Label A_...                 ← Grenouilleau 2019 (HBH-MLA*)
├── Integrated task assignment...       ← Chen 2021 (RMCA)
├── Multi-Goal Multi-Agent...           ← Xu 2022 (LNS-PBS, LNS-wPBS)
├── reference_code/MGMAPD/             ← Implementation code (needs rebuild)
└── history/
    └── conversation_summary.md         ← This file
```
