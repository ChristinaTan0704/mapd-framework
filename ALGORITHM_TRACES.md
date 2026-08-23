# Algorithm Implementation Traces — 17 Reimplementation Methods

This document traces the **code path of each of the 17 reimplementation methods**
(the labels used in `all_results.xlsx`). Every label is one `-a <ALGO>` preset plus a
combination of CLI flags (`--single_agent`, `--mapf`, `--sipp`, `--lns_time`, `--tour`).
The flags per label are defined in `skill_tools/run_method.sh`; the preset → config
mapping is `inc/config.h` `get_preset()`.

Line numbers refer to `src/simulation.cpp` unless noted (current as of commit `cc0d38d`).
Regenerate with the `/match-reference-results` debugging step.

> **Recent changes reflected here**
> - **Agent-start shuffle** (`init` [~40]): MGMAPD methods (Hungarian/LNS + PBS/wPBS)
>   shuffle `mapd_map.agent_starts` with `std::default_random_engine()` to reproduce the
>   reference agent-home permutation — changes PBS priority (= agent index) & parking.
> - **TA-Hybrid Group 1 is now faithful ICBS** (`hybrid_group1_plan` [3752]), not the old
>   fixed-priority search.
> - **Windowed-PBS for the non-SIPP wPBS-MLA\* methods (#7, #9) is now a framework-native
>   solver** (`wpbs_windowed_solve` [8835] → `native_wpbs_solve` [8741], classes
>   `WStateTimeAStar`/`WReservationTable`/`WPBS` in `simulation.cpp`). The former bolt-on
>   `ref_solve.cpp`/`refsolve` module has been **removed**. SIPP wPBS siblings (#11, #13)
>   still use `pbs_core(true)`.

---

## 0. Entry point & top-level dispatch

```
driver.cpp:main()
  set_parameters(config, vm)
    config = get_preset(algo)                              [config.h get_preset]  -- preset per -a
    apply --lns_time / --single_agent / --mapf / --sipp    [driver.cpp:19-33]
  sim.init(map, task, config, tour)                        [24]
    -- MGMAPD agent-home shuffle (gated, see 0.2)          [~40]
  sim.run()                                                [123]
  (optional) sim.realpath_lns_imp(rounds, group)                          -- only with --lns_imp
  sim.fullCollisionCheck(config.name)                      [driver.cpp]
  sim.showTask() / sim.saveOutput()

run()                                                      [123]
  while (!end()):
    release_tasks()
    task_assignment_and_path_planning():                   [~525]
      task_assignment()                                    -- switch assign_method [~857]
      if should_replan(): path_planning()                  [933]
    update_system()                                        [260]
  post-loop: residual end-parking deconfliction
```

### `should_assign()` / `should_replan()` — by trigger
| trigger (config) | assign when | replan? |
|---|---|---|
| `AT_ON_FREE_WAITS` (TP/TPTS) | any agent `finish_time <= t` | false (planning inside assignment) |
| `AT_EVERY_TIMESTEP` (CENTRAL) | `central_has_event_` & free agent exists | `central_has_event_` |
| `AT_ON_NEW_TASK_OR_FREE` (CENTRAL_FIXED) | `central_reassign_event_` & free agent | `central_has_event_` |
| `AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT` (Hungarian/LNS) | assignment event | replan event |
| `AT_ONCE` (TA) | `timestep == 0` | true |

### `task_assignment()` dispatch — `switch(config.assign_method)` [~857]
```
AM_DECOUPLED_GREEDY (TP)        -> assign_decoupled_greedy()   [985]
AM_DECOUPLED_GREEDY_SWAPS (TPTS)-> assign_tpts()               [1057]
AM_HUNGARIAN (CENTRAL / Hung)   -> assign_hungarian() [1411] (CENTRAL, every-step)
                                   OR assign_repeated_hungarian() [4459] (Hungarian+PBS/wPBS/PP)
AM_REPEATED_HUNGARIAN_LNS       -> assign_repeated_hungarian_lns() [4838]
AM_LKH3_TSP / _REASSIGN (TA)    -> assign_ta_tsp()             [2649]
```

### `path_planning()` dispatch [933]
```
if (CENTRAL/CENTRAL_FIXED trigger) && mapf==PBS   -> path_planning_cbs_with_pp()
if (AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT) && mapf==PP -> path_planning_pp_mla() [7625]
switch mapf:
  MAPF_DECOUPLED_PP -> plan_ta_prioritized()[2752] (TA) | path_planning_pp()
  MAPF_CBS          -> path_planning_cbs()          [1586]
  MAPF_PBS          -> path_planning_pbs()          [7755] -> pbs_core(false) [6467]
  MAPF_wPBS         -> path_planning_wpbs()         [8799] -> native (see 0.3) or pbs_core(true)
  MAPF_TA_HYBRID    -> plan_ta_hybrid()             [3992]
```

---

## 0.2 Agent-start shuffle (MGMAPD faithfulness) [`init` ~40]

```
mgmapd_method = (assign_method==AM_HUNGARIAN || AM_REPEATED_HUNGARIAN_LNS)
                && (mapf==MAPF_PBS || mapf==MAPF_wPBS)
if (mgmapd_method):
    std::shuffle(mapd_map.agent_starts, std::default_random_engine())   -- default seed
```
The reference MGMAPD loader collects home ('r') cells in map-scan order then shuffles them
with `std::default_random_engine()` and sets `starts[k]=agent_home_locations[k]`. The reimpl
reproduces the **identical permutation** for MGMAPD methods only. Because **PBS priority = agent
index**, this changes who-yields-to-whom and the order-dependent parking. Applies to **#6–#13**
(Hungarian/LNS × PBS/wPBS × MLA*/MLSIPP). **Not** #16/#17 (they use `--mapf PP`), nor
TP/TPTS/CENTRAL/TA (different reference, no shuffle). Assignment *outcome* is invariant to the
permutation (the Hungarian optimum is order-independent).

## 0.3 Low-level solver dispatch (`--sipp` / `--single_agent`)

**A. `plan_agent` lambda inside `pbs_core` and in `path_planning_pp_mla`:**
```
if config.use_sipp        -> sipp_search()             [5750]   (--sipp)   ==> "MLSIPP" / "PP-SIPP"
else if mla_mode==MLA_SEQ_STA -> seq_sta_plan (seq single-goal A*)
else if mla_mode==MLA_TASKWISE (DEFAULT) -> mla_star_taskwise() [6363]     ==> "MLA*"
else (MLA_SEQ)            -> seq_mla_star()             [5137]
```
Default `mla_mode = MLA_TASKWISE` (config.h) and `run_method.sh` never passes `--mla_mode`, so
the `-MLA*` labels resolve to **`mla_star_taskwise`** (plans each task group via `seq_mla_star`).
`--sipp` overrides all of the above.

**B. `plan_task_token` [2404] (token methods TP / TPTS):**
```
if config.use_sipp        -> multi-goal sipp_search(pickup, delivery)   (no fallback)
else if single_agent==SA_MLA_SEQUENCE -> token_mla_star()   [2279]
else (SA_STA_TASK_EP, DEFAULT)         -> 2x astar(pickup)+astar(delivery) [2176]
```

**`sipp_search` [5750]** — Safe-Interval Path Planning over multi-goal sequences; O(1) endpoint
heuristics; in windowed (wPBS) mode caps at `start+win_look` (env `SIPP_WIN_LOOK`, default 16).

## 0.4 Framework-native windowed PBS (the `ref_solve` replacement) [7798–8460, 8835]

`path_planning_wpbs()` [8799] chooses between two windowed implementations:
```
use_native = (assign_method==AM_HUNGARIAN || AM_REPEATED_HUNGARIAN_LNS)
             && mapf==MAPF_wPBS && !use_sipp        -- i.e. exactly #7 and #9
if use_native: wpbs_windowed_solve()   [8835]
else:          pbs_core(true)          [6467]       -- #11, #13 (SIPP wPBS)
(env REF_SOLVE=1/0 can force native on/off for debugging)
```
`wpbs_windowed_solve()` [8835] builds per-solve inputs from live state, applies the reference
dispersal (`choose_good_endpoint`: task endpoints only, skip own, home fallback), calls
`native_wpbs_solve()` [8741], and writes committed paths back to `token.path`/`agents.path`.
Native solver components (anonymous namespace in `simulation.cpp`, backed by `mapd_map`):
- `WStateTimeAStar` [8161] — dual fibonacci-heap **focal** low level (open by f, focal by
  secondary key), cutoff at `start+window`, two-phase goal sequence.
- `WReservationTable` [7926] — hard constraints from the higher-priority (reachable) set only
  (no soft CAT), window + permanent endpoint hold.
- `WPBS` [8329] / `WPBSNode` [8303] — DFS high level, sequential-prioritized root,
  `find_consistent_paths` child cascade.
This is a faithful in-framework re-expression of the MGMAPD reference solve (the old separate
`ref_solve.cpp`/`namespace refsolve` module is deleted).

---

## The 17 methods

Each entry: **CLI** (from `run_method.sh`) → **preset** (`config.h`) → **trace**.

### 1. TP-STA*  — Token Passing, space-time A*
**CLI:** `-a TP` · **preset:** `DECOUPLED_GREEDY, ON_FREE_WAITS, DECOUPLED_PP, SA_STA_TASK_EP, HOLDING_ENDPOINT`
```
task_assignment() -> AM_DECOUPLED_GREEDY:
  should_assign(): AT_ON_FREE_WAITS -> any agent finish_time<=t
  pick ONE free agent, assign_decoupled_greedy(ag)                            [985]
    build hold set; for it in token.tasks: closest task by BFS h-val
    plan_task_token(ag, task) -> SA_STA default -> astar(pickup)+astar(deliv) [2404/2176]
    else move2EP(ag) / bump finish_time                                        [2484]
  should_replan() -> false (planning happened in assignment)
```

### 2. TPTS-STA*  — Token Passing with Task Swaps, space-time A*
**CLI:** `-a TPTS` · **preset:** `DECOUPLED_GREEDY_SWAPS, ON_FREE_WAITS, DECOUPLED_PP, SA_STA_TASK_EP, HOLDING_ENDPOINT`
```
task_assignment() -> AM_DECOUPLED_GREEDY_SWAPS:
  pick ONE free agent, assign_tpts(ag, depth=0)                                [1057]
    save state; for it in token.tasks (by distance): ag_hide = displaced agent
      plan_task_token(ag, task, ag_hide) -> 2x astar(..., ag_hide)             [2404]
      if swap feasible: assign_tpts(displaced_agent, depth+1)  <-- RECURSIVE
      rollback on failure
```

### 3. CENTRAL-ECBS  — Hungarian (every step) + CBS/ECBS
**CLI:** `-a CENTRAL` · **preset:** `HUNGARIAN, EVERY_TIMESTEP, MAPF_CBS, SA_STA_TASK_EP, HOLDING_ENDPOINT`
```
task_assignment(): AM_HUNGARIAN -> assign_hungarian()                          [1411]
  collect FREE agents + candidate tasks + parking endpoints; cost matrix; dlib::max_cost_assignment()
should_replan() -> central_has_event_
path_planning() -> mapf==CBS -> path_planning_cbs()                            [1586]
  GROUP 1 (CARRYING deliveries): CBSSearch over all delivery agents; astar() fallback
  GROUP 2 (FREE -> pickup/parking): CBSSearch (CBS HL + ECBS LL, cbs.cpp); astar/PP fallback
  (CBS HL expansion cap 200 with A*-fallback — see cbs.cpp)
```

### 4. TA-Hybrid-STA*  — offline LKH3 (reassign) + two-group planning (Group 1 = ICBS)
**CLI:** `-a TA_HYBRID --tour <N>-500.tour` · **preset:** `LKH3_TSP_REASSIGN, ONCE, TA_HYBRID_TWO_GROUP, SA_STA_TASK_EP, DUMMY_PATH`
```
task_assignment() -> AM_LKH3_TSP_REASSIGN -> assign_ta_tsp()                   [2649]
  parse LKH3 tour file, split into per-agent task_sequences
path_planning() -> MAPF_TA_HYBRID_TWO_GROUP -> plan_ta_hybrid()               [3992]
  Group 1 (deliveries): faithful multi-agent ICBS -> hybrid_group1_plan()     [3752]
      high-level CBS over the delivery batch; low-level = two-phase deliver->park
      astar_with_dummy() [3124] extended to honor CBS (loc,time)/edge constraints
      (replaced the old fixed-priority single-agent search; fixes the 40/500 deadlock)
  Group 2 (route to pickups): CostFlow (min-cost max-flow)
  hybrid_replan_dummy() on events
end(): ta_planning_done_
```

### 5. TA-Prioritized-STA*  — offline LKH3 + prioritized planning
**CLI:** `-a TA_PRIORITIZED --tour <N>-500.tour` · **preset:** `LKH3_TSP, ONCE, DECOUPLED_PP, SA_SEQ_STA, DUMMY_PATH`
```
task_assignment() -> AM_LKH3_TSP -> assign_ta_tsp()                            [2649]
path_planning() -> DECOUPLED_PP & SA_SEQ_STA -> plan_ta_prioritized()          [2752]
  sort agents by descending makespan; per agent/task: astar_with_dummy(->pickup,park) then (->deliv,park) [3124]
```

### 6. Hungarian+PBS-MLA*  — repeated Hungarian + PBS, MLA* low-level
**CLI:** `-a HUNGARIAN_PBS` · **preset:** `HUNGARIAN, ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT, MAPF_PBS, SA_MLA_SEQUENCE, DUMMY_PATH`
```
init: agent-start shuffle (0.2)
task_assignment() -> AM_HUNGARIAN (trigger UNASSIGNED_OR_FREE):
  assign_repeated_hungarian()                                                  [4459]
    while remaining tasks: cost = -(estimated arrival); max_cost_assignment; append to task_sequence
path_planning() -> path_planning_pbs() [7755] -> pbs_core(false)               [6467]
  build_goal_sequences() [4987] (goals=[p,d,...,dummy]; choose_dummy_endpoint [4923])
  ROOT: plan every agent via plan_agent -> mla_star_taskwise() [6363]  (MLA*)
  detect conflicts; DFS with priority constraints + nogood pruning; find_consistent_paths cascade
  commit best node (max_hl = 5000 if >30 agents else 50000)
update_system(): AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT -> update_system_pbs() [4357]
```

### 7. Hungarian+wPBS-MLA*  — repeated Hungarian + windowed PBS, MLA*  (framework-native)
**CLI:** `-a HUNGARIAN_wPBS` · **preset:** as #6 but `MAPF_wPBS`, `replan_window` (=10, the reference plan_window)
```
init: agent-start shuffle (0.2)
task_assignment() -> assign_repeated_hungarian()                               [4459]
path_planning() -> path_planning_wpbs() [8799] -> use_native TRUE -> wpbs_windowed_solve() [8835]
  dispersal: choose_good_endpoint (task endpoints only, skip own, home fallback)
  native_wpbs_solve() [8741]: WPBS [8329] high level + WStateTimeAStar [8161] focal low level
    + WReservationTable [7926], backed by mapd_map (see 0.4)
```
This is the in-framework re-expression of the MGMAPD reference windowed solve (formerly the
removed `ref_solve.cpp`). Matches the reference within +3% on the full grid (many cells beat it).

### 8. LNS(1s)+PBS-MLA*  — repeated Hungarian + LNS assignment + PBS, MLA*
**CLI:** `-a LNS_PBS --lns_time 1` · **preset:** `REPEATED_HUNGARIAN_LNS, ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT, MAPF_PBS, SA_MLA_SEQUENCE, DUMMY_PATH, lns_time_limit=1`
```
init: agent-start shuffle (0.2)
task_assignment() -> assign_repeated_hungarian_lns()                           [4838]
  Phase 1: assign_repeated_hungarian() [4459]
  Phase 2: LNS (while elapsed < 1s): destroy (RANDOM/WORST/RELATED) -> regret repair -> accept if cheaper
     (LNS reseeds RNG with time(NULL) -> results vary run-to-run)
path_planning() -> path_planning_pbs() -> pbs_core(false) [6467]   -- MLA* (mla_star_taskwise)
```

### 9. LNS(1s)+wPBS-MLA*  — LNS assignment + windowed PBS, MLA*  (framework-native)
**CLI:** `-a LNS_wPBS --lns_time 1` · **preset:** as #8 but `MAPF_wPBS`
```
init: agent-start shuffle (0.2)
task_assignment() -> assign_repeated_hungarian_lns() [4838]
path_planning() -> path_planning_wpbs() -> use_native TRUE -> wpbs_windowed_solve() [8835]  (as #7)
```
The LNS-optimized assignment feeds the native windowed solve. Matches reference within +3%
(one boundary cell, 30/500 makespan, oscillates around +3% due to LNS's time(NULL) reseed).

### 10. Hungarian+PBS-MLSIPP  — repeated Hungarian + PBS, SIPP low-level
**CLI:** `-a HUNGARIAN_PBS --sipp` · **preset:** #6 preset **+ `use_sipp=true`**
Identical control flow to #6 (incl. agent-shuffle), but in `pbs_core(false)` the `plan_agent`
lambda takes the `config.use_sipp` branch -> **`sipp_search()`** [5750] for root and every cascade
replan. Root SIPP old-path constraints precompressed into CT ranges.

### 11. Hungarian+wPBS-MLSIPP  — repeated Hungarian + windowed PBS, SIPP  (NOT native)
**CLI:** `-a HUNGARIAN_wPBS --sipp` · **preset:** #7 preset **+ `use_sipp=true`**
`use_sipp` ⇒ `use_native=FALSE`, so `path_planning_wpbs()` -> **`pbs_core(true)`** (windowed) with
`plan_agent -> sipp_search()`. Windowed SIPP caps at `start + SIPP_WIN_LOOK`(=16). Agent-shuffle applies.
(Still on baseline quality — a candidate for its own native/integrated port.)

### 12. LNS(1s)+PBS-MLSIPP  — LNS assignment + PBS, SIPP
**CLI:** `-a LNS_PBS --lns_time 1 --sipp` · **preset:** #8 preset **+ `use_sipp=true`**
LNS assignment (#8) + `pbs_core(false)` with `plan_agent -> sipp_search()`. Agent-shuffle applies.

### 13. LNS(1s)+wPBS-MLSIPP  — LNS assignment + windowed PBS, SIPP  (NOT native)
**CLI:** `-a LNS_wPBS --lns_time 1 --sipp` · **preset:** #9 preset **+ `use_sipp=true`**
`use_sipp` ⇒ `use_native=FALSE` -> windowed `pbs_core(true)` with `plan_agent -> sipp_search()`
(windowed SIPP cap as #11). Agent-shuffle applies.

### 14. TP-SIPP  — Token Passing, SIPP low-level
**CLI:** `-a TP --single_agent MLA --sipp` · **preset:** TP preset (single_agent overridden to MLA) **+ `use_sipp=true`**
```
task_assignment() -> AM_DECOUPLED_GREEDY (as #1) -> assign_decoupled_greedy() [985]
  plan_task_token(ag, task): config.use_sipp -> multi-goal sipp_search({pickup,delivery}) [2404/5750]
    validate vs token.path; check delivery can_hold; NO fallback to MLA*/STA* (pure SIPP)
```
(`--single_agent MLA` is inert once `--sipp` wins the dispatch.)

### 15. TPTS-SIPP  — Token Passing w/ swaps, SIPP low-level
**CLI:** `-a TPTS --single_agent MLA --sipp` · **preset:** TPTS preset **+ `use_sipp=true`**
```
task_assignment() -> AM_DECOUPLED_GREEDY_SWAPS (as #2) -> assign_tpts() [1057]
  plan_task_token(ag, task, ag_hide): use_sipp -> sipp_search({pickup,delivery}) skipping ag.id & ag_hide
```

### 16. Hungarian+PP-SIPP  — repeated Hungarian + Prioritized Planning, SIPP
**CLI:** `-a HUNGARIAN_PBS --mapf PP --sipp` · **preset:** #6 preset, `mapf`→`MAPF_DECOUPLED_PP` **+ `use_sipp=true`**
```
(NO agent-shuffle: mapf==PP is outside the shuffle gate)
task_assignment() -> assign_repeated_hungarian() [4459]
path_planning(): AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT & mapf==PP -> path_planning_pp_mla() [7625]
  order: active agents first, idle second; for each: cons=others' paths;
  goals=[pickup,delivery] (task_truncated_size=1) + dummy; use_sipp -> sipp_search(goals,cons) [5750]
```
No PBS tree — single sweep of prioritized SIPP searches (weaker than PBS on quality).

### 17. LNS(1s)+PP-SIPP  — LNS assignment + Prioritized Planning, SIPP
**CLI:** `-a LNS_PBS --mapf PP --lns_time 1 --sipp` · **preset:** #8 preset, `mapf`→`MAPF_DECOUPLED_PP` **+ `use_sipp=true`**
```
(NO agent-shuffle: mapf==PP)
task_assignment() -> assign_repeated_hungarian_lns() [4838]  (as #8)
path_planning() -> path_planning_pp_mla() [7625]  (as #16, SIPP low-level)
```

---

## Shared components (current line numbers)

| Component | Line | Used by |
|---|---|---|
| `assign_decoupled_greedy` | 985 | TP (#1) |
| `assign_tpts` (recursive swaps) | 1057 | TPTS (#2) |
| `assign_hungarian` (CENTRAL, per-step) | 1411 | CENTRAL (#3) |
| `path_planning_cbs` | 1586 | CENTRAL (#3) |
| `astar` (space-time A*) | 2176 | TP/TPTS (STA), CENTRAL, TA groups |
| `token_mla_star` | 2279 | TP/TPTS when `--single_agent MLA` & no `--sipp` |
| `plan_task_token` (dispatch) | 2404 | TP, TPTS |
| `move2EP` (BFS to endpoint) | 2484 | TP, TPTS |
| `assign_ta_tsp` (parse LKH tour) | 2649 | TA-Prioritized, TA-Hybrid |
| `plan_ta_prioritized` | 2752 | TA-Prioritized (#5) |
| `astar_with_dummy` (two-phase, CBS-aware) | 3124 | TA-Prioritized, TA-Hybrid Group-1 ICBS |
| `hybrid_group1_plan` (Group-1 ICBS) | 3752 | TA-Hybrid (#4) |
| `plan_ta_hybrid` (ICBS G1 + CostFlow G2) | 3992 | TA-Hybrid (#4) |
| `update_system_pbs` | 4357 | Hungarian/LNS families |
| `assign_repeated_hungarian` | 4459 | #6,#7,#10,#11,#16 + Phase 1 of LNS |
| `assign_repeated_hungarian_lns` | 4838 | #8,#9,#12,#13,#17 |
| `choose_dummy_endpoint` | 4923 | PBS/PP goal building |
| `build_goal_sequences` | 4987 | pbs_core |
| `seq_mla_star` (single-goal MLA*) | 5137 | seq/taskwise low-level |
| `sipp_search` (SIPP) | 5750 | ALL `--sipp` methods (#10–#17) |
| `mla_star_taskwise` (task-by-task MLA*) | 6363 | `-MLA*` PBS methods (#6, #8; #10/#12 use SIPP) |
| `pbs_core(windowed)` | 6467 | #6,#8,#10,#12 (PBS) and #11,#13 (SIPP wPBS) |
| `pbs_core_sipp` | 7168 | **dead code** (see note) |
| `path_planning_pp_mla` | 7625 | #16, #17 (PP) |
| `path_planning_pbs` | 7755 | PBS wrapper -> pbs_core(false) |
| **native windowed PBS** (`WState`/`WReservationTable`/`WStateTimeAStar`/`WPBS`) | 7798–8460 | #7, #9 (via native path) |
| `path_planning_wpbs` (native vs pbs_core gate) | 8799 | #7,#9 (native) ; #11,#13 (pbs_core(true)) |
| `native_wpbs_solve` (entry) | 8741 | #7, #9 |
| `wpbs_windowed_solve` (adapter) | 8835 | #7, #9 |
| CBS / ECBS | cbs.cpp | CENTRAL-ECBS (#3) |

> Note: `pbs_core_sipp()` [7168] is **dead code** — `path_planning_pbs/_wpbs` call `pbs_core(false/true)`
> (which delegates to `sipp_search` via `plan_agent` when `--sipp` is set), or the native windowed
> solver. The former `ref_solve.cpp`/`namespace refsolve` module has been **deleted**.

---

## Flag → config → code dispatch summary

| # | Label | -a preset | extra flags | assign | high-level MAPF | low-level | shuffle? |
|---|-------|-----------|-------------|--------|-----------------|-----------|:---:|
| 1 | TP-STA* | TP | — | decoupled greedy [985] | (in-assign PP) | 2× astar [2176] | — |
| 2 | TPTS-STA* | TPTS | — | greedy+swaps [1057] | (in-assign PP) | 2× astar | — |
| 3 | CENTRAL-ECBS | CENTRAL | — | hungarian [1411] | CBS [1586] | ECBS (cbs.cpp) | — |
| 4 | TA-Hybrid-STA* | TA_HYBRID | --tour | LKH3 reassign [2649] | two-group [3992] | Group-1 **ICBS** [3752] + CostFlow | — |
| 5 | TA-Prioritized-STA* | TA_PRIORITIZED | --tour | LKH3 [2649] | PP [2752] | astar_with_dummy [3124] | — |
| 6 | Hungarian+PBS-MLA* | HUNGARIAN_PBS | — | rep. hungarian [4459] | PBS [6467] | mla_star_taskwise [6363] | ✓ |
| 7 | Hungarian+wPBS-MLA* | HUNGARIAN_wPBS | — | rep. hungarian | **native wPBS** [8741] | WStateTimeAStar [8161] | ✓ |
| 8 | LNS(1s)+PBS-MLA* | LNS_PBS | --lns_time 1 | rep. hung.+LNS [4838] | PBS | mla_star_taskwise | ✓ |
| 9 | LNS(1s)+wPBS-MLA* | LNS_wPBS | --lns_time 1 | rep. hung.+LNS | **native wPBS** [8741] | WStateTimeAStar | ✓ |
| 10 | Hungarian+PBS-MLSIPP | HUNGARIAN_PBS | --sipp | rep. hungarian | PBS | sipp_search [5750] | ✓ |
| 11 | Hungarian+wPBS-MLSIPP | HUNGARIAN_wPBS | --sipp | rep. hungarian | wPBS pbs_core(true) | sipp_search (win) | ✓ |
| 12 | LNS(1s)+PBS-MLSIPP | LNS_PBS | --lns_time 1 --sipp | rep. hung.+LNS | PBS | sipp_search | ✓ |
| 13 | LNS(1s)+wPBS-MLSIPP | LNS_wPBS | --lns_time 1 --sipp | rep. hung.+LNS | wPBS pbs_core(true) | sipp_search (win) | ✓ |
| 14 | TP-SIPP | TP | --single_agent MLA --sipp | decoupled greedy | (in-assign PP) | sipp_search [2404] | — |
| 15 | TPTS-SIPP | TPTS | --single_agent MLA --sipp | greedy+swaps | (in-assign PP) | sipp_search | — |
| 16 | Hungarian+PP-SIPP | HUNGARIAN_PBS | --mapf PP --sipp | rep. hungarian | PP [7625] | sipp_search | — |
| 17 | LNS(1s)+PP-SIPP | LNS_PBS | --mapf PP --lns_time 1 --sipp | rep. hung.+LNS | PP [7625] | sipp_search | — |

**CLI switches recap**
- `--single_agent STA|MLA` — TP/TPTS low-level (inert when `--sipp` set).
- `--mapf CBS|PBS|wPBS|PP` — overrides preset MAPF (used by #16/#17 to force PP).
- `--sipp` — replaces MLA*/STA* with `sipp_search` everywhere (#10–#17); also disables the native wPBS path (#11/#13 fall back to `pbs_core(true)`).
- `--lns_time <s>` — LNS assignment budget (default 1s; #8,#9,#12,#13,#17).
- `--tour <file>` — LKH3 tour input for the offline TA methods (#4,#5).
- `REF_SOLVE=1|0` (env) — force the native windowed solver on/off (debug; default on for #7/#9).
- `--save_output` — dump paths / task completions / runtime to `./output/`.
