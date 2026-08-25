# Algorithm Implementation Traces — MAPD Reimplementation Methods

This document traces the code paths of the supported methods
(the labels used in `all_results.xlsx`). Every label is one `-a <ALGO>` preset plus a
combination of CLI flags (`--single_agent`, `--mapf`, `--lns_time`, `--tour`).
The flags per label are defined in `skill_tools/run_method.sh`; the preset → config
mapping is `inc/config.h` `get_preset()`.

Line numbers refer to `src/simulation.cpp` unless noted (current as of commit `cc0d38d`).
Regenerate with the `/match-reference-results` debugging step.

> **Recent changes reflected here**
> - **Known agent-start difference** (`init` [~40]): the released MGMAPD loader shuffles
>   agent homes with `std::default_random_engine()`, while the current framework keeps map-scan
>   order. This can change PBS priority (= agent index) and order-dependent parking.
> - **TA-Hybrid Group 1 now uses conflict-based search** rather than the old
>   fixed-priority search; it preserves the paper's joint-planning structure
>   without claiming the reference ICBS conflict-classification optimizations.
> - **Windowed-PBS for both MLA\* and MLSIPP methods is framework-native
>   solver** (`wpbs_windowed_solve` [8835] → `native_wpbs_solve` [8741], classes
>   `WStateTimeAStar`/`WReservationTable`/`WPBS` in `simulation.cpp`). The former bolt-on
>   `ref_solve.cpp`/`refsolve` module has been **removed**.

---

## 0. Entry point & top-level dispatch

```
driver.cpp:main()
  set_parameters(config, vm)
    config = get_preset(algo)                              [config.h get_preset]  -- preset per -a
    apply --lns_time / --single_agent / --mapf             [driver.cpp]
  sim.init(map, task, config, tour)                        [24]
    -- MGMAPD agent-home shuffle (gated, see 0.2)          [~40]
  sim.run()                                                [123]
  (optional) sim.realpath_lns_imp(rounds, group)                          -- only with --lns_imp
  sim.fullCollisionCheck(algorithm_name)                   [driver.cpp]
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
| `AT_EVERY_TIMESTEP` (CENTRAL) | every discrete simulation timestep | every timestep: newly occupied agents first, then all non-carrying agents |
| `AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` (Hungarian/LNS) | a new task is revealed, a strict-PBS task is deferred, or a task agent finishes its retained prefix | replan event |
| `AT_ONCE` (TA) | `timestep == 0` | true |

### `task_assignment()` dispatch — `switch(config.assign_method)` [~857]
```
AM_DECOUPLED_GREEDY (TP)        -> assign_decoupled_greedy()   [985]
AM_DECOUPLED_GREEDY_SWAPS (TPTS)-> assign_tpts()               [1057]
AM_CENTRAL_HUNGARIAN            -> assign_central_hungarian()
AM_REPEATED_HUNGARIAN           -> assign_repeated_hungarian() (Hungarian+PBS/wPBS/PP)
AM_REPEATED_HUNGARIAN_LNS       -> assign_repeated_hungarian_lns() [4838]
AM_LKH3_TSP (TA-Prioritized)     -> assign_ta_tsp()
AM_LKH3_TSP_REASSIGN (TA-Hybrid) -> assign_ta_hybrid()
```

### `path_planning()` dispatch [933]
```
if (AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE) && mapf==PP -> path_planning_pp_mla() [7625]
switch mapf:
  MAPF_PP_PER_TASK       -> path_planning_pp_per_task()
  MAPF_PP_TASK_SEQUENCE  -> path_planning_pp_task_sequence()
  MAPF_CBS               -> path_planning_ecbs()
  MAPF_PBS          -> path_planning_pbs()          [7755] -> pbs_core(false) [6467]
  MAPF_wPBS         -> path_planning_wpbs()         [8799] -> native (see 0.3) or pbs_core(true)
  MAPF_TA_HYBRID_TWO_GROUP -> path_planning_ta_hybrid()
```

---

## 0.2 Agent-start ordering

The released MGMAPD loader collects home (`r`) cells in map-scan order and then calls
`shuffle(agent_home_locations.begin(), agent_home_locations.end(),
std::default_random_engine())`. The current framework uses `mapd_map.agent_starts` directly
in map-scan order and does not shuffle it. This is a remaining implementation difference for
the PBS/wPBS rows because agent index affects PBS priority and order-dependent dummy selection.

## 0.3 Low-level solver dispatch (`--single_agent`)

**A. MLA*/MLSIPP selection inside PBS and PP-per-task:**
```
single_agent==SA_MLSIPP_SEQUENCE -> SIPPPlanner
single_agent==SA_MLA_SEQUENCE    -> MLAStarPlanner
```
`--single_agent` is the only low-level planner-selection option. Use
`--single_agent MLSIPP` to select MLSIPP; the duplicate `use_sipp` config field
and `--sipp` shortcut were removed.

**B. `plan_token_task` (token methods TP / TPTS):**
```
if single_agent==SA_MLSIPP_SEQUENCE -> one MLSIPP search through Task::goals
else (SA_STA_TASK_EP)                -> sequential STA* through Task::goals
```

**`sipp_search` [5750]** — Safe-Interval Path Planning over multi-goal sequences; O(1) endpoint
heuristics; in windowed (wPBS) mode caps at `start+win_look` (env `SIPP_WIN_LOOK`, default 16).

## 0.4 Framework-native windowed PBS (the `ref_solve` replacement) [7798–8460, 8835]

`path_planning_wpbs()` builds the goal sequences and invokes the native windowed solver:
```
native_wpbs_solve(...,
    use_mlsipp = (config.single_agent == SA_MLSIPP_SEQUENCE))
```
`wpbs_windowed_solve()` [8835] builds per-solve inputs from live state, applies the reference
dispersal (`choose_good_endpoint`: task endpoints only, skip own, home fallback), calls
`native_wpbs_solve()` [8741], and writes committed paths back to `token.path`/`agents.path`.
Native solver components (anonymous namespace in `simulation.cpp`, backed by `mapd_map`):
- `WStateTimeAStar` [8161] — dual fibonacci-heap **focal** low level (open by f, focal by
  secondary key), cutoff at `start+window`, ordered multi-goal sequence.
- `WReservationTable` [7926] — hard constraints from the higher-priority (reachable) set only
  (no soft CAT), window + permanent endpoint hold.
- `WPBS` [8329] / `WPBSNode` [8303] — DFS high level, sequential-prioritized root,
  `find_consistent_paths` child cascade.
This is a faithful in-framework re-expression of the MGMAPD reference solve (the old separate
`ref_solve.cpp`/`namespace refsolve` module is deleted).

## 0.5 Dummy paths and endpoint selection: paper vs released code vs framework

The paper defines a dummy endpoint as “an endpoint that [an agent] can move to
and stay indefinitely at without collisions” and defines the goal sequence as
all task goals “plus its dummy endpoint at the end” (Section IV, PDF p.3).
Thus both LNS-PBS and LNS-wPBS have a dummy path; wPBS merely drops the
completeness-oriented restrictions on that endpoint and on the old paths.

For PBS, Section IV-B (PDF p.4) states that task agents choose before free
agents. A new dummy must differ from already selected new dummies, every goal
of every unfinished task, and the old dummies of the *other* M-1 agents. The
algorithm considers task endpoints by increasing shortest-path distance and
uses the agent's start location only when no task endpoint is available.

For wPBS, Section IV-E (PDF p.5) states that it does not defer tasks, does not
use the modified PBS low level that considers old paths, and requires dummy
endpoints only to be pairwise different; they need not avoid unfinished-task
goals or old dummies.

| Reimplementation rows | Paper behavior | Authors' released implementation | Current framework | Status |
|---|---|---|---|---|
| PBS 6, 8, 10, 12 | Dummy path enabled. Busy agents first. Choose the closest available **task endpoint**; avoid new dummies, all unfinished-task goals, and the other agents' old dummies; home is fallback only. | Same basic procedure, but its shared `current_assigned_endpoints` also excludes the selecting agent's own old dummy. It searches only task endpoints and falls back to that agent's home. | `dummy_path=true`, `NEAREST_WITH_STRICT_EXCLUSIONS`. Busy agents first; avoids new dummies, unfinished-task goals, and other agents' old path tails; allows its own old dummy. However, task and home endpoints compete in the same nearest-endpoint search. | Matches the paper except that home endpoints are candidates instead of fallback-only. It matches the paper better than the released code regarding the agent's own old dummy. |
| wPBS 7, 9, 11, 13 | Dummy path remains enabled, but new dummies need only be pairwise different. No task deferral and no avoidance of old paths/dummies. | Clears the old-endpoint set; selects pairwise-distinct task endpoints, skips the last goal itself, then searches home endpoints as fallback. | `dummy_path=true`, `PAIRWISE_TASK_THEN_HOME`; the shared `choose_dummy_endpoint()` implements the same task-endpoint-first, pairwise-distinct, skip-last-goal, home-fallback procedure. | Closely aligned with released code. Skipping the last goal is a released-code detail not explicitly required by the paper. |
| PP 16, 17 | No PP-SIPP variant is defined in this paper; these rows compare against the corresponding PBS assignment/dummy policy. | No direct original implementation for this combination. | Inherits `dummy_path=true` and `NEAREST_WITH_STRICT_EXCLUSIONS`, including PBS-style deferral and endpoint filtering, but uses prioritized SIPP instead of modified PBS. | Correctly transplants the PBS dummy policy structurally, with the same home-candidate mismatch as the PBS rows; it does not inherit the paper's PBS completeness guarantee. |

The Hungarian labels change only task assignment. The paper describes its
Hungarian ablation as using “our Hungarian-based insertion to find a task
assignment (but does not improve it via LNS)” (PDF p.7). Therefore Hungarian
and LNS variants with the same PBS/wPBS path planner share the same dummy-path
and endpoint-selection policy.

## 0.6 Current `MAPDConfig` audit against the paper and released implementation

`MAPDConfig` currently has **14** behavioral fields. The tables below
audit the Hungarian/LNS series (rows 6–13 and the derived PP rows 16–17).

### Section: `mode`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Hungarian/LNS presets remain `MODE_ONLINE`, with `--mode ONLINE`, `OFFLINE`, or `SEMI_ONLINE` available as an explicit override. Online reveals the current release batch, offline reveals every task at time zero, and semi-online reveals the configured number of future release batches. | Reproduction commands use `scenario=KIVAONLINE`; the source also supports offline `KIVA` and `look_ahead_horizon`. Its numeric convention uses `1` for no future look-ahead. | Online means tasks are unknown until release; offline means all tasks are known initially; semi-online knows a finite number of future batches. The paper's horizon `1` example knows the current and next batch. | All three modes are implemented. The current `semi_online_lookahead_batches` follows the paper's numeric convention directly; reference option `2` corresponds to current/paper value `1`. |

### Section: `semi_online_lookahead_batches`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Used only in `MODE_SEMI_ONLINE`. Default `1`; this reveals the current batch plus one distinct future release batch. `0` is online-equivalent. The horizon advances at each actual release batch. | `look_ahead_horizon=1` means no future look-ahead; its loop includes that many batches total. | The horizon is “the number of batches that we know in advance”; horizon `1` knows the current and next batch. | Current naming and values follow the paper rather than the released code's off-by-one convention. |

### Section: `assign_method`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Hungarian rows use `AM_REPEATED_HUNGARIAN`; LNS rows use `AM_REPEATED_HUNGARIAN_LNS` (Hungarian initialization followed by Shaw removal and regret repair). | `lns_time=0` selects Hungarian-only behavior; positive `lns_time` runs LNS. | “LNS starts with an initial task assignment generated by Hungarian-based insertion” and iteratively improves it; the Hungarian ablation omits the LNS improvement. | High-level method matches. Current Shaw weights match the paper, while the released source uses asymmetric 9/1 spatial weights. Current also has a configurable no-improvement early stop not stated in the paper/reference. |

### Section: `assign_trigger`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| `AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` uses an explicit event flag rather than `open_tasks_` occupancy. Truncated suffixes are returned to `open_tasks_` immediately but do not trigger assignment by themselves. Assignment runs when a task is newly revealed, strict PBS defers a task, or an agent finishes its complete materialized prefix and becomes `AG_FREE`. | Triggers on a release period, `new_agent_finish`, and (PBS only) `deferred_task`. Finishing the currently materialized truncated goal sequence causes a new assignment iteration. | Algorithm 1: “if there are new or deferred tasks or any task agent becomes a free agent.” | Aligned. `open_tasks_` is the complete unassigned pool, while trigger state is represented separately. |

### Section: `mapf`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| PBS rows use `MAPF_PBS`; wPBS rows use `MAPF_wPBS`; PP rows use `MAPF_PP_PER_TASK`. | Both reference directories instantiate `PBS`; wPBS is obtained with a finite planning window. | LNS-PBS uses modified PBS with old-path avoidance; LNS-wPBS uses windowed PBS and drops old-path avoidance. | PBS/wPBS align conceptually. PP-SIPP is a deliberate derived comparison with no direct paper/reference counterpart and no PBS completeness guarantee. |

### Section: `single_agent`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| `SA_MLA_SEQUENCE` is the default for PBS/wPBS; `SA_MLSIPP_SEQUENCE` is selectable for the MLSIPP variants. PP variants use MLSIPP. | Reproduction commands use the default `single_agent_solver=ASTAR`, implemented by `StateTimeAStar`; `SIPP` exists as an optional implementation setting. | The paper describes the generalized MLA* low level for sequences of goal locations. | MLA* rows are conceptually aligned, though the implementation is rewritten. MLSIPP and PP-SIPP rows are explicit extensions rather than identical paper methods. |

### Section: `dummy_path`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| `true` for every Hungarian/LNS PBS, wPBS and PP preset. The selected dummy is appended to the goal sequence and held at the path tail. | `KivaSystemOnline` appends dummy goals. The full driver exposes `dummy_paths`; its default is false, while `driver_simple` sets it true, so the released entrypoints are inconsistent. | Every agent maintains a dummy endpoint, and its goal sequence contains all task goals “plus its dummy endpoint at the end.” | Current setting matches the algorithm described by the paper. PP inherits it as an extension. |

### Section: `seed`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Default `0`; LNS calls `srand(config.seed)` on each assignment run. Negative values select `time(NULL)`. Agent-home ordering is not controlled by this field. | Driver accepts `--seed` and calls `srand(seed)`, but `LNS::run()` calls `srand(time(NULL))` again. Agent homes are separately shuffled with an unseeded default engine. | Shaw removal chooses a task randomly; the paper does not specify seed semantics. Reproduction commands pass `--seed=0`. | Current LNS is more reproducible but does not reproduce the released LNS reseeding. Current also lacks the released deterministic agent-home shuffle. |

### Section: `endpoint_strategy`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| PBS/PP use `NEAREST_WITH_STRICT_EXCLUSIONS`; wPBS uses `PAIRWISE_TASK_THEN_HOME`. | PBS maintains old endpoints and unfinished-task goals; wPBS clears them and keeps only newly selected pairwise endpoints. | PBS avoids unfinished-task goals, new dummies and other agents' old dummies. wPBS requires only pairwise-distinct new dummies. | Constraint sets are aligned. Remaining PBS mismatch: current searches task endpoints and all homes together; paper/reference search task endpoints first and use home as fallback. Released PBS also forbids the selecting agent's own old dummy, unlike the paper/current. |

### Section: `task_sequence_limit`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Default `2`; after Hungarian/LNS optimization, PBS/wPBS retain only the first two tasks per agent. Removed suffix tasks become unassigned and return immediately to `open_tasks_`, but are globally reconsidered only at the next new-task/deferred/free-agent event. PP-per-task retains one task because its low level plans one task per call. | Driver default is `1`, but reproduction commands pass `task_truncated_size=2`. Goal construction materializes at most `C` tasks and a later assignment iteration reconstructs task sequences. | “We truncate the task sequence of each agent to a size of at most ... `C`”; experiments use `C=2`. “The remaining tasks are deleted from the task sequences and will be assigned in future iterations.” | PBS/wPBS lifecycle and numeric value now align. PP-per-task is a derived extension and intentionally uses a one-task prefix. |

### Section: `wpbs_replan_window`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| The config default, `HUNGARIAN_wPBS` preset and `LNS_wPBS` preset all use `10`. The CLI overrides it only when `--wpbs_replan_window` is explicitly supplied. | Reproduction commands pass `planning_window=10`; wPBS replans when `timestep == last_plan_timestep + planning_window`. | Experiments set the wPBS window to `w=10` timesteps. | Aligned for both CLI and direct `get_preset()` use. |

### Section: `lns_time_limit`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| Default `1` second for LNS variants, measured with CPU clock. A separate configurable no-improvement limit may stop the search sooner. | `lns_time` defaults to `1`; `LNS::run()` loops until its CPU-clock limit. `0` selects Hungarian-only behavior. | Experiments use a one-second LNS runtime limit. | Nominal value matches. Set `lns_no_improvement_limit=0` to disable the extra early stop and rely on the time budget. |

### Section: `lns_no_improvement_limit`

| Current framework | Released implementation | Paper | Difference/status |
|---|---|---|---|
| LNS-specific field and CLI option. Default `2000`; stops after that many consecutive rejected moves. An accepted improvement resets the counter. `0` disables this condition. | No equivalent iteration limit; the LNS loop normally runs until its CPU-time limit. | No no-improvement iteration limit is specified. | This remains an optional framework optimization, but it is no longer hardcoded and can be disabled for closer reference behavior. |

### Section: algorithm-specific field applicability

| Field | Algorithms that consume it |
|---|---|
| `task_sequence_limit` | PBS and wPBS goal construction; PP-per-task intentionally remains one task |
| `wpbs_replan_window` | wPBS only |
| `lns_time_limit` | LNS assignment only; ignored by Hungarian-only rows |
| `lns_no_improvement_limit` | LNS assignment only; ignored by Hungarian-only rows; `0` disables it |
| `semi_online_lookahead_batches` | Semi-online mode only; `0` gives online-equivalent task visibility |

---

## 0.7 TP/TPTS audit against the 2017 MAPD paper

This section compares the TP and TPTS presets with the released implementation
in `reference_code/CENTRAL-TP-TPTS/COBRA` and with *Lifelong Multi-Agent Path
Finding for Online Pickup and Delivery Tasks*.

### Configuration and execution model

| Item | Our implementation | Released implementation | Paper writing | Status |
|---|---|---|---|---|
| `mode` | Both presets use `MODE_ONLINE`. The generic CLI can override the mode, but that produces an extension rather than the original TP/TPTS experiment. | `run_TOTP()` and `run_TPTR()` add tasks as their release times are reached. | MAPD is an “online setting” where “tasks can enter the system at any time.” | Default presets align. |
| `assign_method` | TP uses `AM_DECOUPLED_GREEDY`; TPTS uses `AM_DECOUPLED_GREEDY_SWAPS`. | `Agent::TOTP()` performs greedy assignment; `Agent::TPTR()` performs recursive task robbing/swapping. | TP is token-passing greedy assignment; TPTS is TP extended with task swaps. | Aligned. |
| `assign_trigger` | `AT_ON_FREE_WAITS`: process one agent with `finish_time <= cur_time_`; all ready agents are processed before advancing time. Ties use agent ID. | Selects the first agent finishing at the current token time, otherwise the earliest-finishing agent. | “Any agent that has reached the end of its path in the token requests the token once per timestep”; requests are handled one after another. | Operationally aligned. The current event-driven loop skips timesteps with no relevant event but preserves the same token order. |
| `mapf` | Preset says `MAPF_PP_PER_TASK`, but the normal MAPF dispatcher is not called. Planning occurs directly inside TP/TPTS assignment against the other committed paths. | Each token holder plans against paths already stored in the token. | TP/TPTS are decoupled; agents “plan their paths one after the other.” | Behavior aligns; the enum name is only an approximate structural label here. |
| `single_agent` | Default `SA_STA_TASK_EP`; `plan_token_task()` runs space-time A*. MLSIPP can be selected as an extension. | `Agent::AStar()` searches `(location,timestep)` states. | “Agent ai finds all paths via A* searches in a state space whose states are pairs of locations and timesteps.” | STA* aligns. TP-SIPP/TPTS-SIPP are extensions, not paper methods. |
| `dummy_path` | Set to `true`, but TP/TPTS do not use it to append a post-delivery leg. They always hold the last location of every committed path and use `choose_dummy_endpoint()` only when Path2 endpoint selection is needed. | Same behavior; no dummy-path switch. | “All MAPD algorithms in this paper ... assume that an agent rests ... forever in the last location of its path.” TP uses `Path2` only when it must vacate a blocking endpoint. | Runtime behavior aligns, but `dummy_path=true` is descriptive/inert for TP/TPTS and should not be interpreted as an appended post-delivery dummy path. |
| `endpoint_strategy` | TP/TPTS use `WAIT_OR_NEAREST_SAFE`. `choose_dummy_endpoint()` decides whether the agent can wait and otherwise returns the nearest reachable safe task/home endpoint. Once relocation is required, the returned endpoint must differ from the current location so a required detour cannot be mistaken for a one-step wait. For a multi-goal task, every post-pickup goal is protected like the original delivery goal. `plan_path2_to_endpoint()` separately constructs and commits the route. | `Move2EP()` combines endpoint selection and route construction in one BFS and protects the delivery location of each two-goal task. | `Path2` chooses “an endpoint” that is not a pending delivery and is not held by another path. The proof guarantees that a non-task endpoint exists but does not restrict the search to non-task endpoints. | Two-goal behavior aligns; the distinct relocation target is required because the unified implementation separates endpoint selection from path construction. Post-pickup-goal protection is the multi-goal extension. |
| Other config fields | `task_sequence_limit`, wPBS and LNS controls, and semi-online look-ahead are not read by default online TP/TPTS. `seed` does not affect their deterministic searches. | No corresponding TP/TPTS controls. | The paper's TP/TPTS algorithms assign one task at a time and do not use LNS, PBS windows, or task-sequence truncation. | Aligned. |

### TP task assignment

| Item | Our implementation | Released implementation | Paper writing | Status |
|---|---|---|---|---|
| Task set | `open_tasks_` contains unassigned tasks; TP removes a task immediately after assignment. | `token.tasks` contains unassigned tasks; `TOTP()` removes the selected task. | “Its task set contains all tasks that have no agents assigned to them.” | Aligned. |
| Eligible task filter | Rejects a task if another committed path ends at any location in its ordered `Task::goals`. | Rejects when the pickup or delivery is held. | `T'` contains tasks for which “no other path in token ends in sj or gj.” | Identical for paper-style two-goal tasks; generalized to every goal for MG-MAPD tasks. |
| Greedy selection | Chooses the eligible task with minimum precomputed endpoint distance from the agent's current location to pickup. Strict `<` retains the first task on a tie. | Same endpoint heuristic and strict comparison. | Select `arg min h(loc(ai), sj)`. | Aligned. |
| `Path1` | Sequential space-time A* searches through every ordered `Task::goals` location. `ag_arrive_start` records the first-goal arrival; task completion and `finish_time` use the final-goal arrival. If the 50,000-expansion safeguard is reached, non-wPBS runs throw an error rather than treating it as an ordinary failed candidate. | Two `AStar()` calls for pickup and delivery. | A cost-minimal collision-free path “from its current location via the pickup ... to the delivery location.” | Identical for two-goal tasks; arbitrary ordered goals are an extension beyond the 2017 implementation/paper. |
| No eligible task | Calls `choose_dummy_endpoint()` with `WAIT_OR_NEAREST_SAFE`. Returning the current location means wait; returning another location is passed to `plan_path2_to_endpoint()`. | Same decision and `Move2EP()` behavior in `TOTP()`. | Wait using `[loc(ai)]`, unless the agent blocks a pending delivery; then use `Path2`. | Aligned. |
| `Path2` | FIFO time-space search for the nearest endpoint that can be held indefinitely, is collision-free, and is not a post-pickup goal of an open task. If movement is required, the selector excludes the origin and `plan_path2_to_endpoint()` constructs the route to the returned endpoint. | `Move2EP()` implements selection and movement together for delivery goals. | Cost-minimal path to an endpoint different from pending delivery locations and other agents' terminal endpoints. | Identical for two-goal tasks; intermediate/final goals are protected for MG-MAPD tasks. Excluding the origin prevents loss of a necessary leave-and-return route at the selection/planning API boundary. |

### TPTS task swapping

| Item | Our implementation | Released implementation | Paper writing | Status |
|---|---|---|---|---|
| Task set lifecycle | Assigned tasks remain in `open_tasks_` while their owner travels to the first ordered goal and are purged at/after that goal. | `token.tasks` retains `TAKEN` tasks and removes them once token time reaches `ag_arrive_start` at pickup. | TPTS contains “all unexecuted tasks,” including an assigned task while its agent is still moving to pickup. | Identical for two-goal tasks; the first ordered goal is the pickup-equivalent for MG-MAPD. |
| Candidate order | Fibonacci heap ordered by static distance to pickup. | Same. | Considers tasks “in order of increasing h-values” to pickup. | Aligned. |
| Steal condition | Uses a static-distance lower-bound filter, then requires the new collision-free path to reach pickup strictly earlier than the current owner. | Same two tests. | A swap is useful only when the new agent reaches pickup “with fewer timesteps than” the assigned agent. | Aligned. |
| Reservation handling | While planning a steal, the displaced owner's old path is hidden so the stealing agent can take over that reservation. | `ag_hide` excludes the old owner from A* constraints. | Algorithm 2 removes the old owner's path from the token before calling `Path1`. | Aligned. |
| Recursive reassignment | Calls `assign_tpts()` recursively for the displaced agent; depth is guarded by the number of agents. | Recursively invokes `old_ag->TPTR(token)` without an explicit depth bound. | The stealing agent sends the token to the displaced agent, which calls `GetTask`. | Aligned in structure; current adds a finite safety guard. |
| Failed-swap rollback | Each candidate snapshots the requesting agent, its reservation row, and the candidate task's owner/arrival/completion fields. A failed low-level attempt or recursive reassignment restores that snapshot before the next candidate. Recursive calls provide the same transactional guarantee for every displaced agent in the chain. | Saves a shallow token copy and only the current agent; task objects are referenced by pointer and are not fully restored after each failed candidate. | “Restore token, task set, and agent assignments” after every unsuccessful swap attempt. | **Aligned with Algorithm 2.** This intentionally fixes a rollback omission retained by the released implementation. |
| Fallback after no task/swap | If safely waiting is impossible, calls the same endpoint-relocation search as TP. It additionally checks whether another future path crosses the current endpoint. | Same additional future-path check and `Move2EP()` fallback. | Uses `Path2`; when recursively called away from an endpoint, failure is permitted. | Closely aligned; the future-path occupancy check is a sensible implementation detail beyond the abbreviated pseudocode condition. |

### Completeness and remaining differences

| Item | Our implementation | Released implementation | Paper writing | Status |
|---|---|---|---|---|
| Terminal holding | Every successful STA* path is extended by holding its final endpoint through `maxtime`. | `updatePath()` does the same. | Every agent rests forever at the last location of its token path. | Aligned. |
| Search completeness | `sta_search()` throws a `runtime_error` after 50,000 expanded nodes for every non-wPBS configuration. wPBS retains the recoverable `-1` return because it is intentionally incomplete. | No fixed expansion-count cutoff; search is bounded by `maxtime`. | Properties 1–4 and Theorems 3/5 rely on `Path1`/`Path2` finding the guaranteed paths on well-formed instances. | The cap is still an implementation safeguard absent from the proof/reference, but a capped non-wPBS search can no longer silently continue and produce a misleading result. |
| Task model | TP/TPTS consume the complete ordered `Task::goals` vector with either sequential STA* or one MLSIPP search. | Two-location pickup/delivery tasks. | The 2017 paper defines one pickup and one delivery per task. | Two-goal inputs remain aligned. Arbitrary ordered goals are now supported as a framework extension beyond the paper/reference implementation. |

### Existing result comparison

The existing STA* validation files compare the current implementation with the
original/reference results. All cases completed and passed collision checking.

| Method | Agents | Frequency | Current makespan / SWT | Original makespan / SWT | Result |
|---|---:|---:|---:|---:|---|
| TP-STA* | 10 | 0.2 | 2532 / 19417 | 2532 / 19270 | Close; SWT +147 |
| TP-STA* | 10 | 0.5 | 1277 / 63595 | 1309 / 66393 | Different trajectory; both metrics lower |
| TP-STA* | 50 | 0.2 | 2540 / 19956 | 2540 / 20016 | Close; SWT -60 |
| TP-STA* | 50 | 0.5 | 1063 / 22100 | 1083 / 21832 | Makespan -20, SWT +268 |
| TPTS-STA* | 10 | 0.2 | 2532 / 14666 | 2532 / 14666 | Identical |
| TPTS-STA* | 10 | 0.5 | 1274 / 65574 | 1274 / 65574 | Identical |
| TPTS-STA* | 50 | 0.2 | 2524 / 11553 | 2524 / 11553 | Identical |
| TPTS-STA* | 50 | 0.5 | 1036 / 12609 | 1036 / 12609 | Identical |

Overall: the default TP/TPTS algorithm structure matches the released code and
paper closely. The STA* node expansion cap is now a fail-fast error for
non-wPBS runs. TPTS now restores each unsuccessful tentative swap before
trying another candidate, matching the paper even though the released code
retains shallow rollback behavior. TP/TPTS also accept arbitrary ordered
`Task::goals`; their first goal retains pickup/stealing semantics and their
final goal determines completion. `dummy_path` remains descriptive/inactive
for TP/TPTS; their endpoint policy is
now explicitly represented by `WAIT_OR_NEAREST_SAFE` in `choose_dummy_endpoint()`,
with route construction handled separately by `plan_path2_to_endpoint()`.

---

## The 17 methods

Each entry: **CLI** (from `run_method.sh`) → **preset** (`config.h`) → **trace**.

### 1. TP-STA*  — Token Passing, space-time A*
**CLI:** `-a TP` · **preset:** `DECOUPLED_GREEDY, ON_FREE_WAITS, PP_PER_TASK, SA_STA_TASK_EP, dummy_path=true, WAIT_OR_NEAREST_SAFE`
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
**CLI:** `-a TPTS` · **preset:** `DECOUPLED_GREEDY_SWAPS, ON_FREE_WAITS, PP_PER_TASK, SA_STA_TASK_EP, dummy_path=true, WAIT_OR_NEAREST_SAFE`
```
task_assignment() -> AM_DECOUPLED_GREEDY_SWAPS:
  pick ONE free agent, assign_tpts(ag, depth=0)                                [1057]
    save state; for it in token.tasks (by distance): ag_hide = displaced agent
      plan_task_token(ag, task, ag_hide) -> 2x astar(..., ag_hide)             [2404]
      if swap feasible: assign_tpts(displaced_agent, depth+1)  <-- RECURSIVE
      failed-branch state handling follows the released code; see audit §0.7
```

### 3. CENTRAL-CBS — per-timestep Hungarian assignment + CBS

**CLI:** `-a CENTRAL-CBS` · **preset:** `CENTRAL_HUNGARIAN, EVERY_TIMESTEP, CBS, SA_STA_TASK_EP, dummy_path=false, NEAREST_AVAILABLE`

```text
each timestep:
  task_assignment() -> AM_CENTRAL_HUNGARIAN
    central_phase1_instant_pickup()
      agents already at an eligible pickup become occupied
    assign_central_hungarian()
      construct T' by excluding conflicts across every ordered task goal
      choose pairwise-distinct parking endpoints through choose_dummy_endpoint()
      run Hungarian matching for every non-carrying agent

  path_planning() -> MAPF_CBS -> path_planning_ecbs()
    Group 1: CBS to the next ordered goal for agents becoming occupied
             or finishing an intermediate task segment this timestep
    Group 2: CBS for every non-carrying agent

  advance_time(): execute exactly one timestep and update pickup/delivery state
```

`ECBSPlanner` is the reusable planner class. CENTRAL passes
`config.ecbs_focal_weight`: its default `1.0` runs optimal CBS, while values
greater than `1.0` run bounded-suboptimal ECBS. The configurable
`config.cbs_high_level_expansion_limit` defaults to `INT_MAX` conflict-tree node
expansions per batch and is exposed as `--cbs_high_level_expansion_limit`.
Assignment
outputs are stored in existing per-agent fields (`current_task`, `status`, and
`last_endpoint`); no CENTRAL-specific persistent vectors or event flags exist.
The existing `Agent::current_goal_index`, shared with other multi-goal methods,
records the next unvisited goal. A task completes only at its final goal.

The implementation follows the paper where it is stricter than the released
code: a task enters `T'` only when both its pickup and delivery are distinct
from already reserved endpoints. The released source checks the pickup in its
active condition and leaves the delivery check commented out.

### 3b. CENTRAL-fixed-CBS — event-driven Hungarian assignment + CBS

**CLI:** `-a CENTRAL-fixed` · **preset:** `CENTRAL_HUNGARIAN, ON_NEW_TASK_OR_AGENT_BECOMES_FREE, CBS, SA_STA_TASK_EP, dummy_path=false, NEAREST_AVAILABLE`

CENTRAL-fixed shares CENTRAL's Hungarian assignment, unified endpoint chooser,
and `ECBSPlanner`. It differs only in scheduling: Hungarian assignment and the
free-agent CBS group run when a new task arrives or an occupied agent becomes
free; reaching a pickup or intermediate task goal runs only the occupied-agent
CBS group for the next segment. Goal events are derived from the existing
`Agent::current_goal_index` and path completion time, while the existing
`new_or_deferred_task_event_` and `new_available_agent_` flags carry assignment
events. No CENTRAL-fixed-specific persistent state is stored.

### 3c. HBH+MLA* — h-value centralized greedy + MLA*

**CLI:** `-a HBH_MLA` · **preset:** `CENTRALIZED_GREEDY, ON_FREE_WAITS, PP_PER_TASK, SA_MLA_SEQUENCE, dummy_path=true, WAIT_OR_NEAREST_FREE_NONTASK`

At each assignment step, `assign_hbh_mla()` gathers every available agent and
every known unassigned task, forms all agent-task pairs, and stable-sorts them
by shortest-path h-value to the task's first goal. It scans that list once. A
pair is committed only when the existing `MLAStarPlanner` finds a path through
all of the task's ordered goals while avoiding all other committed paths.

Remaining agents call the shared `choose_dummy_endpoint()` policy. They wait
when their current endpoint is safe; otherwise they move to the nearest
reachable non-task endpoint that is absent from other agents' future paths.
In semi-online mode, future task batches already revealed by
`release_tasks()` participate in the same pair list, and MLA* enforces the
first goal's actual release time.

HBH stores no method-specific persistent state. It reuses `Agent::current_task`,
`current_goal_index`, `finish_time`, `last_endpoint`, and the committed path
table. The original paper defines pickup/delivery pairs; traversal of arbitrary
`Task::goals` is the framework's multi-goal extension.

### 4. TA-Hybrid-STA*  — offline LKH3 (reassign) + two-group planning
**CLI:** `-a TA_HYBRID --tour <N>-500.tour` · **preset:** `LKH3_TSP_REASSIGN, ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE, TA_HYBRID_TWO_GROUP, SA_STA_TASK_EP, dummy_path=true, RETURN_TO_HOME`
```
task_assignment() -> AM_LKH3_TSP_REASSIGN -> assign_ta_hybrid()
  read the complete offline LKH3 tour at time zero
  on later free-agent events, move an unstarted suffix task when beneficial
path_planning() -> MAPF_TA_HYBRID_TWO_GROUP -> path_planning_ta_hybrid()
  Group 1: CBS over agents continuing an assigned task
  Group 2: local min-cost max-flow for free agents and next pickups
  low level: sequential STA* over every ordered task goal, then the endpoint
  returned by choose_dummy_endpoint(); RETURN_TO_HOME selects the agent's home
```

TA-Hybrid is offline-only. A `--mode ONLINE` or `--mode SEMI_ONLINE` override
is rejected because the LKH3 tour must cover the complete task set.

### 5. TA-Prioritized-STA* — LKH3 + paper-order prioritized planning
**CLI:** `-a TA_PRIORITIZED --tour <N>-500.tour` · **preset:** `LKH3_TSP, ONCE, PP_TASK_SEQUENCE, SA_SEQ_STA, dummy_path=true, RETURN_TO_HOME`

The method is offline-only. A `--mode ONLINE` or `--mode SEMI_ONLINE` override
is rejected because the LKH3 tour must cover the complete task set.
```
task_assignment() -> AM_LKH3_TSP -> assign_ta_tsp()
  parse the LKH3 tour locally; materialize the complete task set once
path_planning() -> MAPF_PP_TASK_SEQUENCE -> path_planning_pp_task_sequence()
  at each priority level:
    tentatively plan every remaining agent against higher-priority paths
    select the agent with the largest actual task-sequence completion time
  for every task: visit every ordered goal, constraining the first by release time
  every STA* goal test reserves a dummy path to the endpoint returned by
  choose_dummy_endpoint(); RETURN_TO_HOME selects the agent's own parking endpoint
```

### 6. Hungarian+PBS-MLA*  — repeated Hungarian + PBS, MLA* low-level
**CLI:** `-a HUNGARIAN_PBS` · **preset:** `REPEATED_HUNGARIAN, ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE, MAPF_PBS, SA_MLA_SEQUENCE, dummy_path=true, FLEXIBLE_STRICT`
```
init: agent-start shuffle (0.2)
task_assignment() -> AM_REPEATED_HUNGARIAN (paper event trigger):
  assignment trigger = new task OR deferred task retry OR task agent became free
  assign_repeated_hungarian()                                                  [4459]
    derive old dummy endpoints from committed path tails
    defer open tasks whose goals overlap those endpoints (FLEXIBLE_STRICT modes)
    while eligible tasks: cost = -(estimated arrival); max_cost_assignment; append to task_sequence
path_planning() -> path_planning_pbs() [7755] -> pbs_core(false)               [6467]
  replan without reassignment if a queued front task is not covered by the committed truncated plan
  build_goal_sequences() [4987] (all remaining goals from up to `task_sequence_limit`
  tasks, then the dummy endpoint; choose_dummy_endpoint [4923])
  ROOT: plan every agent via plan_agent -> mla_star_taskwise() [6363]  (MLA*)
  detect conflicts; DFS with priority constraints + nogood pruning; find_consistent_paths cascade
  commit best node (max_hl = 5000 if >30 agents else 50000)
update_system(): AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE -> update_system_pbs() [4357]
```

### 7. Hungarian+wPBS-MLA*  — repeated Hungarian + windowed PBS, MLA*  (framework-native)
**CLI:** `-a HUNGARIAN_wPBS` · **preset:** as #6 but `MAPF_wPBS`, `wpbs_replan_window` (=10 from the CLI default)
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
**CLI:** `-a LNS_PBS --lns_time 1` · **preset:** `REPEATED_HUNGARIAN_LNS, ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE, MAPF_PBS, SA_MLA_SEQUENCE, dummy_path=true, FLEXIBLE_STRICT, lns_time_limit=1`
```
init: agent-start shuffle (0.2)
task_assignment() -> assign_repeated_hungarian_lns()                           [4838]
  Phase 1: assign_repeated_hungarian() [4459]
  Phase 2: LNS (up to 1s): Shaw-related destroy -> regret repair -> accept if cheaper
    relatedness = 9*(first-goal distance + final-goal distance)
                  + 3*(first-goal-time difference + final-goal-time difference)
    RNG uses the framework-wide `seed` setting
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
**CLI:** `-a HUNGARIAN_PBS --single_agent MLSIPP`
Identical control flow to #6, but the `SA_MLSIPP_SEQUENCE` branch selects
**`sipp_search()`** for root and cascade replans.

### 11. Hungarian+wPBS-MLSIPP  — repeated Hungarian + native windowed PBS, MLSIPP
**CLI:** `-a HUNGARIAN_wPBS --single_agent MLSIPP`
`path_planning_wpbs()` calls `native_wpbs_solve()` with its MLSIPP low-level.

### 12. LNS(1s)+PBS-MLSIPP  — LNS assignment + PBS, SIPP
**CLI:** `-a LNS_PBS --lns_time 1 --single_agent MLSIPP`
LNS assignment (#8) + `pbs_core(false)` with `plan_agent -> sipp_search()`. Agent-shuffle applies.

### 13. LNS(1s)+wPBS-MLSIPP  — LNS assignment + native windowed PBS, MLSIPP
**CLI:** `-a LNS_wPBS --lns_time 1 --single_agent MLSIPP`
LNS assignment feeds `native_wpbs_solve()` with its MLSIPP low-level.

### 14. TP-SIPP  — Token Passing, SIPP low-level
**CLI:** `-a TP --single_agent MLSIPP`
```
task_assignment() -> AM_DECOUPLED_GREEDY (as #1) -> assign_decoupled_greedy() [985]
  plan_token_task(ag, task): SA_MLSIPP_SEQUENCE -> sipp_search(Task::goals)
    validate vs token.path; check final goal can_hold; NO fallback to STA* (pure SIPP)
```
### 15. TPTS-SIPP  — Token Passing w/ swaps, SIPP low-level
**CLI:** `-a TPTS --single_agent MLSIPP`
```
task_assignment() -> AM_DECOUPLED_GREEDY_SWAPS (as #2) -> assign_tpts() [1057]
  plan_token_task(ag, task, ag_hide): SA_MLSIPP_SEQUENCE -> sipp_search(Task::goals)
```

### 16. Hungarian+PP-SIPP  — repeated Hungarian + Prioritized Planning, SIPP
**CLI:** `-a HUNGARIAN_PBS --mapf PP --single_agent MLSIPP`
```
(NO agent-shuffle: mapf==PP is outside the shuffle gate)
task_assignment() -> assign_repeated_hungarian() [4459]
path_planning(): AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE & mapf==PP -> path_planning_pp_mla() [7625]
  order: active agents first, idle second; for each: cons=others' paths;
  goals=all remaining goals of the next task + dummy; MLSIPP -> sipp_search(goals,cons)
```
No PBS tree — single sweep of prioritized SIPP searches (weaker than PBS on quality).

### 17. LNS(1s)+PP-SIPP  — LNS assignment + Prioritized Planning, SIPP
**CLI:** `-a LNS_PBS --mapf PP --lns_time 1 --single_agent MLSIPP`
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
| `astar` (space-time A*) | 2176 | TP/TPTS (STA), TA groups |
| `token_mla_star` | 2279 | TP/TPTS when `--single_agent MLA` |
| `plan_task_token` (dispatch) | 2404 | TP, TPTS |
| `move2EP` (BFS to endpoint) | 2484 | TP, TPTS |
| `assign_ta_tsp` (parse LKH tour) | 2649 | TA-Prioritized, TA-Hybrid |
| `path_planning_pp_task_sequence` | 1658 | TA-Prioritized |
| `astar_with_dummy` (two-phase, CBS-aware) | 3124 | TA-Prioritized, TA-Hybrid Group-1 ICBS |
| `hybrid_group1_plan` (Group-1 ICBS) | 3752 | TA-Hybrid (#4) |
| `plan_ta_hybrid` (ICBS G1 + CostFlow G2) | 3992 | TA-Hybrid (#4) |
| `update_system_pbs` | 4357 | Hungarian/LNS families |
| `assign_repeated_hungarian` | 4459 | #6,#7,#10,#11,#16 + Phase 1 of LNS |
| `assign_repeated_hungarian_lns` | 4838 | #8,#9,#12,#13,#17 |
| `choose_dummy_endpoint` | 4923 | PBS/PP goal building |
| `build_goal_sequences` | 4987 | pbs_core |
| `seq_mla_star` (single-goal MLA*) | 5137 | seq/taskwise low-level |
| `sipp_search` (SIPP) | 5750 | Methods configured with `--single_agent MLSIPP` |
| `mla_star_taskwise` (task-by-task MLA*) | 6363 | `-MLA*` PBS methods (#6, #8; #10/#12 use SIPP) |
| `pbs_core(windowed)` | 6467 | PBS methods (#6,#8,#10,#12) |
| `pbs_core_sipp` | 7168 | **dead code** (see note) |
| `path_planning_pp_per_task` | 1943 | Hungarian/LNS + PP |
| `path_planning_pbs` | 7755 | PBS wrapper -> pbs_core(false) |
| **native windowed PBS** (`WState`/`WReservationTable`/`WStateTimeAStar`/`WPBS`) | 7798–8460 | #7, #9 (via native path) |
| `path_planning_wpbs` | 4991 | wPBS methods (#7,#9,#11,#13) |
| `native_wpbs_solve` (entry) | 4926 | wPBS methods (#7,#9,#11,#13) |

> Note: `pbs_core_sipp()` [7168] is **dead code** — `path_planning_pbs/_wpbs` call `pbs_core(false/true)`
> (which delegates to the configured single-agent planner), or the native windowed
> solver. The former `ref_solve.cpp`/`namespace refsolve` module has been **deleted**.

---

## Flag → config → code dispatch summary

| # | Label | -a preset | extra flags | assign | high-level MAPF | low-level | shuffle? |
|---|-------|-----------|-------------|--------|-----------------|-----------|:---:|
| 1 | TP-STA* | TP | — | decoupled greedy [985] | (in-assign PP) | 2× astar [2176] | — |
| 2 | TPTS-STA* | TPTS | — | greedy+swaps [1057] | (in-assign PP) | 2× astar | — |
| 3 | CENTRAL-CBS | CENTRAL-CBS | `--ecbs_focal_weight`, `--cbs_high_level_expansion_limit` | per-step Hungarian | CBS/ECBS in two groups | `SingleAgentECBS`, focal weight 1.0 and effectively uncapped high-level search by default | — |
| 3b | CENTRAL-fixed-CBS | CENTRAL-fixed | `--ecbs_focal_weight`, `--cbs_high_level_expansion_limit` | event-driven Hungarian | CBS/ECBS in two groups | `SingleAgentECBS`, focal weight 1.0 and effectively uncapped high-level search by default | — |
| 3c | HBH+MLA* | HBH_MLA | `--mode`, `--semi_online_lookahead_batches` | h-value centralized greedy | interleaved prioritized planning | `MLAStarPlanner`, arbitrary ordered goals | — |
| 4 | TA-Hybrid-STA* | TA_HYBRID | --tour | LKH3 reassign [2649] | two-group [3992] | Group-1 **ICBS** [3752] + CostFlow | — |
| 5 | TA-Prioritized-STA* | TA_PRIORITIZED | `--tour`; offline only | complete LKH3 tour | paper-order PP over complete sequences | sequential `astar_with_dummy`, arbitrary ordered goals | — |
| 6 | Hungarian+PBS-MLA* | HUNGARIAN_PBS | — | rep. hungarian [4459] | PBS [6467] | mla_star_taskwise [6363] | ✓ |
| 7 | Hungarian+wPBS-MLA* | HUNGARIAN_wPBS | — | rep. hungarian | **native wPBS** [8741] | WStateTimeAStar [8161] | ✓ |
| 8 | LNS(1s)+PBS-MLA* | LNS_PBS | --lns_time 1 | rep. hung.+LNS [4838] | PBS | mla_star_taskwise | ✓ |
| 9 | LNS(1s)+wPBS-MLA* | LNS_wPBS | --lns_time 1 | rep. hung.+LNS | **native wPBS** [8741] | WStateTimeAStar | ✓ |
| 10 | Hungarian+PBS-MLSIPP | HUNGARIAN_PBS | --single_agent MLSIPP | rep. hungarian | PBS | sipp_search | ✓ |
| 11 | Hungarian+wPBS-MLSIPP | HUNGARIAN_wPBS | --single_agent MLSIPP | rep. hungarian | native wPBS | WSippSearch | ✓ |
| 12 | LNS(1s)+PBS-MLSIPP | LNS_PBS | --lns_time 1 --single_agent MLSIPP | rep. hung.+LNS | PBS | sipp_search | ✓ |
| 13 | LNS(1s)+wPBS-MLSIPP | LNS_wPBS | --lns_time 1 --single_agent MLSIPP | rep. hung.+LNS | native wPBS | WSippSearch | ✓ |
| 14 | TP-SIPP | TP | --single_agent MLSIPP | decoupled greedy | (in-assign PP) | sipp_search | — |
| 15 | TPTS-SIPP | TPTS | --single_agent MLSIPP | greedy+swaps | (in-assign PP) | sipp_search | — |
| 16 | Hungarian+PP-SIPP | HUNGARIAN_PBS | --mapf PP --single_agent MLSIPP | rep. hungarian | PP | sipp_search | — |
| 17 | LNS(1s)+PP-SIPP | LNS_PBS | --mapf PP --lns_time 1 --single_agent MLSIPP | rep. hung.+LNS | PP | sipp_search | — |

**CLI switches recap**
- `--single_agent STA|MLA|MLSIPP` — selects the low-level single-agent planner.
- `--mapf PBS|wPBS|PP|PP_PER_TASK|PP_TASK_SEQUENCE` — overrides preset MAPF.
- `--lns_time <s>` — LNS assignment budget (default 1s; #8,#9,#12,#13,#17).
- `--tour <file>` — LKH3 tour input for the offline TA methods (#4,#5).
- `--save_output` — dump paths / task completions / runtime to `./output/`.
