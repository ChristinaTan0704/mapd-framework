# Algorithm Implementation Traces

All 11 algorithms (+1 post-processing) share the same entry point:

```
run()                                        [line 73]
  while (!end()):                             [line 74]
    release_tasks()                           [line 75]
    task_assignment_and_path_planning()        [line 76]
      task_assignment()                       [line 449]
        instant-pickup detection              [line 456]  -- CENTRAL/CENTRAL_FIXED only
        if should_assign():                   [line 372]
          switch assign_method                [line 504]
      if should_replan():                     [line 363]
        path_planning()                       [line 556]
          -- PP+MLA* for HUNGARIAN/LNS        [line 567]
          -- CBS Group1+PBS Group2 for CENTRAL [line 560]
          -- switch on config.mapf            [line 573]
    update_system()                           [line 77]

driver.cpp post-processing:
  if lns_imp_rounds > 0:
    sim.realpath_lns_imp(rounds, group_size)  [line 4491]
  sim.fullCollisionCheck()                    [line 1740]
  sim.showTask()                              [line 1771]
  if save_output:
    sim.saveOutput(filepath, runtime_ms)      [line 1789]
```

---

## 1. TP -- Token Passing

**Config:** `DECOUPLED_GREEDY, ON_FREE_WAITS, DECOUPLED_PP, STA_TASK_EP, HOLDING_ENDPOINT`

**Switchable:** `--single_agent MLA` to use token MLA* instead of 2x A*

```
run()
 while (!end()):
   release_tasks()
   task_assignment():
     should_assign() -> AT_ON_FREE_WAITS                      [line 376]
       for ag in agents:                                       [line 378]
         if ag.finish_time <= timestep: return true
     case AM_DECOUPLED_GREEDY:                                 [line 505]
       for i in 0..agents.size():                              [line 506]
         pick ONE free agent (earliest finish_time)
       assign_decoupled_greedy(ag):                            [line 606]
         for i in 0..token.path.size():                        [line 610]
           build hold set
         for it in token.tasks:                                [line 615]
           find closest task (min BFS h-val)
         if task found:
           plan_task_token(ag, task):                           [line 1655]
             if MLA: token_mla_star(ag, task)                  [line 1530]
             else:   astar(pickup) + astar(delivery)           [line 1441]
           for i in timestep..maxtime:                         [line 633]
             token.path[ag.id][i] = ag.path[i]
         else: move2EP(ag) or bump finish_time                 [line 1684]
     should_replan() -> false                                  [line 419]
   update_system():
     AT_ON_FREE_WAITS branch:                                  [line 227]
       if free agent: stay, detect deliveries/pickups, return
       else: advance to next event
```

---

## 2. TPTS -- Token Passing with Task Swaps

**Config:** `DECOUPLED_GREEDY_SWAPS, ON_FREE_WAITS, DECOUPLED_PP, STA_TASK_EP, HOLDING_ENDPOINT`

**Switchable:** `--single_agent MLA` to use token MLA* (supports ag_hide for swaps)

```
case AM_DECOUPLED_GREEDY_SWAPS:                                [line 517]
  pick ONE free agent                                          [line 518]
  assign_tpts(ag, depth=0):                                    [line 662]
    save full state
    for it in token.tasks (sorted by distance):                [line 680]
      ag_hide = displaced agent or self
      plan_task_token(ag, task, ag_hide):                       [line 1655]
        if MLA: token_mla_star(ag, task, ag_hide)              [line 1530]
          (skips ag_hide in isConstrained + can_hold)
        else:   astar(pickup, ag_hide) + astar(delivery, ag_hide)
      if swap feasible:
        assign_tpts(displaced_agent, depth+1) <- RECURSIVE     [line 749]
      rollback on failure
```

---

## 3. HBH-MLA* -- Centralized Greedy + MLA*

**Config:** `CENTRALIZED_GREEDY, ON_FREE_WAITS, DECOUPLED_PP, MLA_SEQUENCE, HOLDING_ENDPOINT`

**Switchable:** `--single_agent STA` to use 2x A* instead of MLA*

```
case AM_CENTRALIZED_GREEDY:                                    [line 528]
  assign_centralized_greedy():                                 [line 782]
    for i in 0..agents.size():                                 [line 790]
      collect ALL free agents
    for it in token.tasks:                                     [line 806]
      collect available tasks
    for aid in free_ids:                                       [line 819]
      for t in avail_tasks:                                    [line 820]
        create (agent, task, h_val) pairs
    sort pairs by h_val ascending                              [line 826]
    for p in pairs:                                            [line 832]
      plan_task_token(ag, task):                                [line 1655]
        if MLA: token_mla_star(ag, task)                       [line 1530]
        else:   astar(pickup) + astar(delivery)                [line 1441]
      if success: commit to token.path, remove task
    for aid in free_ids:                                       [line 872]
      remaining free agents: move2EP or bump
```

---

## 4. CENTRAL -- Hungarian + CBS

**Config:** `HUNGARIAN, EVERY_TIMESTEP, CBS, STA_TASK_EP, HOLDING_ENDPOINT`

**Switchable:** `--mapf PBS` to use PBS+MLA* instead of CBS for Group 2

```
run()
 while (!end()):
   release_tasks()
   task_assignment():
     instant-pickup detection:                                 [line 456]
       for i in 0..agents.size():                              [line 458]
         if FREE at task pickup: -> CARRYING
     should_assign() -> AT_EVERY_TIMESTEP                      [line 382]
     case AM_HUNGARIAN:                                        [line 531]
       assign_hungarian():                                     [line 887]
         for i in 0..agents.size():                            [line 893]
           collect FREE agents
         for it in token.tasks:                                [line 905]
           filter candidate tasks
         for aid in phase2_free_ids_:                           [line 925]
           add parking endpoints for extra agents
         for i in 0..N:                                        [line 947]
           for j in 0..N:                                      [line 948]
             build cost matrix
         dlib::max_cost_assignment()                           [line 968]
         store in phase2_free_ids_, phase2_tasks_

     should_replan() -> true (central_has_event_)              [line 423]

     path_planning():                                          [line 556]
       if --mapf PBS:
         path_planning_cbs_with_pp():                          [line 1091]
           GROUP 1: astar(delivery) for CARRYING agents        [line 1093]
           GROUP 2: PBS+MLA* for FREE agents                   [line 1112]
             build goal_seqs = [pickup/parking, dummy]         [line 1140]
             PBS root: mla_star per agent                      [line 3695]
             PBS DFS conflict resolution (max 2000 HL nodes)
             commit best_node paths to token
       else (default CBS):
         path_planning_cbs():                                  [line 996]
           GROUP 1: astar(delivery) for CARRYING agents        [line 1000]
           GROUP 2: CBSSearch for FREE agents                  [line 1028]
             CBS high-level + ECBS low-level                   [cbs.cpp]
             fallback: path_planning_pp() if CBS fails         [line 1080]

   update_system():
     default branch:                                           [line 272]
       advance to next event
       detect deliveries: CARRYING + done -> FREE              [line 312]
       detect pickups: MOVING_TO_PICKUP arrived -> CARRYING    [line 328]
```

---

## 5. CENTRAL_FIXED -- Event-Driven Centralized

**Config:** `HUNGARIAN, ON_NEW_TASK_OR_FREE, CBS, STA_TASK_EP, HOLDING_ENDPOINT`

**Switchable:** `--mapf PBS` (same as CENTRAL)

Same as CENTRAL except:
```
should_assign() -> AT_ON_NEW_TASK_OR_FREE                      [line 389]
  checks central_reassign_event_ (NOT central_has_event_)
  true:  delivery completed, new tasks arrived
  false: pickup arrival only
```

---

## 6. TA_PRIORITIZED -- Offline TSP + Prioritized Planning

**Config:** `LKH3_TSP, ONCE, DECOUPLED_PP, SEQ_STA, DUMMY_PATH`

```
task_assignment() -> AM_LKH3_TSP:                              [line 540]
  assign_ta_tsp():                                             [line 1849]
    parse LKH3 tour file, split into per-agent task_sequences

path_planning() -> DECOUPLED_PP + SEQ_STA:                     [line 574]
  plan_ta_prioritized():                                       [line 1952]
    sort agents by descending makespan
    for each agent: for each task:
      astar_with_dummy(-> pickup, park)                        [line 2144]
      astar_with_dummy(-> delivery, park)                      [line 2144]
```

---

## 7. TA_HYBRID -- Offline TSP + Two-Group Planning

**Config:** `LKH3_TSP_REASSIGN, ONCE, TA_HYBRID_TWO_GROUP, STA_TASK_EP, DUMMY_PATH`

```
path_planning() -> MAPF_TA_HYBRID_TWO_GROUP:                   [line 581]
  plan_ta_hybrid():                                            [line 2829]
    for each timestep:
      Group 1: delivery via astar_with_dummy()                 [line 2144]
      Group 2: route to pickups via CostFlow                   [line 2414]
```

---

## 8. HUNGARIAN_PBS -- Repeated Hungarian + PBS

**Config:** `HUNGARIAN, ON_UNASSIGNED_OR_FREE, PBS, MLA_SEQUENCE, DUMMY_PATH`

**Switchable:** `--mapf PP` to use PP+MLA* instead of PBS

```
run()
 while (!end()):
   release_tasks()
   task_assignment():
     should_assign() -> AT_ON_UNASSIGNED_OR_FREE               [line 397]
     case AM_HUNGARIAN (trigger=ON_UNASSIGNED_OR_FREE):        [line 531]
       assign_repeated_hungarian():                            [line 3249]
         while !remaining_tasks.empty():                       [line 3260]
           for i in 0..row:                                    [line 3265]
             for j in 0..row:                                  [line 3266]
               build cost matrix = -(estimated arrival time)
           dlib::max_cost_assignment()                         [line 3312]
           append to agent.task_sequence

     should_replan() -> true (pbs_has_event_)                  [line 431]

     path_planning():                                          [line 556]
       if --mapf PP:
         path_planning_pp_mla():                               [line 4213]
           save old_paths from token                           [line 4226]
           sort agents: active first                           [line 4233]
           for each agent in order:                            [line 4241]
             cons_paths = other agents' paths (old or new)     [line 4247]
             build goals from FULL task_sequence               [line 4257]
               goals = [p1,d1,p2,d2,...,pN,dN]
             choose non-task dummy endpoint                    [line 4273]
               skip task endpoints, other agents' dummies,
               and cons_path permanent holds
             goals += [dummy]
             mla_star(agent, goals, cons_paths)                [line 3695]
               ONE call through ALL goals
             commit path to token.path                         [line 4298]
       else (default PBS):
         path_planning_pbs():                                  [line 4337]
           pbs_core(windowed=false):                           [line 3934]
             build_goal_sequences():                           [line 3615]
               for each agent: goals = [p,d,...,dummy]
               choose_dummy_endpoint(strict=true)              [line 3569]
             save old_paths                                    [line 3942]
             ROOT: for each agent:                             [line 3972]
               mla_star(goals, cons=planned, old=unplanned)    [line 3695]
             find conflicts                                    [line 4012]
             DFS search (max 5000 HL):                         [line 4060]
               replan lower-priority via mla_star()
             commit best_node paths                            [line 4186]

   update_system():
     AT_ON_UNASSIGNED_OR_FREE branch:                          [line 154]
       scan paths for goal arrivals -> advance
       update_system_pbs():                                    [line 3179]
         detect pickup -> CARRYING
         detect delivery -> pop sequence, FREE or next task
         check periodic replan trigger
```

---

## 9. HUNGARIAN_wPBS -- Repeated Hungarian + Windowed PBS

**Config:** `HUNGARIAN, ON_UNASSIGNED_OR_FREE, wPBS, MLA_SEQUENCE, DUMMY_PATH, replan_window=10`

**Switchable:** `--mapf PP` (same as HUNGARIAN_PBS)

Same as HUNGARIAN_PBS except in pbs_core(windowed=true):
- choose_dummy_endpoint(strict=false) — pairwise mode
- conflict detection limited to replan_window timesteps
- DFS branching: use_old_paths=false (standard PBS)
- periodic replan every replan_window steps

---

## 10. LNS_PBS -- Repeated Hungarian + LNS + PBS

**Config:** `REPEATED_HUNGARIAN_LNS, ON_UNASSIGNED_OR_FREE, PBS, MLA_SEQUENCE, DUMMY_PATH, lns_time_limit=1`

**Switchable:** `--mapf PP` (same as HUNGARIAN_PBS)

```
task_assignment() -> AM_REPEATED_HUNGARIAN_LNS:                [line 537]
  assign_repeated_hungarian_lns():                             [line 3515]
    Phase 1: assign_repeated_hungarian()                       [line 3249]
    Phase 2: LNS improvement (time-limited):                   [line 3529]
      while elapsed < lns_time_limit:
        lns_destroy(): RANDOM/WORST/RELATED                    [line 3383]
        lns_repair(): regret-based re-insertion                [line 3455]
        estimate_sequence_cost() for accept/reject             [line 3360]

path_planning() -> PBS or PP (same switch as HUNGARIAN_PBS)
  pbs_core with max_tasks_per_agent=2
```

---

## 11. LNS_wPBS -- Repeated Hungarian + LNS + Windowed PBS

Same as LNS_PBS except pbs_core(windowed=true).

---

## 12. REALPATH_LNS_IMP -- Post-Processing Anytime Improvement

**Called from:** `driver.cpp` via `--lns_imp <rounds> --lns_imp_group <size>`

```
realpath_lns_imp(num_rounds, group_size):                      [line 4491]
  build agent_task_lists from task.status                      [line 4501]
  for round in 0..num_rounds:                                  [line 4510]
    SNAPSHOT                                                   [line 4514]
    DESTROY: rmca_destroy()                                    [line 4380]
      RANDOM / WORST / MULTIPLE strategies
    CASCADE: remove destroyed + subsequent tasks               [line 4541]
    REPAIR: rmca_repair() — Hungarian assignment               [line 4420]
    REPLAN: PP with per-task astar, finish-time ordered         [line 4567]
    EVALUATE: accept if makespan + SWT both improve            [line 4595]
    REJECT: restore snapshot                                   [line 4602]
```

---

## Shared Components

### Space-Time A* (`astar`) -- [line 1441]
**Used by:** TP(STA), TPTS(STA), HBH(STA), CENTRAL(Group1+Group2), REALPATH_LNS_IMP

### Token-based MLA* (`token_mla_star`) -- [line 1530]
**Used by:** TP(MLA), TPTS(MLA), HBH_MLA(MLA)
- Uses `isConstrained()` against `token.path`
- Supports `ag_hide` for TPTS swap planning
- No `can_hold` at intermediate goals, only at delivery

### Plan Task Token (`plan_task_token`) -- [line 1655]
**Used by:** TP, TPTS, HBH_MLA
- Switch: `config.single_agent == MLA_SEQUENCE` → `token_mla_star()`
- Default: 2x `astar()` (pickup then delivery)

### Two-Phase A* (`astar_with_dummy`) -- [line 2144]
**Used by:** TA_PRIORITIZED, TA_HYBRID

### Multi-Label A* (`mla_star`) -- [line 3695]
**Used by:** HUNGARIAN_PBS, HUNGARIAN_wPBS, LNS_PBS, LNS_wPBS (in pbs_core)
             CENTRAL+PBS (in path_planning_cbs_with_pp Group 2)
             HUNGARIAN+PP (in path_planning_pp_mla)
- Uses `cons_paths` (hard) + `old_paths` (soft)
- Returns max_t-length padded path

### CBS/ECBS -- [cbs.cpp]
**Used by:** CENTRAL(CBS), CENTRAL_FIXED(CBS)

### Min-Cost Max-Flow (`CostFlow`) -- [simulation.h]
**Used by:** TA_HYBRID (Group 2)

### Priority-Based Search (`pbs_core`) -- [line 3934]
**Used by:** HUNGARIAN_PBS, HUNGARIAN_wPBS, LNS_PBS, LNS_wPBS
             CENTRAL+PBS (simplified version in path_planning_cbs_with_pp)

### PP+MLA* (`path_planning_pp_mla`) -- [line 4213]
**Used by:** HUNGARIAN/LNS with `--mapf PP`
- Plans agents one at a time with full task_sequence goals
- Each agent: one `mla_star` call through all goals + dummy
- cons_paths = other agents' old/new paths

### Move to Endpoint (`move2EP`) -- [line 1684]
**Used by:** TP, TPTS, HBH-MLA*

---

## Config Dispatch Summary

| Function | TP | TPTS | HBH | CENTRAL | C_FIX | TA_P | TA_H | H_PBS | H_wPBS | LNS_PBS | LNS_wPBS |
|----------|----|----|-----|---------|-------|------|------|-------|--------|---------|----------|
| trigger | FREE | FREE | FREE | EVERY | NEW_F | ONCE | ONCE | UNAS | UNAS | UNAS | UNAS |
| assign | GRD | GRD_SW | C_GRD | HUNG | HUNG | TSP | TSP_R | HUNG | HUNG | RH_LNS | RH_LNS |
| replan? | false | false | false | true | true | true | true | true | true | true | true |
| mapf | -- | -- | -- | CBS/PBS | CBS/PBS | PP | HYB | PBS/PP | wPBS/PP | PBS/PP | wPBS/PP |
| update | TP | TP | TP | def | def | def | def | PBS | PBS | PBS | PBS |
| solver | STA/MLA | STA/MLA | MLA/STA | ECBS/MLA | ECBS/MLA | 2ph | 2ph+F | MLA* | MLA* | MLA* | MLA* |

**CLI switches:**
- `--single_agent STA|MLA` — switches TP, TPTS, HBH_MLA low-level solver
- `--mapf CBS|PBS|PP` — switches CENTRAL/CENTRAL_FIXED and HUNGARIAN/LNS path planner
- `--save_output` — saves agent paths + task completions + runtime to `./output/`
