# Algorithm Call Traces — Unified MAPD Framework

How each algorithm flows through the pseudocode, with constraint table (ct) lifecycle.

---

## 1. TP (Token Passing)

**Config:** `mode=ONLINE, assign_method=DECOUPLED_GREEDY, single_agent=STA_TASK_EP, deadlock=HOLDING_ENDPOINT`

```
Main (Sec 2)
│ ct = empty
│ For each agent: ct.add(agent_i, [init_loc], init_loc, 0)
│
└─ While not End():
   ├─ GetReleasedTasks(t, ONLINE) (Sec 3)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config) (Sec 2):
   │   ├─ Task_Assignment(ct, config) (Sec 6):
   │   │   ├─ ShouldAssign? (Sec 5) → ON_FREE_WAITS
   │   │   ├─ GetReplanAgents → FREE agents at end of path
   │   │   ├─ GetAvailTasks → unassigned only
   │   │   └─ Assign_Decoupled_Greedy(replan_agents, avail_tasks, ct, config) (Sec 9.1)
   │   │       └─ For each free agent_i:
   │   │           ├─ FilterEndpointConflicts (Sec 14.2)
   │   │           ├─ Pick closest task
   │   │           └─ Plan_Agent_Path(agent_i, ct, config) (Sec 12.1):
   │   │               ├─ ct.delete(agent_i)         // remove old trivial path
   │   │               ├─ Find_Path(agent_i, ct, config) (Sec 12.0) → STA_TASK_EP:
   │   │               │   ├─ STA*(loc → pickup, ct, agent_i)  (Sec 13.1)
   │   │               │   │   └─ uses ct.is_valid_for(agent_i, ...) + ct.can_hold(...)
   │   │               │   ├─ STA*(pickup → delivery, ct, agent_i)
   │   │               │   └─ Return (path, delivery_loc)
   │   │               ├─ agent.path = path
   │   │               ├─ agent.last_endpoint = delivery_loc
   │   │               └─ ct.add(agent_i, path, delivery_loc, t)  // add new
   │   │
   │   │           If no task available:
   │   │           └─ Plan_Dummy_Path(agent_i, ct, config) (Sec 12.1):
   │   │               ├─ ct.delete(agent_i)
   │   │               ├─ Find_Dummy_Path(agent_i, ct, config) (Sec 12.0) → HOLDING_EP
   │   │               │   └─ If loc not a delivery → Return ([loc], loc)
   │   │               │      Else → STA* to safe endpoint
   │   │               └─ ct.add(agent_i, dummy_path, endpoint, t)
   │   │
   │   └─ ShouldReplan? (Sec 15) → false (path planning done inside assignment)
   │
   └─ Update_System (Sec 4)
```

---

## 2. TPTS (Token Passing with Task Swaps)

**Config:** `mode=ONLINE, assign_method=DECOUPLED_GREEDY_SWAPS, single_agent=STA_TASK_EP, deadlock=HOLDING_ENDPOINT`

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, ONLINE) (Sec 3)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config) (Sec 6):
   │   │   ├─ ShouldAssign? → ON_FREE_WAITS
   │   │   ├─ GetReplanAgents → FREE agents at end of path
   │   │   ├─ GetAvailTasks → unassigned OR assigned-but-not-carrying
   │   │   └─ Assign_TPTS(replan_agents, avail_tasks, ct, config) (Sec 9.2)
   │   │       └─ For agent_i → GetTask_TPTS(agent_i, avail_tasks, ct, config):
   │   │           ├─ FilterEndpointConflicts, sort by distance
   │   │           └─ For task_j in sorted:
   │   │               │
   │   │               ├─ CASE 1: task unassigned (status == -1)
   │   │               │   ├─ Plan_Agent_Path(agent_i, ct, config):
   │   │               │   │   ├─ ct.delete(agent_i)
   │   │               │   │   ├─ Find_Path(agent_i, ct, config)
   │   │               │   │   ├─ If success: ct.add(agent_i, new_path, ...)
   │   │               │   │   └─ If fail: ct.add(agent_i, OLD_path, ...) // restore
   │   │               │   ├─ Success → return true
   │   │               │   └─ Fail → restore agent state, try next
   │   │               │
   │   │               └─ CASE 2: swap attempt (task assigned to agent_j)
   │   │                   ├─ saved_j_path = agent_j.path.copy()
   │   │                   ├─ ct.delete(agent_j)                    // free agent_j's space
   │   │                   ├─ Plan_Agent_Path(agent_i, ct, config): // plan agent_i
   │   │                   │   ├─ ct.delete(agent_i)
   │   │                   │   ├─ Find_Path(agent_i, ct, ...)
   │   │                   │   └─ ct.add(agent_i, new_path, ...)
   │   │                   │
   │   │                   ├─ If agent_i reaches pickup FASTER than agent_j (TimeToReach):
   │   │                   │   └─ GetTask_TPTS(agent_j, remaining, ct, config) ← RECURSIVE
   │   │                   │       └─ agent_j plans against ct (which has agent_i's new path)
   │   │                   │       ├─ Success → swap accepted, return true
   │   │                   │       └─ Fail → proceed to rollback
   │   │                   │
   │   │                   └─ ROLLBACK on failure:
   │   │                       ├─ ct.delete(agent_i)                // remove agent_i's path
   │   │                       ├─ agent_j.path = saved_j_path
   │   │                       └─ ct.add(agent_j, saved_j_path, saved_j_ep, t) // restore
   │   │
   │   │           After exhausting all tasks — fallback (Alg 2, Lines 34-43):
   │   │           ├─ If NOT at endpoint (recursive displaced agent):
   │   │           │   └─ Try Find_Dummy_Path → if success, return true; else return false
   │   │           └─ If AT endpoint:
   │   │               └─ Stay (if not at delivery of task in T) or Plan_Dummy_Path
   │   │
   │   │           If GetTask_TPTS returns false:
   │   │           └─ Plan_Dummy_Path(agent_i, ct, config)
   │   │
   │   └─ ShouldReplan? → false
   │
   └─ Update_System (Sec 4)
```

---

## 3. CENTRAL

**Config:** `mode=ONLINE, assign_method=HUNGARIAN, mapf=CBS, single_agent=STA_TASK_EP, deadlock=HOLDING_ENDPOINT`

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, ONLINE) (Sec 3)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config) (Sec 6):
   │   │   ├─ ShouldAssign? → EVERY_TIMESTEP (whenever tasks exist)
   │   │   ├─ GetReplanAgents → ALL agents
   │   │   ├─ GetAvailTasks → unassigned only
   │   │   └─ Assign_Hungarian(replan_agents, avail_tasks, ct, config) (Sec 9.4)
   │   │       ├─ Build cost matrix, solve Hungarian
   │   │       └─ Apply assignment (NO path planning here)
   │   │
   │   └─ ShouldReplan? (Sec 15) → CENTRAL → true
   │       └─ Path_Planning(all_agents, ct, config) (Sec 11) → CBS
   │           └─ Path_Planning_CBS(all_agents, ct, config) (Sec 12.2):
   │               ├─ base_ct = ct.copy_without(all_agents)  // empty
   │               ├─ Root node: For each agent_i:
   │               │   └─ Find_Path_CBS(agent_i, base_ct, {}, config)
   │               │       └─ Find_Path(agent_i, merged_ct, config) → STA_TASK_EP
   │               ├─ CBS loop (best-first, priority_queue):
   │               │   ├─ FindConflict → Branch → Find_Path_CBS with constraints
   │               │   └─ If no conflict → solution found
   │               └─ On solution: batch ct.update for all agents
   │
   └─ Update_System (Sec 4)
```

---

## 4. CENTRAL-fixed

**Config:** `mode=ONLINE, assign_method=HUNGARIAN, assign_trigger=ON_NEW_TASK_OR_FREE, mapf=CBS, single_agent=STA_TASK_EP, deadlock=HOLDING_ENDPOINT`

Same Hungarian + CBS as CENTRAL, but event-driven instead of every-timestep:

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, ONLINE)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config):
   │   │   ├─ ShouldAssign? → ON_NEW_TASK_OR_FREE
   │   │   │   (true ONLY when new task arrives OR agent becomes free)
   │   │   ├─ If triggered:
   │   │   │   ├─ GetReplanAgents → all agents
   │   │   │   ├─ GetAvailTasks → unassigned only
   │   │   │   └─ Assign_Hungarian → same as CENTRAL
   │   │   └─ (assignments persist between events — agents follow planned paths)
   │   │
   │   └─ ShouldReplan? → CENTRAL_FIXED
   │       ├─ If newly occupied agents exist:
   │       │   └─ Path_Planning_CBS(occupied_agents, ct, config)
   │       └─ If assignment just happened:
   │           └─ Path_Planning_CBS(free_agents, ct, config)
   │       (CBS internals same as CENTRAL: copy_without → search → batch update)
   │
   └─ Update_System (Sec 4)
```

**Why CENTRAL-fixed is complete but CENTRAL is not:**
- Assignments persist between events → agents make progress toward task completion
- The thesis proves Properties 6.4/6.5 guarantee CBS always finds paths
- Event-driven trigger ensures each task eventually gets assigned and completed (Theorem 6.3)

---

## 5. HBH-MLA*

**Config:** `mode=ONLINE, assign_method=CENTRALIZED_GREEDY, single_agent=MLA_SEQUENCE, deadlock=HOLDING_ENDPOINT`

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, ONLINE)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config):
   │   │   ├─ ShouldAssign? → ON_FREE_WAITS
   │   │   └─ Assign_Centralized_Greedy(replan_agents, avail_tasks, ct, config) (Sec 9.3)
   │   │       └─ Repeat: pick best (agent, task) pair globally
   │   │           └─ Assign task to agent (NO path planning here)
   │   │
   │   └─ ShouldReplan? → HBH_MLA → true (assignment just happened)
   │       └─ Path_Planning_Decoupled(agents_to_plan, ct, config) (Sec 12.1):
   │           └─ For each agent_i:
   │               └─ Plan_Agent_Path(agent_i, ct, config):
   │                   ├─ ct.delete(agent_i)
   │                   ├─ Find_Path(agent_i, ct, config) → MLA_SEQUENCE:
   │                   │   ├─ BuildGoalSequence → [pickup, delivery]
   │                   │   ├─ MLA*(loc → [pickup, delivery], ct, agent_i) (Sec 13.2)
   │                   │   └─ Return (path, delivery_loc)
   │                   │   (or if no task: Find_Dummy_Path → stay or move to endpoint)
   │                   └─ ct.add(agent_i, path, last_ep, t)
   │
   └─ Update_System
```

---

## 6. TA-Prioritized

**Config:** `mode=OFFLINE, assign_method=LKH3_TSP, single_agent=SEQ_STA, deadlock=DUMMY_PATH`

```
Main (Sec 2)
│ ct = empty; ct.add for each agent's initial path
│ T = all tasks (released at t=0)
│
└─ While not End():
   ├─ GetReleasedTasks(t, OFFLINE)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config):
   │   │   ├─ ShouldAssign? → ONCE (true only at t=0)
   │   │   └─ Assign_TA_TSP (Sec 10.1) → LKH3 mTSP → task sequences per agent
   │   │
   │   └─ ShouldReplan? → TA_PRIORITIZED → true (at t=0)
   │       └─ Path_Planning_Decoupled(all_agents, ct, config) (Sec 12.1):
   │           // Agents sorted by LARGEST estimated execution time first
   │           └─ For each agent_i (in priority order):
   │               └─ Plan_Agent_Path(agent_i, ct, config):
   │                   ├─ ct.delete(agent_i)
   │                   ├─ Find_Path(agent_i, ct, config) → SEQ_STA:
   │                   │   ├─ For each task in [current] + task_sequence:
   │                   │   │   ├─ STA_WithDummyCheck(curr → pickup, ...) (Sec 14.5)
   │                   │   │   │   └─ GoalTest: x==goal AND t>=release AND dummy path exists
   │                   │   │   └─ STA_WithDummyCheck(pickup → delivery, ...)
   │                   │   ├─ STA*(last_delivery → non_task_EP)  // final dummy path
   │                   │   └─ Return (full_path, non_task_EP)
   │                   └─ ct.add(agent_i, full_path, non_task_EP, t)
   │                      // next agent plans against ct including this agent's path
   │
   │   Subsequent timesteps: ShouldAssign? → ONCE → false; ShouldReplan? → false
   │
   └─ Update_System → auto-advance through task sequences
```

---

## 7. TA-Hybrid

**Config:** `mode=OFFLINE, assign_method=LKH3_TSP_REASSIGN, mapf=TA_HYBRID_TWO_GROUP, deadlock=DUMMY_PATH`

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, OFFLINE)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config):
   │   │   ├─ ShouldAssign? → ONCE (true only at t=0)
   │   │   └─ Assign_TA_Hybrid → Assign_TA_TSP (Sec 10.1) — one-shot TSP
   │   │
   │   └─ ShouldReplan? → TA_HYBRID → true (every timestep)
   │       └─ Path_Planning_TA_Hybrid(ct, config) (Sec 10.2):
   │           │
   │           ├─ Group 1: New task agents
   │           │   // Free agents that just arrived at pickup at/after release time
   │           │   // → become task agents (CARRYING)
   │           │   └─ Path_Planning_CBS(group1, ct, config)  // ICBS: pickup → delivery
   │           │       ├─ base_ct = ct.copy_without(group1)
   │           │       ├─ CBS search with Find_Path (STA* with dummy path goal test)
   │           │       └─ ct.update for each Group 1 agent
   │           │
   │           └─ Group 2: Free agents (if free-agent set changed or t==0)
   │               // Agents not yet executing their current task
   │               └─ PlanPathsToPickup_AMAPF(group2, ct, config)
   │                   // Min-cost max-flow: plan paths to pickup locations
   │                   // May SWAP task sequences between agents:
   │                   // "If an agent is assigned the current pickup location of
   │                   //  a different agent, TA-Hybrid replaces its current task
   │                   //  sequence with the task sequence of this different agent."
   │                   ├─ Partition group2 into subgroups (same pickup → different subgroup)
   │                   ├─ For each subgroup: MINCOSTMAXFLOW → plan paths to pickups
   │                   └─ For each subgroup: plan dummy paths to parking
   │
   └─ Update_System (Sec 4)
```

---

## 8. RMCA

**Config:** `mode=OFFLINE/ONLINE, assign_method=GREEDY_INSERT_LNS, coupled=FULLY_COUPLED, deadlock=NO_AVOIDANCE`

```
Main (Sec 2)
└─ Task_Assignment_And_Path_Planning(agents, ct, config) (Sec 2.1):
    │
    ├─ Step 1: Task_Assignment(ct, config):
    │   ├─ ShouldAssign? → ONCE/ON_NEW_TASK
    │   └─ Assign_RMCA(agents, avail_tasks, ct, config) (Sec 10.3)
    │       └─ Regret-based greedy insertion (coupled with path planning)
    │           └─ While |unassigned| > 0:
    │               ├─ For each task: compute best1/best2 marginal cost
    │               ├─ Select task with max regret
    │               └─ ct.update(commit_agent, commit_path, ...)
    │
    ├─ Step 2: ShouldReplan? → false (path planning done inside RMCA)
    │
    └─ Step 3: anytime_improvement=true → REALPATH_LNS_IMP (Sec 16.3)
        // destroy_fn=RMCA_Destroy, repair_fn=Reassign_RMCA
        // time_limit=60s (offline) or 1s (online)
        └─ While runtime < time_limit:
            ├─ RMCA_Destroy → remove eligible tasks
            ├─ Reassign_RMCA → re-assign + re-plan (coupled)
            └─ Accept if improved, else restore
```

---

## 9. LNS-PBS

**Config:** `mode=ONLINE/OFFLINE/SEMI_ONLINE, assign_method=REPEATED_HUNGARIAN_LNS, mapf=PBS, single_agent=MLA_SEQUENCE, deadlock=DUMMY_PATH`

```
Main (Sec 2)
└─ While not End():
   ├─ GetReleasedTasks(t, mode, window)
   │
   ├─ Task_Assignment_And_Path_Planning(ct, config):
   │   ├─ Task_Assignment(ct, config):
   │   │   ├─ ShouldAssign? → ON_UNASSIGNED_OR_FREE
   │   │   ├─ GetAvailTasks → filter: defer tasks whose goals overlap dummy endpoints
   │   │   └─ Assign_TA_LNS (Sec 10.4)
   │   │       ├─ Repeated Hungarian → task sequences
   │   │       └─ LNS destroy/repair → improved sequences
   │   │
   │   ├─ ChooseLastEndpoint (FLEXIBLE_STRICT) for each agent (Sec 14.3)
   │   │   └─ Excludes: assigned dummies + goal locs + old dummies of other agents
   │   │
   │   └─ ShouldReplan? → true (assignment just happened)
   │       └─ Path_Planning_PBS(all_agents, ct, config) (Sec 12.3):
   │           ├─ base_ct = ct.copy_without(all_agents)
   │           ├─ old_paths = save current paths of all agents
   │           │
   │           ├─ Root node: For each agent_i:
   │           │   └─ Find_Path_PBS(agent_i, base_ct, root, ..., old_paths, ...):
   │           │       ├─ planning_ct = base_ct.copy()
   │           │       ├─ For each other agent_j: add OLD path of agent_j
   │           │       │   // (Xu et al. 2022: "avoids the old paths of all other M-1 agents")
   │           │       └─ Find_Path(agent_i, planning_ct, config) → MLA_SEQUENCE:
   │           │           ├─ BuildGoalSequence → [p1, d1, ..., dummy_EP]
   │           │           └─ MLA*([p1, d1, ..., dummy_EP], planning_ct, agent_i)
   │           │
   │           ├─ PBS loop (DFS via stack):
   │           │   ├─ FindConflict(node.paths)
   │           │   └─ Branch: add priority edge (high → low), replan low + descendants:
   │           │       └─ Find_Path_PBS(agent_low, ...):
   │           │           ├─ planning_ct = base_ct.copy()
   │           │           ├─ Higher-priority agents: add NEW paths
   │           │           ├─ Other agents: add OLD paths
   │           │           └─ Find_Path(agent_low, planning_ct, config)
   │           │
   │           └─ On solution found:
   │               For agent_i: ct.update(agent_i, solution_path, last_ep, t)
   │
   └─ Update_System (Sec 4)
```

---

## 10. LNS-wPBS

**Config:** same as LNS-PBS but `mapf=wPBS`

```
Same as LNS-PBS except:
  ├─ GetAvailTasks → NO task deferral (all unfinished tasks)
  ├─ ChooseLastEndpoint (FLEXIBLE_PAIRWISE) (Sec 14.3)
  │   └─ Only pairwise-different constraint (no goal-loc or old-dummy checks)
  │
  └─ Path_Planning_wPBS (Sec 12.4):
      ├─ Uses Path_Planning_Standard_PBS (NOT modified PBS — no old paths)
      │   └─ Find_Path_Standard_PBS: only higher-priority agents' NEW paths
      │      // (Xu et al. 2022, Sec IV-E: "uses the original low level of PBS")
      │
      └─ Periodic trigger: if t % w == 0 → replan ALL agents
         Else → replan triggered agents only
```

---

## Constraint Table Lifecycle Summary

| Operation | Where | Purpose |
|---|---|---|
| `ct.add(agent_i, path, ep, t)` | Main init, Plan_Agent_Path (after planning), TPTS restore, CBS/PBS on solution | Reserve agent's path in ct |
| `ct.delete(agent_i)` | Plan_Agent_Path (before planning), TPTS (free displaced agent's space), RMCA (trial removal) | Free agent's space so it (or others) can replan |
| `ct.update(agent_i, path, ep, t)` | CBS/PBS on solution found, RMCA on commit | Atomic delete+add |
| `ct.copy_without(agents)` | CBS/PBS (build base_ct for replanning group) | Snapshot of non-replanned agents only |
| `ct.is_valid_for(agent_i, ...)` | STA*/MLA* during search | Check if move is collision-free |
| `ct.can_hold(agent_i, v, t)` | STA*/MLA* goal test | Check if agent can terminally wait |

---

## Comparison Matrix

| Algorithm | Assignment trigger | Task method | ct pattern | Path method | Single-agent | Deadlock |
|---|---|---|---|---|---|---|
| **TP** | free waits | decoupled greedy | delete/plan/add per agent | PP | 2x STA* | hold EP |
| **TPTS** | free waits | greedy + swaps | delete/plan/add + save/restore | PP | 2x STA* | hold EP |
| **CENTRAL** | every timestep | Hungarian | copy_without → CBS → batch update | CBS | STA* | hold EP |
| **CENTRAL-fixed** | new task/free | Hungarian | copy_without → CBS → batch update | CBS | STA* | hold EP |
| **HBH-MLA*** | free waits | centralized greedy | delete/plan/add per agent | PP | MLA* | hold EP |
| **TA-Prioritized** | once | LKH3 mTSP | delete/plan/add per agent | PP (sorted by exec time) | STA_WithDummyCheck | dummy path |
| **TA-Hybrid** | once (TSP) | LKH3 (once) | Group 1: CBS batch update; Group 2: AMAPF (may swap sequences) | ICBS + min-cost max-flow | STA* | dummy path |
| **RMCA** | once/new task | regret-based insert + LNS | delete/trial/restore → update on commit | coupled | STA* | none |
| **LNS-PBS** | unassigned/free | repeated Hungarian + LNS | copy_without → modified PBS (DFS + old paths) → batch update | modified PBS | MLA* | dummy path |
| **LNS-wPBS** | unassigned/free | repeated Hungarian + LNS | copy_without → standard PBS (DFS, no old paths) → batch update | standard PBS | MLA* | dummy path |
