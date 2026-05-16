# Unified MAPD Framework — Complete Pseudocode

Based on "Multi-Agent Pickup and Delivery: Formulations and Algorithms" (Table 1).
Supports: TP, TPTS, CENTRAL, CENTRAL-fixed, HBH-MLA*, TA-Prioritized, TA-Hybrid, RMCA, LNS-PBS, LNS-wPBS.

---

## 0. Configuration Enums

```
enum Mode          { ONLINE, OFFLINE, SEMI_ONLINE }
enum AssignType    { IA, TA }
enum AssignMethod  { DECOUPLED_GREEDY, CENTRALIZED_GREEDY, HUNGARIAN,
                     DECOUPLED_GREEDY_SWAPS,
                     LKH3_TSP, LKH3_TSP_REASSIGN,
                     GREEDY_INSERT_LNS,
                     REPEATED_HUNGARIAN_LNS }
enum AssignTrigger { ON_FREE_WAITS, EVERY_TIMESTEP, ON_NEW_TASK_OR_FREE,
                     ON_UNASSIGNED_OR_FREE, ONCE }
enum CoupledMode   { NONE, SWAPS_ONLY, REASSIGN_ONLY, FULLY_COUPLED }
enum MAPFMethod    { DECOUPLED_PP, CBS, PBS, wPBS, TA_HYBRID_TWO_GROUP }
enum SingleAgent   { STA_TASK_EP, STA_NONTASK_EP, MLA_SEQUENCE, SEQ_STA }
enum DeadlockAvoid { HOLDING_ENDPOINT, DUMMY_PATH, NO_AVOIDANCE }
enum EndpointStrategy { TASK_ENDPOINT, FIXED_PARKING, FLEXIBLE_STRICT,
                        FLEXIBLE_PAIRWISE }

// --- Anytime improvement config (optional, applies to any algorithm) ---
config.anytime_improvement = true/false   // whether to run REALPATH_LNS_IMP after planning
config.anytime_time_limit  = 60s / 1s    // wall-clock budget (60s offline, 1s online per step)
config.anytime_destroy_fn  = RMCA_Destroy / LNS_Destroy / ...   // pluggable destroy
config.anytime_repair_fn   = Reassign_RMCA / Reassign_Decoupled / ...  // pluggable repair
```

---

## 0.1 Algorithm Presets (from Table 1)

```
TP:
  mode=ONLINE, assign_type=IA, assign_method=DECOUPLED_GREEDY,
  assign_trigger=ON_FREE_WAITS, coupled=NONE,
  mapf=DECOUPLED_PP, single_agent=STA_TASK_EP or STA_NONTASK_EP,
  deadlock=HOLDING_ENDPOINT, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=false

TPTS:
  mode=ONLINE, assign_type=IA, assign_method=DECOUPLED_GREEDY_SWAPS,
  assign_trigger=ON_FREE_WAITS, coupled=SWAPS_ONLY,
  mapf=DECOUPLED_PP, single_agent=STA_TASK_EP or STA_NONTASK_EP,
  deadlock=HOLDING_ENDPOINT, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=false

CENTRAL:
  mode=ONLINE, assign_type=IA, assign_method=HUNGARIAN,
  assign_trigger=EVERY_TIMESTEP, coupled=NONE,
  mapf=CBS, single_agent=STA_TASK_EP,
  deadlock=HOLDING_ENDPOINT, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=false

CENTRAL_FIXED:
  mode=ONLINE, assign_type=IA, assign_method=HUNGARIAN,
  assign_trigger=ON_NEW_TASK_OR_FREE, coupled=NONE,
  mapf=CBS, single_agent=STA_TASK_EP,
  deadlock=HOLDING_ENDPOINT, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=false

HBH_MLA:
  mode=ONLINE, assign_type=IA, assign_method=CENTRALIZED_GREEDY,
  assign_trigger=ON_FREE_WAITS, coupled=NONE,
  mapf=DECOUPLED_PP, single_agent=MLA_SEQUENCE,
  deadlock=HOLDING_ENDPOINT, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=false

TA_PRIORITIZED:
  mode=OFFLINE, assign_type=TA, assign_method=LKH3_TSP,
  assign_trigger=ONCE, coupled=NONE,
  mapf=DECOUPLED_PP, single_agent=SEQ_STA,
  deadlock=DUMMY_PATH, endpoint_strategy=FIXED_PARKING,
  anytime_improvement=false

TA_HYBRID:
  mode=OFFLINE, assign_type=TA, assign_method=LKH3_TSP_REASSIGN,
  assign_trigger=ONCE, coupled=REASSIGN_ONLY,
  mapf=TA_HYBRID_TWO_GROUP, single_agent=STA_TASK_EP,
  deadlock=DUMMY_PATH, endpoint_strategy=FIXED_PARKING,
  anytime_improvement=false

RMCA:
  mode=OFFLINE or ONLINE, assign_type=TA, assign_method=GREEDY_INSERT_LNS,
  assign_trigger=ONCE (offline) or ON_NEW_TASK (online), coupled=FULLY_COUPLED,
  mapf=DECOUPLED_PP, single_agent=STA_TASK_EP,
  deadlock=NO_AVOIDANCE, endpoint_strategy=TASK_ENDPOINT,
  anytime_improvement=true, anytime_time_limit=60s (offline) / 1s (online),
  anytime_destroy_fn=RMCA_Destroy, anytime_repair_fn=Reassign_RMCA

LNS_PBS:
  mode=ONLINE/OFFLINE/SEMI_ONLINE, assign_type=TA,
  assign_method=REPEATED_HUNGARIAN_LNS,
  assign_trigger=ON_UNASSIGNED_OR_FREE, coupled=NONE,
  mapf=PBS, single_agent=MLA_SEQUENCE,
  deadlock=DUMMY_PATH, endpoint_strategy=FLEXIBLE_STRICT,
  anytime_improvement=false

LNS_wPBS:
  mode=ONLINE/OFFLINE/SEMI_ONLINE, assign_type=TA,
  assign_method=REPEATED_HUNGARIAN_LNS,
  assign_trigger=ON_UNASSIGNED_OR_FREE, coupled=NONE,
  mapf=wPBS, single_agent=MLA_SEQUENCE,
  deadlock=DUMMY_PATH, endpoint_strategy=FLEXIBLE_PAIRWISE,
  anytime_improvement=false
```

---

## 1. System Data Structures

```
// --- Per-task data ---
task[j].pickup_loc          // pickup vertex
task[j].delivery_loc        // delivery vertex
task[j].release_time        // when task becomes known
task[j].status              // -1: unassigned; agent_id: assigned; INT_MAX: finished
task[j].goals[]             // for MG-MAPD: ordered sequence of goal vertices
                            // for standard MAPD: [pickup_loc, delivery_loc]

// --- Per-agent data ---
agent[i].loc                // current vertex at time t
agent[i].initial_loc        // initial vertex (= parking location for offline algorithms)
agent[i].status             // FREE, MOVING_TO_PICKUP, CARRYING
agent[i].path[]             // planned path from current time onward
agent[i].last_endpoint      // the endpoint this agent "holds" (reserved from arrival onward)
agent[i].task_sequence[]    // ordered list of assigned task ids (TA methods assign multiple)
agent[i].current_task       // id of task currently being executed, or -1
agent[i].final_output_path[]// committed output path (one location per timestep)

// --- Global data ---
t                           // current simulation timestep
T                           // set of available unexecuted task ids
ct                          // global constraint table (see Section 1.1)
endpoints                   // set of all endpoint vertices
non_task_endpoints          // endpoints that are not task-goal vertices
all_tasks[]                 // master list of all tasks in the instance
dist[v][g]                  // precomputed shortest-path distances (for heuristics)
```

### 1.1 Constraint Table API

```
// The constraint table stores per-agent reservations. Each agent's entry
// consists of: vertex reservations, edge reservations, and an endpoint hold.
//
// Internally:
//   vertex_reservations: { (vertex, time) → agent_id }
//   edge_reservations:   { (v1, v2, time) → agent_id }
//   endpoint_holds:      { agent_id → (vertex, from_time) }  // reserved to infinity


ct.add(agent_i, path, last_endpoint, start_time):
  // Add reservations for agent_i's path starting at start_time.
  // Precondition: agent_i has no existing entry in ct.
  For t_step in 0..len(path)-1:
    vertex_reservations[(path[t_step], start_time + t_step)] = agent_i
  For t_step in 0..len(path)-2:
    edge_reservations[(path[t_step], path[t_step+1], start_time + t_step)] = agent_i
  endpoint_holds[agent_i] = (last_endpoint, start_time + len(path) - 1)


ct.delete(agent_i):
  // Remove all reservations for agent_i.
  Remove all vertex_reservations where value == agent_i
  Remove all edge_reservations where value == agent_i
  Remove endpoint_holds[agent_i]


ct.update(agent_i, path, last_endpoint, start_time):
  // Replace agent_i's reservations. Equivalent to delete + add.
  ct.delete(agent_i)
  ct.add(agent_i, path, last_endpoint, start_time)


ct.copy():
  // Return a deep copy of the entire constraint table.
  Return deep_copy(ct)


ct.copy_without(agents_set):
  // Return a copy with all reservations for agents in agents_set removed.
  new_ct = ct.copy()
  For agent_i in agents_set:
    new_ct.delete(agent_i)
  Return new_ct


ct.is_valid_for(agent_i, v, v_next, t_curr, t_next):
  // Check if agent_i can move from v to v_next between t_curr and t_next
  // without colliding with ANY OTHER agent's reservations.
  //
  // Vertex conflict: another agent reserves (v_next, t_next)
  If (v_next, t_next) in vertex_reservations
     AND vertex_reservations[(v_next, t_next)] != agent_i:
    Return false
  // Edge conflict: another agent traverses (v_next, v) at same time (swap)
  If (v_next, v, t_curr) in edge_reservations
     AND edge_reservations[(v_next, v, t_curr)] != agent_i:
    Return false
  // Endpoint hold conflict: another agent holds v_next from some time <= t_next
  For (agent_j, (ep, from_t)) in endpoint_holds:
    If agent_j != agent_i AND ep == v_next AND t_next >= from_t:
      Return false
  Return true


ct.can_hold(agent_i, v, from_time):
  // Check that agent_i can terminally wait at v from from_time onward.
  // Fails if any other agent has a vertex reservation at (v, t >= from_time)
  // or an endpoint hold at v.
  For (agent_j, (ep, ft)) in endpoint_holds:
    If agent_j != agent_i AND ep == v:
      Return false
  For t_future in from_time..max_reserved_time:
    If (v, t_future) in vertex_reservations
       AND vertex_reservations[(v, t_future)] != agent_i:
      Return false
  Return true
```

---

## 2. Main Loop

```
Main(config):
  // --- Initialization ---
  t = 0
  T = {}
  ct = empty constraint table

  // Precompute shortest-path distances for heuristics
  For each endpoint g in endpoints:
    dist[*][g] = BFS/Dijkstra from all vertices to g

  // Initialize agents and populate ct with initial positions
  For each agent i:
    agent[i].initial_loc = agent[i].loc      // save for fixed-endpoint strategies
    agent[i].path = [agent[i].loc]
    agent[i].last_endpoint = agent[i].loc    // initial vertex is a non-task endpoint
    agent[i].task_sequence = []
    agent[i].current_task = -1
    agent[i].status = FREE
    agent[i].final_output_path = []
    ct.add(agent_i, [agent[i].loc], agent[i].loc, 0)   // ← init ct

  // --- Main simulation loop ---
  While not End():

    // Step A: Release new tasks based on mode
    T = T ∪ GetReleasedTasks(t, config.mode, config.window)

    // Step B: Assign tasks, plan paths, optionally improve
    Task_Assignment_And_Path_Planning(agents, ct, config)

    // Step C: Execute one timestep
    Update_System()

  // Output final results
  Return ComputeMetrics()
```

---

## 2.1 Task_Assignment_And_Path_Planning

```
Task_Assignment_And_Path_Planning(agents, ct, config):
  // Step 1: Task assignment (trigger check is inside Task_Assignment)
  Task_Assignment(ct, config)

  // Step 2: Path planning for algorithms that decouple it from assignment
  If ShouldReplan(config):
    agents_to_plan = GetAgentsToReplan(config)
    Path_Planning(agents_to_plan, ct, config)

  // Step 3: Optional anytime improvement using real path costs
  // Any algorithm can enable this via config.anytime_improvement=true.
  // Default: on for RMCA, off for all others (can be toggled for ablation).
  If config.anytime_improvement:
    REALPATH_LNS_IMP(agents, ct, config,
                     time_limit   = config.anytime_time_limit,
                     destroy_fn   = config.anytime_destroy_fn,
                     repair_fn    = config.anytime_repair_fn)
```

---

## 3. GetReleasedTasks

```
GetReleasedTasks(curr_t, mode, window):
  Switch mode:
    ONLINE:
      Return { task j : task[j].release_time == curr_t AND j not in T }
    SEMI_ONLINE:
      Return { task j : task[j].release_time <= curr_t + window AND j not in T }
    OFFLINE:
      If curr_t == 0:
        Return { all tasks }   // all tasks become available at t=0
      Else:
        Return {}              // no new tasks after t=0
```

---

## 4. Update_System

```
Update_System():
  t = t + 1

  For each agent i:
    // Pop the next location from the planned path
    If len(agent[i].path) > 1:
      agent[i].path.pop_front()
    agent[i].loc = agent[i].path[0]

    // Append to committed output
    agent[i].final_output_path.append(agent[i].loc)

    // Check if agent reaches pickup location of its current task
    If agent[i].current_task != -1:
      curr_task = agent[i].current_task

      If agent[i].loc == task[curr_task].pickup_loc AND agent[i].status == MOVING_TO_PICKUP:
        agent[i].status = CARRYING

      If agent[i].loc == task[curr_task].delivery_loc AND agent[i].status == CARRYING:
        // Task completed
        task[curr_task].status = INT_MAX
        agent[i].status = FREE
        agent[i].current_task = -1
        T.remove(curr_task)

        // Start next task in sequence if available (TA methods)
        If len(agent[i].task_sequence) > 0:
          next_task = agent[i].task_sequence.pop_front()
          agent[i].current_task = next_task
          agent[i].status = MOVING_TO_PICKUP
          task[next_task].status = i

    // If agent is FREE and has no path left, it's terminally waiting
    If agent[i].status == FREE AND len(agent[i].path) <= 1:
      agent[i].last_endpoint = agent[i].loc
```

---

## 5. ShouldAssign — When to Trigger Task Assignment

```
ShouldAssign(config):
  Switch config.assign_trigger:

    ON_FREE_WAITS:                         // TP, TPTS, HBH-MLA*
      Return exists agent i where:
        agent[i].status == FREE AND
        agent[i] is at end of its path (terminally waiting) AND
        |T| > 0

    EVERY_TIMESTEP:                        // CENTRAL
      Return |T| > 0

    ON_NEW_TASK_OR_FREE:                   // CENTRAL-fixed
      Return (new_tasks_arrived_this_step OR any_agent_became_free_this_step) AND |T| > 0

    ON_UNASSIGNED_OR_FREE:                 // LNS-PBS, LNS-wPBS
      unassigned_tasks_exist = exists task j in T where task[j].status == -1
      any_agent_free = exists agent i where agent[i].status == FREE
      Return unassigned_tasks_exist OR any_agent_free

    ONCE:                                  // TA-Prioritized, TA-Hybrid (offline)
      Return t == 0 AND |T| > 0           // trigger only at t=0 when tasks are first loaded
      // TA-Hybrid also uses ONCE — task sequences are assigned once via TSP.
      // The min-cost max-flow swap happens during path planning, not assignment.
```

---

## 6. Task_Assignment — Dispatcher

```
Task_Assignment(ct, config):
  // Guard: only proceed if the trigger condition is met
  If not ShouldAssign(config):
    Return

  replan_agents = GetReplanAgents(config)
  avail_tasks = GetAvailTasks(config)

  If |replan_agents| == 0 OR |avail_tasks| == 0:
    If config.deadlock in {HOLDING_ENDPOINT, DUMMY_PATH}:
      For agent i in replan_agents where agent[i].current_task == -1:
        Plan_Dummy_Path(agent_i, ct, config)
    Return

  Switch config.assign_method:

    DECOUPLED_GREEDY:
      Assign_Decoupled_Greedy(replan_agents, avail_tasks, ct, config)

    DECOUPLED_GREEDY_SWAPS:
      Assign_TPTS(replan_agents, avail_tasks, ct, config)

    CENTRALIZED_GREEDY:
      Assign_Centralized_Greedy(replan_agents, avail_tasks, ct, config)

    HUNGARIAN:
      Assign_Hungarian(replan_agents, avail_tasks, ct, config)

    LKH3_TSP:
      Assign_TA_TSP(replan_agents, avail_tasks, config)

    LKH3_TSP_REASSIGN:
      Assign_TA_Hybrid(replan_agents, avail_tasks, ct, config)

    GREEDY_INSERT_LNS:
      Assign_RMCA(replan_agents, avail_tasks, ct, config)

    REPEATED_HUNGARIAN_LNS:
      Assign_TA_LNS(replan_agents, avail_tasks, ct, config)
```

---

## 7. GetReplanAgents

```
GetReplanAgents(config):
  Switch config.assign_trigger:

    ON_FREE_WAITS:                         // TP, TPTS, HBH-MLA*
      Return { agent i : agent[i].status == FREE AND
               agent[i] is at end of its path }

    EVERY_TIMESTEP:                        // CENTRAL
      Return all_agents

    ON_NEW_TASK_OR_FREE:                   // CENTRAL-fixed
      Return all_agents

    ON_UNASSIGNED_OR_FREE:                 // LNS-PBS, LNS-wPBS
      Return all_agents

    ONCE:                                  // TA-Prioritized, TA-Hybrid
      Return all_agents                    // at t=0, all agents need assignment + paths
```

---

## 8. GetAvailTasks

```
GetAvailTasks(config):
  Switch config.assign_type:

    IA:
      If config.assign_method == DECOUPLED_GREEDY_SWAPS:  // TPTS
        Return { task j in T :
          task[j].status != INT_MAX AND
          (task[j].status == -1 OR
           (task[j].status >= 0 AND agent[task[j].status].status != CARRYING)) }
      Else:  // TP, CENTRAL, HBH-MLA*
        Return { task j in T : task[j].status == -1 }

    TA:
      If config.assign_method == REPEATED_HUNGARIAN_LNS AND config.mapf == PBS:
        // LNS-PBS: defer tasks whose goal locations conflict with dummy endpoints
        // (Xu et al. 2022, Sec IV, Alg 1 Line 3: "all of whose goal locations
        //  are different from the dummy endpoints of the agents")
        dummy_eps = { agent[i].last_endpoint : for all agents i }
        Return { task j in T : task[j].status != INT_MAX AND
                 no goal in task[j].goals is in dummy_eps }
      Else:
        // LNS-wPBS and others: no deferral (Xu et al. 2022, Sec IV-E:
        //   "LNS-wPBS does not defer any tasks")
        Return { task j in T : task[j].status != INT_MAX }
```

---

## 9. AssignTask — IA Methods

### 9.1 Decoupled Greedy (TP)

```
Assign_Decoupled_Greedy(replan_agents, avail_tasks, ct, config):
  For agent_i in replan_agents:
    filtered = FilterEndpointConflicts(avail_tasks, agent_i)
    If |filtered| == 0:
      Plan_Dummy_Path(agent_i, ct, config)
      Continue

    best_task = argmin over task j in filtered: dist[agent[i].loc][task[j].pickup_loc]

    task[best_task].status = agent_i
    agent[agent_i].current_task = best_task
    agent[agent_i].status = MOVING_TO_PICKUP
    avail_tasks.remove(best_task)

    // Remove old reservations, plan new path, add new reservations
    Plan_Agent_Path(agent_i, ct, config)
```

### 9.2 Decoupled Greedy with Swaps (TPTS)

```
Assign_TPTS(replan_agents, avail_tasks, ct, config):
  For agent_i in replan_agents:
    If not GetTask_TPTS(agent_i, avail_tasks, ct, config):
      Plan_Dummy_Path(agent_i, ct, config)


GetTask_TPTS(agent_i, avail_tasks, ct, config):
  filtered = FilterEndpointConflicts(avail_tasks, agent_i)
  Sort filtered by dist[agent[i].loc][task[j].pickup_loc] ascending

  For task_j in sorted filtered:
    If task[task_j].status == -1:
      // Unassigned task — try direct assignment
      task[task_j].status = agent_i
      agent[agent_i].current_task = task_j
      agent[agent_i].status = MOVING_TO_PICKUP
      success = Plan_Agent_Path(agent_i, ct, config)
      If success:
        avail_tasks.remove(task_j)
        Return true
      Else:
        // Planning failed — restore agent state
        // (Plan_Agent_Path restores ct on failure, see Sec 12.1)
        task[task_j].status = -1
        agent[agent_i].current_task = -1
        agent[agent_i].status = FREE

    Else:
      // Task assigned to another agent — try swap
      agent_j = task[task_j].status

      // Save agent_j's state for rollback
      saved_j_path = agent[agent_j].path.copy()
      saved_j_last_ep = agent[agent_j].last_endpoint

      // Remove agent_j's reservations from ct
      ct.delete(agent_j)

      // Try assigning task_j to agent_i
      task[task_j].status = agent_i
      agent[agent_i].current_task = task_j
      agent[agent_i].status = MOVING_TO_PICKUP
      success_i = Plan_Agent_Path(agent_i, ct, config)

      // Compare time-to-pickup: agent_i must reach pickup faster than agent_j
      // (Paper Alg 2, Line 26: "Compare when a_i reaches s_j on its path
      //  in token to when a_i' reaches s_j on its path in token'")
      i_pickup_time = TimeToReach(agent[agent_i].path, task[task_j].pickup_loc)
      j_pickup_time = TimeToReach(saved_j_path, task[task_j].pickup_loc)

      If success_i AND i_pickup_time < j_pickup_time:
        avail_tasks_without_j = avail_tasks.copy()
        avail_tasks_without_j.remove(task_j)

        // Can displaced agent_j find another task?
        success_j = GetTask_TPTS(agent_j, avail_tasks_without_j, ct, config)
        If success_j:
          Return true

      // Swap failed — full rollback
      // Remove agent_i's new reservations (if planning succeeded)
      If success_i:
        ct.delete(agent_i)
      task[task_j].status = agent_j
      agent[agent_i].current_task = -1
      agent[agent_i].status = FREE
      // Restore agent_j
      agent[agent_j].path = saved_j_path
      agent[agent_j].last_endpoint = saved_j_last_ep
      ct.add(agent_j, saved_j_path, saved_j_last_ep, t)

  // After exhausting all tasks — handle the fallback cases
  // (Ma et al. 2017, Alg 2 Lines 34-43)
  If agent[agent_i].loc is not an endpoint:
    // Recursive displaced agent not at endpoint — try Path2
    ct.delete(agent_i)
    result = Find_Dummy_Path(agent_i, ct, config)
    If result != null:
      (dummy_path, dummy_ep) = result
      agent[agent_i].path = dummy_path
      agent[agent_i].last_endpoint = dummy_ep
      ct.add(agent_i, dummy_path, dummy_ep, t)
      Return true
    ct.add(agent_i, agent[agent_i].path, agent[agent_i].last_endpoint, t)
    Return false
  Else:
    // At an endpoint — stay or move away from delivery locations
    If no task tau_j in T exists with task[tau_j].delivery_loc == agent[agent_i].loc:
      // Safe to stay
      Return true
    Else:
      Plan_Dummy_Path(agent_i, ct, config)
      Return true
```

### 9.3 Centralized Greedy (HBH-MLA*)

```
Assign_Centralized_Greedy(replan_agents, avail_tasks, ct, config):
  // Assignment only — path planning is done separately via Path_Planning_Decoupled.
  // MLA* is complete for well-formed instances (no feasibility check needed here).
  remaining_agents = replan_agents.copy()
  remaining_tasks = avail_tasks.copy()

  While |remaining_agents| > 0 AND |remaining_tasks| > 0:
    best_cost = INF
    best_agent = -1
    best_task = -1

    For agent_i in remaining_agents:
      For task_j in remaining_tasks:
        cost = dist[agent[i].loc][task[j].pickup_loc]
        If cost < best_cost:
          best_cost = cost
          best_agent = agent_i
          best_task = task_j

    If best_task == -1:
      Break

    task[best_task].status = best_agent
    agent[best_agent].current_task = best_task
    agent[best_agent].status = MOVING_TO_PICKUP
    remaining_agents.remove(best_agent)
    remaining_tasks.remove(best_task)

  // Unassigned agents will get dummy paths during Path_Planning_Decoupled
```

### 9.4 Hungarian (Optimal Matching) — CENTRAL, CENTRAL-fixed

```
Assign_Hungarian(replan_agents, avail_tasks, ct, config):
  m = |replan_agents|
  n = |avail_tasks|
  cost_matrix = Matrix(m, n)

  For i in 0..m-1:
    For j in 0..n-1:
      cost_matrix[i][j] = dist[replan_agents[i].loc][avail_tasks[j].pickup_loc]

  matching = Hungarian_Method(cost_matrix)

  For (agent_i, task_j) in matching:
    task[task_j].status = agent_i
    agent[agent_i].current_task = task_j
    agent[agent_i].status = MOVING_TO_PICKUP

  // Path planning handled separately (CBS called from ShouldReplan/Path_Planning)
```

---

## 10. AssignTask — TA Methods

### 10.1 TA via TSP Solver (TA-Prioritized)

```
Assign_TA_TSP(agents, avail_tasks, config):
  task_sequences = SolveMTSP_LKH3(agents, avail_tasks)

  For each agent_i:
    agent[agent_i].task_sequence = task_sequences[agent_i]
    If len(agent[agent_i].task_sequence) > 0:
      first_task = agent[agent_i].task_sequence.pop_front()
      agent[agent_i].current_task = first_task
      agent[agent_i].status = MOVING_TO_PICKUP
      task[first_task].status = agent_i
    For task_j in agent[agent_i].task_sequence:
      task[task_j].status = agent_i
```

### 10.2 TA-Hybrid (TSP + two-group path planning each timestep)

TA-Hybrid does task assignment **once** via TSP (same as TA-Prioritized). But unlike
TA-Prioritized, path planning happens **every timestep** with two groups, and free agents
can swap task sequences via AMAPF. (Liu et al. 2019, Sec 5, Algorithm 1)

```
Assign_TA_Hybrid(replan_agents, avail_tasks, ct, config):
  // Task assignment is done ONCE at t=0 via TSP. No reassignment afterward.
  // (The min-cost max-flow swap happens during path planning, not here.)
  Assign_TA_TSP(all_agents, avail_tasks, config)


// TA-Hybrid's per-timestep path planning (called from Task_Assignment_And_Path_Planning)
// This replaces the generic ShouldReplan/Path_Planning for TA-Hybrid.
// (Liu et al. 2019, Algorithm 1 Lines 9-11)

Path_Planning_TA_Hybrid(ct, config):

  // --- Group 1: New task agents (free agents that just arrived at pickup) ---
  // "TA-Hybrid checks whether one or more free agents are at the pickup
  //  locations of their current tasks at or after their release times and
  //  are not executing them yet. If so, then each such agent turns into a
  //  task agent and is part of Group 1." (Sec 5.2)
  group1 = {}
  For agent_i in all_agents:
    If agent[agent_i].current_task != -1:
      task_j = agent[agent_i].current_task
      If agent[agent_i].status == FREE
         AND agent[agent_i].loc == task[task_j].pickup_loc
         AND t >= task[task_j].release_time:
        agent[agent_i].status = CARRYING  // becomes task agent
        group1.add(agent_i)

  // Plan paths for Group 1 from pickup → delivery via ICBS
  // "TA-Hybrid uses ICBS to plan the next sub-paths for all agents in
  //  Group 1 (and their dummy paths) simultaneously." (Sec 5.2)
  If |group1| > 0:
    Path_Planning_CBS(group1, ct, config)  // ICBS with dummy path goal test

  // --- Group 2: Free agents (not yet executing their current task) ---
  // "TA-Hybrid checks whether one or more agents do not yet execute their
  //  current tasks. If so, then each such agent is a free agent and part
  //  of Group 2." (Sec 5.3)
  // Triggered only when the set of free agents has changed or at t=0.
  // "if the set of free agents has changed or timestep = 0" (Algorithm 1, Line 10)
  If free_agent_set_changed OR t == 0:
    group2 = { agent i : agent[i].current_task != -1
               AND agent[i].status != CARRYING }

    If |group2| > 0:
      // Plan paths for Group 2 from current loc → pickup via min-cost max-flow
      // This may swap task sequences between agents:
      // "If an agent is assigned the current pickup location of a different
      //  agent, TA-Hybrid replaces its current task sequence with the task
      //  sequence of this different agent." (Sec 5.3)
      PlanPathsToPickup_AMAPF(group2, ct, config)
```

### 10.3 RMCA (Greedy Insertion + LNS, Coupled)

```
Assign_RMCA(agents, avail_tasks, ct, config):
  // Coupled: assignment and path planning happen together

  // Phase 1: Regret-based greedy insertion (Chen et al. 2021, Sec IV-C)
  // RMCA selects the task with maximum regret (= difference between
  // second-best and best marginal insertion cost) and assigns it to
  // the best robot. This provides look-ahead vs plain MCA.
  unassigned = avail_tasks.copy()

  While |unassigned| > 0:
    best_regret_task = -1
    best_regret = -INF

    For task_j in unassigned:
      // Find best and second-best insertion for task_j across all agents
      best1_cost = INF
      best2_cost = INF
      best1_agent = -1
      best1_pos = -1
      best1_path = null

      For agent_i in agents:
        For pos in 0..len(agent[agent_i].task_sequence):
          trial_sequence = agent[agent_i].task_sequence.insert(pos, task_j)

          ct.delete(agent_i)
          trial_path = Find_Path_For_Sequence(agent_i, trial_sequence, ct, config)
          ct.add(agent_i, agent[agent_i].path, agent[agent_i].last_endpoint, t)

          If trial_path != null:
            cost = Evaluate_Marginal_Cost(trial_path, agent_i)
            If cost < best1_cost:
              best2_cost = best1_cost
              best1_cost = cost
              best1_agent = agent_i
              best1_path = trial_path
              best1_pos = pos
            Else if cost < best2_cost:
              best2_cost = cost

      regret = best2_cost - best1_cost   // (Chen et al. 2021, Eq. 13-14)
      If regret > best_regret:
        best_regret = regret
        best_regret_task = task_j
        commit_agent = best1_agent
        commit_path = best1_path
        commit_pos = best1_pos

    If best_regret_task != -1 AND commit_agent != -1:
      agent[commit_agent].task_sequence.insert(commit_pos, best_regret_task)
      agent[commit_agent].path = commit_path.path
      agent[commit_agent].last_endpoint = commit_path.last_endpoint
      task[best_regret_task].status = commit_agent
      ct.update(commit_agent, commit_path.path, commit_path.last_endpoint, t)
      unassigned.remove(best_regret_task)
    Else:
      Break  // no feasible insertion found

  // Anytime improvement (if enabled) is called by Task_Assignment_And_Path_Planning
  // after this function returns — see Sec 2.1, Step 3.
```

### 10.4 Repeated Hungarian + LNS (LNS-PBS, LNS-wPBS)

```
Assign_TA_LNS(agents, avail_tasks, ct, config):
  // Phase 1: Build initial assignment via repeated Hungarian
  unassigned_tasks = avail_tasks.copy()
  For round in 1..ceil(|avail_tasks| / |agents|):
    batch = first min(|agents|, |unassigned_tasks|) tasks from unassigned_tasks
    matching = Hungarian_Method(agents, batch)
    For (agent_i, task_j) in matching:
      agent[agent_i].task_sequence.append(task_j)
      task[task_j].status = agent_i
      unassigned_tasks.remove(task_j)

  For agent_i in agents:
    If len(agent[agent_i].task_sequence) > 0 AND agent[agent_i].current_task == -1:
      next = agent[agent_i].task_sequence.pop_front()
      agent[agent_i].current_task = next
      agent[agent_i].status = MOVING_TO_PICKUP

  // Phase 2: LNS improvement of task sequences
  For iter in 1..MAX_LNS_ITERS:
    removed = LNS_Destroy(agents)
    LNS_Repair(removed, agents)
    If solution_improved:
      Accept
    Else:
      Reject and restore
```

---

## 11. Path_Planning — Dispatcher

```
Path_Planning(agents_to_plan, ct, config):
  // TA-Hybrid has its own dedicated two-group path planning
  If config.assign_method == LKH3_TSP_REASSIGN:
    Path_Planning_TA_Hybrid(ct, config)
    Return

  Switch config.mapf:

    DECOUPLED_PP:
      Path_Planning_Decoupled(agents_to_plan, ct, config)

    CBS:
      Path_Planning_CBS(agents_to_plan, ct, config)

    PBS:
      Path_Planning_PBS(agents_to_plan, ct, config)

    wPBS:
      Path_Planning_wPBS(agents_to_plan, ct, config)
```

---

## 12. Path Planning Methods

There are two layers of path planning functions:

1. **`Find_Path`** (pure) — Plans a path for one agent against a given constraint table.
   Does NOT modify ct or agent state. Returns `(path, last_endpoint)` or null.

2. **`Plan_Agent_Path`** (stateful wrapper) — Calls Find_Path, then updates agent state
   and the global ct. Used by decoupled PP and assignment functions.

CBS/PBS use Find_Path directly with their own constraint tables and only update
the global ct when a complete solution is found.

### 12.0 Find_Path — Pure Single-Agent Planner

```
Find_Path(agent_i, ct, config):
  // Plans a collision-free path for agent_i against ct.
  // Does NOT modify ct or agent state.
  // Returns (path, last_endpoint) or null.

  If agent[agent_i].current_task != -1:
    task_j = agent[agent_i].current_task

    Switch config.single_agent:

      STA_TASK_EP:
        // Plan pickup→delivery; last endpoint chosen via ChooseLastEndpoint.
        If agent[agent_i].status == MOVING_TO_PICKUP:
          pickup_path = STA(agent[agent_i].loc, t, task[task_j].pickup_loc, ct, agent_i)
          If pickup_path == null: Return null
          t_at_pickup = t + len(pickup_path) - 1
          delivery_path = STA(task[task_j].pickup_loc, t_at_pickup,
                              task[task_j].delivery_loc, ct, agent_i)
          If delivery_path == null: Return null
          path = pickup_path + delivery_path[1:]
        Else:  // CARRYING
          path = STA(agent[agent_i].loc, t, task[task_j].delivery_loc, ct, agent_i)
          If path == null: Return null
        endpoint = ChooseLastEndpoint(agent_i, config)
        // For TASK_ENDPOINT strategy, this returns delivery_loc.
        // For other strategies (ablation), it returns a different endpoint,
        // and a dummy path from delivery to that endpoint would be appended.
        If endpoint == task[task_j].delivery_loc:
          Return (path, endpoint)
        Else:
          t_at_delivery = t + len(path) - 1
          dummy = STA(task[task_j].delivery_loc, t_at_delivery, endpoint, ct, agent_i)
          If dummy == null: Return null
          Return (path + dummy[1:], endpoint)

      STA_NONTASK_EP:
        // Plan pickup→delivery→endpoint; endpoint chosen via ChooseLastEndpoint.
        If agent[agent_i].status == MOVING_TO_PICKUP:
          pickup_path = STA(agent[agent_i].loc, t, task[task_j].pickup_loc, ct, agent_i)
          If pickup_path == null: Return null
          t_at_pickup = t + len(pickup_path) - 1
          delivery_path = STA(task[task_j].pickup_loc, t_at_pickup,
                              task[task_j].delivery_loc, ct, agent_i)
          If delivery_path == null: Return null
          task_path = pickup_path + delivery_path[1:]
        Else:
          task_path = STA(agent[agent_i].loc, t, task[task_j].delivery_loc, ct, agent_i)
          If task_path == null: Return null

        // Append dummy path to chosen endpoint
        t_at_delivery = t + len(task_path) - 1
        endpoint = ChooseLastEndpoint(agent_i, config)
        dummy = STA(task[task_j].delivery_loc, t_at_delivery, endpoint, ct, agent_i)
        If dummy == null: Return null
        path = task_path + dummy[1:]
        Return (path, endpoint)

      MLA_SEQUENCE:
        goals = BuildGoalSequence(agent_i)
        endpoint = ChooseLastEndpoint(agent_i, config)
        If endpoint == goals[last]:
          // TASK_ENDPOINT: last goal IS the endpoint — no dummy needed
          path = MLA(agent[agent_i].loc, t, goals, ct, agent_i)
          If path == null: Return null
          Return (path, endpoint)
        Else:
          // FIXED_PARKING / FLEXIBLE: append endpoint to goal sequence
          goals.append(endpoint)
          path = MLA(agent[agent_i].loc, t, goals, ct, agent_i)
          If path == null: Return null
          Return (path, endpoint)

      SEQ_STA:
        // "Reserving dummy paths": each sub-path's goal test verifies that
        // a dummy path to the endpoint exists from the goal at the arrival time.
        // (Liu et al., AAMAS 2019, Sec 4)
        full_path = []
        curr_loc = agent[agent_i].loc
        curr_t = t
        endpoint = ChooseLastEndpoint(agent_i, config)

        all_tasks_to_plan = [agent[agent_i].current_task] + agent[agent_i].task_sequence

        For task_k in all_tasks_to_plan:
          p = STA_WithDummyCheck(curr_loc, curr_t, task[task_k].pickup_loc,
                                 task[task_k].release_time, endpoint, ct, agent_i)
          If p == null: Return null
          If len(full_path) > 0:
            full_path = full_path + p[1:]
          Else:
            full_path = p
          curr_t = curr_t + len(p) - 1
          curr_loc = task[task_k].pickup_loc

          d = STA_WithDummyCheck(curr_loc, curr_t, task[task_k].delivery_loc,
                                 0, endpoint, ct, agent_i)
          If d == null: Return null
          full_path = full_path + d[1:]
          curr_t = curr_t + len(d) - 1
          curr_loc = task[task_k].delivery_loc

        // Final dummy path (the only one the agent actually executes)
        dummy = STA(curr_loc, curr_t, endpoint, ct, agent_i)
        If dummy == null: Return null
        full_path = full_path + dummy[1:]
        Return (full_path, endpoint)

  Else:
    // No task — plan dummy path
    Return Find_Dummy_Path(agent_i, ct, config)


Find_Dummy_Path(agent_i, ct, config):
  // Pure version: returns (path, last_endpoint) or null. Does NOT modify ct.
  // Uses ChooseLastEndpoint to determine where to go.

  Switch config.deadlock:

    HOLDING_ENDPOINT:
      // If agent is at a safe location, stay. Otherwise move to chosen endpoint.
      If agent[agent_i].loc not in { task[j].delivery_loc : j in T, task[j].status != INT_MAX }:
        Return ([agent[agent_i].loc], agent[agent_i].loc)
      Else:
        endpoint = ChooseLastEndpoint(agent_i, config)
        dummy = STA(agent[agent_i].loc, t, endpoint, ct, agent_i)
        If dummy != null:
          Return (dummy, endpoint)
        Return null

    DUMMY_PATH:
      endpoint = ChooseLastEndpoint(agent_i, config)
      If endpoint == agent[agent_i].loc:
        Return ([agent[agent_i].loc], endpoint)
      dummy = STA(agent[agent_i].loc, t, endpoint, ct, agent_i)
      If dummy == null: Return null
      Return (dummy, endpoint)

    NO_AVOIDANCE:
      Return ([agent[agent_i].loc], agent[agent_i].loc)
```

### 12.1 Plan_Agent_Path — Stateful Wrapper (Decoupled PP)

```
Plan_Agent_Path(agent_i, ct, config):
  // Removes agent_i's old reservations, plans a new path, adds new reservations.
  // Updates agent state. Returns true on success, false on failure.

  // Step 1: Remove old reservations so agent doesn't block itself
  ct.delete(agent_i)

  // Step 2: Plan path against ct (which now has everyone else's paths)
  result = Find_Path(agent_i, ct, config)

  If result == null:
    // Planning failed — restore old reservations
    ct.add(agent_i, agent[agent_i].path, agent[agent_i].last_endpoint, t)
    Return false

  // Step 3: Apply result
  (new_path, new_last_endpoint) = result
  agent[agent_i].path = new_path
  agent[agent_i].last_endpoint = new_last_endpoint
  ct.add(agent_i, new_path, new_last_endpoint, t)
  Return true


Plan_Dummy_Path(agent_i, ct, config):
  // Convenience wrapper for dummy path planning.
  // Same delete-plan-add pattern.

  ct.delete(agent_i)

  result = Find_Dummy_Path(agent_i, ct, config)

  If result == null:
    ct.add(agent_i, agent[agent_i].path, agent[agent_i].last_endpoint, t)
    Return false

  (new_path, new_last_endpoint) = result
  agent[agent_i].path = new_path
  agent[agent_i].last_endpoint = new_last_endpoint
  ct.add(agent_i, new_path, new_last_endpoint, t)
  Return true


Path_Planning_Decoupled(agents_to_plan, ct, config):
  // Plan paths one agent at a time in priority order.
  // Each agent's new path is added to ct before the next agent plans.
  //
  // For TA-Prioritized: agents are ordered by LARGEST estimated execution
  // time first, so agents with longer task sequences have fewer constraints
  // (Liu et al. 2019, Sec 4: "giving priority to agents with larger
  //  estimated execution times").
  If config.assign_method == LKH3_TSP:
    Sort agents_to_plan by estimated_execution_time DESCENDING

  For agent_i in agents_to_plan:
    Plan_Agent_Path(agent_i, ct, config)
```

### 12.2 Conflict-Based Search (CBS) — CENTRAL, CENTRAL-fixed, TA-Hybrid(occupied)

```
Path_Planning_CBS(agents_to_plan, ct, config):
  // Step 1: Build base constraint table (agents NOT being replanned)
  base_ct = ct.copy_without(agents_to_plan)

  // Step 2: CBS high-level search
  root.cbs_constraints = {}     // per-agent CBS constraints (initially empty)
  root.paths = {}
  root.last_endpoints = {}
  For agent_i in agents_to_plan:
    result = Find_Path_CBS(agent_i, base_ct, root.cbs_constraints, config)
    root.paths[agent_i] = result.path
    root.last_endpoints[agent_i] = result.last_endpoint
  root.cost = Sum_Of_Costs(root.paths)

  OPEN = priority_queue sorted by cost
  OPEN.push(root)

  While OPEN not empty:
    node = OPEN.pop()

    conflict = FindConflict(node.paths)
    If conflict == null:
      // Solution found — update global ct with new paths
      For agent_i in agents_to_plan:
        ct.update(agent_i, node.paths[agent_i], node.last_endpoints[agent_i], t)
        agent[agent_i].path = node.paths[agent_i]
        agent[agent_i].last_endpoint = node.last_endpoints[agent_i]
      Return true

    // Branch: create two child nodes
    For each agent_k in {conflict.agent1, conflict.agent2}:
      child = node.copy()
      child.cbs_constraints.add(new_constraint_for(agent_k, conflict))
      result = Find_Path_CBS(agent_k, base_ct, child.cbs_constraints, config)
      If result != null:
        child.paths[agent_k] = result.path
        child.last_endpoints[agent_k] = result.last_endpoint
        child.cost = Sum_Of_Costs(child.paths)
        OPEN.push(child)

  Return false


Find_Path_CBS(agent_i, base_ct, cbs_constraints, config):
  // Low-level CBS planner for one agent.
  // Merges base_ct (paths of non-replanned agents) with CBS constraints.
  // Does NOT include paths of other replanning agents — those are handled
  // by the CBS high-level conflict detection and branching.

  merged_ct = base_ct.copy()
  merged_ct.add_cbs_constraints(agent_i, cbs_constraints)
  // cbs_constraints for agent_i are negative constraints: (vertex, time) or (edge, time)
  // that agent_i is forbidden from using. These are stored separately from
  // reservation-based constraints but checked the same way in is_valid_for.

  Return Find_Path(agent_i, merged_ct, config)


FindConflict(paths):
  For all pairs (i, j):
    For each timestep t_step:
      If paths[i][t_step] == paths[j][t_step]:
        Return VertexConflict(i, j, paths[i][t_step], t_step)
      If paths[i][t_step] == paths[j][t_step+1] AND paths[i][t_step+1] == paths[j][t_step]:
        Return EdgeConflict(i, j, paths[i][t_step], paths[i][t_step+1], t_step)
    // Also check endpoint holds: if agent i holds EP_i and agent j's path crosses it
    // after agent i arrives there (or vice versa)
  Return null
```

### 12.3 Priority-Based Search (PBS) — LNS-PBS

```
Path_Planning_PBS(agents_to_plan, ct, config):
  // Step 1: Build base constraint table (agents NOT being replanned)
  base_ct = ct.copy_without(agents_to_plan)

  // Step 2: Save old paths from previous iteration for modified PBS
  // (Xu et al. 2022, Sec IV-C: "it plans a time-minimal path for each
  //  agent that avoids the old paths of all other M-1 agents")
  old_paths = {}
  old_endpoints = {}
  For agent_i in agents_to_plan:
    old_paths[agent_i] = agent[agent_i].path.copy()
    old_endpoints[agent_i] = agent[agent_i].last_endpoint

  // Step 3: PBS high-level search
  root.priority_graph = empty DAG
  root.paths = {}
  root.last_endpoints = {}
  For agent_i in agents_to_plan:
    result = Find_Path_PBS(agent_i, base_ct, root, agents_to_plan,
                           old_paths, old_endpoints, config)
    root.paths[agent_i] = result.path
    root.last_endpoints[agent_i] = result.last_endpoint
  root.cost = Sum_Of_Costs(root.paths)

  // PBS uses depth-first search (Xu et al. 2022, Sec IV-C; Ma et al. 2019).
  // The number of expanded PT nodes is bounded by the max depth O(M^2).
  OPEN = stack  // DFS, not best-first
  OPEN.push(root)

  While OPEN not empty:
    node = OPEN.pop()  // DFS: last-in, first-out

    conflict = FindConflict(node.paths)
    If conflict == null:
      // Solution found — update global ct
      For agent_i in agents_to_plan:
        ct.update(agent_i, node.paths[agent_i], node.last_endpoints[agent_i], t)
        agent[agent_i].path = node.paths[agent_i]
        agent[agent_i].last_endpoint = node.last_endpoints[agent_i]
      Return true

    For (high, low) in [(conflict.agent1, conflict.agent2),
                        (conflict.agent2, conflict.agent1)]:
      child = node.copy()
      child.priority_graph.add_edge(high -> low)
      If not child.priority_graph.has_cycle():
        affected = topological_descendants(child.priority_graph, low)
        For agent_k in affected (in topological order):
          result = Find_Path_PBS(agent_k, base_ct, child, agents_to_plan,
                                 old_paths, old_endpoints, config)
          child.paths[agent_k] = result.path
          child.last_endpoints[agent_k] = result.last_endpoint
        child.cost = Sum_Of_Costs(child.paths)
        OPEN.push(child)

  Return false


Find_Path_PBS(agent_i, base_ct, node, agents_to_plan,
              old_paths, old_endpoints, config):
  // Modified PBS low-level for LNS-PBS (Xu et al. 2022, Sec IV-C).
  // Agent_i plans avoiding:
  //   1. base_ct (non-replanned agents)
  //   2. NEW paths of higher-priority replanning agents
  //   3. OLD paths of all other replanning agents (no priority relation)

  planning_ct = base_ct.copy()
  For agent_j in agents_to_plan:
    If agent_j == agent_i:
      Continue
    If node.priority_graph has path (j -> i):
      // agent_j has higher priority — use its NEW path
      planning_ct.add(agent_j, node.paths[agent_j], node.last_endpoints[agent_j], t)
    Else:
      // agent_j has no priority over agent_i — use its OLD path
      planning_ct.add(agent_j, old_paths[agent_j], old_endpoints[agent_j], t)

  Return Find_Path(agent_i, planning_ct, config)
```

### 12.4 Windowed PBS (wPBS) — LNS-wPBS

```
Path_Planning_wPBS(agents_to_plan, ct, config):
  // LNS-wPBS uses STANDARD PBS, NOT the modified version with old paths
  // (Xu et al. 2022, Sec IV-E: "wPBS uses the original low level of PBS
  //  instead of our modified version, i.e., it does not consider the old
  //  paths of the agents.")
  If periodic_replan_trigger(t, config.replan_window):
    Path_Planning_Standard_PBS(all_agents, ct, config)
  Else:
    Path_Planning_Standard_PBS(agents_to_plan, ct, config)


Path_Planning_Standard_PBS(agents_to_plan, ct, config):
  // Standard PBS without old-path avoidance (used by LNS-wPBS).
  // Same structure as Path_Planning_PBS (Sec 12.3) but Find_Path_PBS
  // only adds higher-priority agents' NEW paths — no old paths.
  base_ct = ct.copy_without(agents_to_plan)

  root.priority_graph = empty DAG
  root.paths = {}
  root.last_endpoints = {}
  For agent_i in agents_to_plan:
    result = Find_Path_Standard_PBS(agent_i, base_ct, root, agents_to_plan, config)
    root.paths[agent_i] = result.path
    root.last_endpoints[agent_i] = result.last_endpoint

  OPEN = stack  // DFS
  OPEN.push(root)

  While OPEN not empty:
    node = OPEN.pop()
    conflict = FindConflict(node.paths)
    If conflict == null:
      For agent_i in agents_to_plan:
        ct.update(agent_i, node.paths[agent_i], node.last_endpoints[agent_i], t)
        agent[agent_i].path = node.paths[agent_i]
        agent[agent_i].last_endpoint = node.last_endpoints[agent_i]
      Return true

    For (high, low) in [(conflict.agent1, conflict.agent2),
                        (conflict.agent2, conflict.agent1)]:
      child = node.copy()
      child.priority_graph.add_edge(high -> low)
      If not child.priority_graph.has_cycle():
        affected = topological_descendants(child.priority_graph, low)
        For agent_k in affected (in topological order):
          result = Find_Path_Standard_PBS(agent_k, base_ct, child, agents_to_plan, config)
          child.paths[agent_k] = result.path
          child.last_endpoints[agent_k] = result.last_endpoint
        OPEN.push(child)
  Return false


Find_Path_Standard_PBS(agent_i, base_ct, node, agents_to_plan, config):
  // Standard PBS low-level: only higher-priority agents' NEW paths.
  // No old paths considered (unlike modified Find_Path_PBS in Sec 12.3).
  planning_ct = base_ct.copy()
  For agent_j in agents_to_plan:
    If agent_j != agent_i AND node.priority_graph has path (j -> i):
      planning_ct.add(agent_j, node.paths[agent_j], node.last_endpoints[agent_j], t)
  Return Find_Path(agent_i, planning_ct, config)


periodic_replan_trigger(t, w):
  Return (t % w == 0)
```

### 12.5 Path_Planning_All — Used After TA Initial Assignment

```
Path_Planning_All(ct, config):
  Switch config.mapf:
    DECOUPLED_PP:
      For agent_i in all_agents:
        Plan_Agent_Path(agent_i, ct, config)
    CBS:
      Path_Planning_CBS(all_agents, ct, config)
    PBS:
      Path_Planning_PBS(all_agents, ct, config)
    wPBS:
      Path_Planning_wPBS(all_agents, ct, config)
```

---

## 13. Single-Agent Search Algorithms

### 13.1 Space-Time A* (STA*)

```
STA(start_loc, start_time, goal_loc, ct, agent_i):
  // A* search in space-time graph for agent_i.
  // Uses ct.is_valid_for(agent_i, ...) to check against other agents' reservations.
  // Returns: path as list of vertices, or null on failure.

  OPEN = priority queue by f = g + h
  start_state = (start_loc, start_time)
  g[start_state] = 0
  f[start_state] = dist[start_loc][goal_loc]
  OPEN.push(start_state, f[start_state])
  parent = {}

  While OPEN not empty:
    (v, t_curr) = OPEN.pop()

    If v == goal_loc:
      If ct.can_hold(agent_i, v, t_curr):
        Return ReconstructPath(parent, start_state, (v, t_curr))

    For (v', t') in { (u, t_curr+1) : (v,u) in E } ∪ { (v, t_curr+1) }:
      If ct.is_valid_for(agent_i, v, v', t_curr, t'):
        new_g = g[(v, t_curr)] + 1
        If new_g < g.get((v', t'), INF):
          g[(v', t')] = new_g
          f[(v', t')] = new_g + dist[v'][goal_loc]
          parent[(v', t')] = (v, t_curr)
          OPEN.push((v', t'), f[(v', t')])

  Return null
```

### 13.2 Multi-Label A* (MLA*)

```
MLA(start_loc, start_time, goals[], ct, agent_i):
  // A* with labels for ordered goal sequence, planning for agent_i.

  K = len(goals)
  start_state = (start_loc, start_time, 0)
  g[start_state] = 0
  h_val = MLA_Heuristic(start_loc, 0, goals)
  OPEN = priority queue
  OPEN.push(start_state, g[start_state] + h_val)
  parent = {}

  While OPEN not empty:
    (v, t_curr, k) = OPEN.pop()

    If k == K:
      If ct.can_hold(agent_i, v, t_curr):
        Return ReconstructPath(parent, start_state, (v, t_curr, k))
      Continue

    For (v', t') in { (u, t_curr+1) : (v,u) in E } ∪ { (v, t_curr+1) }:
      If ct.is_valid_for(agent_i, v, v', t_curr, t'):
        If v' == goals[k]:
          k' = k + 1
        Else:
          k' = k

        new_state = (v', t', k')
        new_g = g[(v, t_curr, k)] + 1
        If new_g < g.get(new_state, INF):
          g[new_state] = new_g
          h = MLA_Heuristic(v', k', goals)
          parent[new_state] = (v, t_curr, k)
          OPEN.push(new_state, new_g + h)

  Return null


MLA_Heuristic(v, k, goals):
  If k >= len(goals): Return 0
  h = dist[v][goals[k]]
  For k' in k..len(goals)-2:
    h += dist[goals[k']][goals[k'+1]]
  Return h
```

### 13.3 Safe Interval Path Planning (SIPP) — optional faster alternative

```
SIPP(start_loc, start_time, goal_loc, ct, agent_i):
  // Same interface as STA but uses safe intervals for efficiency.
  // Safe interval = maximal contiguous range of timesteps when vertex is unoccupied.

  safe_intervals = ct.compute_safe_intervals(agent_i)

  // States: (vertex, safe_interval_index)
  // [Implementation follows standard SIPP algorithm]
  // Returns path or null.
  ...
```

---

## 14. Helper Functions

### 14.1 Build Goal Sequence for an Agent

```
BuildGoalSequence(agent_i):
  goals = []

  If agent[agent_i].current_task != -1:
    task_j = agent[agent_i].current_task
    If agent[agent_i].status == MOVING_TO_PICKUP:
      goals.append(task[task_j].pickup_loc)
    goals.append(task[task_j].delivery_loc)

  For task_k in agent[agent_i].task_sequence:
    goals.append(task[task_k].pickup_loc)
    goals.append(task[task_k].delivery_loc)

  Return goals
```

### 14.2 Filter Tasks by Endpoint Conflicts

```
FilterEndpointConflicts(avail_tasks, agent_i):
  occupied_endpoints = { agent[j].last_endpoint : j != agent_i, agent[j] is at end of path }

  Return { task j in avail_tasks :
    task[j].pickup_loc not in occupied_endpoints AND
    task[j].delivery_loc not in occupied_endpoints }
```

### 14.3 Choose Last Endpoint

Unified endpoint chooser. All algorithms call `ChooseLastEndpoint` which dispatches
based on `config.endpoint_strategy`. This makes it easy to swap strategies for ablation.

The four strategies correspond to the framework paper (Sec 4.4.2):
- **TASK_ENDPOINT**: ℓ_i = g[K] (delivery loc) — zero-length dummy path. (Choice 1)
- **FIXED_PARKING**: ℓ_i = agent's initial vertex (parking location). (Choice 2)
- **FLEXIBLE_STRICT**: ℓ_i chosen from endpoints, excluding assigned + goals + old dummies. (Choice 3, strict)
- **FLEXIBLE_PAIRWISE**: ℓ_i chosen from endpoints, excluding assigned only. (Choice 3, relaxed)

```
ChooseLastEndpoint(agent_i, config, old_dummy_endpoints=null):
  // Unified dispatcher — returns the last endpoint for agent_i's path.
  // Change config.endpoint_strategy to swap strategies for ablation.

  Switch config.endpoint_strategy:

    TASK_ENDPOINT:
      // Choice 1: hold the task's final goal (delivery loc).
      // Used by: TP, TPTS, CENTRAL, CENTRAL-fixed, HBH-MLA*, RMCA.
      // (Ma et al. 2017; framework paper Sec 4.4.2 Choice 1)
      If agent[agent_i].current_task != -1:
        Return task[agent[agent_i].current_task].delivery_loc
      Else:
        // No task — fall through to flexible selection for a safe endpoint
        Return ChooseFlexibleEndpoint(agent_i,
                 { agent[j].last_endpoint : j != agent_i },
                 agent[agent_i].loc)

    FIXED_PARKING:
      // Choice 2: always return the agent's fixed parking location (initial vertex).
      // Used by: TA-Prioritized, TA-Hybrid.
      // (Liu et al. 2019, Sec 4; framework paper Sec 4.4.2 Choice 2)
      Return agent[agent_i].initial_loc

    FLEXIBLE_STRICT:
      // Choice 3 (strict): exclude assigned + all goal locs + old dummies.
      // Used by: LNS-PBS.
      // (Xu et al. 2022, Sec IV-B)
      already_assigned = { agent[j].last_endpoint : j != agent_i,
                           agent[j].last_endpoint already chosen this iteration }
      all_goal_locs = { loc : task k in T, task[k].status != INT_MAX,
                        loc in task[k].goals }
      old_others = { old_dummy_endpoints[j] : j != agent_i }
      forbidden = already_assigned ∪ all_goal_locs ∪ old_others
      last_goal = last element of agent[agent_i].goal_sequence (before dummy)
      Return ChooseFlexibleEndpoint(agent_i, forbidden, last_goal)

    FLEXIBLE_PAIRWISE:
      // Choice 3 (relaxed): exclude assigned dummies only (pairwise different).
      // Used by: LNS-wPBS.
      // (Xu et al. 2022, Sec IV-E)
      already_assigned = { agent[j].last_endpoint : j != agent_i,
                           agent[j].last_endpoint already chosen this iteration }
      forbidden = already_assigned
      last_goal = last element of agent[agent_i].goal_sequence (before dummy)
      Return ChooseFlexibleEndpoint(agent_i, forbidden, last_goal)


ChooseFlexibleEndpoint(agent_i, forbidden, proximity_ref):
  // Generic flexible endpoint chooser (internal).
  // Returns the closest admissible endpoint to proximity_ref.
  candidates = { e in endpoints : e not in forbidden }
  Return argmin e in candidates: dist[proximity_ref][e]
```

### 14.4 TimeToReach — Used by TPTS Swap Condition

```
TimeToReach(path, target_loc):
  // Return the timestep offset at which the path first visits target_loc.
  // Used by TPTS to compare pickup arrival times (Ma et al. 2017, Alg 2 Line 26).
  For i in 0..len(path)-1:
    If path[i] == target_loc:
      Return i
  Return INF  // target not on path
```

### 14.5 STA_WithDummyCheck — Used by TA-Prioritized

```
STA_WithDummyCheck(start_loc, start_time, goal_loc, release_time,
                   dummy_endpoint, ct, agent_i):
  // STA* with modified goal test for "reserving dummy paths"
  // (Liu et al. 2019, Sec 4).
  // A node (x, t) is accepted as a goal iff:
  //   (1) x == goal_loc
  //   (2) t >= release_time
  //   (3) STA*(x, t, dummy_endpoint, ct, agent_i) succeeds
  // If the goal test at (3) fails, the search continues to find
  // a later arrival time where a dummy path exists.

  // Identical to STA* (Sec 13.1) except the goal test is:
  GoalTest(v, t_curr):
    If v != goal_loc: Return false
    If t_curr < release_time: Return false
    dummy = STA(v, t_curr, dummy_endpoint, ct, agent_i)
    If dummy == null: Return false
    If not ct.can_hold(agent_i, dummy_endpoint, t_curr + len(dummy) - 1):
      Return false
    Return true

  // Run STA* with this GoalTest; return path or null.
```

### 14.6 End Condition

```
End():
  Switch config.mode:
    OFFLINE:
      Return all tasks j have task[j].status == INT_MAX
    ONLINE, SEMI_ONLINE:
      Return all tasks j have task[j].status == INT_MAX AND no more tasks will arrive
```

### 14.7 Compute Metrics

```
ComputeMetrics():
  total_service = 0
  For each task j:
    service_time = completion_time[j] - task[j].release_time
    total_service += service_time
  avg_service = total_service / num_tasks

  makespan = max over all tasks j: completion_time[j]

  throughput(t) = |{ task j : completion_time[j] in [t-99, t] }|

  Return { avg_service, makespan, throughput }
```

---

## 15. ShouldReplan — When to Trigger Path Planning

```
ShouldReplan(config):
  // For TP and TPTS, path planning is done inside Task_Assignment via
  // Plan_Agent_Path. For all other algorithms, path planning is separate.

  Switch config:
    HBH_MLA:
      Return assignment_just_happened      // after centralized greedy assigns, plan paths

    CENTRAL, CENTRAL_FIXED:
      Return newly_occupied_agents_exist OR assignment_just_happened

    TA_PRIORITIZED:
      Return assignment_just_happened      // at t=0, after TSP assigns sequences, plan all paths

    TA_HYBRID:
      Return true  // TA-Hybrid always calls Path_Planning_TA_Hybrid each timestep
                   // (it handles Group 1/Group 2 logic internally)

    wPBS:
      Return assignment_just_happened OR periodic_replan_trigger(t, config.replan_window)

    Default:
      Return false


GetAgentsToReplan(config):
  Switch config:
    HBH_MLA:
      Return { agent i : agent[i].current_task != -1 OR agent[i].status == FREE }
      // assigned agents get MLA* paths; unassigned free agents get dummy paths

    CENTRAL:
      Return all_agents

    CENTRAL_FIXED:
      If newly_occupied_agents_exist:
        Return newly_occupied_agents
      Else:
        Return all_free_agents

    TA_PRIORITIZED:
      Return all_agents                    // at t=0, plan paths for all agents

    TA_HYBRID:
      Return all_agents  // Path_Planning_TA_Hybrid splits into Group 1/2 internally

    wPBS:
      Return all_agents

    Default:
      Return agents needing replanning
```

---

## 16. LNS Subroutines

There are two distinct LNS procedures used by different algorithms:
- **RMCA_Destroy** (Chen et al. 2021, Sec IV-D): destroy random / destroy worst / destroy multiple
- **LNS_Destroy** (Xu et al. 2022, Sec IV-A): Shaw removal (spatial+temporal relatedness)

Similarly for repair:
- **RMCA**: re-assigns destroyed tasks via full RMCA (coupled with path planning)
- **LNS-PBS/wPBS**: regret-based re-insertion using estimated costs (decoupled)

### 16.1 RMCA Destroy Strategies (Chen et al. 2021, Sec IV-D)

```
RMCA_Destroy(agents, group_size):
  method = random_choice(DESTROY_RANDOM, DESTROY_WORST, DESTROY_MULTIPLE)

  Switch method:
    DESTROY_RANDOM:
      // "randomly selects a group of tasks from all assigned tasks"
      Return random_sample(all assigned tasks, group_size)

    DESTROY_WORST:
      // "randomly selects a group of tasks from the agent with the worst TTD"
      worst_agent = argmax agent_i: total_travel_delay(agent_i)
      Return random_sample(tasks of worst_agent, group_size)

    DESTROY_MULTIPLE:
      // "selects a group of agents that have the worst sum of TTD,
      //  then randomly destroys one task from each agent"
      worst_agents = top-group_size agents by total_travel_delay
      Return { random task from each agent in worst_agents }
```

### 16.2 LNS-PBS/wPBS Destroy + Repair (Xu et al. 2022, Sec IV-A)

```
LNS_Destroy(agents):
  // Shaw removal: spatial + temporal relatedness
  // (Xu et al. 2022, Sec IV-A)
  method = random_choice(RANDOM, WORST, RELATED)

  Switch method:
    RANDOM:
      Return random sample of tasks from all agent sequences
    WORST:
      Return tasks with highest marginal cost increase
    RELATED:
      Pick a random task; return nearby tasks (by distance)


LNS_Repair(removed_tasks, agents):
  While |removed_tasks| > 0:
    best_regret = -INF
    best_task = -1
    For task_j in removed_tasks:
      costs = []
      For agent_i in agents:
        For pos in all valid insertion positions:
          c = compute_insertion_cost(task_j, agent_i, pos)
          costs.append(c)
      sort(costs)
      regret = costs[1] - costs[0]
      If regret > best_regret:
        best_regret = regret
        best_task = task_j
        best_agent, best_pos = argmin insertion

    agent[best_agent].task_sequence.insert(best_pos, best_task)
    task[best_task].status = best_agent
    removed_tasks.remove(best_task)
```

### 16.3 REALPATH_LNS_IMP — Generic Anytime Improvement

A generic LNS-based anytime improvement that uses **real (collision-aware) path
costs** to evaluate solutions. Can be applied to any algorithm's output by plugging
in appropriate destroy and repair functions.

Originally proposed for RMCA (Chen et al. 2021, Algorithm 3):
> *"After finding an initial solution based on RMCA, we make use of an anytime
> improvement strategy on the solution. This strategy is based on the concept
> of Large Neighbourhood Search (LNS). As shown in Algorithm 3, the algorithm
> will continuously destroy some assigned tasks from the current solution and
> reassign these tasks using RMCA. If a better solution is found, we adopt the
> new solution, and otherwise we keep the current solution. We keep destroying
> and re-assigning until time out."* (Sec IV-D)

The structure is algorithm-agnostic: destroy some tasks, re-assign them (with
real path planning), accept if improved.

**Offline vs. Online usage:**

- **Offline (one-shot):** Called once after the initial solution. Runs until a
  wall-clock time limit (e.g., 60s). Iterates destroy/repair over all assigned tasks.

- **Online (lifelong):** Called every timestep when new tasks arrive. Operates on
  all released-but-not-yet-picked-up tasks (not just newly arrived ones):
  > *"At each timestep, after adding newly released tasks to the unassigned task
  > set P_u, the system performs RMCA(r) on current assignments set A, and runs
  > the anytime improvement process on all released tasks that are not yet picked
  > up. The RMCA(r) uses the anytime improvement strategy of destroy random with
  > a group size of 5. As the anytime improvement triggers at every timestep when
  > new tasks arrive, and involves all released yet unpicked up tasks, we set the
  > improvement time as 1 second in each run."* (Sec V-B)

```
REALPATH_LNS_IMP(agents, ct, config, time_limit, destroy_fn, repair_fn,
                 eligible_tasks=null):
  // Generic anytime improvement using real (collision-aware) path costs.
  //
  // Parameters:
  //   agents:         all agents with current assignments and paths
  //   ct:             global constraint table (modified in-place)
  //   config:         algorithm configuration
  //   time_limit:     wall-clock time budget (e.g., 60s offline, 1s online)
  //   destroy_fn:     function(agents, group_size, eligible) → set of removed tasks
  //                   e.g., RMCA_Destroy (Sec 16.1) or LNS_Destroy (Sec 16.2)
  //   repair_fn:      function(agents, removed_tasks, ct, config) → re-assigns tasks
  //                   e.g., Reassign_RMCA (coupled, regret-based)
  //                         or any other assignment+planning procedure
  //   eligible_tasks: which tasks can be destroyed and re-assigned.
  //                   If null, defaults to all assigned-but-not-yet-picked-up tasks.
  //                   Online: "all released tasks that are not yet picked up" (Sec V-B)
  //                   Offline: all assigned tasks
  //
  // The repair_fn must update agent task sequences, paths, and ct.

  If eligible_tasks == null:
    eligible_tasks = { task j : task[j].status >= 0        // assigned to some agent
                       AND task[j].status != INT_MAX       // not yet completed
                       AND agent[task[j].status].status != CARRYING }  // not picked up

  While runtime < time_limit:
    // Save current state for rollback
    saved_state = snapshot(agents, ct)

    // Destroy: remove a subset of eligible (not-yet-picked-up) tasks
    removed = destroy_fn(agents, group_size, eligible_tasks)
    For task_j in removed:
      Remove task_j from its agent's task_sequence
      task[task_j].status = -1  // mark unassigned

    // Repair: re-assign and re-plan collision-free paths for removed tasks
    repair_fn(agents, removed, ct, config)

    // Accept or reject based on real path cost
    If cost(agents) <= cost(saved_state):
      Accept new solution
      // Update eligible_tasks to reflect new assignments
    Else:
      Restore saved_state  // rollback agents, ct
```

**Example usage by different algorithms:**

```
// RMCA offline (Chen et al. 2021): 60s budget, RMCA destroy + repair
REALPATH_LNS_IMP(agents, ct, config, time_limit=60s,
                 destroy_fn=RMCA_Destroy,        // Sec 16.1
                 repair_fn=Reassign_RMCA)         // regret-based coupled insertion

// RMCA online (Chen et al. 2021, Sec V-B): 1s per timestep, destroy_random, group_size=5
REALPATH_LNS_IMP(agents, ct, config, time_limit=1s,
                 destroy_fn=RMCA_Destroy,        // with DESTROY_RANDOM, group_size=5
                 repair_fn=Reassign_RMCA,
                 eligible_tasks=all_released_not_picked_up)

// Applied to TP/HBH-MLA* output for post-hoc improvement:
REALPATH_LNS_IMP(agents, ct, config, time_limit=10s,
                 destroy_fn=LNS_Destroy,          // Sec 16.2 (Shaw removal)
                 repair_fn=Reassign_Decoupled)     // any decoupled assignment + PP

// Applied to LNS-PBS output with different destroy strategy:
REALPATH_LNS_IMP(agents, ct, config, time_limit=5s,
                 destroy_fn=RMCA_Destroy,
                 repair_fn=Reassign_TA_LNS)        // repeated Hungarian + LNS
```

---

## 17. Constraint Table Lifecycle Summary

```
WHERE ct IS INITIALIZED:
  Main() → ct = empty; then ct.add() for each agent's initial trivial path

WHERE ct IS MODIFIED (add/delete/update):

  Plan_Agent_Path(agent_i, ct, config):
    ct.delete(agent_i)                    // remove old
    result = Find_Path(agent_i, ct, ...)  // plan against remaining
    ct.add(agent_i, new_path, ...)        // add new
    (on failure: ct.add old path back)

  Plan_Dummy_Path(agent_i, ct, config):
    Same delete-plan-add pattern as Plan_Agent_Path

  Path_Planning_CBS(agents_to_plan, ct, config):
    base_ct = ct.copy_without(agents_to_plan)   // read-only copy for planning
    ... CBS search uses base_ct + CBS constraints ...
    On solution found:
      For agent_i in agents_to_plan:
        ct.update(agent_i, solution_path, ...)   // batch update global ct

  Path_Planning_PBS(agents_to_plan, ct, config):
    base_ct = ct.copy_without(agents_to_plan)   // read-only copy for planning
    ... PBS search uses base_ct + priority constraints ...
    On solution found:
      For agent_i in agents_to_plan:
        ct.update(agent_i, solution_path, ...)   // batch update global ct

  Assign_TPTS (swap logic):
    ct.delete(agent_j)                    // remove displaced agent's path
    Plan_Agent_Path(agent_i, ct, ...)     // plan stealer's path (does delete+add)
    On swap failure:
      ct.delete(agent_i)                  // remove stealer's path if it was added
      ct.add(agent_j, saved_path, ...)    // restore displaced agent's path

  Assign_RMCA (coupled):
    ct.delete(agent_i)                    // temporarily remove for trial
    ... plan trial path against ct ...
    ct.add(agent_i, old_path, ...)        // restore after trial
    On best found:
      ct.update(agent_i, best_path, ...)  // commit best

WHERE ct IS READ (not modified):
  Find_Path(agent_i, ct, config):
    Calls STA/MLA which use ct.is_valid_for() and ct.can_hold()

  Find_Path_CBS / Find_Path_PBS:
    Build merged_ct or planning_ct from base_ct + algorithm constraints
    Call Find_Path with that ct
```

---

## 18. Algorithm Instantiation Summary

| Algorithm | Call Sequence |
|-----------|--------------|
| **TP** | `Main` → `GetReleasedTasks(ONLINE)` → `Assign_Decoupled_Greedy` → `Plan_Agent_Path` (delete/Find_Path(STA_TASK_EP)/add) + `Plan_Dummy_Path` |
| **TPTS** | `Main` → `GetReleasedTasks(ONLINE)` → `Assign_TPTS` → `Plan_Agent_Path` + swap (delete/add/restore) + `Plan_Dummy_Path` |
| **CENTRAL** | `Main` → `GetReleasedTasks(ONLINE)` → `Assign_Hungarian` → `Path_Planning_CBS` (copy_without → CBS search with Find_Path → batch update) |
| **CENTRAL-fixed** | `Main` → `GetReleasedTasks(ONLINE)` → `Assign_Hungarian` → `Path_Planning_CBS` (same as CENTRAL) |
| **HBH-MLA*** | `Main` → `GetReleasedTasks(ONLINE)` → `Assign_Centralized_Greedy` (assignment only) → `Path_Planning_Decoupled` → `Plan_Agent_Path` (delete/Find_Path(MLA_SEQUENCE)/add) per agent |
| **TA-Prioritized** | `Main` → `Assign_TA_TSP` (once) → `Path_Planning_Decoupled` → `Plan_Agent_Path` (delete/Find_Path(SEQ_STA)/add) per agent |
| **TA-Hybrid** | `Main` → `Assign_TA_TSP` (once at t=0) → every timestep: `Path_Planning_TA_Hybrid` (Group 1: ICBS pickup→delivery; Group 2: AMAPF current→pickup with sequence swaps) |
| **RMCA** | `Main` → `Task_Assignment_And_Path_Planning` → `Assign_RMCA` (regret-based coupled insertion) → `REALPATH_LNS_IMP` (anytime, config-enabled) |
| **LNS-PBS** | `Main` → `Assign_TA_LNS` → `Path_Planning_PBS` (copy_without → PBS search with Find_Path(MLA_SEQUENCE) → batch update) |
| **LNS-wPBS** | `Main` → `Assign_TA_LNS` → `Path_Planning_wPBS` (copy_without → PBS + periodic replan → batch update) |
