# Algorithm Step-by-Step Summaries with Original Paper Quotes

Each algorithm is summarized step-by-step. Every step quotes from the **original paper** that proposed the algorithm.

---

## 1. TP (Token Passing)

**Paper:** Ma et al., "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks," AAMAS 2017.

**Step 1. Initialize the token.**
> *"The system initializes the token with the trivial paths where all agents rest in their initial locations."* (Sec 4.1)

**Step 2. Release tasks online each timestep.**
> *"In each timestep, the system adds all new tasks, if any, to the task set."* (Sec 4.1)

**Step 3. Free agents request the token one at a time.**
> *"Any agent that has reached the end of its path in the token requests the token once per timestep."* (Sec 4.1)

**Step 4. Filter tasks by endpoint conflicts.**
> *"The agent with the token chooses a task from the task set such that no path of other agents in the token ends in the pickup or delivery location of the task."* (Sec 4.1, Algorithm 1 Line 7)

**Step 5. Assign the closest task using greedy heuristic.**
> *"The selected task is one with minimum h-value, defined as the Manhattan distance between the agent's current location and the pickup location."* (Sec 4.1)

**Step 6. Plan path via sequential A\* calls.**
> *"The agent computes a path from its current location to the pickup location using an A\* algorithm, and then a path from the pickup location to the delivery location, again via A\*."* (Sec 4.1)

**Step 7. Paths avoid collisions via the token (constraint table).**
> *"The paths it finds must not collide with paths in the token inherited from the previous agent, where the token contains the paths already determined for other agents."* (Sec 4.1)

**Step 8. Handle free agents with no task — stay or move to endpoint.**
> *"If the agent is not in the delivery location of a task in the task set, then it updates its path in the token with the trivial path where it rests in its current location. Otherwise, to avoid deadlocks, it calls function Path2 to update its path in the token with a cost-minimal path that moves from its current location to an endpoint."* (Sec 4.1)

**Step 9. All agents move one step.**
> *"Finally, the agent returns the token to the system and moves along its path in the token."* (Sec 4.1)

**Completeness:**
> *"Theorem 3. All well-formed MAPD instances are solvable, and TP solves them."* (Sec 4.1)

---

## 2. TPTS (Token Passing with Task Swaps)

**Paper:** Ma et al., "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks," AAMAS 2017.

**Step 1. Same initialization and task release as TP.**

**Step 2. Task set includes all unexecuted tasks (not just unassigned).**
> *"Token Passing with Task Swaps (TPTS) is similar to TP except that its task set now contains all unexecuted tasks, rather than only all tasks that have no agents assigned."* (Sec 4.2)

**Step 3. Agent can steal tasks from non-carrying agents.**
> *"An agent with the token can assign itself not only to a task that has no agent assigned but also to a task that is already assigned another agent as long as that agent is still moving to the pickup location of the task."* (Sec 4.2)

**Step 4. Swap condition: new agent must be closer.**
> *"This might be beneficial when the former agent can move to the pickup location of the task in fewer timesteps than the latter agent."* (Sec 4.2)

**Step 5. Displaced agent recursively tries to find a new task.**
> *"The former agent therefore sends the token to the latter agent so that the latter agent can try to assign itself to a new task."* (Sec 4.2)

**Step 6. Rollback on failure.**
> *"In all other cases, agent a_i reverses all changes to the paths in the token, task set, and agent assignments and then considers the next task."* (Sec 4.2)

**Completeness:**
> *"Theorem 5. TPTS solves all well-formed MAPD instances."* (Sec 4.2)

---

## 3. CENTRAL

**Paper:** Ma et al., "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks," AAMAS 2017.

**Step 1. Every timestep, assign endpoints to all agents.**
> *"In each timestep, CENTRAL first assigns endpoints to all agents and then solves the resulting MAPF instance to plan paths for all agents from their current locations to their assigned endpoints simultaneously."* (Sec 5)

**Step 2. Use Hungarian method for assignment.**
> *"CENTRAL assigns each free agent an endpoint in X to satisfy all constraints. It uses the Hungarian Method for this purpose."* (Sec 5)

**Step 3. Use CBS for joint path planning.**
> *"CENTRAL uses the optimally effective MAPF algorithm Conflict-Based Search to plan collision-free paths for all agents from their current locations to their assigned endpoints simultaneously."* (Sec 5)

**Step 4. Plan in two groups for efficiency.**
> *"First, it plans paths for all agents that become occupied in the current timestep to their assigned endpoints... Then, it plans paths for all free agents to their assigned endpoints."* (Sec 5)

**Step 5. All agents move one step; repeat.**
> *"Finally, all agents move along their paths for one timestep and the procedure repeats."* (Sec 5)

**Completeness:**
> *"We want CENTRAL to be reasonably efficient and effective but do not require that it is optimally effective or even solves all well-formed MAPD instances."* (Sec 5)

---

## 3b. CENTRAL (fixed)

**Paper:** Ma, "Target Assignment and Path Planning for Navigation Tasks with Teams of Agents," Ph.D. thesis, USC, 2020 (Chapter 6, Section 6.5.3).

CENTRAL-fixed is described as an extension of CENTRAL in Section 6.5.3 of Ma's dissertation. The key difference is the trigger frequency: CENTRAL-fixed does not reassign every timestep.

**Step 1. Same two-phase task/endpoint-assignment procedure as CENTRAL.**
> *"At each time step, CENTRAL executes the task/endpoint-assignment procedure in two phases to assign endpoints to agents."* (Sec 6.5.1)

**Step 2. But trigger assignment only on events, not every timestep.**
> *"Task assignment can be changed so that new tasks that have just been added to the system or agents that have just become free can be taken into account at each time step."* (Sec 6.5.3, Extension 2)

The dissertation describes how the second phase of the task/endpoint-assignment procedure can be limited to event-driven triggers:
> *"the task/endpoint-assignment procedure can consider only unassigned tasks and free agents with no task assigned and thus do not reassign tasks to agents that have already been assigned tasks. This can make CENTRAL less effective but makes it more efficient because the target-assignment and MAPF problems are all smaller."* (Sec 6.5.3, Extension 4)

**Step 3. Use Hungarian method for assignment (same as CENTRAL).**
> *"CENTRAL assigns each free agent an endpoint in X to satisfy all constraints. It uses the Hungarian Method for this purpose."* (Sec 6.5.1)

**Step 4. Use CBS for path planning in two stages (same as CENTRAL).**
> *"CENTRAL uses a version of CBS to plan collision-free paths for all agents from their current vertices to their assigned endpoints simultaneously."* (Sec 6.5.2)

> *"We noticed that CENTRAL becomes significantly more efficient if it plans paths in two stages... In the first stage, if any agents become occupied... CENTRAL plans paths for all these agents to their assigned endpoints... In the second stage, if any new tasks are added to the system or any agents become free at the current time step... it plans paths for all free agents to their assigned endpoints."* (Sec 6.5.2)

**Step 5. Assignments remain stable between events.**
The key structural difference: once an agent is assigned a pickup vertex and starts moving toward it, CENTRAL-fixed keeps that assignment until the agent reaches the pickup vertex or a new event occurs. This prevents the oscillation issue of CENTRAL.

**Step 6. Each agent moves one step; advance time.**

**Completeness:**
> *"Theorem 6.3. CENTRAL is long-term robust for all well-formed MAPD problem instances."* (Sec 6.5.2)

The event-driven variant preserves completeness because the key properties (Property 6.4 and 6.5) that guarantee CBS can always find paths still hold, and the event-driven trigger ensures progress: once an agent is assigned a task and no new events occur, it follows its planned path to completion.

---

## 4. HBH-MLA\*

**Paper:** Grenouilleau et al., "A Multi-Label A\* Algorithm for Multi-Agent Pathfinding," ICAPS 2019.

**Step 1. At each timestep, get available agents and open tasks.**
> *"The algorithm first retrieves the list of currently available agents (agents with no assigned task, resting at a location) and the list of open tasks (tasks released and not yet assigned)."* (Sec H-Value-Based Heuristic, Algorithm 2)

**Step 2. Sort agent-task pairs by h-value (centralized greedy).**
> *"A list of agent-task pairs is created and sorted in increasing order of h-value."* (Sec H-Value-Based Heuristic)

**Step 3. Try each pair using MLA\* for path planning.**
> *"HBH then scans the list and tries each agent-task pair (a, τ) using MLA\*. If the assignment is feasible, HBH assigns a to τ and updates a's path to the one found by MLA\*."* (Sec H-Value-Based Heuristic)

**Step 4. MLA\* plans through pickup and delivery in one search.**
> *"We propose a multi-label A\* algorithm (MLA\*) which combines the search for a path to the pickup location with that of the delivery location (and more generally any other goals that follow sequentially)."* (Sec Multi-Label A\* Algorithm)

**Step 5. MLA\* uses labels to track goal progress.**
> *"The label indicates the current state of the node; if ℓ_n = 1 the agent is seeking a path to the pickup location, while if ℓ_n = 2 the agent is seeking a path to the delivery location."* (Sec Multi-Label A\* Algorithm)

**Step 6. Free agents move to closest free endpoint.**
> *"If some agents remain available, HBH checks their current locations and move them toward the closest free endpoint if necessary. Endpoints are locations at which agents are authorized to rest indefinitely."* (Sec H-Value-Based Heuristic)

---

## 5. TA-Prioritized

**Paper:** Liu et al., "Task and Path Planning for Multi-Agent Pickup and Delivery," AAMAS 2019.

TA-Prioritized is an **offline** algorithm with two strictly sequential phases: (1) assign all tasks to agents once via a TSP solver, then (2) plan collision-free paths once. Neither phase is ever repeated.

### Phase 1: Task Assignment (once, Sec 3)

**Step 1. Construct a directed weighted graph G' for the TSP.**

Each agent and each task becomes a vertex. Edge weights encode travel times between locations:
> *"Our MAPD algorithms first construct a directed weighted graph G' = (V', E') with V' = A ∪ T, where vertex α_i ∈ A represents agent a_i and vertex τ_i ∈ T represents task t_i."* (Sec 3.1)

**Step 2. Solve the TSP to get one task sequence per agent.**

A Hamiltonian cycle on G' is partitioned into M parts — one per agent — each defining that agent's ordered task sequence:
> *"Our MAPD algorithms use the LKH-3 TSP solver to plan a good Hamiltonian cycle on G' for their objectives."* (Sec 3.2)

> *"Since a Hamiltonian cycle visits each agent vertex exactly once, it can be partitioned into M parts, where each part consists of an agent vertex, a sequence of task vertices, and another agent vertex (in this order). Since a Hamiltonian cycle also visits each task vertex exactly once, the M parts can be converted to M task sequences, one for each agent."* (Sec 3.1)

The objective is to minimize makespan (max execution time across agents):
> *"The primary objective of our MAPD algorithms is to minimize the makespan max(M_i), which is the largest execution time of all task sequences."* (Sec 3)

Task assignment is done **once** and **never revisited** — the TSP ignores collisions:
> *"Both MAPD algorithms first assign the tasks to agents. They compute one task sequence for each agent by solving a special TSP, which ignores collisions and thus uses estimated travel times between locations."* (Sec 1.2)

### Phase 2: Prioritized Path Planning (once, Sec 4)

**Step 3. Plan paths for agents one at a time using improved prioritized planning.**

Agents are committed one per iteration. The key improvement over basic PP is that TA-Prioritized uses **tentative planning** to decide the agent ordering dynamically:
> *"The path-planning part of TA-Prioritized uses an improved version of prioritized planning to plan collision-free paths for the agents to execute all of their tasks according to their task sequences."* (Sec 4)

> *"TA-Prioritized improves on this technique by choosing the next agent only after it has planned a path for an agent. This way, it can choose the next agent based on the actual execution times (that take the paths of the previous agents into account). For each remaining agent, it tentatively assumes that it chooses this agent next and plans a path for it. It then chooses the agent next whose path has the largest execution time, and the procedure repeats."* (Sec 4)

> *"TA-Prioritized obtains collision-free paths for all agents after M iterations, during each of which it plans paths for at most M remaining agents."* (Sec 4)

Where M is the number of agents:
> *"A MAPD problem consists of a set of M agents A = {a1, . . . , aM}."* (Sec 2)

The intuition: agents with longer paths get committed first so they face fewer constraints, reducing overall makespan.

**Step 4. Each agent's path is a concatenation of sub-paths, planned one by one.**

For an agent with task sequence [t1, t2, t3], the sub-paths are: start→pickup1, pickup1→delivery1, delivery1→pickup2, pickup2→delivery2, delivery2→pickup3, pickup3→delivery3:
> *"The path of an agent is a concatenation of several sub-paths according to its task sequence, namely, from its start location to the pickup location of its first task, to the delivery location of its first task, to the pickup location of its second task, and so on, ending at the delivery location of its last task."* (Sec 4)

> *"TA-Prioritized constructs the path sub-path by sub-path, where a sub-path moves the agent from its current location to its goal location, which is either the pickup location of a task at or after its release time or the delivery location of a task."* (Sec 4)

Sub-paths must avoid all previously committed agents' paths and cannot visit remaining agents' parking locations:
> *"All sub-paths have to avoid collisions with the paths of all previous agents and cannot contain the parking locations of all remaining agents."* (Sec 4)

**Step 5. "Reserving dummy paths" — each sub-path's goal test verifies a dummy path to parking exists.**

The A* search for each sub-path uses a **modified goal test** that requires a feasible escape route to the agent's parking location. This prevents the agent from reaching a goal state from which it would be stuck:
> *"TA-Prioritized implements 'reserving dummy paths' by changing the goal-test function of the A\* search for each sub-path: An A\* node (x,t) is a goal node iff (1) location x is the goal location of the agent, (2) time step t is at or after the release time of the task if the goal location is the pickup location of the task, and (3) the A\* search is able to plan a 'dummy path' from location x at time step t to the parking location of the agent."* (Sec 4)

> *"This dummy path has to 'hold' the parking location (that is, allow the agent to stay there forever), avoid collisions with the paths of all previous agents (and their final dummy paths), and cannot contain the parking locations of all remaining agents."* (Sec 4)

If the goal test at condition (3) fails, the A* search does not accept that goal state and keeps searching for a later arrival time where a dummy path exists.

**Step 6. Dummy paths are never executed — except the final one.**

The dummy path exists only to **prove** that the next sub-path can be planned. Once the next sub-path is actually planned, it replaces the dummy path:
> *"An agent never moves along its dummy path, except for its last one (the 'final' dummy path), that moves it from the delivery location of its last task to its parking location and holds it, since the purpose of a dummy path is only to guarantee that the subsequent sub-path for the agent (and its dummy path), that replace this dummy path, exists."* (Sec 4)

> *"All sub-paths also have to avoid collisions with the final dummy paths of all previous agents."* (Sec 4)

### Completeness

> *"Whenever TA-Prioritized plans a sub-path for an agent (and its dummy path) for well-formed MAPD instances, it is guaranteed to find collision-free ones (and is thus complete) since they exist."* (Sec 4)

The proof is by induction — each agent starts at its parking location, waits for all previous agents to finish, then visits its goals via paths that avoid all other parking locations (guaranteed to exist by well-formedness):
> *"Each agent starts in its parking location. Assume that the agent is in its parking location. It can stay there until all previous agents have moved along their paths (and their final dummy paths) to their parking locations. Then, it can first move to its goal location, then move back to its parking location, and finally stay there for as many time steps as needed. This path does not collide with the paths of the previous agents because it avoids the parking locations of the previous agents and their paths avoid its parking location."* (Sec 4)

---

## 6. TA-Hybrid

**Paper:** Liu et al., "Task and Path Planning for Multi-Agent Pickup and Delivery," AAMAS 2019.

**Step 1. Same task assignment as TA-Prioritized (LKH-3 TSP).**

**Step 2. Two-group path planning: ICBS for task agents, min-cost max-flow for free agents.**
> *"Group 1: New task agents. TA-Hybrid plans sub-paths for them from their current locations to the delivery locations of their current tasks... TA-Hybrid thus uses Improved Conflict-Based Search (ICBS)... Group 2: Free agents. TA-Hybrid plans sub-paths for them from their current locations to the pickup locations of the next tasks in their task sequences... TA-Hybrid thus uses a polynomial-time min-cost max-flow algorithm."* (Sec 5.1)

**Step 3. Free agents can swap pickup locations via AMAPF.**
> *"If an agent is assigned the current pickup location of a different agent, TA-Hybrid replaces its current task sequence with the task sequence of this different agent, which can improve the resulting makespan."* (Sec 5.3)

**Step 4. Reserving dummy paths for deadlock avoidance.**
> *"TA-Hybrid uses 'reserving dummy paths' for both Procedures PLANPATHSTODELIVERY and PLANPATHSTOPICKUP to guarantee completeness."* (Sec 5.1)

**Completeness:**
> *"Whenever TA-Hybrid plans a sub-path for an agent (and its dummy path) for well-formed MAPD instances, it is guaranteed to find collision-free ones (and is thus complete) since they exist."* (Sec 5.2)

---

## 7. RMCA

**Paper:** Chen et al., "Integrated Task Assignment and Path Planning for Capacitated Multi-Agent Pickup and Delivery," IEEE RA-L 2021.

**Step 1. Coupled task assignment and path planning.**
> *"In this work we propose a new coupled method where task assignment choices are informed by actual delivery costs instead of by lower-bound estimates."* (Sec I)

**Step 2. Greedy insertion using marginal cost.**
> *"MCA... select a task i\* to be assigned to robot k\* while satisfying: (k\*, i\*, q\*_1, q\*_2) = argmin {t((o_k ⊕ s_i) ⊕ g_i) − t(o_k)}"* (Sec IV-B, Eq. 12)

**Step 3. Regret-based variant (RMCA) for better task selection.**
> *"RMCA chooses the next task to be assigned based on the difference in the marginal cost of inserting the task into the best robot's route and the second-best robot's route, and then assigns the task to the robot that has the lowest marginal cost to transport the task."* (Sec IV-C)

**Step 4. Path planning uses prioritized planning with STA\*.**
> *"The planPath() function uses prioritised planning with space-time A\*, which is fast and effective, to plan a single path for agent k following its ordered action sequence o_k while avoiding collisions with any other agents' existing paths."* (Sec IV-A)

**Step 5. LNS anytime improvement.**
> *"The algorithm will continuously destroy some assigned tasks from the current solution and reassign these tasks using RMCA. If a better solution is found, we adopt the new solution."* (Sec IV-D, Algorithm 3)

**Completeness:**
> *"It is worth noting that the path planning part of Algorithm 1 might be incomplete as the prioritised planning is known to be incomplete."* (Sec IV-A)

---

## 8. LNS-PBS

**Paper:** Xu et al., "Multi-Goal Multi-Agent Pickup and Delivery," IROS 2022.

**Step 1. Trigger assignment on events.**
> *"When new tasks are released by the system, tasks are deferred from the previous iteration, or a task agent becomes a free agent, we start a new iteration."* (Sec IV, Algorithm 1 Line 2)

**Step 2. Assign task sequences using LNS with Hungarian-based insertion.**
> *"LNS starts with an initial task assignment generated by Hungarian-based insertion and iteratively improves it using Shaw removal and regret-based re-insertion until a user-specified runtime limit is reached."* (Sec IV-A)

**Step 3. Assign dummy endpoints to all agents.**
> *"The dummy endpoint of each agent needs to be different from the already assigned dummy endpoints, all goal locations of the uncompleted tasks, and the old dummy endpoints of the other M−1 agents in the previous iteration."* (Sec IV-B)

**Step 4. Plan paths using modified PBS.**
> *"When PBS generates the root node of the PT, it plans a time-minimal path for each agent that avoids the old paths of all other M-1 agents."* (Sec IV-C)

**Step 5. PBS low-level uses MLA\* for goal sequences.**
> *"Li et al. generalize the low level of PBS to planning time-minimal paths for agents with sequences of goal locations."* (Sec IV-C)

**Step 6. Modified PBS avoids old paths for completeness.**
> *"When PBS resolves a collision between two agents, for each agent whose path needs to be re-planned in each child node, PBS plans a time-minimal path for it that avoids collisions with the new paths of all higher-priority agents and the old paths of all other agents (that do not have a higher priority than it)."* (Sec IV-C)

**Completeness:**
> *"Theorem 1: Given a well-formed MG-MAPD instance with a finite number of tasks, LNS-PBS is guaranteed to find collision-free paths in finite time that allow each agent to execute all tasks assigned to it."* (Sec IV-D)

---

## 9. LNS-wPBS

**Paper:** Xu et al., "Multi-Goal Multi-Agent Pickup and Delivery," IROS 2022.

**Step 1–3. Same as LNS-PBS (LNS assignment + dummy endpoints).**

**Step 4. Use windowed PBS (wPBS) instead of full PBS.**
> *"LNS-wPBS is a variant of LNS-PBS that, unlike LNS-PBS, uses windowed PBS (wPBS) for planning collision-free paths for only the first w timesteps and then plan path again once the agents have moved for w timesteps."* (Sec IV-E)

**Step 5. Additional periodic replanning every w timesteps.**
> Algorithm 1, Lines 6-8: *"else if agents have moved w timesteps then: Assign a dummy endpoint to each agent; Plan paths for all agents using wPBS"*

**Step 6. Simplified dummy-endpoint constraints (no old-path avoidance).**
> *"Since LNS-wPBS gives up the completeness guarantee, we further simplify it in three respects: First, LNS-wPBS does not defer any tasks... Second, wPBS uses the original low level of PBS instead of our modified version, i.e., it does not consider the old paths of the agents. Third, the assigned dummy endpoints need only to be pairwise different from each other."* (Sec IV-E)

**Completeness:**
> *"This makes LNS-wPBS more efficient than LNS-PBS but incomplete because there is no guarantee that the agents can reach their goal locations in a finite number of timesteps."* (Sec IV-E)
