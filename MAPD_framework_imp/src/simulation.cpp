#include "simulation.h"
#include "cbs.h"
#include <algorithm>
#include <set>
#include <ctime>
#include <fstream>
#include <climits>
#include <unordered_map>
#include <dlib/optimization/max_cost_assignment.h>

// ============================================================
// Section 0: Initialization (Pseudocode Section 2)
// ============================================================

void Simulation::init(const string& map_file, const string& task_file, const MAPDConfig& cfg,
                      const string& tour_file) {
    tour_file_ = tour_file;
    config = cfg;
    mapd_map.load(map_file);
    maxtime = mapd_map.maxtime;

    all_tasks = load_tasks(task_file, mapd_map.endpoints);

    task_indices_by_time.resize(maxtime);
    t_task = 0;
    for (int i = 0; i < (int)all_tasks.size(); i++) {
        int rt = all_tasks[i].release_time;
        if (rt >= 0 && rt < (int)maxtime) {
            task_indices_by_time[rt].push_back(i);
            if (rt > t_task) t_task = rt;
        }
    }

    agents.resize(mapd_map.num_agents);
    token.path.resize(mapd_map.num_agents);
    token.my_map = mapd_map.grid;
    token.my_endpoints = mapd_map.is_endpoint;
    token.timestep = 0;

    for (int i = 0; i < mapd_map.num_agents; i++) {
        agents[i].init(i, mapd_map.agent_starts[i], mapd_map.col, mapd_map.row, maxtime);
        token.path[i].resize(maxtime);
        for (unsigned int k = 0; k < maxtime; k++)
            token.path[i][k] = mapd_map.agent_starts[i];
    }

    // Init loop state
    last_released_time_ = -1;  // no tasks released yet; release_tasks() will handle t=0
    ta_planning_done_ = false;
    agent_pending_task.assign(agents.size(), nullptr);
    tp_agent_ = nullptr;

    // Set event flags so the first iteration's task_assignment runs.
    // release_tasks() will load t=0 tasks before task_assignment runs.
    central_has_event_ = true;
    central_reassign_event_ = true;
    central_first_iter_ = true;

    // Init state for PBS
    pbs_has_event_ = true;
    pbs_last_replan_time_ = 0;
}

// ============================================================
// Section 1: Unified Main Loop (Pseudocode Section 2)
//
//   While not End():
//     Step A: release_tasks
//     Step B: task_assignment_and_path_planning
//     Step C: update_system (transitions + advance + release)
// ============================================================

void Simulation::run() {
    while (!end()) {
        release_tasks();                      // Step A
        task_assignment_and_path_planning();   // Step B
        update_system();                       // Step C
    }
}

// ============================================================
// Section 1.1: End Condition (Pseudocode Section 14.6)
// ============================================================

bool Simulation::end() const {
    // Offline algorithms that do all planning in a single iteration:
    // They need at least one iteration to run, then they're done.
    // ta_planning_done_ is set after the first iteration completes.
    if (config.mode == MODE_OFFLINE && config.assign_method == AM_LKH3_TSP)
        return ta_planning_done_;
    if (config.mapf == MAPF_TA_HYBRID_TWO_GROUP)
        return ta_planning_done_;

    if (!token.tasks.empty()) return false;
    if ((int)token.timestep <= t_task) return false;
    // For PBS online: also check task_sequences
    if (config.assign_trigger == AT_ON_UNASSIGNED_OR_FREE) {
        for (auto& a : agents)
            if (!a.task_sequence.empty()) return false;
    }
    for (auto& a : agents)
        if (a.status != AG_FREE && a.finish_time > token.timestep) return false;
    return true;
}

// ============================================================
// Section 1.2: Release Tasks (Pseudocode Section 3)
//   On first call (t=0), tasks are already loaded by init().
//   On subsequent calls, task release is handled by update_system().
//   This function exists for structural clarity.
// ============================================================

void Simulation::release_tasks() {
    // Pseudocode Section 3 — GetReleasedTasks(t, mode)
    // Release all tasks from last_released_time_+1 up to current token.timestep.
    // This handles both the initial t=0 release and subsequent releases
    // after update_system() advances the timestep.

    int from = last_released_time_ + 1;
    int to = (int)token.timestep;

    switch (config.mode) {
    case MODE_ONLINE:
    case MODE_SEMI_ONLINE:
        for (int t = from; t <= to && t < (int)maxtime; t++) {
            for (int idx : task_indices_by_time[t])
                token.tasks.push_back(&all_tasks[idx]);
        }
        break;
    case MODE_OFFLINE:
        // All tasks released at t=0
        if (last_released_time_ < 0) {
            for (int i = 0; i < (int)all_tasks.size(); i++)
                token.tasks.push_back(&all_tasks[i]);
        }
        break;
    }

    last_released_time_ = to;
}

// ============================================================
// Section 1.3: Update System (Pseudocode Section 4)
//   Merged: state transitions + advance timestep + release tasks
//   Order:
//     1. Detect state transitions (deliveries, pickups)
//     2. Advance to next event timestep
//     3. Release tasks for skipped timesteps
//     4. Update agent locations
// ============================================================

void Simulation::update_system() {
    // --- HUNGARIAN/LNS online mode ---
    if (config.assign_trigger == AT_ON_UNASSIGNED_OR_FREE) {
        // Advance to next event timestep
        unsigned int next_ts = maxtime;

        // Check when any agent arrives at its current task's first/last goal
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].task_sequence.empty()) continue;
            int task_id = agents[i].task_sequence.front();
            Task& task = all_tasks[task_id];
            int first_goal = task.goals.empty() ? task.pickup_loc : task.goals[0];
            int last_goal = task.goals.size() >= 2 ? task.goals[min((int)task.goals.size()-1, 1)] : first_goal;
            int target_loc = -1;
            int min_time = 0;

            if (agents[i].status == AG_MOVING_TO_PICKUP) {
                target_loc = first_goal;
                min_time = task.release_time;
            } else if (agents[i].status == AG_CARRYING) {
                target_loc = last_goal;
                min_time = 0;
            }

            if (target_loc >= 0) {
                for (unsigned int t = token.timestep + 1; t < maxtime; t++) {
                    if ((int)agents[i].path[t] == target_loc && (int)t >= min_time) {
                        if (t < next_ts) next_ts = t;
                        break;
                    }
                }
            }
        }

        // Check for new task releases
        for (unsigned int t = token.timestep + 1; t < maxtime && t <= next_ts; t++) {
            if (!task_indices_by_time[t].empty()) {
                if (t < next_ts) next_ts = t;
                break;
            }
        }

        // If no event found but agents have tasks, advance by 1
        if (next_ts >= maxtime) {
            bool has_work = false;
            for (auto& a : agents) {
                if (!a.task_sequence.empty()) { has_work = true; break; }
            }
            if (has_work || !token.tasks.empty())
                next_ts = token.timestep + 1;
            else
                return;  // truly done
        }
        if (next_ts <= token.timestep) next_ts = token.timestep + 1;

        if (next_ts >= maxtime) {
            cerr << "PBS: exceeded maxtime=" << maxtime << endl;
            return;
        }

        // Advance timestep and update locations
        // (task release is handled by release_tasks() at the start of next iteration)
        token.timestep = next_ts;
        for (auto& ag : agents) {
            if (token.timestep < maxtime)
                ag.loc = ag.path[token.timestep];
        }

        // Reset and re-detect events at the new timestep for the next iteration
        pbs_has_event_ = false;
        update_system_pbs();
        return;
    }

    // --- TP/TPTS mode: check if a free agent needs processing first ---
    if (config.assign_trigger == AT_ON_FREE_WAITS) {
        for (auto& ag : agents) {
            if (ag.finish_time <= token.timestep) {
                // Stay at current timestep to process this agent next iteration.
                // Reset event flags and detect transitions at current time.
                central_has_event_ = false;
                central_reassign_event_ = false;

                // Detect completed deliveries
                for (int i = 0; i < (int)agents.size(); i++) {
                    if (agents[i].status == AG_CARRYING && agents[i].finish_time <= token.timestep) {
                        Task* task = agent_pending_task[i];
                        if (task) {
                            task->completion_time = agents[i].finish_time;
                            token.tasks.remove(task);
                            agent_pending_task[i] = nullptr;
                        }
                        agents[i].status = AG_FREE;
                        agents[i].current_task = -1;
                    }
                }

                // Detect pickup arrivals
                for (int i = 0; i < (int)agents.size(); i++) {
                    if (agents[i].status == AG_MOVING_TO_PICKUP && agents[i].finish_time <= token.timestep) {
                        Task* task = agent_pending_task[i];
                        if (task && (int)agents[i].loc == task->pickup_loc) {
                            agents[i].status = AG_CARRYING;
                        } else {
                            agents[i].status = AG_FREE;
                            if (task) task->status = -1;
                            agent_pending_task[i] = nullptr;
                        }
                    }
                }

                // Check for new tasks at current timestep
                if (token.timestep < maxtime && !task_indices_by_time[token.timestep].empty()) {
                    // already released
                }
                return;  // don't advance
            }
        }
    }

    // --- Default mode (CENTRAL, CENTRAL_FIXED, TA-Prioritized, TA-Hybrid, TP/TPTS) ---

    // Step 1: Detect state transitions at current timestep
    // (Reset event flags first — they will be set based on the NEW timestep)
    // We detect transitions at the current timestep before advancing,
    // but the event flags will be set for the NEXT iteration after advancing.

    // Step 2: Advance to next event timestep
    unsigned int next_ts = maxtime;
    for (auto& ag : agents) {
        if (ag.finish_time > token.timestep && ag.finish_time < next_ts)
            next_ts = ag.finish_time;
    }
    for (unsigned int t = token.timestep + 1; t < maxtime && t <= next_ts; t++) {
        if (!task_indices_by_time[t].empty()) {
            if (t < next_ts) next_ts = t;
            break;
        }
    }
    if (next_ts >= maxtime) {
        bool any_busy = false;
        for (auto& a : agents)
            if (a.status != AG_FREE && a.finish_time > token.timestep) { any_busy = true; break; }
        if (!any_busy && token.tasks.empty()) return;
        next_ts = token.timestep + 1;
    }
    if (next_ts <= token.timestep) next_ts = token.timestep + 1;

    // Step 3: Advance timestep and update agent locations
    // (task release is handled by release_tasks() at the start of next iteration)
    token.timestep = next_ts;
    for (auto& ag : agents)
        ag.loc = ag.path[token.timestep];

    // Step 5: Detect state transitions at the NEW timestep
    //   (sets event flags for the next iteration's task_assignment_and_path_planning)
    central_has_event_ = false;
    central_reassign_event_ = false;

    // Detect completed deliveries (CARRYING -> FREE)
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_CARRYING && agents[i].finish_time <= token.timestep) {
            Task* task = agent_pending_task[i];
            if (task) {
                task->completion_time = agents[i].finish_time;
                token.tasks.remove(task);
                agent_pending_task[i] = nullptr;
            }
            agents[i].status = AG_FREE;
            agents[i].current_task = -1;
            central_has_event_ = true;
            central_reassign_event_ = true;
        }
    }

    // Detect pickup arrivals (MOVING_TO_PICKUP -> CARRYING)
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_MOVING_TO_PICKUP && agents[i].finish_time <= token.timestep) {
            Task* task = agent_pending_task[i];
            if (task && (int)agents[i].loc == task->pickup_loc) {
                agents[i].status = AG_CARRYING;
                central_has_event_ = true;
            } else {
                agents[i].status = AG_FREE;
                if (task) task->status = -1;
                agent_pending_task[i] = nullptr;
                central_has_event_ = true;
                central_reassign_event_ = true;
            }
        }
    }

    // Check if new tasks arrived at the new timestep
    if (token.timestep < maxtime && !task_indices_by_time[token.timestep].empty()) {
        central_has_event_ = true;
        central_reassign_event_ = true;
    }
}

// ============================================================
// Section 2: Task Assignment and Path Planning
//   (Pseudocode Section 2.1)
//
//   1. task_assignment()      — Phase 1 delivery (CENTRAL) + should_assign() guard + dispatcher
//   2. if should_replan():
//        path_planning()      — dispatcher (CBS, PP, PBS, etc.)
// ============================================================

void Simulation::task_assignment_and_path_planning() {
    task_assignment();

    if (should_replan())
        path_planning();
}

// ============================================================
// Section 3: Should Assign — Trigger Check
//   (Pseudocode Section 5)
// ============================================================

bool Simulation::should_assign() const {
    if (token.tasks.empty()) return false;

    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        // TP / TPTS: any free agent at end of path
        for (auto& ag : agents)
            if (ag.finish_time <= token.timestep) return true;
        return false;

    case AT_EVERY_TIMESTEP:
        // CENTRAL: whenever any event occurred and free agents exist
        if (!central_has_event_) return false;
        for (auto& ag : agents)
            if (ag.status == AG_FREE) return true;
        return false;

    case AT_ON_NEW_TASK_OR_FREE:
        // CENTRAL_FIXED: only when agent became free or new tasks arrived
        // (pickup arrival triggers Phase 1 delivery, NOT Phase 2 reassignment)
        if (!central_reassign_event_) return false;
        for (auto& ag : agents)
            if (ag.status == AG_FREE) return true;
        return false;

    case AT_ON_UNASSIGNED_OR_FREE:
        // HUNGARIAN_PBS/wPBS: when unassigned tasks exist OR any agent is free (no tasks queued)
        if (pbs_has_event_) return true;
        return false;

    case AT_ONCE:
        // TA-Prioritized: assign once at t=0
        return token.timestep == 0;

    default:
        return false;
    }
}

// ============================================================
// Section 4: Should Replan — Separate Path Planning Needed?
//   (Pseudocode Section 15)
// ============================================================

bool Simulation::should_replan() const {
    // Pseudocode Section 15 — ShouldReplan
    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        // TP/TPTS/HBH-MLA*: path planning is done inside assignment
        return false;

    case AT_EVERY_TIMESTEP:
        // CENTRAL: replan when event occurred
        return central_has_event_;

    case AT_ON_NEW_TASK_OR_FREE:
        // CENTRAL_FIXED: replan when event occurred
        return central_has_event_;

    case AT_ON_UNASSIGNED_OR_FREE:
        // PBS/wPBS: replan when event occurred
        return pbs_has_event_;

    case AT_ONCE:
        // TA-Prioritized/TA-Hybrid: replan after initial assignment
        return true;

    default:
        return false;
    }
}

// ============================================================
// Section 5: Task Assignment — Dispatcher
//   (Pseudocode Section 6)
// ============================================================

void Simulation::task_assignment() {
    // Pseudocode Section 6 — Task_Assignment dispatcher

    // For CENTRAL/CENTRAL_FIXED: detect instant pickups
    // (free agent standing on a task's pickup → transition to CARRYING)
    // This must happen before should_assign() so these agents are not
    // treated as FREE by the Hungarian assignment.
    if (config.assign_trigger == AT_EVERY_TIMESTEP ||
        config.assign_trigger == AT_ON_NEW_TASK_OR_FREE) {
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].status != AG_FREE) continue;
            for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
                Task* task = *it;
                if (task->status != -1) continue;
                if ((int)agents[i].loc == task->pickup_loc) {
                    bool goal_held = false;
                    for (int j = 0; j < (int)agents.size(); j++) {
                        if (j == i) continue;
                        if ((int)token.path[j][maxtime - 1] == task->delivery_loc) {
                            goal_held = true; break;
                        }
                    }
                    if (!goal_held) {
                        task->status = i;
                        agents[i].status = AG_CARRYING;
                        agents[i].current_task = task->id;
                        agent_pending_task[i] = task;
                        central_has_event_ = true;
                        break;
                    }
                }
            }
        }
    }

    // Clear phase2 data at the start of every iteration to prevent stale data
    phase2_free_ids_.clear();
    phase2_tasks_.clear();
    phase2_goal_locs_.clear();
    phase2_goal_eps_.clear();

    // Guard: only proceed if the trigger condition is met
    if (!should_assign()) {
        // For TP/TPTS/HBH: bump one free agent to avoid infinite loop when no tasks
        if (config.assign_trigger == AT_ON_FREE_WAITS) {
            for (auto& ag : agents) {
                if (ag.finish_time <= token.timestep) {
                    ag.finish_time = token.timestep + 1;
                    break;
                }
            }
        }
        return;
    }

    switch (config.assign_method) {
    case AM_DECOUPLED_GREEDY: {
        // Pick free agent with earliest finish_time (one at a time)
        Agent* ag = nullptr;
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].finish_time <= token.timestep) {
                if (!ag) ag = &agents[i];
                else if (agents[i].finish_time < ag->finish_time) ag = &agents[i];
            }
        }
        if (ag) assign_decoupled_greedy(*ag);
        break;
    }
    case AM_DECOUPLED_GREEDY_SWAPS: {
        Agent* ag = nullptr;
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].finish_time <= token.timestep) {
                if (!ag) ag = &agents[i];
                else if (agents[i].finish_time < ag->finish_time) ag = &agents[i];
            }
        }
        if (ag) assign_tpts(*ag);
        break;
    }
    case AM_CENTRALIZED_GREEDY:
        assign_centralized_greedy();
        break;
    case AM_HUNGARIAN:
        if (config.assign_trigger == AT_ON_UNASSIGNED_OR_FREE)
            assign_repeated_hungarian();
        else
            assign_hungarian();
        break;
    case AM_REPEATED_HUNGARIAN_LNS:
        assign_repeated_hungarian_lns();
        break;
    case AM_LKH3_TSP:
        assign_ta_tsp();
        break;
    case AM_LKH3_TSP_REASSIGN:
        assign_ta_tsp();  // same tour parsing; plan_ta_hybrid() handles the rest
        break;
    default:
        break;
    }
}

// ============================================================
// Section 6: Path Planning — Dispatcher
//   (Pseudocode Section 11)
// ============================================================

void Simulation::path_planning() {
    // For CENTRAL/CENTRAL_FIXED with PBS override: use CBS wrapper
    // but replace CBS Group 2 with PBS-based planning
    if ((config.assign_trigger == AT_EVERY_TIMESTEP ||
         config.assign_trigger == AT_ON_NEW_TASK_OR_FREE) &&
        config.mapf == MAPF_PBS) {
        path_planning_cbs_with_pp();
        return;
    }

    // HUNGARIAN/LNS with PP override: PP + per-task MLA*
    if (config.assign_trigger == AT_ON_UNASSIGNED_OR_FREE &&
        config.mapf == MAPF_DECOUPLED_PP) {
        path_planning_pp_mla();
        return;
    }

    switch (config.mapf) {
    case MAPF_DECOUPLED_PP:
        if (config.single_agent == SA_SEQ_STA ||
            (config.single_agent == SA_MLA_SEQUENCE &&
             config.assign_method == AM_LKH3_TSP)) {
            plan_ta_prioritized();
            ta_planning_done_ = true;
        } else {
            path_planning_pp();
        }
        break;
    case MAPF_CBS:
        path_planning_cbs();
        break;
    case MAPF_PBS:
        path_planning_pbs();
        break;
    case MAPF_wPBS:
        path_planning_wpbs();
        break;
    case MAPF_TA_HYBRID_TWO_GROUP:
        plan_ta_hybrid();
        ta_planning_done_ = true;
        break;
    default:
        path_planning_pp();
        break;
    }
}

// ============================================================
// Section 8: Task Assignment — Decoupled Greedy (TP)
//   (Pseudocode Section 9.1)
// ============================================================

bool Simulation::assign_decoupled_greedy(Agent& ag) {
    ag.loc = ag.path[token.timestep];

    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)token.path.size(); i++) {
        if (i != ag.id) hold[token.path[i][maxtime - 1]] = true;
    }

    Task* best_task = nullptr;
    for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
        if (hold[(*it)->pickup_loc] || hold[(*it)->delivery_loc]) continue;
        if (best_task == nullptr ||
            mapd_map.endpoints[(*it)->pickup].h_val[ag.loc] <
            mapd_map.endpoints[best_task->pickup].h_val[ag.loc]) {
            best_task = *it;
        }
    }

    if (best_task == nullptr) {
        bool move = false;
        for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
            if ((*it)->delivery_loc == (int)ag.loc) { move = true; break; }
        }
        if (move) {
            if (move2EP(ag)) {
                for (unsigned int i = token.timestep; i < token.path[ag.id].size(); i++)
                    token.path[ag.id][i] = ag.path[i];
                return true;
            }
        } else {
            ag.finish_time = token.timestep + 1;
            return true;
        }
    } else {
        auto result = plan_task_token(ag, *best_task);
        if (result.first < 0) return false;

        for (unsigned int i = token.timestep; i < token.path[ag.id].size(); i++)
            token.path[ag.id][i] = ag.path[i];

        ag.finish_time = result.second + best_task->goal_wait_time;
        ag.current_task = best_task->id;
        best_task->status = ag.id;
        best_task->ag_arrive_start = result.first;
        best_task->completion_time = result.second;
        token.tasks.remove(best_task);
        return true;
    }
    return false;
}

// ============================================================
// Section 9: Task Assignment — Decoupled Greedy with Swaps (TPTS)
//   (Pseudocode Section 9.2)
// ============================================================

bool Simulation::assign_tpts(Agent& ag, int depth) {
    if (depth >= (int)agents.size()) return false;
    vector<vector<unsigned int>> saved_paths(token.path.size());
    for (int i = 0; i < (int)token.path.size(); i++)
        saved_paths[i] = token.path[i];
    vector<Agent> saved_agents = agents;

    ag.loc = ag.path[token.timestep];

    struct HN { int h; Task* task; };
    vector<HN> sorted_tasks;
    for (auto it = token.tasks.begin(); it != token.tasks.end(); it++)
        sorted_tasks.push_back({mapd_map.endpoints[(*it)->pickup].h_val[ag.loc], *it});
    sort(sorted_tasks.begin(), sorted_tasks.end(), [](const HN& a, const HN& b) { return a.h < b.h; });

    for (auto& hn : sorted_tasks) {
        Task* task = hn.task;

        if (task->status == -1 ||
            (task->status >= 0 && task->ag_arrive_start >= 0 &&
             task->ag_arrive_start > (int)token.timestep + hn.h)) {
            bool occupied = false;
            for (int i = 0; i < (int)token.path.size(); i++) {
                if (i == ag.id) continue;
                if (task->status >= 0 && i == task->status) continue;
                if (token.path[i][maxtime - 1] == (unsigned int)task->delivery_loc ||
                    token.path[i][maxtime - 1] == (unsigned int)task->pickup_loc) {
                    occupied = true; break;
                }
            }
            if (occupied) continue;

            int ag_hide = (task->status >= 0) ? task->status : -1;
            auto plan_result = plan_task_token(ag, *task, ag_hide);
            int arrive_start = plan_result.first;
            int arrive_goal = plan_result.second;

            if (arrive_start >= 0 && (task->status == -1 || arrive_start < task->ag_arrive_start)) {
                if (arrive_goal >= 0) {
                    for (unsigned int i = token.timestep; i < token.path[ag.id].size(); i++)
                        token.path[ag.id][i] = ag.path[i];
                    ag.finish_time = arrive_goal + task->goal_wait_time;

                    if (task->status == -1) {
                        task->status = ag.id;
                        task->ag_arrive_start = arrive_start;
                        task->completion_time = arrive_goal;
                        token.tasks.remove(task);
                        return true;
                    } else {
                        Agent* old_ag = &agents[task->status];
                        int saved_status = task->status;
                        int saved_arrive = task->ag_arrive_start;
                        int saved_completion = task->completion_time;
                        task->status = ag.id;
                        task->ag_arrive_start = arrive_start;
                        task->completion_time = arrive_goal;

                        if (assign_tpts(*old_ag, depth + 1)) return true;

                        task->status = saved_status;
                        task->ag_arrive_start = saved_arrive;
                        task->completion_time = saved_completion;
                    }
                }
            }

            for (int i = 0; i < (int)token.path.size(); i++)
                token.path[i] = saved_paths[i];
            agents = saved_agents;
            ag.loc = ag.path[token.timestep];
        }
    }

    if (token.my_endpoints[ag.loc]) {
        bool need_move = false;
        for (auto it = token.tasks.begin(); it != token.tasks.end() && !need_move; it++)
            if ((*it)->delivery_loc == (int)ag.loc) need_move = true;
        for (unsigned int t = token.timestep; t < maxtime && !need_move; t++)
            for (int i = 0; i < (int)agents.size() && !need_move; i++)
                if (i != ag.id && token.path[i][t] == (unsigned int)ag.loc) need_move = true;
        if (need_move) {
            if (move2EP(ag)) {
                for (unsigned int i = token.timestep; i < token.path[ag.id].size(); i++)
                    token.path[ag.id][i] = ag.path[i];
                return true;
            } else {
                for (int i = 0; i < (int)token.path.size(); i++) token.path[i] = saved_paths[i];
                agents = saved_agents;
                return false;
            }
        } else {
            for (unsigned int i = token.timestep + 1; i < maxtime; i++) {
                ag.path[i] = ag.path[token.timestep];
                token.path[ag.id][i] = ag.path[token.timestep];
            }
            ag.finish_time = token.timestep + 1;
            return true;
        }
    } else {
        if (move2EP(ag)) {
            for (unsigned int i = token.timestep; i < token.path[ag.id].size(); i++)
                token.path[ag.id][i] = ag.path[i];
            return true;
        } else {
            for (int i = 0; i < (int)token.path.size(); i++) token.path[i] = saved_paths[i];
            agents = saved_agents;
            return false;
        }
    }
}

// ============================================================
// Section 9.3: Task Assignment — Centralized Greedy (HBH-MLA*)
//   (Pseudocode Section 9.3 — Assign_Centralized_Greedy)
//   Assigns ALL free agents to tasks using centralized greedy:
//   sort all (agent, task) pairs by h-value, greedily assign best pair.
//   Assignment only — path planning done separately by plan_hbh_mla().
// ============================================================

void Simulation::assign_centralized_greedy() {
    // HBH: centralized greedy with INTERLEAVED assignment and path planning.
    // "HBH scans the list and tries each agent-task pair (a, τ) using MLA*.
    //  If the assignment is feasible, HBH assigns a to τ and updates a's path."
    //  (Grenouilleau et al., ICAPS 2019, Algorithm 2)

    // Collect free agents
    vector<int> free_ids;
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].finish_time <= token.timestep)
            free_ids.push_back(i);
    }
    if (free_ids.empty()) return;

    // Build hold set (endpoints occupied by non-free agents)
    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++) {
        bool is_free = false;
        for (int fid : free_ids) if (fid == i) { is_free = true; break; }
        if (!is_free) hold[token.path[i][maxtime - 1]] = true;
    }

    // Collect available tasks
    vector<Task*> avail_tasks;
    for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
        if ((*it)->status != -1) continue;
        if (hold[(*it)->pickup_loc] || hold[(*it)->delivery_loc]) continue;
        avail_tasks.push_back(*it);
    }

    // Create all (agent, task) pairs sorted by h-value (ascending)
    struct AgentTaskPair {
        int agent_id;
        Task* task;
        int h_val;
    };
    vector<AgentTaskPair> pairs;
    for (int aid : free_ids) {
        for (Task* t : avail_tasks) {
            int h = mapd_map.endpoints[t->pickup].h_val[agents[aid].loc];
            if (h < INT_MAX)
                pairs.push_back({aid, t, h});
        }
    }
    sort(pairs.begin(), pairs.end(),
         [](const AgentTaskPair& a, const AgentTaskPair& b) { return a.h_val < b.h_val; });

    // Scan sorted list: try each pair, plan path, commit if feasible
    set<int> done_agents;
    set<int> done_tasks;
    for (auto& p : pairs) {
        if (done_agents.count(p.agent_id)) continue;
        if (done_tasks.count(p.task->id)) continue;

        Agent& ag = agents[p.agent_id];
        Task& task = *p.task;

        auto result = plan_task_token(ag, task);
        if (result.first < 0) continue;

        int arrive_start = result.first;
        int arrive_goal = result.second;

        // Feasible — commit path to token
        for (unsigned int t = token.timestep; t < maxtime; t++)
            token.path[ag.id][t] = ag.path[t];

        task.status = ag.id;
        task.ag_arrive_start = arrive_start;
        task.completion_time = arrive_goal;
        ag.current_task = task.id;
        ag.finish_time = arrive_goal + task.goal_wait_time;
        token.tasks.remove(p.task);

        done_agents.insert(p.agent_id);
        done_tasks.insert(p.task->id);
    }

    // Handle remaining free agents: move off delivery endpoints if needed
    for (int aid : free_ids) {
        if (done_agents.count(aid)) continue;
        Agent& ag = agents[aid];
        bool need_move = false;
        for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
            if ((*it)->delivery_loc == (int)ag.loc) { need_move = true; break; }
        }
        if (need_move) {
            if (move2EP(ag)) {
                for (unsigned int t = token.timestep; t < token.path[ag.id].size(); t++)
                    token.path[ag.id][t] = ag.path[t];
            } else {
                ag.finish_time = token.timestep + 1;
            }
        } else {
            ag.finish_time = token.timestep + 1;
        }
    }
}

// ============================================================
// Section 10: Task Assignment — Hungarian (CENTRAL)
//   (Pseudocode Section 9.4)
//   Populates phase2_* members for path_planning()
// ============================================================

void Simulation::assign_hungarian() {
    phase2_free_ids_.clear();
    phase2_tasks_.clear();
    phase2_goal_locs_.clear();
    phase2_goal_eps_.clear();

    for (int i = 0; i < (int)agents.size(); i++)
        if (agents[i].status == AG_FREE) phase2_free_ids_.push_back(i);
    if (phase2_free_ids_.empty()) return;

    // Build hold set from non-free agents
    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++)
        if (agents[i].status != AG_FREE) hold[token.path[i][maxtime - 1]] = true;

    // Candidate tasks (pickup/delivery not held)
    vector<Task*> candidate_tasks;
    vector<int> candidate_ep_indices;
    for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
        if ((*it)->status != -1) continue;
        if (!hold[(*it)->pickup_loc] && !hold[(*it)->delivery_loc]) {
            bool dup = false;
            for (auto* ct : candidate_tasks) {
                if (ct->pickup_loc == (*it)->pickup_loc ||
                    ct->delivery_loc == (*it)->delivery_loc) { dup = true; break; }
            }
            if (!dup) {
                candidate_tasks.push_back(*it);
                candidate_ep_indices.push_back((*it)->pickup);
                hold[(*it)->pickup_loc] = true;
                hold[(*it)->delivery_loc] = true;
            }
        }
    }
    int num_tasks_available = candidate_tasks.size();

    // Add parking endpoints for extra agents
    if ((int)candidate_ep_indices.size() < (int)phase2_free_ids_.size()) {
        for (int aid : phase2_free_ids_) {
            int best_ep = -1, best_d = mapd_map.col * mapd_map.row;
            for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                int loc = mapd_map.endpoints[e].loc;
                if (hold[loc]) continue;
                if (loc == (int)agents[aid].loc) { best_ep = e; best_d = 0; break; }
                int d = mapd_map.endpoints[e].h_val[agents[aid].loc];
                if (d > 0 && d < best_d) { best_d = d; best_ep = e; }
            }
            if (best_ep >= 0) {
                hold[mapd_map.endpoints[best_ep].loc] = true;
                candidate_ep_indices.push_back(best_ep);
            }
        }
    }

    int N = max((int)phase2_free_ids_.size(), (int)candidate_ep_indices.size());
    if (N == 0) return;

    // Build cost matrix
    int BIG = 2 * mapd_map.col * mapd_map.row;
    dlib::matrix<int> cost_matrix(N, N);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i >= (int)phase2_free_ids_.size()) {
                cost_matrix(i, j) = 0;
            } else {
                int aid = phase2_free_ids_[i];
                if (j < (int)candidate_ep_indices.size()) {
                    int ep_idx = candidate_ep_indices[j];
                    int d = mapd_map.endpoints[ep_idx].h_val[agents[aid].loc];
                    if (d == INT_MAX) d = BIG;
                    if (j < num_tasks_available)
                        cost_matrix(i, j) = (BIG - d) * N * N;
                    else
                        cost_matrix(i, j) = BIG * N - d;
                } else {
                    cost_matrix(i, j) = 0;
                }
            }
        }
    }

    vector<long> assignment = dlib::max_cost_assignment(cost_matrix);

    // Store assignment results for path_planning()
    phase2_tasks_.resize(phase2_free_ids_.size(), nullptr);
    phase2_goal_locs_.resize(phase2_free_ids_.size());
    phase2_goal_eps_.resize(phase2_free_ids_.size());

    for (int i = 0; i < (int)phase2_free_ids_.size(); i++) {
        int dest_idx = assignment[i];
        if (dest_idx >= (int)candidate_ep_indices.size()) {
            phase2_goal_locs_[i] = agents[phase2_free_ids_[i]].loc;
            phase2_goal_eps_[i] = findEndpointIndex(agents[phase2_free_ids_[i]].loc);
            continue;
        }
        int ep_idx = candidate_ep_indices[dest_idx];
        phase2_goal_locs_[i] = mapd_map.endpoints[ep_idx].loc;
        phase2_goal_eps_[i] = ep_idx;
        if (dest_idx < num_tasks_available)
            phase2_tasks_[i] = candidate_tasks[dest_idx];
    }
}

// ============================================================
// Section 11: Path Planning — CBS
//   (Pseudocode Section 12.2)
//   Uses phase2_* members set by assign_hungarian()
// ============================================================

void Simulation::path_planning_cbs() {
    // --- Plan delivery for newly CARRYING agents ---
    // Agents transitioned to CARRYING by update_system() or instant pickup
    // in task_assignment() need delivery paths planned.
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_CARRYING && agent_pending_task[i] &&
            agents[i].finish_time <= token.timestep) {
            Task* task = agent_pending_task[i];
            int begin = token.timestep + task->start_wait_time;
            int arrive = astar(agents[i], task->pickup_loc, begin,
                               mapd_map.endpoints[task->delivery], i);
            if (arrive >= 0) {
                for (unsigned int t = token.timestep; t < maxtime; t++)
                    token.path[i][t] = agents[i].path[t];
                agents[i].finish_time = arrive + task->goal_wait_time;
            } else {
                agents[i].status = AG_FREE;
                task->status = -1;
                agent_pending_task[i] = nullptr;
            }
        }
    }

    // --- Plan paths to pickups/parking for FREE agents via CBS ---
    if (phase2_free_ids_.empty()) return;

    // Build cons_paths from non-free agents
    vector<vector<int>> cons_paths_cbs;
    for (int i = 0; i < (int)agents.size(); i++) {
        bool is_free = false;
        for (int fid : phase2_free_ids_) if (fid == i) { is_free = true; break; }
        if (!is_free) {
            vector<int> cp(maxtime);
            for (unsigned int t = 0; t < maxtime; t++) cp[t] = (int)token.path[i][t];
            cons_paths_cbs.push_back(cp);
        }
    }

    vector<int> cbs_indices, cbs_starts, cbs_goals, cbs_ep_indices;
    for (int i = 0; i < (int)phase2_free_ids_.size(); i++) {
        if (phase2_goal_eps_[i] >= 0) {
            cbs_indices.push_back(i);
            cbs_starts.push_back(agents[phase2_free_ids_[i]].loc);
            cbs_goals.push_back(phase2_goal_locs_[i]);
            cbs_ep_indices.push_back(phase2_goal_eps_[i]);
        } else {
            agents[phase2_free_ids_[i]].finish_time = token.timestep + 1;
        }
    }

    bool used_cbs = false;
    if (!cbs_indices.empty()) {
        CBSSearch cbs(mapd_map.grid, cbs_starts, cbs_goals, cbs_ep_indices,
                      cons_paths_cbs, token.timestep, mapd_map.col,
                      config.ecbs_weight, mapd_map.endpoints, maxtime);
        if (cbs.run()) {
            used_cbs = true;
            for (int ci = 0; ci < (int)cbs_indices.size(); ci++) {
                int i = cbs_indices[ci];
                int aid = phase2_free_ids_[i];
                if (cbs.paths[ci].empty()) { agents[aid].finish_time = token.timestep + 1; continue; }
                for (int t = 0; t < (int)cbs.paths[ci].size(); t++) {
                    if (token.timestep + t < maxtime) {
                        token.path[aid][token.timestep + t] = cbs.paths[ci][t];
                        agents[aid].path[token.timestep + t] = cbs.paths[ci][t];
                    }
                }
                int last_loc = cbs.paths[ci].back();
                for (unsigned int t = token.timestep + cbs.paths[ci].size(); t < maxtime; t++) {
                    token.path[aid][t] = last_loc;
                    agents[aid].path[t] = last_loc;
                }
                agents[aid].finish_time = token.timestep + cbs.paths[ci].size() - 1;
                if (phase2_tasks_[i] != nullptr) {
                    agents[aid].status = AG_MOVING_TO_PICKUP;
                    agents[aid].current_task = phase2_tasks_[i]->id;
                    phase2_tasks_[i]->status = aid;
                    agent_pending_task[aid] = phase2_tasks_[i];
                }
            }
        }
    }

    // Fallback to PP if CBS fails
    if (!used_cbs) path_planning_pp();
}

// ============================================================
// Section 11b: Path Planning — CBS Group1 + PBS Group2
//   For CENTRAL/CENTRAL_FIXED with --mapf PBS override.
//   Group 1: delivery paths for CARRYING agents (same as CBS version)
//   Group 2: pickup/parking paths for FREE agents via PBS+MLA*
//            (PBS DFS conflict resolution with MLA* low-level)
// ============================================================

void Simulation::path_planning_cbs_with_pp() {
    // --- GROUP 1: delivery for CARRYING agents (same as path_planning_cbs) ---
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_CARRYING && agent_pending_task[i] &&
            agents[i].finish_time <= token.timestep) {
            Task* task = agent_pending_task[i];
            int begin = token.timestep + task->start_wait_time;
            int arrive = astar(agents[i], task->pickup_loc, begin,
                               mapd_map.endpoints[task->delivery], i);
            if (arrive >= 0) {
                for (unsigned int t = token.timestep; t < maxtime; t++)
                    token.path[i][t] = agents[i].path[t];
                agents[i].finish_time = arrive + task->goal_wait_time;
            } else {
                agents[i].status = AG_FREE;
                task->status = -1;
                agent_pending_task[i] = nullptr;
            }
        }
    }

    // --- GROUP 2: FREE agents to pickups/parking via PBS+MLA* ---
    if (phase2_free_ids_.empty()) return;

    // Collect valid agents with goals
    vector<int> pbs_ids;
    vector<Task*> pbs_tasks_vec;
    vector<vector<pair<int,int>>> goal_seqs;
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;
    vector<int> assigned_dummies(num_ag, -1);

    for (int i = 0; i < (int)phase2_free_ids_.size(); i++) {
        if (phase2_goal_eps_[i] >= 0) {
            int aid = phase2_free_ids_[i];
            pbs_ids.push_back(aid);
            pbs_tasks_vec.push_back(phase2_tasks_[i]);
            // Goal = [pickup/parking, dummy]
            vector<pair<int,int>> goals;
            goals.push_back({phase2_goal_locs_[i], 0});
            int dummy = choose_dummy_endpoint(aid, phase2_goal_locs_[i],
                                               assigned_dummies, false);
            assigned_dummies[aid] = dummy;
            goals.push_back({dummy, 0});
            goal_seqs.push_back(goals);
        } else {
            agents[phase2_free_ids_[i]].finish_time = token.timestep + 1;
        }
    }

    if (pbs_ids.empty()) return;
    int n = (int)pbs_ids.size();

    // Build external cons_paths (non-Group2 agents)
    set<int> pbs_set(pbs_ids.begin(), pbs_ids.end());
    vector<vector<int>> ext_cons;
    for (int a = 0; a < num_ag; a++) {
        if (pbs_set.count(a)) continue;
        vector<int> cp(max_t);
        for (int t = 0; t < max_t; t++) cp[t] = (int)token.path[a][t];
        ext_cons.push_back(cp);
    }

    // Pre-compute task groups for task-by-task mode
    bool use_taskwise_cbs = (config.mla_mode != MLA_SEQ);
    vector<vector<vector<pair<int,int>>>> pbs_task_groups;
    if (use_taskwise_cbs) {
        pbs_task_groups.resize(n);
        for (int idx = 0; idx < n; idx++) {
            int aid = pbs_ids[idx];
            pbs_task_groups[idx] = split_into_task_groups(aid, goal_seqs[idx]);
        }
    }

    auto plan_pbs_agent = [&](int idx, const vector<vector<int>>& cons) -> vector<int> {
        int aid = pbs_ids[idx];
        if (use_taskwise_cbs)
            return mla_star_taskwise(aid, (int)agents[aid].loc, (int)token.timestep,
                                      pbs_task_groups[idx], cons, {}, false);
        return seq_mla_star(aid, (int)agents[aid].loc, (int)token.timestep,
                             goal_seqs[idx], cons, {}, false);
    };

    // --- PBS root node: plan each agent via MLA* ---
    PBSNode* root = new PBSNode();
    root->paths.resize(n);
    for (int idx = 0; idx < n; idx++) {
        // cons = external + already-planned Group2 agents
        vector<vector<int>> cons = ext_cons;
        for (int j = 0; j < idx; j++)
            cons.push_back(root->paths[j]);

        root->paths[idx] = plan_pbs_agent(idx, cons);

        if (root->paths[idx].empty())
            root->paths[idx].resize(max_t, (int)agents[pbs_ids[idx]].loc);
    }

    // Find conflicts
    for (int a1 = 0; a1 < n; a1++) {
        for (int a2 = a1 + 1; a2 < n; a2++) {
            for (int t = (int)token.timestep; t < max_t; t++) {
                if (root->paths[a1][t] == root->paths[a2][t]) {
                    root->conflicts.emplace_back(a1, a2,
                        root->paths[a1][t], -1, t);
                    break;
                }
                if (t > (int)token.timestep &&
                    root->paths[a1][t] == root->paths[a2][t-1] &&
                    root->paths[a1][t-1] == root->paths[a2][t]) {
                    root->conflicts.emplace_back(a1, a2,
                        root->paths[a1][t], root->paths[a2][t], t);
                    break;
                }
            }
        }
    }
    root->num_collisions = (int)root->conflicts.size();

    // --- PBS DFS search ---
    PBSNode* best_node = root;
    if (!root->conflicts.empty()) {
        vector<PBSNode*> all_nodes;
        all_nodes.push_back(root);
        stack<PBSNode*> dfs_stack;
        dfs_stack.push(root);

        best_node->conflict = root->conflicts.front();
        for (auto& c : root->conflicts)
            if (get<4>(c) < get<4>(best_node->conflict))
                best_node->conflict = c;
        best_node->earliest_collision = get<4>(best_node->conflict);

        int hl = 0, max_hl = 2000;
        while (!dfs_stack.empty() && hl < max_hl) {
            PBSNode* curr = dfs_stack.top(); dfs_stack.pop();
            if (curr->conflicts.empty()) { best_node = curr; break; }

            auto chosen = curr->conflicts.front();
            for (auto& c : curr->conflicts)
                if (get<4>(c) < get<4>(chosen)) chosen = c;
            curr->conflict = chosen;
            curr->earliest_collision = get<4>(chosen);
            if (curr->earliest_collision > best_node->earliest_collision ||
                (curr->earliest_collision == best_node->earliest_collision &&
                 curr->num_collisions < best_node->num_collisions))
                best_node = curr;
            hl++;

            int a1 = get<0>(chosen), a2 = get<1>(chosen);
            for (int c = 0; c < 2; c++) {
                int lower = (c == 0) ? a1 : a2;
                int higher = (c == 0) ? a2 : a1;

                PBSNode* child = new PBSNode();
                child->parent = curr;
                child->priorities.copy(curr->priorities);
                child->priorities.add(lower, higher);
                if (child->priorities.connected(higher, lower)) {
                    delete child; continue;
                }
                child->paths = curr->paths;

                // Replan lower-priority agent
                set<int> hp = child->priorities.get_higher_priority(lower);
                vector<vector<int>> cons_lower = ext_cons;
                for (int j = 0; j < n; j++)
                    if (j != lower && hp.count(j))
                        cons_lower.push_back(child->paths[j]);

                child->paths[lower] = plan_pbs_agent(lower, cons_lower);

                if (child->paths[lower].empty()) {
                    delete child; continue;
                }

                // Find conflicts in child
                child->conflicts.clear();
                for (int i = 0; i < n; i++) {
                    if (i == lower) continue;
                    for (int t = (int)token.timestep; t < max_t; t++) {
                        if (child->paths[lower][t] == child->paths[i][t]) {
                            child->conflicts.emplace_back(lower, i,
                                child->paths[lower][t], -1, t);
                            break;
                        }
                        if (t > (int)token.timestep &&
                            child->paths[lower][t] == child->paths[i][t-1] &&
                            child->paths[lower][t-1] == child->paths[i][t]) {
                            child->conflicts.emplace_back(lower, i,
                                child->paths[lower][t], child->paths[i][t], t);
                            break;
                        }
                    }
                }
                child->num_collisions = (int)child->conflicts.size();
                all_nodes.push_back(child);
                dfs_stack.push(child);
            }
        }
        for (auto* nd : all_nodes) if (nd != best_node) delete nd;
    }

    // --- Commit best node paths ---
    for (int idx = 0; idx < n; idx++) {
        int aid = pbs_ids[idx];
        for (int t = 0; t < max_t; t++) {
            token.path[aid][t] = best_node->paths[idx][t];
            agents[aid].path[t] = best_node->paths[idx][t];
        }
        // Find actual arrival at goal (first goal in goal_seqs)
        int goal_loc = goal_seqs[idx][0].first;
        int arrive = (int)token.timestep;
        for (int t = (int)token.timestep; t < max_t; t++) {
            if (best_node->paths[idx][t] == goal_loc) { arrive = t; break; }
        }
        agents[aid].finish_time = arrive;
        if (pbs_tasks_vec[idx] != nullptr) {
            agents[aid].status = AG_MOVING_TO_PICKUP;
            agents[aid].current_task = pbs_tasks_vec[idx]->id;
            pbs_tasks_vec[idx]->status = aid;
            agent_pending_task[aid] = pbs_tasks_vec[idx];
        }
    }
    delete best_node;
}

// ============================================================
// Section 12: Path Planning — Prioritized Planning (PP)
//   (Pseudocode Section 12.1)
//   Uses phase2_* members set by assign_hungarian()
// ============================================================

void Simulation::path_planning_pp() {
    if (phase2_free_ids_.empty()) return;

    for (int i = 0; i < (int)phase2_free_ids_.size(); i++) {
        int aid = phase2_free_ids_[i];
        int ep_idx = phase2_goal_eps_[i];
        if (ep_idx < 0) { agents[aid].finish_time = token.timestep + 1; continue; }

        int arrive = astar(agents[aid], agents[aid].loc, token.timestep,
                           mapd_map.endpoints[ep_idx], aid);
        if (arrive >= 0) {
            for (unsigned int t = token.timestep; t < maxtime; t++)
                token.path[aid][t] = agents[aid].path[t];
            agents[aid].finish_time = arrive;
            if (phase2_tasks_[i] != nullptr) {
                agents[aid].status = AG_MOVING_TO_PICKUP;
                agents[aid].current_task = phase2_tasks_[i]->id;
                phase2_tasks_[i]->status = aid;
                agent_pending_task[aid] = phase2_tasks_[i];
            }
        } else {
            agents[aid].finish_time = token.timestep + 1;
        }
    }
}

// ============================================================
// Section 12.1: HBH-MLA* — Decoupled PP using MLA*
//   (Pseudocode Section 12.1 — Path_Planning_Decoupled with MLA*)
//   Plans paths for all agents that just got assigned tasks.
//   Uses MLA* (multi-label A*) to plan pickup→delivery in one search.
//   Free agents with no task get moved to nearest free endpoint.
// ============================================================

void Simulation::plan_hbh_mla() {
    // Plan paths for agents that were just assigned by centralized greedy,
    // plus handle free agents that couldn't get a task.
    for (int i = 0; i < (int)agents.size(); i++) {
        Agent& ag = agents[i];

        // Skip agents that are already busy (not free, not just-assigned)
        if (ag.finish_time > token.timestep && ag.status != AG_MOVING_TO_PICKUP)
            continue;

        if (ag.current_task < 0 || ag.status != AG_MOVING_TO_PICKUP) {
            // Agent has no task — handle free agent
            if (ag.finish_time <= token.timestep) {
                // Check if agent needs to move off a delivery endpoint
                bool need_move = false;
                for (auto it = token.tasks.begin(); it != token.tasks.end(); it++) {
                    if ((*it)->delivery_loc == (int)ag.loc) { need_move = true; break; }
                }
                if (need_move) {
                    if (move2EP(ag)) {
                        for (unsigned int t = token.timestep; t < token.path[ag.id].size(); t++)
                            token.path[ag.id][t] = ag.path[t];
                    } else {
                        ag.finish_time = token.timestep + 1;
                    }
                } else {
                    ag.finish_time = token.timestep + 1;
                }
            }
            continue;
        }

        Task& task = all_tasks[ag.current_task];

        // Build goals: [pickup, delivery] — MLA* handles both in one search
        vector<pair<int,int>> goals;
        goals.push_back({task.pickup_loc, 0});   // pickup, no release_time constraint
        goals.push_back({task.delivery_loc, 0});  // delivery

        // Use existing TP-style A* for pickup, then delivery
        // This is equivalent to MLA* but uses the proven token constraint system
        int arrive_start = astar(ag, ag.loc, token.timestep,
                                 mapd_map.endpoints[task.pickup], ag.id);
        if (arrive_start < 0) {
            ag.status = AG_FREE;
            ag.current_task = -1;
            task.status = -1;
            ag.finish_time = token.timestep + 1;
            continue;
        }

        int arrive_goal = astar(ag, task.pickup_loc,
                                arrive_start + task.start_wait_time,
                                mapd_map.endpoints[task.delivery], ag.id);
        vector<int> result;
        if (arrive_goal >= 0) {
            // Build result from agent's path
            for (unsigned int t = token.timestep; t <= (unsigned int)arrive_goal; t++)
                result.push_back(ag.path[t]);
        }

        if (arrive_goal >= 0) {
            // Path already written to ag.path by astar() calls
            // Write to token.path
            for (unsigned int t = token.timestep; t < maxtime; t++)
                token.path[ag.id][t] = ag.path[t];

            task.ag_arrive_start = arrive_start;
            task.status = ag.id;
            task.completion_time = arrive_goal;

            ag.finish_time = arrive_goal + task.goal_wait_time;
            token.tasks.remove(&task);
        } else {
            // MLA* failed — revert agent state
            ag.status = AG_FREE;
            ag.current_task = -1;
            task.status = -1;
            ag.finish_time = token.timestep + 1;
        }
    }
}

// ============================================================
// Section 13: Single-Agent Search — Space-Time A* (STA*)
//   (Pseudocode Section 13.1)
// ============================================================

bool Simulation::isConstrained(int agent_id, int curr_id, int next_id, int next_timestep, int ag_hide) {
    if (!token.my_map[next_id]) return true;
    for (int ag = 0; ag < (int)token.path.size(); ag++) {
        if (ag == agent_id || ag == ag_hide) continue;
        if (token.path[ag][next_timestep] == (unsigned int)next_id) return true;
        if (token.path[ag][next_timestep - 1] == (unsigned int)next_id &&
            token.path[ag][next_timestep] == (unsigned int)curr_id) return true;
    }
    return false;
}

int Simulation::astar(Agent& ag, int start_loc, int begin_time, const Endpoint& goal, int ag_hide) {
    int goal_location = goal.loc;
    heap_open_t open_list;
    map<unsigned int, SearchNode*> allNodes;

    SearchNode* start = new SearchNode(start_loc, 0, goal.h_val[start_loc], nullptr, begin_time);
    open_list.push(start);
    allNodes.insert(make_pair((unsigned int)start_loc, start));

    while (!open_list.empty()) {
        SearchNode* curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;

        if (curr->loc == goal_location) {
            bool can_hold = true;
            for (unsigned int i = curr->timestep + 1; i < maxtime; i++) {
                for (int j = 0; j < (int)token.path.size(); j++) {
                    if (j != ag.id && j != ag_hide && (int)token.path[j][i] == curr->loc) {
                        can_hold = false; break;
                    }
                }
                if (!can_hold) break;
            }
            if (can_hold) {
                updatePath(ag, *curr);
                int t = curr->timestep;
                releaseNodes(allNodes);
                return t;
            }
        }

        if ((unsigned int)curr->timestep >= maxtime - 1) continue;

        int action[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};
        for (int i = 0; i < 5; i++) {
            int next_id = curr->loc + action[i];
            int next_timestep = curr->timestep + 1;
            if (!isConstrained(ag.id, curr->loc, next_id, next_timestep, ag_hide)) {
                int next_g = curr->g_val + 1;
                int next_h = goal.h_val[next_id];
                SearchNode* next = new SearchNode(next_id, next_g, next_h, curr, next_timestep);
                unsigned int key = next->loc + next->g_val * mapd_map.row * mapd_map.col;
                if (allNodes.find(key) == allNodes.end()) {
                    allNodes.insert(make_pair(key, next));
                    open_list.push(next);
                } else {
                    delete next;
                }
            }
        }
    }

    releaseNodes(allNodes);
    return -1;
}

void Simulation::updatePath(Agent& ag, const SearchNode& goal_node) {
    for (unsigned int i = goal_node.timestep + 1; i < ag.path.size(); i++)
        ag.path[i] = goal_node.loc;
    const SearchNode* curr = &goal_node;
    while (curr != nullptr) {
        ag.path[curr->timestep] = curr->loc;
        curr = curr->parent;
    }
}

// ============================================================
// Token-based MLA*: plan pickup+delivery in one search
//   Uses isConstrained() against token.path.
//   Returns (arrive_start, arrive_goal) or (-1,-1) on failure.
//   Writes planned path to ag.path.
// ============================================================

struct MLATokenNode {
    int loc, g_val, h_val, timestep, goal_id;
    MLATokenNode* parent;
    MLATokenNode(int l, int g, int h, int t, int gi, MLATokenNode* p)
        : loc(l), g_val(g), h_val(h), timestep(t), goal_id(gi), parent(p) {}
    int f() const { return g_val + h_val; }
};

struct CmpMLAToken {
    bool operator()(const MLATokenNode* a, const MLATokenNode* b) const {
        if (a->f() != b->f()) return a->f() > b->f();
        return a->g_val <= b->g_val;
    }
};

pair<int,int> Simulation::token_mla_star(Agent& ag, Task& task, int ag_hide) {
    int goal_locs[2] = {task.pickup_loc, task.delivery_loc};
    int goal_release[2] = {task.release_time, 0};
    int num_goals = 2;

    const vector<int>* h_ptrs[2] = {
        &mapd_map.endpoints[task.pickup].h_val,
        &mapd_map.endpoints[task.delivery].h_val
    };

    auto compute_h = [&](int loc, int gi) -> int {
        if (gi >= num_goals) return 0;
        int h = (*h_ptrs[gi])[loc];
        if (h == INT_MAX) return INT_MAX;
        for (int g = gi; g < num_goals - 1; g++) {
            int d = (*h_ptrs[g + 1])[goal_locs[g]];
            if (d == INT_MAX) return INT_MAX;
            h += d;
        }
        return h;
    };

    int start_h = compute_h(ag.loc, 0);
    if (start_h == INT_MAX) return {-1, -1};

    priority_queue<MLATokenNode*, vector<MLATokenNode*>, CmpMLAToken> mla_open;
    vector<MLATokenNode*> mla_all;
    map<tuple<int,int,int>, MLATokenNode*> mla_closed;

    auto* mla_start = new MLATokenNode(ag.loc, 0, start_h, token.timestep, 0, nullptr);
    mla_open.push(mla_start);
    mla_all.push_back(mla_start);

    MLATokenNode* mla_solution = nullptr;
    int mla_max_t = min((int)maxtime, (int)token.timestep + 1000);

    while (!mla_open.empty() && (int)mla_all.size() < 200000) {
        auto* curr = mla_open.top(); mla_open.pop();

        int gi = curr->goal_id;
        if (gi < num_goals && curr->loc == goal_locs[gi]
            && curr->timestep >= goal_release[gi]) {
            gi++;
        }

        if (gi >= num_goals) {
            bool can_hold = true;
            for (unsigned int t = curr->timestep + 1; t < maxtime; t++) {
                for (int j = 0; j < (int)token.path.size(); j++) {
                    if (j == ag.id || j == ag_hide) continue;
                    if ((int)token.path[j][t] == curr->loc) {
                        can_hold = false; break;
                    }
                }
                if (!can_hold) break;
            }
            if (can_hold) { mla_solution = curr; break; }
        }

        if (curr->timestep >= mla_max_t - 1) continue;

        auto key = make_tuple(curr->loc, gi, curr->g_val);
        if (mla_closed.count(key)) continue;
        mla_closed[key] = curr;

        int action[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};
        for (int a = 0; a < 5; a++) {
            int nloc = curr->loc + action[a];
            int nt = curr->timestep + 1;
            if (!isConstrained(ag.id, curr->loc, nloc, nt, ag_hide >= 0 ? ag_hide : ag.id)) {
                int ngi = gi;
                if (ngi < num_goals && nloc == goal_locs[ngi]
                    && nt >= goal_release[ngi]) {
                    ngi++;
                }
                int nh = compute_h(nloc, ngi);
                if (nh == INT_MAX) continue;
                auto nkey = make_tuple(nloc, ngi, curr->g_val + 1);
                if (mla_closed.count(nkey)) continue;
                auto* child = new MLATokenNode(nloc, curr->g_val + 1, nh, nt, ngi, curr);
                mla_open.push(child);
                mla_all.push_back(child);
            }
        }
    }

    if (!mla_solution) {
        for (auto* n : mla_all) delete n;
        return {-1, -1};
    }

    // Extract path
    vector<int> result;
    for (auto* n = mla_solution; n; n = n->parent)
        result.push_back(n->loc);
    reverse(result.begin(), result.end());

    // Write to ag.path
    for (int t = 0; t < (int)result.size() && ((int)token.timestep + t) < (int)maxtime; t++)
        ag.path[token.timestep + t] = result[t];
    int end_loc = result.back();
    for (unsigned int t = token.timestep + result.size(); t < maxtime; t++)
        ag.path[t] = end_loc;

    // Extract arrival times
    int arrive_start = -1;
    for (int t = 0; t < (int)result.size(); t++) {
        int abs_t = (int)token.timestep + t;
        if (result[t] == task.pickup_loc && abs_t >= task.release_time) {
            arrive_start = abs_t; break;
        }
    }
    int arrive_goal = (int)token.timestep + (int)result.size() - 1;

    for (auto* n : mla_all) delete n;

    if (arrive_start < 0) return {-1, -1};
    return {arrive_start, arrive_goal};
}

// ============================================================
// plan_task_token: switch between 2x A* and token MLA*
//   based on config.single_agent
// ============================================================

pair<int,int> Simulation::plan_task_token(Agent& ag, Task& task, int ag_hide) {
    if (config.single_agent == SA_MLA_SEQUENCE) {
        return token_mla_star(ag, task, ag_hide);
    }

    // Default: 2x sequential A* (STA_TASK_EP)
    int hide = (ag_hide >= 0) ? ag_hide : ag.id;
    vector<unsigned int> saved_path(ag.path.begin(), ag.path.end());

    int arrive_start = astar(ag, ag.loc, token.timestep,
                             mapd_map.endpoints[task.pickup], hide);
    if (arrive_start < 0) return {-1, -1};

    int arrive_goal = astar(ag, task.pickup_loc,
                            arrive_start + task.start_wait_time,
                            mapd_map.endpoints[task.delivery], hide);
    if (arrive_goal < 0) {
        ag.path.assign(saved_path.begin(), saved_path.end());
        return {-1, -1};
    }

    return {arrive_start, arrive_goal};
}

// ============================================================
// Section 14: Dummy Path / Move to Endpoint (BFS)
//   (Pseudocode Section 12.0)
// ============================================================

bool Simulation::move2EP(Agent& ag) {
    queue<SearchNode*> Q;
    map<unsigned int, SearchNode*> allNodes;
    int action[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    SearchNode* start = new SearchNode(ag.loc, 0, nullptr, token.timestep);
    allNodes.insert(make_pair((unsigned int)ag.loc, start));
    Q.push(start);

    while (!Q.empty()) {
        SearchNode* v = Q.front(); Q.pop();
        if ((unsigned int)v->timestep >= maxtime - 1) continue;

        if (token.my_endpoints[v->loc]) {
            bool occupied = false;
            for (unsigned int t = v->timestep; t < maxtime && !occupied; t++)
                for (int i = 0; i < (int)agents.size() && !occupied; i++)
                    if (i != ag.id && token.path[i][t] == (unsigned int)v->loc) occupied = true;
            if (!occupied)
                for (auto it = token.tasks.begin(); it != token.tasks.end() && !occupied; it++)
                    if ((*it)->delivery_loc == v->loc) occupied = true;
            if (!occupied) {
                updatePath(ag, *v);
                ag.finish_time = v->timestep;
                releaseNodes(allNodes);
                return true;
            }
        }

        for (int i = 0; i < 5; i++) {
            int next_id = v->loc + action[i];
            int next_t = v->timestep + 1;
            if (!isConstrained(ag.id, v->loc, next_id, next_t, ag.id)) {
                unsigned int key = next_id + (unsigned int)(v->g_val + 1) * mapd_map.row * mapd_map.col;
                if (allNodes.find(key) == allNodes.end()) {
                    SearchNode* next = new SearchNode(next_id, v->g_val + 1, v, next_t);
                    allNodes.insert(make_pair(key, next));
                    Q.push(next);
                }
            }
        }
    }

    releaseNodes(allNodes);
    return false;
}

// ============================================================
// Section 15: Helpers
// ============================================================

void Simulation::releaseNodes(map<unsigned int, SearchNode*>& table) {
    for (auto& p : table) delete p.second;
    table.clear();
}

bool Simulation::fullCollisionCheck(const string& alg_name) const {
    bool ok = true;
    int collision_count = 0;
    int num_ag = token.path.size();
    unsigned int T = maxtime;

    for (int a1 = 0; a1 < num_ag; a1++) {
        for (int a2 = a1 + 1; a2 < num_ag; a2++) {
            for (unsigned int t = 0; t < T; t++) {
                if (token.path[a1][t] == token.path[a2][t]) {
                    if (collision_count < 10)
                        cout << "[" << alg_name << "] VERTEX COLLISION: agents " << a1 << " and " << a2
                             << " at loc " << token.path[a1][t] << " time " << t << endl;
                    ok = false; collision_count++;
                }
                if (t > 0 && token.path[a1][t] == token.path[a2][t-1] &&
                    token.path[a1][t-1] == token.path[a2][t]) {
                    if (collision_count < 10)
                        cout << "[" << alg_name << "] EDGE COLLISION: agents " << a1 << " and " << a2
                             << " on edge " << token.path[a1][t-1] << "-" << token.path[a1][t]
                             << " time " << t << endl;
                    ok = false; collision_count++;
                }
            }
        }
    }
    if (ok) cout << "[" << alg_name << "] COLLISION CHECK PASSED" << endl;
    else cout << "[" << alg_name << "] COLLISION CHECK FAILED - " << collision_count << " collisions" << endl;
    return ok;
}

void Simulation::showTask() const {
    int makespan = 0;
    int total_wait = 0;
    int tasks_done = 0;

    for (auto& t : all_tasks) {
        if (t.completion_time > 0) {
            tasks_done++;
            if (t.completion_time > makespan) makespan = t.completion_time;
            total_wait += (t.completion_time - t.release_time);
        }
    }

    cout << "Finishing Timestep:\t" << makespan << endl;
    cout << "Sum of Task Waiting Time:\t" << total_wait << endl;
    cout << "Tasks completed:\t" << tasks_done << "/" << all_tasks.size() << endl;
}

void Simulation::saveOutput(const string& filepath, double runtime_ms) const {
    ofstream out(filepath);
    if (!out.is_open()) {
        cerr << "Error: cannot open output file: " << filepath << endl;
        return;
    }

    // Compute metrics
    int makespan = 0;
    int total_wait = 0;
    int tasks_done = 0;
    for (auto& t : all_tasks) {
        if (t.completion_time > 0) {
            tasks_done++;
            if (t.completion_time > makespan) makespan = t.completion_time;
            total_wait += (t.completion_time - t.release_time);
        }
    }

    // Header
    out << "# MAPD Framework Output" << endl;
    out << "# Algorithm: " << config.name << endl;
    out << "# Map: " << mapd_map.raw_row << "x" << mapd_map.raw_col
        << " (" << mapd_map.workpoint_num << " task eps, "
        << mapd_map.num_agents << " agents)" << endl;
    out << "# Tasks: " << all_tasks.size() << endl;
    out << "# Makespan: " << makespan << endl;
    out << "# Sum of Task Waiting Time: " << total_wait << endl;
    out << "# Tasks completed: " << tasks_done << "/" << all_tasks.size() << endl;
    out << "# Runtime: " << runtime_ms << " ms" << endl;
    out << endl;

    // Agent paths: one line per agent, space-separated locations
    out << "# Agent paths (agent_id: loc_t0 loc_t1 ... loc_makespan)" << endl;
    for (int i = 0; i < (int)agents.size(); i++) {
        out << i << ":";
        for (int t = 0; t <= makespan && t < (int)maxtime; t++)
            out << " " << agents[i].path[t];
        out << endl;
    }
    out << endl;

    // Task completions
    out << "# Tasks (task_id release_time pickup_loc delivery_loc agent completion_time)" << endl;
    for (auto& t : all_tasks) {
        out << t.id << " " << t.release_time << " " << t.pickup_loc
            << " " << t.delivery_loc << " " << t.status
            << " " << t.completion_time << endl;
    }

    out.close();
}

// ============================================================
// Section 16: TA-Prioritized — Task Assignment (Tour File Parsing)
//   Parse LKH3 tour file, build per-agent task sequences.
//   Tour format: node IDs 1..agent_cnt = agent depots,
//                agent_cnt+1..node_cnt = tasks (task id = node - agent_cnt - 1)
// ============================================================

void Simulation::assign_ta_tsp() {
    if (tour_file_.empty()) {
        cerr << "Error: TA_PRIORITIZED requires --tour parameter" << endl;
        exit(1);
    }

    int agent_cnt = (int)agents.size();
    int task_cnt = (int)all_tasks.size();
    int node_cnt = task_cnt + agent_cnt;

    ifstream fin(tour_file_);
    if (!fin.is_open()) {
        cerr << "Tour file not found: " << tour_file_ << endl;
        exit(1);
    }

    // Skip to TOUR_SECTION
    string line;
    while (getline(fin, line)) {
        if (line.find("TOUR_SECTION") != string::npos)
            break;
    }

    // Parse tour: build per-sequence task queues
    // Each sequence starts with a depot node (1..agent_cnt)
    vector<deque<int>> tsp_seqs;          // tsp_seqs[seq_id] = list of task ids
    vector<int> seq_agent_map;            // seq_agent_map[seq_id] = agent id

    deque<int> current_seq;
    int current_agent = -1;
    bool first_depot = true;

    for (int i = 0; i < node_cnt; i++) {
        int u;
        fin >> u;
        if (u <= agent_cnt) {
            // This is a depot node for agent (u-1)
            if (!first_depot) {
                tsp_seqs.push_back(current_seq);
                seq_agent_map.push_back(current_agent);
                current_seq.clear();
            }
            current_agent = u - 1;
            first_depot = false;
        } else {
            // This is a task node: task_id = u - agent_cnt - 1
            int task_id = u - agent_cnt - 1;
            current_seq.push_back(task_id);
        }
    }
    // Push the last sequence
    if (!first_depot) {
        tsp_seqs.push_back(current_seq);
        seq_agent_map.push_back(current_agent);
    }
    fin.close();

    // Assign task sequences to agents
    for (int s = 0; s < (int)tsp_seqs.size(); s++) {
        int aid = seq_agent_map[s];
        agents[aid].task_sequence = tsp_seqs[s];
    }

    // Mark all tasks as assigned (remove from token.tasks)
    // For offline mode, we handle all tasks in planning
    // token.tasks will be cleared after planning
}

// ============================================================
// Section 17: TA-Prioritized — BFS All-Pairs Distances
// ============================================================

void Simulation::compute_all_pairs_bfs() {
    int map_size = mapd_map.row * mapd_map.col;
    all_pairs_dist_.resize(map_size);
    for (int i = 0; i < map_size; i++) {
        all_pairs_dist_[i].resize(map_size, INT_MAX);
        if (!mapd_map.grid[i]) continue;
        all_pairs_dist_[i][i] = 0;
        queue<int> Q;
        Q.push(i);
        while (!Q.empty()) {
            int u = Q.front(); Q.pop();
            for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                int v = u + d;
                if (v >= 0 && v < map_size &&
                    abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                    mapd_map.grid[v] &&
                    all_pairs_dist_[i][v] > all_pairs_dist_[i][u] + 1) {
                    all_pairs_dist_[i][v] = all_pairs_dist_[i][u] + 1;
                    Q.push(v);
                }
            }
        }
    }
}

// ============================================================
// Section 18: TA-Prioritized — Path Planning
//   Sort agents by estimated makespan (descending), then plan
//   each agent's full task sequence with two-phase A*.
// ============================================================

void Simulation::plan_ta_prioritized() {
    int num_ag = (int)agents.size();
    int map_size = mapd_map.row * mapd_map.col;

    // Build the sequence order (matching tour file order) for tie-breaking.
    vector<int> ta_seq_order;
    {
        ifstream fin2(tour_file_);
        string line2;
        while (getline(fin2, line2))
            if (line2.find("TOUR_SECTION") != string::npos) break;
        int agent_cnt = (int)agents.size();
        int task_cnt = (int)all_tasks.size();
        int node_cnt = task_cnt + agent_cnt;
        bool first = true;
        int cur_ag = -1;
        for (int i = 0; i < node_cnt; i++) {
            int u; fin2 >> u;
            if (u <= agent_cnt) {
                if (!first) ta_seq_order.push_back(cur_ag);
                cur_ag = u - 1;
                first = false;
            }
        }
        if (!first) ta_seq_order.push_back(cur_ag);
        fin2.close();
    }

    // Map agent_id -> seq_index (tour file order)
    vector<int> agent_to_seq(num_ag, 0);
    for (int j = 0; j < (int)ta_seq_order.size(); j++)
        agent_to_seq[ta_seq_order[j]] = j;

    // Use pre-computed endpoint h_vals for distance estimation.
    // endpoints[task.pickup].h_val[loc] = BFS distance from endpoints[task.pickup].loc to loc
    // This replaces the expensive all-pairs BFS.

    // For each agent's parking endpoint: find the endpoint index for agent's initial_loc
    // Agent i's parking is endpoints[workpoint_num + i]
    int wp = mapd_map.workpoint_num;

    // Estimate completion time for each sequence using endpoint h_vals
    struct SeqInfo {
        int agent_id;
        int seq_idx;      // tour file sequence order (for tie-breaking)
        int est_makespan;
    };
    vector<SeqInfo> seq_infos;

    for (int i = 0; i < num_ag; i++) {
        int t = 0;
        int loc = agents[i].initial_loc;
        for (int task_id : agents[i].task_sequence) {
            Task& task = all_tasks[task_id];
            // Distance from loc to pickup_loc = endpoints[task.pickup].h_val[loc]
            int d_to_pickup = mapd_map.endpoints[task.pickup].h_val[loc];
            if (d_to_pickup == INT_MAX) d_to_pickup = 0;
            t += d_to_pickup;
            if (t < task.release_time)
                t = task.release_time;
            // Distance from pickup_loc to delivery_loc = endpoints[task.delivery].h_val[task.pickup_loc]
            int d_to_delivery = mapd_map.endpoints[task.delivery].h_val[task.pickup_loc];
            if (d_to_delivery == INT_MAX) d_to_delivery = 0;
            t += d_to_delivery;
            loc = task.delivery_loc;
        }
        seq_infos.push_back({i, agent_to_seq[i], t});
    }

    // Sort by estimated makespan descending, tie-break by lower seq_idx first
    sort(seq_infos.begin(), seq_infos.end(),
         [](const SeqInfo& a, const SeqInfo& b) {
             if (a.est_makespan != b.est_makespan)
                 return a.est_makespan > b.est_makespan;
             return a.seq_idx < b.seq_idx;
         });

    // Plan paths for each agent in priority order
    for (int pi = 0; pi < num_ag; pi++) {
        int aid = seq_infos[pi].agent_id;
        Agent& ag = agents[aid];

        cerr << "Planning agent " << pi << "/" << num_ag
             << " (agent " << aid << ", " << ag.task_sequence.size() << " tasks, est="
             << seq_infos[pi].est_makespan << ")" << endl;

        // Build constraint paths from all already-planned agents
        vector<vector<int>> cons_paths;
        for (int j = 0; j < pi; j++) {
            int other_aid = seq_infos[j].agent_id;
            vector<int> cp(maxtime);
            for (unsigned int t = 0; t < maxtime; t++)
                cp[t] = (int)agents[other_aid].path[t];
            cons_paths.push_back(cp);
        }

        int park_loc = ag.initial_loc;

        if (config.single_agent == SA_MLA_SEQUENCE) {
            for (int j = pi + 1; j < num_ag; j++) {
                int other_aid = seq_infos[j].agent_id;
                int other_park = agents[other_aid].initial_loc;
                vector<int> park_cons(maxtime, other_park);
                cons_paths.push_back(park_cons);
            }

            if (config.mla_mode == MLA_SEQ) {
                // SeqMLA*: plan ALL tasks in one search
                vector<pair<int,int>> all_goals;
                for (int task_id : ag.task_sequence) {
                    Task& task = all_tasks[task_id];
                    all_goals.push_back({task.pickup_loc, task.release_time});
                    all_goals.push_back({task.delivery_loc, 0});
                }
                all_goals.push_back({park_loc, 0});

                vector<int> path = seq_mla_star(aid, ag.initial_loc, 0,
                                                 all_goals, cons_paths, {}, false);
                if (path.empty()) {
                    cerr << "SeqMLA* failed for agent " << aid << endl;
                } else {
                    for (int t = 0; t < (int)path.size() && t < (int)maxtime; t++)
                        ag.path[t] = path[t];
                    int el = path.back();
                    for (int t = (int)path.size(); t < (int)maxtime; t++)
                        ag.path[t] = el;

                    // Extract per-task arrival times
                    int gi = 0;
                    int task_idx = 0;
                    for (int t = 0; t < (int)path.size() && gi < (int)all_goals.size() - 1; t++) {
                        if (path[t] == all_goals[gi].first && t >= all_goals[gi].second) {
                            if (gi % 2 == 0) {
                                int tid = ag.task_sequence[task_idx];
                                all_tasks[tid].ag_arrive_start = t;
                                all_tasks[tid].status = aid;
                            } else {
                                int tid = ag.task_sequence[task_idx];
                                all_tasks[tid].completion_time = t;
                                task_idx++;
                            }
                            gi++;
                        }
                    }
                }
            } else {
                // Task-by-task MLA* (default for TA-Prioritized)
                int cur_loc = ag.initial_loc;
                int cur_time = 0;

                for (int task_id : ag.task_sequence) {
                    Task& task = all_tasks[task_id];

                    vector<pair<int,int>> goals;
                    goals.push_back({task.pickup_loc, task.release_time});
                    goals.push_back({task.delivery_loc, 0});
                    goals.push_back({park_loc, 0});

                    vector<int> path = seq_mla_star(aid, cur_loc, cur_time,
                                                     goals, cons_paths, {}, false);

                    if (path.empty()) {
                        cerr << "MLA* failed for agent " << aid
                             << " task " << task_id << endl;
                        break;
                    }

                    for (int t = cur_time; t < (int)path.size() && t < (int)maxtime; t++)
                        ag.path[t] = path[t];
                    int el = path.back();
                    for (int t = (int)path.size(); t < (int)maxtime; t++)
                        ag.path[t] = el;

                    int pickup_t = -1, delivery_t = -1;
                    int gi = 0;
                    for (int t = cur_time; t < (int)path.size() && gi < 2; t++) {
                        if (path[t] == goals[gi].first && t >= goals[gi].second) {
                            if (gi == 0) pickup_t = t;
                            else delivery_t = t;
                            gi++;
                        }
                    }

                    task.ag_arrive_start = pickup_t;
                    task.completion_time = delivery_t;
                    task.status = aid;

                    if (delivery_t >= 0) {
                        cur_loc = task.delivery_loc;
                        cur_time = delivery_t;
                    }
                }
            }
        } else {
            // Default: 2x astar_with_dummy per task
            const vector<int>& h_park = mapd_map.endpoints[wp + aid].h_val;
            int t = 0;

            for (int task_id : ag.task_sequence) {
                Task& task = all_tasks[task_id];
                const vector<int>& h_pickup = mapd_map.endpoints[task.pickup].h_val;

                int pickup_arrive = astar_with_dummy(ag, ag.path[t], t,
                                                      task.pickup_loc, park_loc,
                                                      h_pickup, h_park,
                                                      cons_paths, task.release_time);
                if (pickup_arrive < 0) {
                    cerr << "Error: astar_with_dummy failed for agent " << aid
                         << " task " << task_id << " (pickup)" << endl;
                    break;
                }

                if (pickup_arrive < task.release_time)
                    pickup_arrive = task.release_time;
                task.ag_arrive_start = pickup_arrive;
                task.status = aid;

                const vector<int>& h_delivery = mapd_map.endpoints[task.delivery].h_val;
                int delivery_arrive = astar_with_dummy(ag, ag.path[pickup_arrive], pickup_arrive,
                                                        task.delivery_loc, park_loc,
                                                        h_delivery, h_park,
                                                        cons_paths, 0);
                if (delivery_arrive < 0) {
                    cerr << "Error: astar_with_dummy failed for agent " << aid
                         << " task " << task_id << " (delivery)" << endl;
                    break;
                }

                task.completion_time = delivery_arrive;
                t = delivery_arrive;
            }
        }

        // Copy agent's path to token.path
        for (unsigned int tt = 0; tt < maxtime; tt++)
            token.path[aid][tt] = ag.path[tt];
    }

    // Clear token.tasks since all are processed
    token.tasks.clear();
}

// ============================================================
// Section 19: TA-Prioritized — Two-Phase A* (STA with Dummy Check)
//   Matches reference SingleAgentICBS::findPath with goal_optimal=true.
//   Phase 1: A* toward goal_loc using h_goal heuristic
//   Phase 2 (after goal reached): A* toward park_loc using h_park heuristic
//   Terminal: at park_loc AND vis_goal AND can hold forever
//   Returns absolute timestep when goal_loc was reached, or -1 on failure.
//
//   Key details matching reference:
//   - Node identity: (loc, timestep, vis_goal) — three-part key
//   - vis_goal is set when popping a node, NOT during child generation
//   - goal_optimal: once any popped node sets vis_goal, non-vis_goal nodes skip expansion
//   - release_time is RELATIVE (g_val >= release_time means enough steps from start)
//   - Heuristic switches from h_goal to h_park based on curr->vis_goal (the parent's state)
// ============================================================

struct DummySearchNode {
    int loc;
    int g_val;
    int h_val;
    int timestep;
    DummySearchNode* parent;
    bool in_openlist;
    bool vis_goal;       // has the path through this node visited the sub-goal?
    int goal_length;     // g_val+1 at the time goal was first reached

    DummySearchNode(int l, int g, int h, DummySearchNode* p, int t, bool vg, int gl)
        : loc(l), g_val(g), h_val(h), parent(p), timestep(t), in_openlist(true),
          vis_goal(vg), goal_length(gl) {}

    int getFVal() const { return g_val + h_val; }
};

struct CompareDummyNode {
    // min-heap by f-val, tie-break by higher g-val
    bool operator()(const DummySearchNode* n1, const DummySearchNode* n2) const {
        if (n1->getFVal() != n2->getFVal()) return n1->getFVal() > n2->getFVal();
        return n1->g_val <= n2->g_val;
    }
};

int Simulation::astar_with_dummy(Agent& ag, int start_loc, int start_time,
                                  int goal_loc, int park_loc,
                                  const vector<int>& h_goal, const vector<int>& h_park,
                                  const vector<vector<int>>& cons_paths,
                                  int release_time) {
    int rel_release = release_time - start_time;
    if (rel_release < 0) rel_release = 0;

    typedef priority_queue<DummySearchNode*, vector<DummySearchNode*>, CompareDummyNode> dummy_heap_t;
    dummy_heap_t open_list;
    unordered_map<unsigned int, DummySearchNode*> allNodes;

    int initial_h = h_goal[start_loc];
    if (initial_h == INT_MAX) initial_h = 0;

    int map_size = mapd_map.row * mapd_map.col;
    int max_t = (int)maxtime;
    int action[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    DummySearchNode* start_node = new DummySearchNode(start_loc, 0, initial_h, nullptr,
                                                       start_time, false, 0);
    allNodes[(unsigned int)start_loc] = start_node;
    open_list.push(start_node);

    bool global_vis_goal = false;

    while (!open_list.empty()) {
        DummySearchNode* curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;

        // goal_optimal: after goal found, skip nodes that haven't visited goal
        bool do_expand = true;
        if (global_vis_goal && !curr->vis_goal && goal_loc != park_loc)
            do_expand = false;

        // Mark goal reached (at pop time, matching reference)
        if (!curr->vis_goal && curr->loc == goal_loc && curr->g_val >= rel_release) {
            curr->vis_goal = true;
            curr->goal_length = curr->g_val + 1;
            global_vis_goal = true;
        }

        // Terminal: at parking AND visited goal AND can hold forever
        if (curr->vis_goal && curr->loc == park_loc) {
            bool can_hold = true;
            for (int ci = 0; ci < (int)cons_paths.size() && can_hold; ci++) {
                for (int t = curr->timestep + 1; t < max_t && can_hold; t++) {
                    if (cons_paths[ci][t] == curr->loc)
                        can_hold = false;
                }
            }
            if (can_hold) {
                int goal_arrival_abs = start_time + curr->goal_length - 1;
                vector<int> path_locs;
                DummySearchNode* node = curr;
                while (node != nullptr) {
                    path_locs.push_back(node->loc);
                    node = node->parent;
                }
                reverse(path_locs.begin(), path_locs.end());

                for (int i = 0; i < (int)path_locs.size(); i++) {
                    int t = start_time + i;
                    if (t < max_t) ag.path[t] = path_locs[i];
                }
                for (int t = start_time + (int)path_locs.size(); t < max_t; t++)
                    ag.path[t] = park_loc;

                for (auto& p : allNodes) delete p.second;
                return goal_arrival_abs;
            }
        }

        if (!do_expand) continue;
        if (curr->timestep >= max_t - 1) continue;

        for (int i = 0; i < 5; i++) {
            int next_loc = curr->loc + action[i];
            int next_t = curr->timestep + 1;

            if (next_loc < 0 || next_loc >= map_size) continue;
            if (!mapd_map.grid[next_loc]) continue;
            if (abs(next_loc % mapd_map.col - curr->loc % mapd_map.col) > 1) continue;

            bool constrained = false;
            for (int ci = 0; ci < (int)cons_paths.size() && !constrained; ci++) {
                if (cons_paths[ci][next_t] == next_loc)
                    constrained = true;
                else if (cons_paths[ci][next_t] == curr->loc &&
                         cons_paths[ci][next_t - 1] == next_loc)
                    constrained = true;
            }
            if (constrained) continue;

            int next_g = curr->g_val + 1;
            int next_h;
            if (curr->vis_goal && goal_loc != park_loc)
                next_h = h_park[next_loc];
            else
                next_h = h_goal[next_loc];
            if (next_h == INT_MAX) next_h = 0;

            // Key: loc + g_val * map_size (matching reference scheme)
            unsigned int key = next_loc + (unsigned int)next_g * map_size;
            if (allNodes.find(key) == allNodes.end() && next_g < max_t - start_time) {
                DummySearchNode* next = new DummySearchNode(next_loc, next_g, next_h,
                                                             curr, next_t,
                                                             curr->vis_goal, curr->goal_length);
                allNodes[key] = next;
                open_list.push(next);
            }
        }
    }

    for (auto& p : allNodes) delete p.second;
    return -1;
}

int Simulation::findEndpointIndex(int loc) const {
    for (int i = 0; i < (int)mapd_map.endpoints.size(); i++)
        if (mapd_map.endpoints[i].loc == loc) return i;
    return -1;
}

// ============================================================
// Section 20: CostFlow — Min-Cost Max-Flow Implementation
//   Ported from reference TA-Hybrid/CostFlow.cpp
// ============================================================

CostFlow::CostFlow(int node_cnt, int source, int sink)
    : cost(0), node_cnt(node_cnt), source(source), sink(sink) {
    head.assign(node_cnt, -1);
    pre.assign(node_cnt, -1);
    in_queue.assign(node_cnt, false);
    dis.assign(node_cnt, COSTFLOW_INF);
}

void CostFlow::AddEdge(int from, int to, int capcity, int cost, int loc) {
    CostFlowEdge e;
    e.from = from;
    e.to = to;
    e.origin_capcity = capcity;
    e.capcity = capcity;
    e.cost = cost;
    e.loc = loc;
    e.next = head[from];
    edges.push_back(e);
    head[from] = (int)edges.size() - 1;
}

void CostFlow::AddEdges(int from, int to, int capcity, int cost, int loc) {
    AddEdge(from, to, capcity, cost, loc);
    AddEdge(to, from, 0, -cost, loc);
}

void CostFlow::RemoveEdges(int from, int to) {
    for (int i = head[from]; i != -1; i = edges[i].next)
        if (edges[i].to == to && edges[i].capcity > 0) {
            edges[i].capcity--;
            edges[i].origin_capcity--;
            return;
        }
}

bool CostFlow::SPFA() {
    while (!Q.empty()) Q.pop();
    for (int i = 0; i < node_cnt; i++) {
        dis[i] = COSTFLOW_INF;
        in_queue[i] = false;
    }
    dis[source] = 0;
    Q.push(source);
    in_queue[source] = true;
    while (!Q.empty()) {
        int u = Q.front();
        Q.pop();
        in_queue[u] = false;
        for (int i = head[u]; i != -1; i = edges[i].next)
            if (edges[i].capcity > 0) {
                int v = edges[i].to, c = edges[i].cost;
                int dd = dis[u] + c;
                if (dis[v] > dd) {
                    dis[v] = dd;
                    pre[v] = i;
                    if (!in_queue[v]) {
                        Q.push(v);
                        in_queue[v] = true;
                    }
                }
            }
    }
    return dis[sink] < COSTFLOW_INF;
}

int CostFlow::MinCostFlow() {
    int flow = 0;
    while (SPFA()) {
        flow++;
        int u = sink;
        while (u != source) {
            edges[pre[u]].capcity--;
            edges[pre[u] ^ 1].capcity++;
            cost += edges[pre[u]].cost;
            u = edges[pre[u]].from;
        }
    }
    return flow;
}

vector<vector<int>> CostFlow::GetPath() {
    vector<vector<int>> paths;
    for (int i = head[source]; i != -1; i = edges[i].next)
        if (edges[i].capcity < edges[i].origin_capcity) {
            vector<int> path;
            path.push_back(edges[i].loc);
            int u = edges[i].to;
            while (u != sink) {
                int v = -1;
                for (int j = head[u]; j != -1; j = edges[j].next)
                    if (edges[j].capcity < edges[j].origin_capcity) {
                        edges[j].capcity = edges[j].origin_capcity;
                        v = edges[j].to;
                        if (edges[j].loc != -1)
                            path.push_back(edges[j].loc);
                        u = v;
                        break;
                    }
                if (v == -1) {
                    cerr << "CostFlow::GetPath ERROR" << endl;
                    break;
                }
            }
            paths.push_back(path);
        }
    return paths;
}

// ============================================================
// Section 21: TA-Hybrid — Cost Estimation
//   Estimates completion time for a task sequence starting at time t.
// ============================================================

int Simulation::hybrid_cost(int t, queue<Task*> seq) {
    int loc = -1;
    while (!seq.empty()) {
        Task* task = seq.front();
        seq.pop();
        if (!task->delivering) {
            if (loc != -1)
                t += all_pairs_dist_[loc][task->pickup_loc];
            if (t < task->release_time)
                t = task->release_time;
            t += all_pairs_dist_[task->pickup_loc][task->delivery_loc];
        } else {
            if (loc != -1)
                t += all_pairs_dist_[loc][task->delivery_loc];
            else
                t += all_pairs_dist_[task->pickup_loc][task->delivery_loc];
        }
        loc = task->delivery_loc;
    }
    return t;
}

// ============================================================
// Section 22: TA-Hybrid — CalcFlow (Build and Solve Cost Flow)
//   Builds time-expanded network for Group 2 agents routing to pickups.
// ============================================================

void Simulation::hybrid_calc_flow(vector<Agent*>& flow_agents, vector<Task*>& flow_tasks,
                                   const vector<vector<int>>& cons_paths,
                                   vector<int> len, int& flow,
                                   vector<vector<int>>& paths) {
    int map_size = (int)mapd_map.grid.size();
    int maxtimestep = 0;
    for (int i = 0; i < (int)flow_tasks.size(); i++)
        maxtimestep = max(maxtimestep, len[i] - (int)hybrid_timestep_);
    maxtimestep++;

    // Node layout (computed, no vector-of-vectors):
    //   0 = source, 1 = sink
    //   For each location u and timestep t (0..maxtimestep):
    //     in_node(u, t)  = 2 + (u * (maxtimestep+1) + t) * 2
    //     out_node(u, t) = 2 + (u * (maxtimestep+1) + t) * 2 + 1
    //   task_node(i) = 2 + map_size * (maxtimestep+1) * 2 + i
    int T1 = maxtimestep + 1;
    auto in_node = [&](int u, int t) { return 2 + (u * T1 + t) * 2; };
    auto out_node = [&](int u, int t) { return 2 + (u * T1 + t) * 2 + 1; };
    int task_base = 2 + map_size * T1 * 2;
    int node_cnt = task_base + (int)flow_tasks.size();
    int source = 0, sink = 1;

    // Map location -> task index (for linking task nodes)
    vector<int> loc_task(map_size, -1);
    for (int i = 0; i < (int)flow_tasks.size(); i++)
        loc_task[flow_tasks[i]->pickup_loc] = i;

    CostFlow costflow(node_cnt, source, sink);

    // Source -> agent starting positions
    for (int i = 0; i < (int)flow_agents.size(); i++)
        costflow.AddEdges(source, out_node(flow_agents[i]->loc, 0), 1, 0, i);

    // Time-expanded edges
    int col = mapd_map.col;
    for (int t = 0; t < maxtimestep; t++) {
        for (int u = 0; u < map_size; u++) {
            if (!mapd_map.grid[u]) continue;
            costflow.AddEdges(in_node(u, t), out_node(u, t), 1, 0, -1);
            costflow.AddEdges(out_node(u, t), in_node(u, t + 1), 1, 1, u);

            // If this is a task pickup location and agent shouldn't pass through before hold_time
            if (loc_task[u] != -1 &&
                t + (int)hybrid_timestep_ >= flow_tasks[loc_task[u]]->hold_time &&
                flow_tasks[loc_task[u]]->hold_time != 0)
                continue;

            int offset[4] = {-1, 1, -col, col};
            for (int j = 0; j < 4; j++) {
                int v = u + offset[j];
                if (v >= 0 && v < map_size && abs(u % col - v % col) < 2 && mapd_map.grid[v])
                    costflow.AddEdges(out_node(u, t), in_node(v, t + 1), 1, 1, v);
            }
        }
    }

    // Task nodes -> sink
    // Note: We relax the release_time constraint in the cost flow to keep
    // the network small. Agents can arrive at pickups early and wait.
    // The actual pickup trigger (Group 1) still checks release_time.
    for (int i = 0; i < (int)flow_tasks.size(); i++) {
        Task* task = flow_tasks[i];
        int mint = max(task->hold_time - (int)hybrid_timestep_, 0);
        int maxt = min(len[i] - (int)hybrid_timestep_, maxtimestep - 1);
        for (int t = mint; t <= maxt; t++)
            costflow.AddEdges(out_node(task->pickup_loc, t), task_base + i, 1, 0, -1);
        costflow.AddEdges(task_base + i, sink, 1, 0, -1);
    }

    // Remove vertex constraints from constraint paths
    for (int i = 0; i < (int)cons_paths.size(); i++) {
        for (int t = 1; t < maxtimestep; t++)
            if ((int)hybrid_timestep_ + t < (int)cons_paths[i].size()) {
                int u = cons_paths[i][hybrid_timestep_ + t];
                costflow.RemoveEdges(in_node(u, t), out_node(u, t));
            }
    }

    // Remove edge constraints from constraint paths
    for (int i = 0; i < (int)cons_paths.size(); i++) {
        for (int t = 0; t < maxtimestep; t++)
            if ((int)hybrid_timestep_ + t + 1 < (int)cons_paths[i].size()) {
                int u_next = cons_paths[i][hybrid_timestep_ + t + 1];
                int u_cur = cons_paths[i][hybrid_timestep_ + t];
                costflow.RemoveEdges(out_node(u_next, t), in_node(u_cur, t + 1));
            }
    }

    flow = costflow.MinCostFlow();
    paths = costflow.GetPath();
}

// ============================================================
// Section 23: TA-Hybrid — GoHome (Plan dummy paths to parking)
//   After Group 2 routes agents to pickups, plan dummy paths from
//   arrival point to parking location using astar_with_dummy.
// ============================================================

int Simulation::hybrid_go_home(vector<Agent*>& ags) {
    int map_size = (int)mapd_map.grid.size();

    for (int i = 0; i < (int)ags.size(); i++) {
        int t = ags[i]->dummy_start_step;
        int start_loc = ags[i]->path[t];
        int park_loc = ags[i]->park_loc;

        // Build constraint paths from all other agents
        vector<vector<int>> cons_paths;
        for (int j = 0; j < (int)agents.size(); j++) {
            if (agents[j].id != ags[i]->id) {
                vector<int> cp(maxtime);
                for (unsigned int tt = 0; tt < maxtime; tt++)
                    cp[tt] = (int)agents[j].path[tt];
                cons_paths.push_back(cp);
            }
        }

        // BFS heuristics for goal=start_loc and park=park_loc
        // For GoHome, goal_loc = start_loc (trivial, already there)
        // park_loc = agent's parking location
        // Use endpoint h_vals if available, otherwise compute inline
        vector<int> h_goal(map_size, 0);  // goal is start_loc, h=0 everywhere since we're already there
        // Actually compute proper BFS from start_loc
        {
            queue<int> bfs_q;
            h_goal.assign(map_size, INT_MAX);
            h_goal[start_loc] = 0;
            bfs_q.push(start_loc);
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                    int v = u + d;
                    if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                        mapd_map.grid[v] && h_goal[v] > h_goal[u] + 1) {
                        h_goal[v] = h_goal[u] + 1;
                        bfs_q.push(v);
                    }
                }
            }
        }

        vector<int> h_park_vec(map_size, INT_MAX);
        {
            queue<int> bfs_q;
            h_park_vec[park_loc] = 0;
            bfs_q.push(park_loc);
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                    int v = u + d;
                    if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                        mapd_map.grid[v] && h_park_vec[v] > h_park_vec[u] + 1) {
                        h_park_vec[v] = h_park_vec[u] + 1;
                        bfs_q.push(v);
                    }
                }
            }
        }

        // Use astar_with_dummy: goal_loc=start_loc (trivially reached), park_loc=parking
        int result = astar_with_dummy(*ags[i], start_loc, t,
                                       start_loc, park_loc,
                                       h_goal, h_park_vec,
                                       cons_paths, 0);
        if (result < 0) {
            cerr << "GoHome Error! agent " << ags[i]->id
                 << " start=" << start_loc << " park=" << park_loc << endl;
            return start_loc;
        }
    }
    return -1;
}

// ============================================================
// Section 24: TA-Hybrid — ReplanDummyPath
//   Check if a given agent's dummy path collides with others.
//   If so, replan it.
// ============================================================

bool Simulation::hybrid_replan_dummy(Agent* ag) {
    if (ag->dummy_start_step >= (int)maxtime - 1)
        return true;

    // Check for collisions from timestep+1 onward
    bool collision = false;
    for (int i = 0; i < (int)agents.size() && !collision; i++) {
        if (agents[i].id == ag->id) continue;
        for (int j = (int)hybrid_timestep_ + 1; !collision && j < (int)maxtime; j++) {
            if ((int)ag->path[j] == (int)agents[i].path[j])
                collision = true;
            else if (j > 0 && (int)ag->path[j] == (int)agents[i].path[j - 1] &&
                     (int)ag->path[j - 1] == (int)agents[i].path[j])
                collision = true;
        }
    }
    if (!collision) return true;

    int t = ag->dummy_start_step;
    int start_loc = ag->path[t];
    int park_loc = ag->park_loc;
    int map_size = (int)mapd_map.grid.size();

    vector<vector<int>> cons_paths;
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].id == ag->id) continue;
        vector<int> cp(maxtime);
        for (unsigned int tt = 0; tt < maxtime; tt++)
            cp[tt] = (int)agents[i].path[tt];
        cons_paths.push_back(cp);
    }

    vector<int> h_goal(map_size, INT_MAX);
    {
        queue<int> bfs_q;
        h_goal[start_loc] = 0;
        bfs_q.push(start_loc);
        while (!bfs_q.empty()) {
            int u = bfs_q.front(); bfs_q.pop();
            for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                int v = u + d;
                if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                    mapd_map.grid[v] && h_goal[v] > h_goal[u] + 1) {
                    h_goal[v] = h_goal[u] + 1;
                    bfs_q.push(v);
                }
            }
        }
    }

    vector<int> h_park_vec(map_size, INT_MAX);
    {
        queue<int> bfs_q;
        h_park_vec[park_loc] = 0;
        bfs_q.push(park_loc);
        while (!bfs_q.empty()) {
            int u = bfs_q.front(); bfs_q.pop();
            for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                int v = u + d;
                if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                    mapd_map.grid[v] && h_park_vec[v] > h_park_vec[u] + 1) {
                    h_park_vec[v] = h_park_vec[u] + 1;
                    bfs_q.push(v);
                }
            }
        }
    }

    int result = astar_with_dummy(*ag, start_loc, t,
                                   start_loc, park_loc,
                                   h_goal, h_park_vec,
                                   cons_paths, 0);
    return (result >= 0);
}

// ============================================================
// Section 25: TA-Hybrid — AssignNewTask (steal tasks from other sequences)
// ============================================================

void Simulation::hybrid_assign_new_task(int id) {
    Task* chosetask = nullptr;
    int tmax = 0, seq_id = -1;

    for (int i = 0; i < (int)agents.size(); i++) {
        if (i == id || hybrid_seqs_[i].empty()) continue;
        if (hybrid_seqs_[i].front()->delivering && (int)hybrid_seqs_[i].size() == 1)
            continue;

        queue<Task*> seq = hybrid_seqs_[i];
        int t = (int)hybrid_timestep_;
        int loc = (int)agents[hybrid_prefer_agent_[i]].path[hybrid_timestep_];
        Task* task = nullptr;
        while (!seq.empty()) {
            task = seq.front();
            seq.pop();
            if (!task->delivering) {
                t += all_pairs_dist_[loc][task->pickup_loc];
                if (t < task->release_time)
                    t = task->release_time;
                t += all_pairs_dist_[task->pickup_loc][task->delivery_loc];
            } else {
                t += all_pairs_dist_[loc][task->delivery_loc];
            }
            loc = task->delivery_loc;
        }

        int prefer_id = hybrid_prefer_agent_[id];
        int d_to_start = all_pairs_dist_[agents[prefer_id].loc][task->pickup_loc];
        int d_start_goal = all_pairs_dist_[task->pickup_loc][task->delivery_loc];
        if (t > tmax &&
            (int)hybrid_timestep_ + d_to_start > task->release_time - 10 &&
            (int)hybrid_timestep_ + d_to_start + d_start_goal < t) {
            tmax = t;
            chosetask = task;
            seq_id = i;
        }
    }

    if (seq_id != -1) {
        // Remove chosetask from seq_id's queue
        queue<Task*> new_seq;
        while (!hybrid_seqs_[seq_id].empty()) {
            Task* task = hybrid_seqs_[seq_id].front();
            hybrid_seqs_[seq_id].pop();
            if (task != chosetask)
                new_seq.push(task);
        }
        hybrid_seqs_[seq_id] = new_seq;
        chosetask->seq_id = id;
        hybrid_seqs_[id].push(chosetask);

        // Recalculate global makespan
        hybrid_global_makespan_ = 0;
        for (int i = 0; i < (int)agents.size(); i++)
            hybrid_global_makespan_ = max(hybrid_global_makespan_,
                                          hybrid_cost((int)hybrid_timestep_, hybrid_seqs_[i]));
    }
}

// ============================================================
// Section 26: TA-Hybrid — Group 1 Delivery Planning
//   Plan delivery paths for agents at pickup locations using
//   prioritized A* with dummy path search.
// ============================================================

bool Simulation::hybrid_group1_plan(vector<Agent*>& delivery_agents,
                                     vector<Agent*>& constraint_agents) {
    if (delivery_agents.empty()) return true;
    int map_size = (int)mapd_map.grid.size();

    for (int i = 0; i < (int)delivery_agents.size(); i++) {
        Agent* ag = delivery_agents[i];
        int goal_loc = ag->goal_loc;
        int park_loc = ag->park_loc;

        // Build constraint paths from all other agents
        vector<vector<int>> cons_paths;
        for (int j = 0; j < (int)agents.size(); j++) {
            if (agents[j].id != ag->id) {
                // Use non_dummy_path up to dummy_start_step, then rest of path
                vector<int> cp(maxtime);
                int ds = agents[j].dummy_start_step;
                for (int t = 0; t <= ds && t < (int)maxtime; t++)
                    cp[t] = (int)agents[j].path[t];
                for (int t = ds + 1; t < (int)maxtime; t++)
                    cp[t] = (int)agents[j].path[t];
                cons_paths.push_back(cp);
            }
        }

        // Compute heuristics
        vector<int> h_goal(map_size, INT_MAX);
        {
            queue<int> bfs_q;
            h_goal[goal_loc] = 0;
            bfs_q.push(goal_loc);
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                    int v = u + d;
                    if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                        mapd_map.grid[v] && h_goal[v] > h_goal[u] + 1) {
                        h_goal[v] = h_goal[u] + 1;
                        bfs_q.push(v);
                    }
                }
            }
        }

        vector<int> h_park_vec(map_size, INT_MAX);
        {
            queue<int> bfs_q;
            h_park_vec[park_loc] = 0;
            bfs_q.push(park_loc);
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                    int v = u + d;
                    if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                        mapd_map.grid[v] && h_park_vec[v] > h_park_vec[u] + 1) {
                        h_park_vec[v] = h_park_vec[u] + 1;
                        bfs_q.push(v);
                    }
                }
            }
        }

        int start_loc = (int)ag->path[hybrid_timestep_];
        int result = astar_with_dummy(*ag, start_loc, (int)hybrid_timestep_,
                                       goal_loc, park_loc,
                                       h_goal, h_park_vec,
                                       cons_paths, 0);
        if (result < 0) {
            cerr << "Group1 delivery planning failed for agent " << ag->id << endl;
            return false;
        }

        // Find the timestep when agent arrives at goal_loc
        int arrive_goal = result;
        ag->dummy_start_step = arrive_goal;
        ag->task_ptr->ag_arrive_goal = arrive_goal;

        // Replan dummy paths for constraint agents that might collide
        for (int j = 0; j < (int)constraint_agents.size(); j++) {
            hybrid_replan_dummy(constraint_agents[j]);
        }
    }
    return true;
}

// ============================================================
// Section 27: TA-Hybrid — Main Per-Timestep Loop
//   This is the core TA-Hybrid algorithm.
// ============================================================

void Simulation::plan_ta_hybrid() {
    int num_ag = (int)agents.size();
    int map_size = (int)mapd_map.grid.size();

    // Compute all-pairs BFS distances
    compute_all_pairs_bfs();

    // Build TA-Hybrid task sequences from agent task_sequences (set by assign_ta_tsp)
    hybrid_seqs_.resize(num_ag);
    hybrid_prefer_agent_.resize(num_ag);
    hybrid_tsp_agent_.resize(num_ag);

    for (int i = 0; i < num_ag; i++) {
        queue<Task*> q;
        for (int task_id : agents[i].task_sequence) {
            all_tasks[task_id].seq_id = i;
            q.push(&all_tasks[task_id]);
        }
        hybrid_seqs_[i] = q;
        hybrid_prefer_agent_[i] = i;  // initially, agent i is preferred for sequence i
        hybrid_tsp_agent_[i] = i;
    }

    // Initialize agent park locations
    for (int i = 0; i < num_ag; i++) {
        agents[i].park_loc = agents[i].initial_loc;
        agents[i].dummy_start_step = (int)maxtime - 1;
        agents[i].delivering = false;
        agents[i].task_ptr = nullptr;
    }

    // Calculate initial global makespan
    hybrid_global_makespan_ = 0;
    for (int i = 0; i < num_ag; i++)
        hybrid_global_makespan_ = max(hybrid_global_makespan_, hybrid_cost(0, hybrid_seqs_[i]));

    cerr << "TA-Hybrid: initial global_makespan = " << hybrid_global_makespan_ << endl;
    cerr << "TA-Hybrid: map_size = " << map_size << ", maxtime = " << maxtime << endl;

    int makespan = 0;
    int cnt = 0;  // completed tasks counter

    // Main per-timestep loop
    for (hybrid_timestep_ = 0; ; hybrid_timestep_++) {
        if (hybrid_timestep_ % 50 == 0)
            cerr << "Timestep " << hybrid_timestep_ << " (completed: " << cnt << "/" << all_tasks.size() << ")" << endl;

        // Check termination
        if ((int)hybrid_timestep_ > t_task) {
            bool finish = true;
            for (int i = 0; i < num_ag; i++)
                if (!hybrid_seqs_[i].empty()) { finish = false; break; }
            if (finish) {
                makespan = (int)hybrid_timestep_ - 1;
                cerr << "TA-Hybrid: completed at makespan = " << makespan << endl;
                break;
            }
        }

        if (hybrid_timestep_ > maxtime - 2) {
            cerr << "TA-Hybrid: exceeded maxtime!" << endl;
            break;
        }

        // Update agent locations
        for (int i = 0; i < num_ag; i++)
            agents[i].loc = agents[i].path[hybrid_timestep_];

        // Try to assign new tasks to empty sequences
        for (int i = 0; i < num_ag; i++)
            if (hybrid_seqs_[i].empty())
                hybrid_assign_new_task(i);

        // Check task completion (delivery done)
        for (int i = 0; i < num_ag; i++) {
            if (hybrid_seqs_[i].empty()) continue;
            Task* task = hybrid_seqs_[i].front();
            if (task->delivering && (int)hybrid_timestep_ == task->ag_arrive_goal) {
                task->delivering = false;
                task->completion_time = task->ag_arrive_goal;
                task->ag->delivering = false;
                task->ag->task_ptr = nullptr;
                hybrid_seqs_[i].pop();
                hybrid_prefer_agent_[i] = task->ag->id;
                cnt++;
                if (hybrid_seqs_[i].empty()) {
                    task->ag->dummy_start_step = (int)maxtime - 1;
                }
            }
        }

        // ============ Group 1: Agents at pickup → plan delivery ============
        vector<Agent*> ag_icbs;
        vector<bool> is_constraint(num_ag, true);

        for (int i = 0; i < num_ag; i++) {
            if (hybrid_seqs_[i].empty()) continue;
            Task* task = hybrid_seqs_[i].front();
            if (!task->delivering &&
                (int)hybrid_timestep_ >= task->release_time &&
                (int)agents[hybrid_prefer_agent_[i]].loc == task->pickup_loc) {
                int aid = hybrid_prefer_agent_[i];
                task->ag_arrive_start = (int)hybrid_timestep_;
                agents[aid].task_ptr = task;
                agents[aid].goal_loc = task->delivery_loc;
                agents[aid].delivering = true;
                ag_icbs.push_back(&agents[aid]);
                task->ag = &agents[aid];
                task->delivering = true;
                is_constraint[aid] = false;
            }
        }

        if (!ag_icbs.empty()) {
            vector<Agent*> cons_agents;
            for (int i = 0; i < num_ag; i++)
                if (is_constraint[i])
                    cons_agents.push_back(&agents[i]);
            hybrid_group1_plan(ag_icbs, cons_agents);
        }

        // ============ Group 2: Free agents → route to pickups via cost flow ============
        vector<bool> taskvis(num_ag, false);
        vector<bool> agentvis(num_ag, false);

        while (true) {
            bool same_dest = false;
            vector<Agent*> ag_costflow;
            vector<Task*> task_costflow;
            vector<bool> prefer(num_ag, false);

            for (int i = 0; i < num_ag; i++) {
                if (hybrid_seqs_[i].empty()) continue;
                Task* task = hybrid_seqs_[i].front();
                if (!taskvis[i] && !task->delivering) {
                    bool flag = false;
                    for (int j = 0; j < (int)task_costflow.size(); j++)
                        if (task_costflow[j]->pickup_loc == task->pickup_loc) {
                            flag = true;
                            same_dest = true;
                        }
                    if (flag) continue;
                    taskvis[i] = true;
                    task_costflow.push_back(task);
                    prefer[hybrid_prefer_agent_[i]] = true;
                }
            }

            vector<bool> in_costflow(num_ag, false);
            for (int i = 0; i < num_ag; i++)
                if (prefer[agents[i].id] && !agentvis[i] && !agents[i].delivering) {
                    ag_costflow.push_back(&agents[i]);
                    agentvis[i] = true;
                    in_costflow[i] = true;
                }

            // Reset hold times
            for (int i = 0; i < (int)task_costflow.size(); i++)
                task_costflow[i]->hold_time = 0;

            // Save non-dummy paths
            for (int i = 0; i < num_ag; i++) {
                vector<int> ndp;
                for (int j = 0; j <= agents[i].dummy_start_step && j < (int)maxtime; j++)
                    ndp.push_back((int)agents[i].path[j]);
                agents[i].non_dummy_path = ndp;
            }

            if (ag_costflow.empty() && task_costflow.empty()) break;

            // Inner loop: solve cost flow, then GoHome, handle conflicts
            int replan_iter = 0;
            while (true) {
                if (++replan_iter > 30) break;  // safety limit
                while (true) {
                    // Build constraint paths from non-costflow agents
                    vector<vector<int>> cons_paths;
                    for (int i = 0; i < num_ag; i++)
                        if (!in_costflow[i])
                            cons_paths.push_back(agents[i].non_dummy_path);

                    // Calculate time horizon for each task.
                    // Since we relaxed the release_time constraint in the flow,
                    // agents just need to reach the pickup (not wait until release_time).
                    // So we use a simple horizon: current_timestep + routing_budget.
                    int routing_budget = min(map_size / 4 + (int)ag_costflow.size() * 5 + 50, 300);
                    vector<int> len;
                    for (int i = 0; i < (int)task_costflow.size(); i++) {
                        int t = (int)hybrid_timestep_;
                        while (hybrid_cost(t + 1, hybrid_seqs_[task_costflow[i]->seq_id]) <=
                               hybrid_global_makespan_)
                            t++;
                        // Cap: agent only needs routing_budget steps to reach any pickup
                        t = min(t, (int)hybrid_timestep_ + routing_budget);
                        len.push_back(t);
                    }

                    int flow;
                    vector<vector<int>> paths;
                    hybrid_calc_flow(ag_costflow, task_costflow, cons_paths, len, flow, paths);

                    if (flow < (int)ag_costflow.size()) {
                        hybrid_global_makespan_++;
                        continue;
                    }

                    // Apply flow paths to agents
                    // path_len[flow_idx] = absolute timestep when flow path ends
                    int n_flow = (int)ag_costflow.size();
                    vector<int> path_len(n_flow, 0);
                    for (int i = 0; i < (int)paths.size(); i++) {
                        int flow_idx = paths[i][0];  // index within ag_costflow
                        path_len[flow_idx] = (int)hybrid_timestep_ + (int)paths[i].size() - 1;
                        if ((int)paths[i].size() == 1) {
                            for (int j = (int)hybrid_timestep_ + 1; j < (int)maxtime; j++)
                                ag_costflow[flow_idx]->path[j] = ag_costflow[flow_idx]->path[hybrid_timestep_];
                            continue;
                        }
                        for (int j = 1; j < (int)paths[i].size(); j++)
                            ag_costflow[flow_idx]->path[hybrid_timestep_ + j] = paths[i][j];
                        // Hold endpoint
                        for (int j = (int)paths[i].size() + (int)hybrid_timestep_; j < (int)maxtime; j++)
                            ag_costflow[flow_idx]->path[j] = paths[i].back();
                    }

                    // Resolve edge conflicts by swapping paths between flow agents
                    while (true) {
                        bool edge_conflict = false;
                        for (int i = 0; i < n_flow; i++)
                            for (int j = i + 1; j < n_flow; j++) {
                                // paths[i][0] and paths[j][0] are flow indices
                                int fi = paths[i][0], fj = paths[j][0];
                                Agent* a_i = ag_costflow[fi];
                                Agent* a_j = ag_costflow[fj];
                                for (int t = (int)hybrid_timestep_; t < min(path_len[fi], path_len[fj]); t++) {
                                    if ((int)a_i->path[t] == (int)a_j->path[t + 1] &&
                                        (int)a_j->path[t] == (int)a_i->path[t + 1]) {
                                        edge_conflict = true;
                                        swap(path_len[fi], path_len[fj]);
                                        for (int tt = t + 1; tt < (int)maxtime; tt++)
                                            swap(a_i->path[tt], a_j->path[tt]);
                                    }
                                }
                            }
                        if (!edge_conflict) break;
                    }

                    // Update agent states after flow
                    for (int i = 0; i < n_flow; i++) {
                        Agent* ag = ag_costflow[i];
                        ag->dummy_start_step = path_len[i];
                        int loc = (int)ag->path[(int)maxtime - 1];
                        ag->goal_loc = loc;
                        for (int j = 0; j < (int)task_costflow.size(); j++)
                            if (loc == task_costflow[j]->pickup_loc) {
                                ag->release_time_agent = task_costflow[j]->release_time;
                                hybrid_prefer_agent_[task_costflow[j]->seq_id] = ag->id;
                            }
                    }

                    // GoHome: plan dummy paths
                    int flag = hybrid_go_home(ag_costflow);
                    if (flag == -1) break;  // success

                    // GoHome failed at location flag - set hold_time for that task
                    int task_id = -1;
                    for (int i = 0; i < (int)task_costflow.size(); i++)
                        if (task_costflow[i]->pickup_loc == flag)
                            task_id = i;
                    if (task_id == -1) {
                        cerr << "TA-Hybrid: cannot find task for failed GoHome" << endl;
                        break;
                    }
                    task_costflow[task_id]->hold_time = task_costflow[task_id]->release_time;
                    // Extend hold_time based on other agents at that location
                    for (int i = 0; i < num_ag; i++)
                        if (!in_costflow[i])
                            for (int t = task_costflow[task_id]->hold_time + 1; t < (int)agents[i].path.size(); t++)
                                if ((int)agents[i].path[t] == flag)
                                    task_costflow[task_id]->hold_time = t;
                }  // end inner while (GoHome retry)

                // Replan dummy paths for all agents
                bool ok = true;
                for (int i = 0; i < num_ag; i++) {
                    if (!hybrid_replan_dummy(&agents[i])) {
                        ok = false;
                        // Update non_dummy_path to current path
                        vector<int> ndp(maxtime);
                        for (unsigned int t = 0; t < maxtime; t++)
                            ndp[t] = (int)agents[i].path[t];
                        agents[i].non_dummy_path = ndp;
                    }
                }
                if (ok) break;
            }  // end outer while (replan dummy retry)

            if (!same_dest) break;
        }  // end while (same_dest)

        // ============ Re-check Group 1 after Group 2 ============
        ag_icbs.clear();
        for (int i = 0; i < num_ag; i++) is_constraint[i] = true;

        for (int i = 0; i < num_ag; i++) {
            if (hybrid_seqs_[i].empty()) continue;
            Task* task = hybrid_seqs_[i].front();
            if (!task->delivering &&
                (int)hybrid_timestep_ >= task->release_time &&
                (int)agents[hybrid_prefer_agent_[i]].loc == task->pickup_loc) {
                int aid = hybrid_prefer_agent_[i];
                task->ag_arrive_start = (int)hybrid_timestep_;
                agents[aid].task_ptr = task;
                agents[aid].goal_loc = task->delivery_loc;
                agents[aid].delivering = true;
                ag_icbs.push_back(&agents[aid]);
                task->ag = &agents[aid];
                task->delivering = true;
                is_constraint[aid] = false;
            }
        }

        if (!ag_icbs.empty()) {
            vector<Agent*> cons_agents;
            for (int i = 0; i < num_ag; i++)
                if (is_constraint[i])
                    cons_agents.push_back(&agents[i]);
            hybrid_group1_plan(ag_icbs, cons_agents);
        }

        // Update token.path for collision checking
        for (int i = 0; i < num_ag; i++)
            for (unsigned int t = 0; t < maxtime; t++)
                token.path[i][t] = agents[i].path[t];
    }

    // Copy final paths to token.path and set task completion info
    for (int i = 0; i < num_ag; i++)
        for (unsigned int t = 0; t < maxtime; t++)
            token.path[i][t] = agents[i].path[t];

    token.tasks.clear();
} // end plan_ta_hybrid

// ============================================================
// Section 28: PBS Online — Update System
//   Detect completed task goals from agent positions.
//   When agent is at delivery location of current task, pop it.
// ============================================================

void Simulation::update_system_pbs() {
    pbs_has_event_ = false;

    // Check if new tasks arrived
    if (token.timestep < maxtime && !task_indices_by_time[token.timestep].empty())
        pbs_has_event_ = true;

    // Advance goal state machine: detect completed goals
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].task_sequence.empty()) continue;
        int task_id = agents[i].task_sequence.front();
        Task& task = all_tasks[task_id];

        int first_goal = task.goals.empty() ? task.pickup_loc : task.goals[0];
        int last_goal = task.goals.size() >= 2 ? task.goals[min((int)task.goals.size()-1, 1)] : first_goal;

        if (agents[i].status == AG_MOVING_TO_PICKUP &&
            (int)agents[i].loc == first_goal &&
            (int)token.timestep >= (unsigned int)task.release_time) {
            agents[i].status = AG_CARRYING;
        }

        if (agents[i].status == AG_CARRYING &&
            (int)agents[i].loc == last_goal) {
            task.completion_time = (int)token.timestep;
            task.status = INT_MAX;
            agents[i].task_sequence.pop_front();
            agents[i].current_task = -1;

            if (!agents[i].task_sequence.empty()) {
                agents[i].status = AG_MOVING_TO_PICKUP;
                agents[i].current_task = agents[i].task_sequence.front();
            } else {
                agents[i].status = AG_FREE;
            }
            pbs_has_event_ = true;
        }
    }

    // Check if any agent is free (no tasks) and there are unassigned tasks
    if (!pbs_has_event_ && !token.tasks.empty()) {
        for (auto& a : agents) {
            if (a.task_sequence.empty() && a.status == AG_FREE) {
                pbs_has_event_ = true;
                break;
            }
        }
    }

    // Trigger periodic replanning to prevent agents from getting stuck
    if (!pbs_has_event_) {
        bool any_active = false;
        for (auto& a : agents) {
            if (!a.task_sequence.empty()) { any_active = true; break; }
        }
        if (any_active && (int)token.timestep - pbs_last_replan_time_ >= config.replan_window) {
            pbs_has_event_ = true;
        }
    }

    if (pbs_has_event_)
        pbs_last_replan_time_ = (int)token.timestep;
}

// ============================================================
// Section 29: Repeated Hungarian Assignment
//   Assign tasks from token.tasks to agents' task_sequences
//   in rounds. Each round assigns min(num_agents, remaining) tasks.
// ============================================================

void Simulation::assign_repeated_hungarian() {
    // Collect unassigned tasks
    vector<Task*> remaining_tasks;
    for (auto it = token.tasks.begin(); it != token.tasks.end(); ++it) {
        if ((*it)->status == -1)
            remaining_tasks.push_back(*it);
    }
    if (remaining_tasks.empty()) return;

    int num_ag = (int)agents.size();

    while (!remaining_tasks.empty()) {
        int remain = (int)remaining_tasks.size();
        int row = max(num_ag, remain);

        dlib::matrix<int> cost_mat(row, row);
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < row; j++) {
                if (i >= num_ag || j >= remain) {
                    cost_mat(i, j) = INT_MIN;
                } else {
                    Task* task = remaining_tasks[j];
                    Agent& ag = agents[i];

                    // Compute estimated arrival time at task's last goal
                    int est_time = 0;
                    if (!ag.task_sequence.empty()) {
                        int loc = (int)ag.loc;
                        int t = (int)token.timestep;
                        for (int tid : ag.task_sequence) {
                            Task& tt = all_tasks[tid];
                            int ng = min((int)tt.goals.size(), 2);
                            for (int g = 0; g < ng; g++) {
                                int gloc = tt.goals[g];
                                int ep_idx = -1;
                                for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                                    if (mapd_map.endpoints[e].loc == gloc) { ep_idx = e; break; }
                                }
                                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                                if (d == INT_MAX) d = 0;
                                t += d;
                                if (g == 0 && t < tt.release_time) t = tt.release_time;
                                loc = gloc;
                            }
                        }
                        int first_goal_loc = task->goals.empty() ? task->pickup_loc : task->goals[0];
                        int ep_idx = -1;
                        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                            if (mapd_map.endpoints[e].loc == first_goal_loc) { ep_idx = e; break; }
                        }
                        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                        if (d == INT_MAX) d = 0;
                        est_time = t + d;
                    } else {
                        int first_goal_loc = task->goals.empty() ? task->pickup_loc : task->goals[0];
                        int ep_idx = -1;
                        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                            if (mapd_map.endpoints[e].loc == first_goal_loc) { ep_idx = e; break; }
                        }
                        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[ag.loc] : 0;
                        if (d == INT_MAX) d = 0;
                        est_time = (int)token.timestep + d;
                    }
                    est_time = max(est_time, task->release_time);
                    // Add distance to second goal (capped to first 2 goals)
                    if (task->goals.size() >= 2) {
                        int g0 = task->goals[0];
                        int g1 = task->goals[min((int)task->goals.size()-1, 1)];
                        int ep_idx = -1;
                        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                            if (mapd_map.endpoints[e].loc == g1) { ep_idx = e; break; }
                        }
                        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[g0] : 0;
                        if (d == INT_MAX) d = 0;
                        est_time += d;
                    }

                    // Negative cost for max_cost_assignment (minimize arrival time)
                    cost_mat(i, j) = -est_time;
                }
            }
        }

        vector<long> assignment = dlib::max_cost_assignment(cost_mat);

        vector<Task*> assigned;
        for (int i = 0; i < num_ag; i++) {
            if (assignment[i] < remain) {
                Task* task = remaining_tasks[assignment[i]];
                agents[i].task_sequence.push_back(task->id);
                task->status = i;
                assigned.push_back(task);
            }
        }

        // Remove assigned tasks from remaining and from token.tasks
        for (Task* t : assigned) {
            remaining_tasks.erase(
                remove(remaining_tasks.begin(), remaining_tasks.end(), t),
                remaining_tasks.end());
            token.tasks.remove(t);
        }
    }
}

// ============================================================
// Section 29b: Repeated Hungarian + LNS (LNS-PBS, LNS-wPBS)
//   Phase 1: Repeated Hungarian (reuses assign_repeated_hungarian)
//   Phase 2: LNS destroy/repair loop for improvement
// ============================================================

int Simulation::estimate_sequence_cost(int agent_id) const {
    const Agent& ag = agents[agent_id];
    int loc = (int)ag.loc;
    int t = (int)token.timestep;
    for (int tid : ag.task_sequence) {
        const Task& tt = all_tasks[tid];
        int ng = min((int)tt.goals.size(), 2);
        for (int g = 0; g < ng; g++) {
            int gloc = tt.goals[g];
            int ep_idx = -1;
            for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
                if (mapd_map.endpoints[e].loc == gloc) { ep_idx = e; break; }
            }
            int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
            if (d == INT_MAX) d = 0;
            t += d;
            if (g == 0 && t < tt.release_time) t = tt.release_time;
            loc = gloc;
        }
    }
    return t;
}

void Simulation::lns_destroy(vector<int>& removed_tasks) {
    // Collect all tasks in agent sequences (not currently being carried)
    vector<int> eligible;
    for (auto& ag : agents) {
        for (int tid : ag.task_sequence) {
            if (ag.status == AG_CARRYING && ag.current_task == tid) continue;
            eligible.push_back(tid);
        }
    }
    if (eligible.empty()) return;

    int method = rand() % 3;
    int group_size = max(1, min((int)eligible.size(), (int)agents.size()));

    if (method == 0) {
        // RANDOM: random sample
        random_shuffle(eligible.begin(), eligible.end());
        for (int i = 0; i < group_size && i < (int)eligible.size(); i++)
            removed_tasks.push_back(eligible[i]);
    } else if (method == 1) {
        // WORST: tasks from the agent with worst (highest) estimated cost
        int worst_ag = -1;
        int worst_cost = -1;
        for (int i = 0; i < (int)agents.size(); i++) {
            int c = estimate_sequence_cost(i);
            if (c > worst_cost) { worst_cost = c; worst_ag = i; }
        }
        if (worst_ag >= 0) {
            for (int tid : agents[worst_ag].task_sequence) {
                if (agents[worst_ag].status == AG_CARRYING && agents[worst_ag].current_task == tid)
                    continue;
                removed_tasks.push_back(tid);
                if ((int)removed_tasks.size() >= group_size) break;
            }
        }
    } else {
        // RELATED: pick a random task, find spatially nearby tasks
        int seed_idx = rand() % (int)eligible.size();
        int seed_tid = eligible[seed_idx];
        int seed_loc = all_tasks[seed_tid].goals.empty() ?
                       all_tasks[seed_tid].pickup_loc : all_tasks[seed_tid].goals[0];
        removed_tasks.push_back(seed_tid);

        vector<pair<int,int>> dists;
        for (int tid : eligible) {
            if (tid == seed_tid) continue;
            int tloc = all_tasks[tid].goals.empty() ?
                       all_tasks[tid].pickup_loc : all_tasks[tid].goals[0];
            int dx = abs(tloc / mapd_map.col - seed_loc / mapd_map.col);
            int dy = abs(tloc % mapd_map.col - seed_loc % mapd_map.col);
            dists.push_back({dx + dy, tid});
        }
        sort(dists.begin(), dists.end());
        for (auto& p : dists) {
            removed_tasks.push_back(p.second);
            if ((int)removed_tasks.size() >= group_size) break;
        }
    }

    // Actually remove from agent sequences
    for (int tid : removed_tasks) {
        all_tasks[tid].status = -1;
        for (auto& ag : agents) {
            auto it = find(ag.task_sequence.begin(), ag.task_sequence.end(), tid);
            if (it != ag.task_sequence.end()) {
                ag.task_sequence.erase(it);
                break;
            }
        }
    }
}

void Simulation::lns_repair(vector<int>& removed_tasks) {
    // Regret-based re-insertion
    while (!removed_tasks.empty()) {
        int best_task = -1;
        int best_agent = -1;
        int best_pos = -1;
        int best_cost = INT_MAX;
        int best_regret = INT_MIN;

        for (int rt : removed_tasks) {
            int best1_cost = INT_MAX, best1_agent = -1, best1_pos = -1;
            int best2_cost = INT_MAX;

            for (int a = 0; a < (int)agents.size(); a++) {
                int seq_len = (int)agents[a].task_sequence.size();
                // Try inserting at each position (0..seq_len)
                // Skip position 0 if agent is carrying the front task
                int start_pos = 0;
                if (agents[a].status == AG_CARRYING && !agents[a].task_sequence.empty()
                    && agents[a].task_sequence.front() == agents[a].current_task)
                    start_pos = 1;

                for (int p = start_pos; p <= seq_len; p++) {
                    // Temporarily insert
                    agents[a].task_sequence.insert(agents[a].task_sequence.begin() + p, rt);
                    int c = estimate_sequence_cost(a);
                    agents[a].task_sequence.erase(agents[a].task_sequence.begin() + p);

                    if (c < best1_cost) {
                        best2_cost = best1_cost;
                        best1_cost = c;
                        best1_agent = a;
                        best1_pos = p;
                    } else if (c < best2_cost) {
                        best2_cost = c;
                    }
                }
            }

            int regret = (best2_cost == INT_MAX) ? 0 : best2_cost - best1_cost;
            if (regret > best_regret || (regret == best_regret && best1_cost < best_cost)) {
                best_regret = regret;
                best_task = rt;
                best_agent = best1_agent;
                best_pos = best1_pos;
                best_cost = best1_cost;
            }
        }

        if (best_task < 0 || best_agent < 0) break;

        agents[best_agent].task_sequence.insert(
            agents[best_agent].task_sequence.begin() + best_pos, best_task);
        all_tasks[best_task].status = best_agent;
        removed_tasks.erase(
            remove(removed_tasks.begin(), removed_tasks.end(), best_task),
            removed_tasks.end());
    }
}

void Simulation::assign_repeated_hungarian_lns() {
    // Phase 1: Build initial assignment via repeated Hungarian
    assign_repeated_hungarian();

    if (config.lns_time_limit <= 0) return;

    // Phase 2: LNS improvement
    clock_t lns_start = clock();
    double time_limit_ms = config.lns_time_limit * 1000.0;

    while (true) {
        double elapsed = (double)(clock() - lns_start) / CLOCKS_PER_SEC * 1000.0;
        if (elapsed >= time_limit_ms) break;

        // Save current assignment state
        vector<deque<int>> saved_seqs(agents.size());
        vector<int> saved_statuses(all_tasks.size());
        int saved_cost = 0;
        for (int i = 0; i < (int)agents.size(); i++) {
            saved_seqs[i] = agents[i].task_sequence;
            saved_cost += estimate_sequence_cost(i);
        }
        for (int i = 0; i < (int)all_tasks.size(); i++)
            saved_statuses[i] = all_tasks[i].status;

        // Destroy
        vector<int> removed;
        lns_destroy(removed);
        if (removed.empty()) continue;

        // Repair
        lns_repair(removed);

        // Evaluate
        int new_cost = 0;
        for (int i = 0; i < (int)agents.size(); i++)
            new_cost += estimate_sequence_cost(i);

        if (new_cost >= saved_cost) {
            // Reject: restore
            for (int i = 0; i < (int)agents.size(); i++)
                agents[i].task_sequence = saved_seqs[i];
            for (int i = 0; i < (int)all_tasks.size(); i++)
                all_tasks[i].status = saved_statuses[i];
        }
    }
}

// ============================================================
// Section 30: Choose Dummy Endpoint
//   FLEXIBLE_STRICT (PBS): avoid all task goals + other agents' dummies
//   FLEXIBLE_PAIRWISE (wPBS): avoid only other agents' dummies
// ============================================================

int Simulation::choose_dummy_endpoint(int agent_id, int last_goal_loc,
                                       const vector<int>& assigned_dummies,
                                       bool strict) {
    set<int> forbidden;

    // Always forbid other agents' assigned dummies
    for (int i = 0; i < (int)assigned_dummies.size(); i++) {
        if (i != agent_id && assigned_dummies[i] >= 0)
            forbidden.insert(assigned_dummies[i]);
    }

    // In strict mode, also forbid all task goals in the system
    if (strict) {
        for (auto it = token.tasks.begin(); it != token.tasks.end(); ++it) {
            for (int gloc : (*it)->goals) forbidden.insert(gloc);
        }
        for (auto& ag : agents) {
            for (int tid : ag.task_sequence) {
                for (int gloc : all_tasks[tid].goals) forbidden.insert(gloc);
            }
        }
    }

    // Find closest non-forbidden endpoint
    int best_loc = -1;
    int best_dist = INT_MAX;
    for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
        int loc = mapd_map.endpoints[e].loc;
        if (forbidden.count(loc)) continue;
        int d = mapd_map.endpoints[e].h_val[last_goal_loc];
        if (d < best_dist) {
            best_dist = d;
            best_loc = loc;
        }
    }
    // Fallback: agent's initial location
    if (best_loc < 0) best_loc = agents[agent_id].initial_loc;
    return best_loc;
}

// ============================================================
// Section 31: Build Goal Sequences from Task Sequences
//   For each agent, compute: [pickup1, delivery1, ..., dummy_endpoint]
//   Returns goal_sequences[agent] = vector<pair<loc, release_time>>
// ============================================================

vector<vector<pair<int,int>>> Simulation::build_goal_sequences() {
    int num_ag = (int)agents.size();
    bool strict = (config.mapf == MAPF_PBS);
    vector<int> assigned_dummies(num_ag, -1);
    vector<vector<pair<int,int>>> goal_seqs(num_ag);

    // Truncate to max tasks per agent for planning efficiency
    // Use 2 for LNS-PBS/wPBS (matches reference task_truncated_size=2)
    int max_tasks_per_agent = (config.assign_method == AM_REPEATED_HUNGARIAN_LNS) ? 2 : 3;

    for (int i = 0; i < num_ag; i++) {
        int count = 0;
        for (int tid : agents[i].task_sequence) {
            if (count >= max_tasks_per_agent) break;
            Task& task = all_tasks[tid];
            // Multi-goal support: use task.goals for goal sequence
            // Cap to first 2 goals per task (matches reference behavior)
            int num_goals = min((int)task.goals.size(), 2);
            bool carrying = (agents[i].status == AG_CARRYING && tid == agents[i].current_task);
            if (num_goals <= 1) {
                // Single-goal task
                if (!carrying)
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
                // For single-goal: pickup == delivery, so carrying means done
            } else {
                // Two-goal task (pickup + delivery)
                if (carrying) {
                    goal_seqs[i].push_back({task.goals[1], 0});
                } else {
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
                    goal_seqs[i].push_back({task.goals[1], 0});
                }
            }
            count++;
        }
    }

    // Choose dummy endpoints
    for (int i = 0; i < num_ag; i++) {
        int last_loc;
        if (goal_seqs[i].empty())
            last_loc = (int)agents[i].loc;
        else
            last_loc = goal_seqs[i].back().first;

        int dummy = choose_dummy_endpoint(i, last_loc, assigned_dummies, strict);
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }

    return goal_seqs;
}

// ============================================================
// Section 32: MLA* — Multi-Label A* for Ordered Goal Sequences
//   State: (location, timestep, goal_id)
//   Increments goal_id when at goals[goal_id].loc after release_time
//   Terminal: goal_id == num_goals
// ============================================================

struct MLANode {
    int loc;
    int g_val;
    int h_val;
    int timestep;
    int goal_id;
    MLANode* parent;

    MLANode(int l, int g, int h, int t, int gi, MLANode* p)
        : loc(l), g_val(g), h_val(h), timestep(t), goal_id(gi), parent(p) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareMLANode {
    bool operator()(const MLANode* a, const MLANode* b) const {
        if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
        return a->g_val <= b->g_val;
    }
};

vector<int> Simulation::seq_mla_star(int agent_id, int start_loc, int start_time,
                                  const vector<pair<int,int>>& goals,
                                  const vector<vector<int>>& cons_paths,
                                  const vector<vector<int>>& old_paths,
                                  bool use_old_paths,
                                  bool skip_holding,
                                  int constraint_window) {
    if (goals.empty()) return {start_loc};

    int map_size = mapd_map.row * mapd_map.col;
    int max_t = (int)maxtime;
    int num_goals = (int)goals.size();
    int action[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    // Limit search horizon
    int search_horizon = min(max_t, start_time + 1000);

    // Use pre-computed endpoint h_vals where possible, otherwise compute BFS
    vector<const vector<int>*> h_vals_ptr(num_goals, nullptr);
    vector<vector<int>> h_vals_computed;  // storage for goals not matching endpoints
    for (int g = 0; g < num_goals; g++) {
        int gloc = goals[g].first;
        // Check if this location has a pre-computed endpoint h_val
        bool found = false;
        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
            if (mapd_map.endpoints[e].loc == gloc) {
                h_vals_ptr[g] = &mapd_map.endpoints[e].h_val;
                found = true;
                break;
            }
        }
        if (!found) {
            // Compute BFS
            h_vals_computed.emplace_back(map_size, INT_MAX);
            auto& hv = h_vals_computed.back();
            hv[gloc] = 0;
            queue<int> bfs_q;
            bfs_q.push(gloc);
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                    int v = u + d;
                    if (v >= 0 && v < map_size && mapd_map.grid[v] &&
                        hv[v] > hv[u] + 1) {
                        hv[v] = hv[u] + 1;
                        bfs_q.push(v);
                    }
                }
            }
            h_vals_ptr[g] = &h_vals_computed.back();
        }
    }

    // Compute heuristic: sum of distances through remaining goals
    auto compute_h = [&](int loc, int goal_id) -> int {
        if (goal_id >= num_goals) return 0;
        int h = (*h_vals_ptr[goal_id])[loc];
        if (h == INT_MAX) return INT_MAX;
        for (int g = goal_id; g < num_goals - 1; g++) {
            int d = (*h_vals_ptr[g + 1])[goals[g].first];
            if (d == INT_MAX) return INT_MAX;
            h += d;
        }
        return h;
    };

    // Constraint checking: higher-priority paths (hard constraints)
    // In windowed mode, only enforce within the constraint window
    auto is_constrained_hard = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        if (!mapd_map.grid[next_loc]) return true;
        if (constraint_window > 0 && abs_t > constraint_window) return false;
        for (auto& cp : cons_paths) {
            int cp_loc = (abs_t < (int)cp.size()) ? cp[abs_t] : cp.back();
            int cp_loc_prev = (abs_t - 1 >= 0 && abs_t - 1 < (int)cp.size()) ? cp[abs_t - 1] : (cp.empty() ? -1 : cp.back());
            if (cp_loc == next_loc) return true;
            if (cp_loc == curr_loc && cp_loc_prev == next_loc) return true;
        }
        return false;
    };

    // PBS: also check old_paths (within window or limited horizon)
    int old_path_horizon = (constraint_window > 0) ? constraint_window : start_time + 300;
    auto is_constrained_old = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        if (!use_old_paths) return false;
        if (abs_t > old_path_horizon) return false;
        for (auto& op : old_paths) {
            int op_loc = (abs_t < (int)op.size()) ? op[abs_t] : op.back();
            int op_loc_prev = (abs_t - 1 >= 0 && abs_t - 1 < (int)op.size()) ? op[abs_t - 1] : (op.empty() ? -1 : op.back());
            if (op_loc == next_loc) return true;
            if (op_loc == curr_loc && op_loc_prev == next_loc) return true;
        }
        return false;
    };

    // Get earliest holding time from constraints (within search horizon)
    // Skip for intermediate task-by-task calls (agent won't actually hold)
    int last_goal_loc = goals.back().first;
    int earliest_holding = 0;
    if (!skip_holding) {
        int hold_scan_limit = (constraint_window > 0) ? constraint_window + 1 : search_horizon;
        for (auto& cp : cons_paths) {
            for (int t = min((int)cp.size(), hold_scan_limit) - 1; t >= 0; t--) {
                if (cp[t] == last_goal_loc) {
                    earliest_holding = max(earliest_holding, t + 1);
                    break;
                }
            }
        }
        if (use_old_paths) {
            for (auto& op : old_paths) {
                for (int t = min({(int)op.size(), hold_scan_limit, old_path_horizon + 1}) - 1; t >= 0; t--) {
                    if (op[t] == last_goal_loc) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
                }
            }
        }
    }

    typedef priority_queue<MLANode*, vector<MLANode*>, CompareMLANode> mla_heap_t;
    mla_heap_t open_list;
    map<tuple<int,int,int>, MLANode*> allNodes;

    int init_h = compute_h(start_loc, 0);
    if (init_h == INT_MAX) return {};

    MLANode* root = new MLANode(start_loc, 0, init_h, start_time, 0, nullptr);
    open_list.push(root);
    allNodes[{start_loc, 0, 0}] = root;

    MLANode* solution = nullptr;
    int mla_expanded = 0;
    int mla_max_nodes = 500000;

    while (!open_list.empty() && mla_expanded < mla_max_nodes) {
        MLANode* curr = open_list.top();
        open_list.pop();
        mla_expanded++;

        // Advance goal_id if at current goal and past release time
        int gi = curr->goal_id;
        if (gi < num_goals && curr->loc == goals[gi].first) {
            int rel = goals[gi].second;
            if (curr->timestep >= rel) {
                // For last goal: check holding constraint
                if (gi == num_goals - 1 && curr->timestep < earliest_holding) {
                    // Can't hold yet
                } else {
                    gi++;
                }
            }
        }

        // Terminal: visited all goals
        if (gi >= num_goals) {
            solution = curr;
            break;
        }

        if (curr->timestep >= search_horizon - 1) continue;

        int child_gi = gi;

        for (int i = 0; i < 5; i++) {
            int next_loc = curr->loc + action[i];
            int next_t = curr->timestep + 1;

            if (next_loc < 0 || next_loc >= map_size) continue;
            if (abs(next_loc % mapd_map.col - curr->loc % mapd_map.col) > 1) continue;
            if (is_constrained_hard(curr->loc, next_loc, next_t)) continue;
            if (is_constrained_old(curr->loc, next_loc, next_t)) continue;

            int next_g = curr->g_val + 1;
            int next_h = compute_h(next_loc, child_gi);
            if (next_h == INT_MAX) continue;

            auto key = make_tuple(next_loc, child_gi, next_g);
            if (allNodes.find(key) == allNodes.end()) {
                MLANode* next_node = new MLANode(next_loc, next_g, next_h, next_t,
                                                  child_gi, curr);
                allNodes[key] = next_node;
                open_list.push(next_node);
            }
        }
    }

    vector<int> result;
    if (solution) {
        vector<int> path_locs;
        MLANode* node = solution;
        while (node != nullptr) {
            path_locs.push_back(node->loc);
            node = node->parent;
        }
        reverse(path_locs.begin(), path_locs.end());

        result.resize(max_t);
        for (int t = 0; t < start_time && t < max_t; t++)
            result[t] = start_loc;
        for (int i = 0; i < (int)path_locs.size(); i++) {
            int t = start_time + i;
            if (t < max_t) result[t] = path_locs[i];
        }
        int last_loc = path_locs.back();
        for (int t = start_time + (int)path_locs.size(); t < max_t; t++)
            result[t] = last_loc;
    }

    for (auto& p : allNodes) delete p.second;
    return result;
}

// ============================================================
// Section 32b: Split Goal Sequence into Per-Task Groups
// ============================================================

vector<vector<pair<int,int>>> Simulation::split_into_task_groups(
    int agent_id,
    const vector<pair<int,int>>& goal_seq) const
{
    vector<vector<pair<int,int>>> groups;
    int idx = 0;
    int max_tasks = (config.assign_method == AM_REPEATED_HUNGARIAN_LNS) ? 2 : 3;
    int count = 0;

    for (int tid : agents[agent_id].task_sequence) {
        if (count >= max_tasks) break;
        const Task& task = all_tasks[tid];
        int num_goals = min((int)task.goals.size(), 2);
        bool carrying = (agents[agent_id].status == AG_CARRYING &&
                          tid == agents[agent_id].current_task);

        vector<pair<int,int>> group;
        if (num_goals <= 1) {
            if (!carrying && idx < (int)goal_seq.size())
                group.push_back(goal_seq[idx++]);
        } else {
            if (carrying) {
                if (idx < (int)goal_seq.size())
                    group.push_back(goal_seq[idx++]);
            } else {
                if (idx < (int)goal_seq.size())
                    group.push_back(goal_seq[idx++]);
                if (idx < (int)goal_seq.size())
                    group.push_back(goal_seq[idx++]);
            }
        }
        if (!group.empty())
            groups.push_back(group);
        count++;
    }

    if (idx < (int)goal_seq.size())
        groups.push_back({goal_seq[idx]});

    return groups;
}

// ============================================================
// Section 32c: Task-by-Task MLA* — Plans Each Task Separately
// ============================================================

vector<int> Simulation::mla_star_taskwise(
    int agent_id, int start_loc, int start_time,
    const vector<vector<pair<int,int>>>& task_groups,
    const vector<vector<int>>& cons_paths,
    const vector<vector<int>>& old_paths,
    bool use_old_paths,
    int constraint_window)
{
    if (task_groups.empty()) return {start_loc};

    int max_t = (int)maxtime;
    vector<int> full_path(max_t, start_loc);
    for (int t = 0; t < min(start_time, max_t); t++)
        full_path[t] = start_loc;

    int cur_loc = start_loc;
    int cur_time = start_time;

    for (int g = 0; g < (int)task_groups.size(); g++) {
        const auto& goals = task_groups[g];
        if (goals.empty()) continue;

        bool is_last = (g == (int)task_groups.size() - 1);

        vector<int> segment = seq_mla_star(agent_id, cur_loc, cur_time,
                                            goals, cons_paths, old_paths,
                                            use_old_paths, !is_last,
                                            constraint_window);
        if (segment.empty()) return {};

        for (int t = cur_time; t < max_t; t++)
            full_path[t] = segment[t];

        // Find where last goal was reached using goal advancement
        int gi = 0;
        bool found = false;
        for (int t = cur_time; t < max_t && gi < (int)goals.size(); t++) {
            if (segment[t] == goals[gi].first && t >= goals[gi].second) {
                gi++;
                if (gi == (int)goals.size()) {
                    cur_loc = goals.back().first;
                    cur_time = t;
                    found = true;
                    break;
                }
            }
        }
        if (!found) return {};
    }

    return full_path;
}

// ============================================================
// Section 33: PBS — Find Earliest Conflict Between Two Paths
// ============================================================

bool Simulation::pbs_find_conflict(const vector<int>& p1, const vector<int>& p2,
                                    int a1, int a2,
                                    tuple<int,int,int,int,int>& conflict) {
    int max_len = (int)max(p1.size(), p2.size());

    for (int t = 0; t < max_len; t++) {
        int loc1 = (t < (int)p1.size()) ? p1[t] : p1.back();
        int loc2 = (t < (int)p2.size()) ? p2[t] : p2.back();

        if (loc1 == loc2) {
            conflict = make_tuple(a1, a2, loc1, -1, t);
            return true;
        }

        if (t > 0) {
            int prev_loc1 = (t - 1 < (int)p1.size()) ? p1[t-1] : p1.back();
            int prev_loc2 = (t - 1 < (int)p2.size()) ? p2[t-1] : p2.back();
            if (loc1 == prev_loc2 && loc2 == prev_loc1) {
                conflict = make_tuple(a1, a2, loc1, loc2, t);
                return true;
            }
        }
    }
    return false;
}

// ============================================================
// Section 34: PBS Core — Shared DFS-Based Search
// ============================================================

bool Simulation::pbs_core(bool windowed) {
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;

    vector<vector<pair<int,int>>> goal_seqs = build_goal_sequences();

    bool use_taskwise = (config.mla_mode != MLA_SEQ);
    vector<vector<vector<pair<int,int>>>> all_task_groups;
    if (use_taskwise) {
        all_task_groups.resize(num_ag);
        for (int i = 0; i < num_ag; i++)
            all_task_groups[i] = split_into_task_groups(i, goal_seqs[i]);
    }

    int cons_window = windowed ? (int)token.timestep + config.replan_window + 1 : -1;

    auto plan_agent = [&](int aid, int loc, int time,
                          const vector<vector<int>>& cons,
                          const vector<vector<int>>& old,
                          bool use_old) -> vector<int> {
        if (use_taskwise)
            return mla_star_taskwise(aid, loc, time, all_task_groups[aid],
                                      cons, old, use_old, cons_window);
        return seq_mla_star(aid, loc, time, goal_seqs[aid], cons, old, use_old,
                             false, cons_window);
    };

    // Save old paths
    vector<vector<int>> old_paths(num_ag);
    for (int i = 0; i < num_ag; i++) {
        old_paths[i].resize(max_t);
        for (int t = 0; t < max_t; t++)
            old_paths[i][t] = (int)token.path[i][t];
    }

    // Create root - plan each agent against old_paths of non-planned agents
    // and new paths of already-planned agents
    PBSNode* root = new PBSNode();
    root->paths.resize(num_ag);

    // Initialize paths with current token.path (preserves history)
    for (int i = 0; i < num_ag; i++) {
        root->paths[i].resize(max_t);
        for (int t = 0; t < max_t; t++)
            root->paths[i][t] = (int)token.path[i][t];
    }

    // Sort agents: active agents (with tasks) first, then idle agents
    vector<int> plan_order;
    for (int i = 0; i < num_ag; i++) {
        if (!agents[i].task_sequence.empty())
            plan_order.push_back(i);
    }
    for (int i = 0; i < num_ag; i++) {
        if (agents[i].task_sequence.empty())
            plan_order.push_back(i);
    }

    vector<bool> planned(num_ag, false);
    for (int idx = 0; idx < num_ag; idx++) {
        int i = plan_order[idx];
        vector<vector<int>> cons;
        vector<vector<int>> old_for_agent;

        // Already-planned agents use their NEW paths
        for (int j = 0; j < num_ag; j++) {
            if (planned[j])
                cons.push_back(root->paths[j]);
        }

        // Not-yet-planned agents use old_paths
        for (int j = 0; j < num_ag; j++) {
            if (j != i && !planned[j])
                old_for_agent.push_back(old_paths[j]);
        }

        vector<int> path = plan_agent(i, (int)agents[i].loc, (int)token.timestep,
                                       cons, old_for_agent, true);
        if (path.empty()) {
            path = plan_agent(i, (int)agents[i].loc, (int)token.timestep,
                               cons, {}, false);
            if (path.empty()) {
                // Keep current path
                for (int t = (int)token.timestep; t < max_t; t++)
                    root->paths[i][t] = (int)agents[i].loc;
            } else {
                for (int t = (int)token.timestep; t < max_t; t++)
                    root->paths[i][t] = path[t];
            }
        } else {
            for (int t = (int)token.timestep; t < max_t; t++)
                root->paths[i][t] = path[t];
        }
        planned[i] = true;
    }

    // Find all conflicts (windowed: only check within window)
    int conflict_horizon = windowed ? (int)token.timestep + config.replan_window + 1 : max_t;
    for (int a1 = 0; a1 < num_ag; a1++) {
        for (int a2 = a1 + 1; a2 < num_ag; a2++) {
            // Check conflicts only within horizon
            for (int t = (int)token.timestep; t < conflict_horizon && t < max_t; t++) {
                int loc1 = (t < (int)root->paths[a1].size()) ? root->paths[a1][t] : root->paths[a1].back();
                int loc2 = (t < (int)root->paths[a2].size()) ? root->paths[a2][t] : root->paths[a2].back();
                if (loc1 == loc2) {
                    root->conflicts.emplace_back(a1, a2, loc1, -1, t);
                    break;
                }
                if (t > 0) {
                    int pl1 = (t-1 < (int)root->paths[a1].size()) ? root->paths[a1][t-1] : root->paths[a1].back();
                    int pl2 = (t-1 < (int)root->paths[a2].size()) ? root->paths[a2][t-1] : root->paths[a2].back();
                    if (loc1 == pl2 && loc2 == pl1) {
                        root->conflicts.emplace_back(a1, a2, loc1, loc2, t);
                        break;
                    }
                }
            }
        }
    }
    root->num_collisions = (int)root->conflicts.size();

    if (root->conflicts.empty()) {
        for (int i = 0; i < num_ag; i++) {
            for (int t = 0; t < max_t; t++) {
                token.path[i][t] = root->paths[i][t];
                agents[i].path[t] = root->paths[i][t];
            }
        }
        delete root;
        return true;
    }

    vector<PBSNode*> all_nodes;
    all_nodes.push_back(root);
    stack<PBSNode*> dfs_stack;
    dfs_stack.push(root);
    PBSNode* best_node = root;
    best_node->conflict = root->conflicts.front();
    for (auto& c : root->conflicts) {
        if (get<4>(c) < get<4>(best_node->conflict))
            best_node->conflict = c;
    }
    best_node->earliest_collision = get<4>(best_node->conflict);
    int hl_expanded = 0;
    int max_hl = 5000;

    while (!dfs_stack.empty() && hl_expanded < max_hl) {
        PBSNode* curr = dfs_stack.top();
        dfs_stack.pop();

        if (curr->conflicts.empty()) {
            best_node = curr;
            break;
        }

        auto chosen = curr->conflicts.front();
        for (auto& c : curr->conflicts) {
            if (get<4>(c) < get<4>(chosen))
                chosen = c;
        }
        curr->conflict = chosen;
        curr->earliest_collision = get<4>(chosen);

        if (curr->earliest_collision > best_node->earliest_collision ||
            (curr->earliest_collision == best_node->earliest_collision &&
             curr->num_collisions < best_node->num_collisions)) {
            best_node = curr;
        }

        hl_expanded++;

        int a1 = get<0>(chosen);
        int a2 = get<1>(chosen);

        PBSNode* children[2];
        bool child_valid[2] = {true, true};

        for (int c = 0; c < 2; c++) {
            children[c] = new PBSNode();
            children[c]->parent = curr;
            children[c]->priorities.copy(curr->priorities);
            children[c]->paths = curr->paths;

            int lower = (c == 0) ? a1 : a2;
            int higher = (c == 0) ? a2 : a1;

            if (children[c]->priorities.connected(higher, lower)) {
                child_valid[c] = false;
                delete children[c];
                children[c] = nullptr;
                continue;
            }

            children[c]->priorities.add(lower, higher);
            children[c]->priority = {lower, higher};

            set<int> higher_set = children[c]->priorities.get_higher_priority(lower);
            vector<vector<int>> cons;
            vector<vector<int>> old_for_agent;

            for (int hp : higher_set)
                cons.push_back(children[c]->paths[hp]);

            // Both PBS and wPBS: use old_paths for non-priority agents
            for (int j = 0; j < num_ag; j++) {
                if (j == lower) continue;
                if (higher_set.count(j)) continue;
                old_for_agent.push_back(old_paths[j]);
            }

            vector<int> new_path = plan_agent(lower, (int)agents[lower].loc,
                                              (int)token.timestep,
                                              cons, old_for_agent, !windowed);
            if (new_path.empty()) {
                child_valid[c] = false;
                delete children[c];
                children[c] = nullptr;
                continue;
            }

            // Only update from current timestep onward
            for (int t = (int)token.timestep; t < max_t; t++)
                children[c]->paths[lower][t] = new_path[t];

            children[c]->conflicts.clear();
            for (int j = 0; j < num_ag; j++) {
                if (j == lower) continue;
                // Check conflicts within horizon
                for (int t = (int)token.timestep; t < conflict_horizon && t < max_t; t++) {
                    int loc_l = (t < (int)children[c]->paths[lower].size()) ? children[c]->paths[lower][t] : children[c]->paths[lower].back();
                    int loc_j = (t < (int)children[c]->paths[j].size()) ? children[c]->paths[j][t] : children[c]->paths[j].back();
                    if (loc_l == loc_j) {
                        children[c]->conflicts.emplace_back(lower, j, loc_l, -1, t);
                        break;
                    }
                    if (t > 0) {
                        int pl_l = (t-1 < (int)children[c]->paths[lower].size()) ? children[c]->paths[lower][t-1] : children[c]->paths[lower].back();
                        int pl_j = (t-1 < (int)children[c]->paths[j].size()) ? children[c]->paths[j][t-1] : children[c]->paths[j].back();
                        if (loc_l == pl_j && loc_j == pl_l) {
                            children[c]->conflicts.emplace_back(lower, j, loc_l, loc_j, t);
                            break;
                        }
                    }
                }
            }
            for (auto& conf : curr->conflicts) {
                if (get<0>(conf) != lower && get<1>(conf) != lower)
                    children[c]->conflicts.push_back(conf);
            }
            children[c]->num_collisions = (int)children[c]->conflicts.size();

            all_nodes.push_back(children[c]);
        }

        if (child_valid[0] && child_valid[1]) {
            int cost0 = children[0]->num_collisions;
            int cost1 = children[1]->num_collisions;
            if (cost0 < cost1) {
                dfs_stack.push(children[1]);
                dfs_stack.push(children[0]);
            } else {
                dfs_stack.push(children[0]);
                dfs_stack.push(children[1]);
            }
        } else if (child_valid[0]) {
            dfs_stack.push(children[0]);
        } else if (child_valid[1]) {
            dfs_stack.push(children[1]);
        }
    }

    for (int i = 0; i < num_ag; i++) {
        for (int t = 0; t < max_t; t++) {
            token.path[i][t] = best_node->paths[i][t];
            agents[i].path[t] = best_node->paths[i][t];
        }
    }

    if (!best_node->conflicts.empty()) {
        cerr << "PBS: best-effort (HL expanded: " << hl_expanded
             << ", collisions: " << best_node->num_collisions << ")" << endl;
    }

    for (auto* n : all_nodes) delete n;
    return true;
}

// ============================================================
// Section 35: PBS Path Planning — Wrapper
// ============================================================

// ============================================================
// Section 35b: PP+MLA* Path Planning for HUNGARIAN/LNS
//   Alternative to PBS/wPBS. Plans each agent one at a time
//   using token-based A*/MLA*, committing to token after each.
//   Uses build_goal_sequences() + choose_dummy_endpoint().
// ============================================================

void Simulation::path_planning_pp_mla() {
    // PP + MLA*: plan agents one at a time.
    // Each agent plans through ALL its assigned tasks in one mla_star call:
    //   goals = [pickup1, delivery1, pickup2, delivery2, ..., dummy]
    // cons_paths = all other agents' paths (old or already-replanned).
    // Those paths already go through their tasks and hold at their dummies.

    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;

    // Save old paths
    vector<vector<int>> old_paths(num_ag);
    for (int i = 0; i < num_ag; i++) {
        old_paths[i].resize(max_t);
        for (int t = 0; t < max_t; t++)
            old_paths[i][t] = (int)token.path[i][t];
    }

    // Plan order: active agents first, idle second
    vector<pair<int,int>> plan_order;
    for (int i = 0; i < num_ag; i++) {
        int pri = agents[i].task_sequence.empty() ? 1 : 0;
        plan_order.push_back({pri, i});
    }
    sort(plan_order.begin(), plan_order.end());

    vector<vector<int>> new_paths(num_ag);
    vector<bool> planned(num_ag, false);
    vector<int> agent_dummies(num_ag, -1);

    for (auto& po : plan_order) {
        int i = po.second;
        Agent& ag = agents[i];

        // Build cons_paths from all OTHER agents
        vector<vector<int>> cons_paths;
        for (int j = 0; j < num_ag; j++) {
            if (j == i) continue;
            if (planned[j])
                cons_paths.push_back(new_paths[j]);
            else
                cons_paths.push_back(old_paths[j]);
        }

        // Build goal sequence: ALL tasks in task_sequence + dummy endpoint
        vector<pair<int,int>> goals;
        for (int tid : ag.task_sequence) {
            Task& task = all_tasks[tid];
            int ng = min((int)task.goals.size(), 2);
            if (ag.status == AG_CARRYING && tid == ag.current_task) {
                // Already carrying: skip pickup
                if (ng >= 2)
                    goals.push_back({task.goals[1], 0});
                else
                    goals.push_back({task.goals[0], 0});
            } else {
                if (ng <= 1) {
                    goals.push_back({task.goals[0], task.release_time});
                } else {
                    goals.push_back({task.goals[0], task.release_time});
                    goals.push_back({task.goals[1], 0});
                }
            }
        }

        // Choose non-task dummy endpoint
        // Must not be another agent's dummy or permanently held by a cons_path agent
        int dummy_loc = ag.initial_loc;
        int best_d = INT_MAX;
        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
            if (mapd_map.endpoints[e].is_task_endpoint) continue;
            int loc = mapd_map.endpoints[e].loc;
            bool taken = false;
            for (int j = 0; j < num_ag; j++)
                if (j != i && agent_dummies[j] == loc) { taken = true; break; }
            if (taken) continue;
            for (auto& cp : cons_paths) {
                if (!cp.empty() && cp.back() == loc) { taken = true; break; }
            }
            if (taken) continue;
            int last_goal = goals.empty() ? (int)ag.loc : goals.back().first;
            int d = mapd_map.endpoints[e].h_val[last_goal];
            if (d < best_d) { best_d = d; dummy_loc = loc; }
        }
        agent_dummies[i] = dummy_loc;
        goals.push_back({dummy_loc, 0});

        // Plan with MLA* — SeqMLA* or task-by-task based on config
        vector<int> path;
        if (config.mla_mode != MLA_SEQ) {
            vector<vector<pair<int,int>>> tg = split_into_task_groups(i, goals);
            path = mla_star_taskwise(i, (int)ag.loc, (int)token.timestep,
                                      tg, cons_paths, {}, false);
        } else {
            path = seq_mla_star(i, (int)ag.loc, (int)token.timestep,
                                 goals, cons_paths, {}, false);
        }

        // Build absolute-timestep path
        new_paths[i].resize(max_t);
        if (!path.empty()) {
            for (int t = 0; t < max_t; t++)
                new_paths[i][t] = path[t];  // mla_star already pads to max_t
        } else {
            // Failed — stay in place
            for (int t = 0; t < max_t; t++)
                new_paths[i][t] = (int)ag.loc;
        }

        planned[i] = true;

        // Commit to token and agent path
        for (int t = 0; t < max_t; t++) {
            token.path[i][t] = new_paths[i][t];
            agents[i].path[t] = new_paths[i][t];
        }
    }

    // Set status
    for (int i = 0; i < num_ag; i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
        }
    }
}

// ============================================================
// Section 35: PBS Path Planning — Wrapper
// ============================================================

void Simulation::path_planning_pbs() {
    pbs_core(false);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
        }
    }
}

// ============================================================
// Section 36: wPBS Path Planning — Wrapper
// ============================================================

void Simulation::path_planning_wpbs() {
    pbs_core(true);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
        }
    }
}

// ============================================================
// REALPATH_LNS_IMP — Generic Anytime Improvement
//   (Chen et al. 2021, Algorithm 3 / Sec IV-D)
//
//   1. Destroy: remove tasks from agents (cascade subsequent)
//   2. Repair:  Hungarian assignment of destroyed tasks to agents
//   3. Replan:  PP with MLA* + flexible dummy endpoints
//   4. Accept/Reject based on real path cost
// ============================================================

int Simulation::compute_realpath_cost() const {
    int cost = 0;
    for (auto& t : all_tasks) {
        if (t.completion_time > 0)
            cost += (t.completion_time - t.release_time);
    }
    return cost;
}

void Simulation::rmca_destroy(vector<int>& removed, int group_size) {
    vector<int> eligible;
    for (auto& t : all_tasks)
        if (t.status >= 0 && t.status != INT_MAX && t.completion_time > 0)
            eligible.push_back(t.id);
    if (eligible.empty()) return;
    group_size = min(group_size, (int)eligible.size());

    int method = rand() % 3;
    if (method == 0) {
        random_shuffle(eligible.begin(), eligible.end());
        for (int i = 0; i < group_size; i++) removed.push_back(eligible[i]);
    } else if (method == 1) {
        sort(eligible.begin(), eligible.end(), [this](int a, int b) {
            return (all_tasks[a].completion_time - all_tasks[a].release_time) >
                   (all_tasks[b].completion_time - all_tasks[b].release_time);
        });
        for (int i = 0; i < group_size; i++) removed.push_back(eligible[i]);
    } else {
        vector<pair<int,int>> ac;
        for (int a = 0; a < (int)agents.size(); a++) {
            int ttd = 0;
            for (int tid : eligible)
                if (all_tasks[tid].status == a)
                    ttd += (all_tasks[tid].completion_time - all_tasks[tid].release_time);
            ac.push_back({-ttd, a});
        }
        sort(ac.begin(), ac.end());
        for (auto& p : ac) {
            if ((int)removed.size() >= group_size) break;
            vector<int> ae;
            for (int tid : eligible)
                if (all_tasks[tid].status == p.second) ae.push_back(tid);
            if (!ae.empty()) removed.push_back(ae[rand() % ae.size()]);
        }
    }
}

bool Simulation::replan_agent_path(int agent_id) { return true; }

void Simulation::rmca_repair(vector<int>& removed, vector<vector<int>>& agent_task_lists) {
    // Hungarian assignment: destroyed tasks → agents, minimizing estimated arrival
    int num_ag = (int)agents.size();
    int num_tasks_to_assign = (int)removed.size();
    if (num_tasks_to_assign == 0) return;

    // Assign in rounds (like assign_repeated_hungarian)
    while (!removed.empty()) {
        int remain = (int)removed.size();
        int dim = max(num_ag, remain);

        dlib::matrix<int> cost_mat(dim, dim);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                if (i >= num_ag || j >= remain) {
                    cost_mat(i, j) = INT_MIN;
                } else {
                    Task& task = all_tasks[removed[j]];
                    // Estimate: time for agent i to finish current tasks + reach this task
                    int last_time = 0, last_loc = agents[i].initial_loc;
                    if (!agent_task_lists[i].empty()) {
                        int lt = agent_task_lists[i].back();
                        if (all_tasks[lt].completion_time > 0) {
                            last_time = all_tasks[lt].completion_time;
                            last_loc = all_tasks[lt].delivery_loc;
                        } else {
                            // Estimate from heuristic
                            int loc = agents[i].initial_loc;
                            int t = 0;
                            for (int tid : agent_task_lists[i]) {
                                Task& tt = all_tasks[tid];
                                int d = mapd_map.endpoints[tt.pickup].h_val[loc];
                                if (d == INT_MAX) d = 0;
                                t += d; t = max(t, tt.release_time);
                                int d2 = mapd_map.endpoints[tt.delivery].h_val[tt.pickup_loc];
                                if (d2 == INT_MAX) d2 = 0;
                                t += d2;
                                loc = tt.delivery_loc;
                            }
                            last_time = t;
                            last_loc = all_tasks[agent_task_lists[i].back()].delivery_loc;
                        }
                    }
                    int d = mapd_map.endpoints[task.pickup].h_val[last_loc];
                    if (d == INT_MAX) d = 9999;
                    int est = max(last_time + d, task.release_time);
                    int d2 = mapd_map.endpoints[task.delivery].h_val[task.pickup_loc];
                    if (d2 == INT_MAX) d2 = 9999;
                    est += d2;
                    cost_mat(i, j) = -est;
                }
            }
        }

        vector<long> assignment = dlib::max_cost_assignment(cost_mat);

        vector<int> assigned;
        for (int i = 0; i < num_ag; i++) {
            if (assignment[i] < remain) {
                int tid = removed[assignment[i]];
                agent_task_lists[i].push_back(tid);
                all_tasks[tid].status = i;
                assigned.push_back(tid);
            }
        }
        for (int tid : assigned)
            removed.erase(std::remove(removed.begin(), removed.end(), tid), removed.end());
        if (assigned.empty()) break;
    }
}

int Simulation::realpath_lns_imp(int num_rounds, int group_size) {
    if (num_rounds <= 0) return 0;

    int num_ag = (int)agents.size();
    int num_tasks = (int)all_tasks.size();
    int improvements = 0;

    // Build per-agent task lists ordered by completion_time
    vector<vector<int>> agent_task_lists(num_ag);
    for (int i = 0; i < num_tasks; i++) {
        if (all_tasks[i].status >= 0 && all_tasks[i].status < num_ag)
            agent_task_lists[all_tasks[i].status].push_back(i);
    }
    for (int a = 0; a < num_ag; a++) {
        sort(agent_task_lists[a].begin(), agent_task_lists[a].end(),
             [this](int x, int y) { return all_tasks[x].completion_time < all_tasks[y].completion_time; });
    }

    srand(42);  // deterministic but different from simulation

    for (int round = 0; round < num_rounds; round++) {
        int current_cost = compute_realpath_cost();

        // --- Snapshot ---
        vector<vector<unsigned int>> saved_agent_paths(num_ag);
        vector<vector<unsigned int>> saved_token_paths(num_ag);
        for (int i = 0; i < num_ag; i++) {
            saved_agent_paths[i].assign(agents[i].path.begin(), agents[i].path.end());
            saved_token_paths[i].assign(token.path[i].begin(), token.path[i].end());
        }
        vector<int> saved_task_status(num_tasks);
        vector<int> saved_task_completion(num_tasks);
        vector<int> saved_task_arrive(num_tasks);
        for (int i = 0; i < num_tasks; i++) {
            saved_task_status[i] = all_tasks[i].status;
            saved_task_completion[i] = all_tasks[i].completion_time;
            saved_task_arrive[i] = all_tasks[i].ag_arrive_start;
        }
        vector<vector<int>> saved_atl = agent_task_lists;

        // --- Destroy: remove tasks and cascade ---
        vector<int> destroyed_tids;
        rmca_destroy(destroyed_tids, group_size);
        if (destroyed_tids.empty()) continue;

        // Remove destroyed tasks and cascade: for each affected agent,
        // find the EARLIEST destroyed task and remove everything from there.
        set<int> destroyed_set(destroyed_tids.begin(), destroyed_tids.end());
        set<int> affected_agents;
        for (int a = 0; a < num_ag; a++) {
            auto& atl = agent_task_lists[a];
            int cut = -1;
            for (int i = 0; i < (int)atl.size(); i++) {
                if (destroyed_set.count(atl[i])) { cut = i; break; }
            }
            if (cut >= 0) {
                for (int j = cut; j < (int)atl.size(); j++) {
                    int tid = atl[j];
                    if (!destroyed_set.count(tid)) destroyed_tids.push_back(tid);
                    destroyed_set.insert(tid);
                    all_tasks[tid].status = -1;
                    all_tasks[tid].completion_time = -1;
                }
                affected_agents.insert(a);
                atl.resize(cut);
            }
        }

        // --- Repair: Hungarian assignment ---
        vector<int> to_assign = destroyed_tids;
        rmca_repair(to_assign, agent_task_lists);

        // Track all affected agents (original + new owners)
        for (int tid : destroyed_tids) {
            int a = all_tasks[tid].status;
            if (a >= 0 && a < num_ag) affected_agents.insert(a);
        }

        // --- Replan: PP with per-task astar, finish-time ordered ---
        // Clear affected agents' paths from their replan points.
        // Then plan tasks one at a time (earliest finish first across
        // all affected agents), committing each to the token before
        // planning the next. This matches the original simulation.

        // Initialize replan state per affected agent
        struct ReplanState {
            int cur_loc, cur_time, next_task_idx;
        };
        vector<ReplanState> rs(num_ag);
        for (int a : affected_agents) {
            auto& atl = agent_task_lists[a];
            int first_bad = (int)atl.size();
            for (int i = 0; i < (int)atl.size(); i++) {
                if (all_tasks[atl[i]].completion_time <= 0) { first_bad = i; break; }
            }
            rs[a].next_task_idx = first_bad;
            rs[a].cur_loc = agents[a].initial_loc;
            rs[a].cur_time = 0;
            if (first_bad > 0) {
                rs[a].cur_time = all_tasks[atl[first_bad - 1]].completion_time;
                rs[a].cur_loc = all_tasks[atl[first_bad - 1]].delivery_loc;
            }
            // Clear path from replan point
            for (unsigned int t = max(0, rs[a].cur_time); t < maxtime; t++) {
                agents[a].path[t] = rs[a].cur_loc;
                token.path[a][t] = rs[a].cur_loc;
            }
        }

        int total_to_plan = 0;
        for (int a : affected_agents)
            total_to_plan += (int)agent_task_lists[a].size() - rs[a].next_task_idx;

        int total_planned = 0;
        bool all_ok = true;

        while (total_planned < total_to_plan && all_ok) {
            // Pick the affected agent with earliest cur_time
            int best_a = -1, best_t = INT_MAX;
            for (int a : affected_agents) {
                if (rs[a].next_task_idx < (int)agent_task_lists[a].size()) {
                    if (rs[a].cur_time < best_t) { best_t = rs[a].cur_time; best_a = a; }
                }
            }
            if (best_a < 0) break;

            int a = best_a;
            int idx = rs[a].next_task_idx;
            int tid = agent_task_lists[a][idx];
            Task& task = all_tasks[tid];
            Agent& ag = agents[a];

            // Plan pickup
            int arrive_start = astar(ag, rs[a].cur_loc, rs[a].cur_time,
                                     mapd_map.endpoints[task.pickup], ag.id);
            if (arrive_start < 0) { all_ok = false; break; }

            // Plan delivery
            int pt = max(arrive_start, task.release_time) + task.start_wait_time;
            int arrive_goal = astar(ag, task.pickup_loc, pt,
                                    mapd_map.endpoints[task.delivery], ag.id);
            if (arrive_goal < 0) { all_ok = false; break; }

            // Commit this agent's path to token
            for (unsigned int t = 0; t < maxtime; t++)
                token.path[a][t] = ag.path[t];

            task.ag_arrive_start = arrive_start;
            task.completion_time = arrive_goal;
            rs[a].cur_loc = task.delivery_loc;
            rs[a].cur_time = arrive_goal + task.goal_wait_time;
            rs[a].next_task_idx++;
            total_planned++;
        }

        // --- Evaluate: accept only if BOTH makespan and SWT improve ---
        int new_cost = all_ok ? compute_realpath_cost() : INT_MAX;
        int new_makespan = 0;
        int valid_ct = 0;
        for (auto& t : all_tasks) {
            if (t.completion_time > 0) {
                valid_ct++;
                if (t.completion_time > new_makespan) new_makespan = t.completion_time;
            }
        }
        int current_makespan = 0;
        for (int i = 0; i < num_tasks; i++)
            if (saved_task_completion[i] > current_makespan) current_makespan = saved_task_completion[i];

        if (all_ok && valid_ct == num_tasks
            && new_cost <= current_cost && new_makespan <= current_makespan
            && (new_cost < current_cost || new_makespan < current_makespan)) {
            improvements++;
            cerr << "  IMPROVED R" << round << ": makespan " << current_makespan
                 << "->" << new_makespan << " SWT " << current_cost
                 << "->" << new_cost << " destroyed=" << destroyed_tids.size() << endl;
        } else {
            // Reject — restore
            for (int i = 0; i < num_ag; i++) {
                agents[i].path.assign(saved_agent_paths[i].begin(), saved_agent_paths[i].end());
                for (unsigned int t = 0; t < maxtime; t++)
                    token.path[i][t] = saved_token_paths[i][t];
            }
            for (int i = 0; i < num_tasks; i++) {
                all_tasks[i].status = saved_task_status[i];
                all_tasks[i].completion_time = saved_task_completion[i];
                all_tasks[i].ag_arrive_start = saved_task_arrive[i];
            }
            agent_task_lists = saved_atl;
        }
    }

    return improvements;
}
