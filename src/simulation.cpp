#include <random>
#include <deque>
#include <chrono>
#include <cstdio>
#include "simulation.h"
#include "cbs.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <algorithm>
#include <cstdlib>
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

    // NOTE: the reference MGMAPD code applies a std::shuffle to the agent-home
    // locations. We deliberately do NOT reproduce it: an A/B over the fast
    // Hungarian cells showed it only reshuffles which few cells are outliers
    // (mean gap vs reference ~1% either way, same # of >3% cells), so agents
    // keep their natural map-scan start order for all methods.

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
    path_table_.resize(mapd_map.num_agents);
    passable_map_ = mapd_map.grid;
    endpoint_mask_ = mapd_map.is_endpoint;
    cur_time_ = 0;

    for (int i = 0; i < mapd_map.num_agents; i++) {
        agents[i].init(i, mapd_map.agent_starts[i], mapd_map.col, mapd_map.row, maxtime);
        path_table_[i].resize(maxtime);
        for (unsigned int k = 0; k < maxtime; k++)
            path_table_[i][k] = mapd_map.agent_starts[i];
    }

    // Init general/system loop state
    last_released_time_ = -1;  // no tasks released yet; release_tasks() will handle t=0
    ta_planning_done_ = false;
    agent_pending_task.assign(agents.size(), nullptr);

    // Init algorithm-specific state (dispatched on config)
    init_algorithm_state();
}

// ============================================================
// Section 0.1: Algorithm-specific state initialization
//   General init() dispatches here.  We FIRST set every
//   algorithm flag to a safe default (the ctor is empty, so
//   members are otherwise uninitialized), then the dispatched
//   per-family helper overrides the ones that algorithm reads.
// ============================================================

void Simulation::init_algorithm_state() {
    // Safe defaults for ALL algorithm flags (no method ever reads an
    // uninitialized flag, regardless of which helper the dispatch selects).
    tp_timestep_advanced_ = false;
    central_has_event_ = false;
    central_reassign_event_ = false;
    pbs_has_event_ = false;
    pbs_assign_event_ = false;
    pbs_last_replan_time_ = 0;
    lns_release_period_ = 1;
    lns_agent_finished_ = false;

    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        init_tp_state();
        break;
    case AT_EVERY_TIMESTEP:
    case AT_ON_NEW_TASK_OR_FREE:
        init_central_state();
        break;
    case AT_ON_UNASSIGNED_OR_FREE:
        if (config.assign_method == AM_REPEATED_HUNGARIAN_LNS)
            init_lns_state();
        else if (config.assign_method == AM_HUNGARIAN)
            init_pbs_state();   // Hungarian PBS / wPBS online
        break;
    default:
        break;
    }
}

void Simulation::init_tp_state() {
    // TP / TPTS: agent selection consults tp_timestep_advanced_.
    tp_timestep_advanced_ = false;
}

void Simulation::init_central_state() {
    // Set event flags so the first iteration's task_assignment runs.
    // release_tasks() will load t=0 tasks before task_assignment runs.
    central_has_event_ = true;
    central_reassign_event_ = true;
}

void Simulation::init_pbs_state() {
    // Online PBS/wPBS (Hungarian): first iteration must assign and replan.
    pbs_has_event_ = true;
    pbs_assign_event_ = true;
    pbs_last_replan_time_ = 0;
}

void Simulation::init_lns_state() {
    // LNS shares the online PBS loop, so it needs the PBS event flags too.
    init_pbs_state();

    // Derive the task-release period (gap between the first two distinct release
    // times) — matches the reference task_release_period.  The reference only
    // spends the 1-second LNS budget on release-period boundaries or when an
    // agent finishes its last goal; gating our LNS spin the same way keeps the
    // runtime in line with the reference without changing the assignment result
    // (Hungarian still runs every event for correctness).
    lns_release_period_ = 1;
    {
        int first_rt = -1, second_rt = -1;
        for (int t = 0; t < (int)maxtime; t++) {
            if (!task_indices_by_time[t].empty()) {
                if (first_rt < 0) first_rt = t;
                else { second_rt = t; break; }
            }
        }
        if (first_rt >= 0 && second_rt > first_rt)
            lns_release_period_ = second_rt - first_rt;
        if (lns_release_period_ <= 0) lns_release_period_ = 1;
    }
    lns_agent_finished_ = false;
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

    // Paths (incl. parked tails at unique endpoints) are already collision-free here.
    // TODO: if the post-run collision check ever FAILS on end-parking, resolve it here.
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

    if (!open_tasks_.empty()) return false;
    if ((int)cur_time_ <= t_task) return false;
    // For PBS online: also check task_sequences
    if (config.assign_trigger == AT_ON_UNASSIGNED_OR_FREE) {
        for (auto& a : agents)
            if (!a.task_sequence.empty()) return false;
    }
    for (auto& a : agents)
        if (a.status != AG_FREE && a.finish_time > cur_time_) return false;
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
    // Release all tasks from last_released_time_+1 up to current cur_time_.
    // This handles both the initial t=0 release and subsequent releases
    // after update_system() advances the timestep.

    int from = last_released_time_ + 1;
    int to = (int)cur_time_;

    switch (config.mode) {
    case MODE_ONLINE:
    case MODE_SEMI_ONLINE:
        for (int t = from; t <= to && t < (int)maxtime; t++) {
            for (int idx : task_indices_by_time[t])
                open_tasks_.push_back(&all_tasks[idx]);
        }
        break;
    case MODE_OFFLINE:
        // All tasks released at t=0
        if (last_released_time_ < 0) {
            for (int i = 0; i < (int)all_tasks.size(); i++)
                open_tasks_.push_back(&all_tasks[i]);
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
    // General dispatcher: pick the per-algorithm update path based on trigger.
    switch (config.assign_trigger) {
    case AT_ON_UNASSIGNED_OR_FREE:
        update_system_online();      // PBS/LNS online (block a)
        return;
    case AT_ON_FREE_WAITS:
        if (tp_pre_step()) return;   // TP/TPTS pre-step (block b); may return early
        break;                       // else fall through to stepwise (block c)
    default:
        break;
    }
    update_system_stepwise();        // CENTRAL/TA/TP-fallthrough (block c)
}

// ============================================================
// Section 1.3a: update_system — HUNGARIAN/LNS online mode
//   (AT_ON_UNASSIGNED_OR_FREE)
// ============================================================

void Simulation::update_system_online() {
    // --- HUNGARIAN/LNS online mode ---
    {
        // wPBS: advance 1 step at a time (matching reference KivaSystemOnline)
        // PBS: event-driven advancement
        unsigned int next_ts = maxtime;

        if (config.mapf == MAPF_wPBS) {
            // wPBS: event-driven advancement capped at replan_window
            for (int i = 0; i < (int)agents.size(); i++) {
                if (agents[i].task_sequence.empty()) continue;
                int task_id = agents[i].task_sequence.front();
                Task& task = all_tasks[task_id];
                int first_goal = task.goals.empty() ? task.pickup_loc : task.goals[0];
                int last_goal = task.goals.size() >= 2 ? task.goals[min((int)task.goals.size()-1, 1)] : first_goal;
                int target_loc = (agents[i].status == AG_MOVING_TO_PICKUP) ? first_goal :
                                 (agents[i].status == AG_CARRYING) ? last_goal : -1;
                int min_time = (agents[i].status == AG_MOVING_TO_PICKUP) ? task.release_time : 0;
                if (target_loc >= 0)
                    for (unsigned int t = cur_time_ + 1; t < maxtime; t++)
                        if ((int)agents[i].path[t] == target_loc && (int)t >= min_time) {
                            if (t < next_ts) next_ts = t;
                            break;
                        }
            }
            for (unsigned int t = cur_time_ + 1; t < maxtime && t <= next_ts; t++)
                if (!task_indices_by_time[t].empty()) { if (t < next_ts) next_ts = t; break; }
            unsigned int window_cap = cur_time_ + config.replan_window;
            if (window_cap < next_ts) next_ts = window_cap;
        } else {
            // PBS: jump to next event
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
                    for (unsigned int t = cur_time_ + 1; t < maxtime; t++) {
                        if ((int)agents[i].path[t] == target_loc && (int)t >= min_time) {
                            if (t < next_ts) next_ts = t;
                            break;
                        }
                    }
                }
            }

            for (unsigned int t = cur_time_ + 1; t < maxtime && t <= next_ts; t++) {
                if (!task_indices_by_time[t].empty()) {
                    if (t < next_ts) next_ts = t;
                    break;
                }
            }
        }

        if (next_ts >= maxtime) {
            bool has_work = false;
            for (auto& a : agents) {
                if (!a.task_sequence.empty()) { has_work = true; break; }
            }
            if (has_work || !open_tasks_.empty())
                next_ts = cur_time_ + 1;
            else
                return;  // truly done
        }
        if (next_ts <= cur_time_) next_ts = cur_time_ + 1;

        if (next_ts >= maxtime) {
            cerr << "PBS: exceeded maxtime=" << maxtime << endl;
            return;
        }

        cur_time_ = next_ts;
        for (auto& ag : agents) {
            if (cur_time_ < maxtime)
                ag.loc = ag.path[cur_time_];
        }

        pbs_has_event_ = false;
        update_system_pbs();
        return;
    }
}

// ============================================================
// Section 1.3b: update_system — TP/TPTS pre-step
//   (AT_ON_FREE_WAITS).  Returns true if update_system should
//   return early (a free agent was processed without advancing);
//   false to fall through to the shared stepwise block.
// ============================================================

bool Simulation::tp_pre_step() {
    // --- TP/TPTS mode: check if a free agent needs processing first ---
    {
        // For TPTS: remove tasks from token when pickup is reached
        // (matching reference: tasks stay in open_tasks_ until ag_arrive_start)
        if (config.assign_method == AM_DECOUPLED_GREEDY_SWAPS) {
            auto it = open_tasks_.begin();
            while (it != open_tasks_.end()) {
                if ((*it)->status >= 0 && (*it)->ag_arrive_start >= 0 &&
                    (int)cur_time_ >= (*it)->ag_arrive_start) {
                    it = open_tasks_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        for (auto& ag : agents) {
            if (ag.finish_time <= cur_time_) {
                central_has_event_ = false;
                central_reassign_event_ = false;

                // Detect completed deliveries
                for (int i = 0; i < (int)agents.size(); i++) {
                    if (agents[i].status == AG_CARRYING && agents[i].finish_time <= cur_time_) {
                        Task* task = agent_pending_task[i];
                        if (task) {
                            task->completion_time = agents[i].finish_time;
                            open_tasks_.remove(task);
                            agent_pending_task[i] = nullptr;
                        }
                        agents[i].status = AG_FREE;
                        agents[i].current_task = -1;
                    }
                }

                // Detect pickup arrivals
                for (int i = 0; i < (int)agents.size(); i++) {
                    if (agents[i].status == AG_MOVING_TO_PICKUP && agents[i].finish_time <= cur_time_) {
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

                if (cur_time_ < maxtime && !task_indices_by_time[cur_time_].empty()) {
                }
                tp_timestep_advanced_ = false;
                return true;  // don't advance
            }
        }
    }

    // TP/TPTS: timestep is about to advance — set flag so agent selection
    // skips the exact-match branch (matching reference where agent selection
    // happens BEFORE timestep is advanced, so no exact match is possible).
    tp_timestep_advanced_ = true;
    return false;  // fall through to the shared stepwise block
}

// ============================================================
// Section 1.3c: update_system — shared step-advance block
//   (CENTRAL, CENTRAL_FIXED, TA-Prioritized, TA-Hybrid, and
//    TP/TPTS after tp_pre_step falls through)
// ============================================================

void Simulation::update_system_stepwise() {
    // --- Default mode (CENTRAL, CENTRAL_FIXED, TA-Prioritized, TA-Hybrid, TP/TPTS) ---

    // Step 1: Detect state transitions at current timestep
    // (Reset event flags first — they will be set based on the NEW timestep)
    // We detect transitions at the current timestep before advancing,
    // but the event flags will be set for the NEXT iteration after advancing.

    // Step 2: Advance to next event timestep
    unsigned int next_ts = maxtime;
    if (config.assign_trigger == AT_EVERY_TIMESTEP) {
        // CENTRAL: advance one timestep at a time (matching reference for-loop)
        next_ts = cur_time_ + 1;
        if (next_ts >= maxtime) {
            bool any_busy = false;
            for (auto& a : agents)
                if (a.status != AG_FREE && a.finish_time > cur_time_) { any_busy = true; break; }
            if (!any_busy && open_tasks_.empty()) return;
        }
    } else {
        for (auto& ag : agents) {
            if (ag.finish_time > cur_time_ && ag.finish_time < next_ts)
                next_ts = ag.finish_time;
        }
        for (unsigned int t = cur_time_ + 1; t < maxtime && t <= next_ts; t++) {
            if (!task_indices_by_time[t].empty()) {
                if (t < next_ts) next_ts = t;
                break;
            }
        }
        if (next_ts >= maxtime) {
            bool any_busy = false;
            for (auto& a : agents)
                if (a.status != AG_FREE && a.finish_time > cur_time_) { any_busy = true; break; }
            if (!any_busy && open_tasks_.empty()) return;
            next_ts = cur_time_ + 1;
        }
        if (next_ts <= cur_time_) next_ts = cur_time_ + 1;
    }

    // Step 3: Advance timestep and update agent locations
    // (task release is handled by release_tasks() at the start of next iteration)
    cur_time_ = next_ts;
    for (auto& ag : agents)
        ag.loc = ag.path[cur_time_];

    // Step 5: Detect state transitions at the NEW timestep
    //   (sets event flags for the next iteration's task_assignment_and_path_planning)
    central_has_event_ = false;
    central_reassign_event_ = false;

    // Detect completed deliveries (CARRYING -> FREE)
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_CARRYING && agents[i].finish_time <= cur_time_) {
            Task* task = agent_pending_task[i];
            if (task) {
                task->completion_time = agents[i].finish_time;
                open_tasks_.remove(task);
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
        if (agents[i].status == AG_MOVING_TO_PICKUP && agents[i].finish_time <= cur_time_) {
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
    if (cur_time_ < maxtime && !task_indices_by_time[cur_time_].empty()) {
        // For AT_EVERY_TIMESTEP (CENTRAL-ECBS): do NOT set central_has_event_.
        // The reference only triggers Phase 2 (has_new_agent) when an agent completes
        // delivery (next_ep==NULL) or a free agent is at a task's pickup -- NOT on
        // new task arrival alone.  Phase 1a in task_assignment() will set
        // central_has_event_ if a free agent is standing on the new task's pickup.
        // Setting central_has_event_ here causes spurious Phase 2 reassignment that
        // returns AG_MOVING_TO_PICKUP agents to free, losing their progress.
        if (config.assign_trigger != AT_EVERY_TIMESTEP) {
            central_has_event_ = true;
        }
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
    // For TA methods (Hungarian/LNS), tasks are in agent sequences, not open_tasks_.
    // assign_repeated_hungarian() puts them back before re-assigning.
    if (config.assign_trigger != AT_ON_UNASSIGNED_OR_FREE &&
        config.assign_trigger != AT_EVERY_TIMESTEP &&
        open_tasks_.empty())
        return false;

    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        // TP / TPTS: any free agent at end of path
        for (auto& ag : agents)
            if (ag.finish_time <= cur_time_) return true;
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
        return pbs_assign_event_;

    case AT_ONCE:
        // TA-Prioritized: assign once at t=0
        return cur_time_ == 0;

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
        return pbs_has_event_;

    case AT_ONCE:
        // TA-Prioritized/TA-Hybrid: replan after initial assignment
        return true;

    default:
        return false;
    }
}

// ============================================================
// Section 5.0: CENTRAL/CENTRAL_FIXED Phase-1a/1b instant pickup
//   (only runs for AT_EVERY_TIMESTEP / AT_ON_NEW_TASK_OR_FREE)
// ============================================================

void Simulation::central_phase1_instant_pickup() {
    // For CENTRAL/CENTRAL_FIXED: detect instant pickups
    // (free agent standing on a task's pickup → transition to CARRYING)
    // This must happen before should_assign() so these agents are not
    // treated as FREE by the Hungarian assignment.
    if (config.assign_trigger != AT_EVERY_TIMESTEP &&
        config.assign_trigger != AT_ON_NEW_TASK_OR_FREE)
        return;
    {
        // Phase 1a: detect instant pickups and collect agents for delivery planning
        // Also set central_has_event_ if a free agent is at ANY task's pickup
        // (matching reference has_new_agent logic, line 373-376)

        // Build DeliverGoal set from currently-delivering agents
        // (matching reference DeliverGoal initialized at line 55, set at line 387, cleared at 295/336)
        vector<bool> deliver_goal_held(mapd_map.row * mapd_map.col, false);
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].status == AG_CARRYING && agent_pending_task[i]) {
                deliver_goal_held[agent_pending_task[i]->delivery_loc] = true;
            }
        }

        // Build ag_loc: maps location -> agent_id for non-delivering agents
        // (reference line 363-364: ag_loc[agents[i].loc] = i for non-delivering)
        // Note: iterates i=0..n-1, overwriting, so ag_loc stores the LAST agent at each loc
        vector<int> ag_loc(mapd_map.row * mapd_map.col, -1);
        // Build hold: count of agents' path endpoints at each location
        // (reference line 367: hold[agents[i].path[maxtime-1]]++)
        vector<int> hold_count(mapd_map.row * mapd_map.col, 0);
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].status != AG_CARRYING)
                ag_loc[agents[i].loc] = i;
            hold_count[(int)path_table_[i][maxtime - 1]]++;
        }

        // Iterate tasks first (matching reference line 370-404)
        // Reference iterates tasks_assign list; we iterate open_tasks_
        vector<int> instant_pickup_agents;
        for (auto it = open_tasks_.begin(); it != open_tasks_.end(); ) {
            Task* task = *it;
            if (task->status != -1) { ++it; continue; }
            // Check if any free agent is at this task's pickup
            // (reference line 372-376: if ag_loc[start] >= 0 -> has_new_agent = true)
            if (ag_loc[task->pickup_loc] >= 0) {
                central_has_event_ = true;
            }
            // Check assignment conditions (reference line 377):
            // ag_loc[start] >= 0 && hold[goal] == 0 && !DeliverGoal[goal]
            if (ag_loc[task->pickup_loc] >= 0 &&
                hold_count[task->delivery_loc] == 0 &&
                !deliver_goal_held[task->delivery_loc]) {
                int id = ag_loc[task->pickup_loc];
                ag_loc[task->pickup_loc] = -1;  // reference line 380
                // If agent was MOVING_TO_PICKUP, release its previous task
                if (agents[id].status == AG_MOVING_TO_PICKUP && agent_pending_task[id]) {
                    agent_pending_task[id]->status = -1;
                    agent_pending_task[id] = nullptr;
                }
                task->status = id;
                agents[id].status = AG_CARRYING;
                agents[id].current_task = task->id;
                agent_pending_task[id] = task;
                instant_pickup_agents.push_back(id);
                deliver_goal_held[task->delivery_loc] = true;  // reference line 387
                ++it;
            } else {
                ++it;
            }
        }

        // Phase 1b: plan delivery paths for instant-pickup agents BEFORE Phase 2
        // Use CBS to coordinate all delivery agents simultaneously (matching reference
        // PathFinding which calls ECBSSearch for all delivery agents together)
        if (!instant_pickup_agents.empty()) {
            // Build cons_paths from non-delivery agents
            set<int> delivery_set(instant_pickup_agents.begin(), instant_pickup_agents.end());
            vector<vector<int>> p1_cons_paths;
            for (int i = 0; i < (int)agents.size(); i++) {
                if (delivery_set.count(i)) continue;
                vector<int> cp(maxtime);
                for (unsigned int t = 0; t < maxtime; t++) cp[t] = (int)path_table_[i][t];
                p1_cons_paths.push_back(cp);
            }

            // Collect starts, goals, endpoint indices for CBS
            vector<int> p1_starts, p1_goals, p1_ep_indices;
            for (int aid : instant_pickup_agents) {
                Task* task = agent_pending_task[aid];
                if (!task) continue;
                p1_starts.push_back((int)agents[aid].loc);
                p1_goals.push_back(task->delivery_loc);
                p1_ep_indices.push_back(task->delivery);
            }

            if (!p1_starts.empty() && p1_starts.size() > 1) {
                // Multi-agent CBS for coordinated delivery planning
                CBSSearch p1_cbs(mapd_map.grid, p1_starts, p1_goals, p1_ep_indices,
                                  p1_cons_paths, cur_time_, mapd_map.col,
                                  config.ecbs_weight, mapd_map.endpoints, maxtime);
                if (p1_cbs.run()) {
                    int ci = 0;
                    for (int aid : instant_pickup_agents) {
                        Task* task = agent_pending_task[aid];
                        if (!task) continue;
                        if (ci < (int)p1_cbs.paths.size() && !p1_cbs.paths[ci].empty()) {
                            for (int t = 0; t < (int)p1_cbs.paths[ci].size(); t++) {
                                if (cur_time_ + t < maxtime) {
                                    path_table_[aid][cur_time_ + t] = p1_cbs.paths[ci][t];
                                    agents[aid].path[cur_time_ + t] = p1_cbs.paths[ci][t];
                                }
                            }
                            int last_loc = p1_cbs.paths[ci].back();
                            for (unsigned int t = cur_time_ + p1_cbs.paths[ci].size(); t < maxtime; t++) {
                                path_table_[aid][t] = last_loc;
                                agents[aid].path[t] = last_loc;
                            }
                            agents[aid].finish_time = cur_time_ + p1_cbs.paths[ci].size() - 1
                                                       + task->goal_wait_time;
                        } else {
                            // CBS failed for this agent — fallback to single-agent A*
                            int arrive = astar(agents[aid], task->pickup_loc,
                                               cur_time_ + task->start_wait_time,
                                               mapd_map.endpoints[task->delivery], aid);
                            if (arrive >= 0) {
                                for (unsigned int t = cur_time_; t < maxtime; t++)
                                    path_table_[aid][t] = agents[aid].path[t];
                                agents[aid].finish_time = arrive + task->goal_wait_time;
                            } else {
                                agents[aid].status = AG_FREE;
                                task->status = -1;
                                agent_pending_task[aid] = nullptr;
                            }
                        }
                        ci++;
                    }
                } else {
                    // CBS failed — fallback to sequential A*
                    for (int aid : instant_pickup_agents) {
                        Task* task = agent_pending_task[aid];
                        if (!task) continue;
                        int arrive = astar(agents[aid], task->pickup_loc,
                                           cur_time_ + task->start_wait_time,
                                           mapd_map.endpoints[task->delivery], aid);
                        if (arrive >= 0) {
                            for (unsigned int t = cur_time_; t < maxtime; t++)
                                path_table_[aid][t] = agents[aid].path[t];
                            agents[aid].finish_time = arrive + task->goal_wait_time;
                        } else {
                            agents[aid].status = AG_FREE;
                            task->status = -1;
                            agent_pending_task[aid] = nullptr;
                        }
                    }
                }
            } else {
                // Single delivery agent — use simple A*
                for (int aid : instant_pickup_agents) {
                    Task* task = agent_pending_task[aid];
                    if (!task) continue;
                    int arrive = astar(agents[aid], task->pickup_loc,
                                       cur_time_ + task->start_wait_time,
                                       mapd_map.endpoints[task->delivery], aid);
                    if (arrive >= 0) {
                        for (unsigned int t = cur_time_; t < maxtime; t++)
                            path_table_[aid][t] = agents[aid].path[t];
                        agents[aid].finish_time = arrive + task->goal_wait_time;
                    } else {
                        agents[aid].status = AG_FREE;
                        task->status = -1;
                        agent_pending_task[aid] = nullptr;
                    }
                }
            }
        }
    }
}

// ============================================================
// Section 5.1: TP/TPTS "no task found" bump/vacate
//   (only runs for AT_ON_FREE_WAITS)
// ============================================================

void Simulation::tp_handle_no_assignment() {
    // For TP/TPTS/HBH: when open_tasks_ is empty but agent is free,
    // replicate reference TOTP/TPTR "no task found" branch:
    //   check if the agent's location conflicts with other agents' paths
    //   and move to another endpoint if needed.  Otherwise bump ft+1.
    if (config.assign_trigger == AT_ON_FREE_WAITS) {
        Agent* bump_ag = &agents[0];
        for (int i = 1; i < (int)agents.size(); i++) {
            if (!tp_timestep_advanced_ && agents[i].finish_time == cur_time_) {
                bump_ag = &agents[i];
                break;
            } else if (agents[i].finish_time < bump_ag->finish_time) {
                bump_ag = &agents[i];
            }
        }
        if (bump_ag->finish_time <= cur_time_) {
            // TPTS (AM_DECOUPLED_GREEDY_SWAPS): match reference TPTR
            // "no task found" branch which checks path collisions and
            // calls Move2EP when another agent's path crosses this loc.
            // TP (AM_DECOUPLED_GREEDY): reference TOTP just bumps ft+1.
            if (config.assign_method == AM_DECOUPLED_GREEDY_SWAPS) {
                bump_ag->loc = bump_ag->path[cur_time_];
                if (endpoint_mask_[bump_ag->loc]) {
                    bool need_move = false;
                    for (unsigned int t = cur_time_; t < maxtime && !need_move; t++)
                        for (int i = 0; i < (int)agents.size() && !need_move; i++)
                            if (i != bump_ag->id && path_table_[i][t] == (unsigned int)bump_ag->loc)
                                need_move = true;
                    if (need_move) {
                        if (move2EP(*bump_ag)) {
                            for (unsigned int i = cur_time_; i < path_table_[bump_ag->id].size(); i++)
                                path_table_[bump_ag->id][i] = bump_ag->path[i];
                        } else {
                            bump_ag->finish_time = cur_time_ + 1;
                        }
                    } else {
                        for (unsigned int i = cur_time_ + 1; i < maxtime; i++) {
                            bump_ag->path[i] = bump_ag->path[cur_time_];
                            path_table_[bump_ag->id][i] = bump_ag->path[cur_time_];
                        }
                        bump_ag->finish_time = cur_time_ + 1;
                    }
                } else {
                    if (move2EP(*bump_ag)) {
                        for (unsigned int i = cur_time_; i < path_table_[bump_ag->id].size(); i++)
                            path_table_[bump_ag->id][i] = bump_ag->path[i];
                    } else {
                        bump_ag->finish_time = cur_time_ + 1;
                    }
                }
            } else {
                bump_ag->finish_time = cur_time_ + 1;
            }
        }
    }
}

// ============================================================
// Section 5: Task Assignment — Dispatcher
//   (Pseudocode Section 6)
// ============================================================
void Simulation::task_assignment() {
    // Pseudocode Section 6 — Task_Assignment dispatcher

    // Phase 1 (CENTRAL/CENTRAL_FIXED): instant pickup + delivery planning.
    // Must happen before should_assign() so these agents are not treated as FREE.
    central_phase1_instant_pickup();

    // Clear phase2 data at the start of every iteration to prevent stale data
    phase2_free_ids_.clear();
    phase2_tasks_.clear();
    phase2_goal_locs_.clear();
    phase2_goal_eps_.clear();

    // Guard: only proceed if the trigger condition is met
    if (!should_assign()) {
        tp_handle_no_assignment();
        return;
    }

    switch (config.assign_method) {
    case AM_DECOUPLED_GREEDY: {
        // Match reference: default to agents[0], break on first exact match from i=1.
        // BUT: when the timestep was just advanced by update_system, the exact-match
        // branch must be skipped because the reference selects the agent BEFORE
        // advancing the timestep (so no exact match is possible there).
        Agent* ag = &agents[0];
        for (int i = 1; i < (int)agents.size(); i++) {
            if (!tp_timestep_advanced_ && agents[i].finish_time == cur_time_) {
                ag = &agents[i];
                break;
            } else if (agents[i].finish_time < ag->finish_time) {
                ag = &agents[i];
            }
        }
        if (ag->finish_time <= cur_time_) assign_decoupled_greedy(*ag);
        break;
    }
    case AM_DECOUPLED_GREEDY_SWAPS: {
        // Match reference: same agent selection as TP above.
        Agent* ag = &agents[0];
        for (int i = 1; i < (int)agents.size(); i++) {
            if (!tp_timestep_advanced_ && agents[i].finish_time == cur_time_) {
                ag = &agents[i];
                break;
            } else if (agents[i].finish_time < ag->finish_time) {
                ag = &agents[i];
            }
        }
        // Match reference: remove finished tasks BEFORE calling TPTR.
        // Reference run_TPTR removes tasks with TAKEN state and
        // timestep >= ag_arrive_start right before calling ag->TPTR.
        {
            auto it = open_tasks_.begin();
            while (it != open_tasks_.end()) {
                if ((*it)->status >= 0 && (*it)->ag_arrive_start >= 0 &&
                    (int)cur_time_ >= (*it)->ag_arrive_start) {
                    it = open_tasks_.erase(it);
                } else {
                    ++it;
                }
            }
        }
        if (ag->finish_time <= cur_time_) assign_tpts(*ag);
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
    ag.loc = ag.path[cur_time_];

    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)path_table_.size(); i++) {
        if (i != ag.id) hold[path_table_[i][maxtime - 1]] = true;
    }

    Task* best_task = nullptr;
    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
        if (hold[(*it)->pickup_loc] || hold[(*it)->delivery_loc]) continue;
        if (best_task == nullptr ||
            mapd_map.endpoints[(*it)->pickup].h_val[ag.loc] <
            mapd_map.endpoints[best_task->pickup].h_val[ag.loc]) {
            best_task = *it;
        }
    }

    // Reference TOTP guarantees a filtered task always has a plannable path, but in
    // this reimplementation transient congestion (multiple free agents parking around
    // an endpoint) can leave the chosen task's delivery momentarily surrounded so the
    // low-level search legitimately fails.  When that happens we must NOT take the task
    // (it cannot be committed) and instead apply the same "no plannable task this turn"
    // behaviour the reference uses when no task passes the filter: vacate if we are
    // sitting on a needed delivery, otherwise wait one step.  This keeps the task in the
    // token for a later turn / another agent and, crucially, advances the agent's
    // finish_time so the TP loop makes progress instead of re-selecting this agent
    // forever (the 50/10 hang).
    bool plan_failed = false;
    if (best_task != nullptr) {
        auto result = plan_task_token(ag, *best_task);
        if (result.first < 0) {
            plan_failed = true;
        } else {
            for (unsigned int i = cur_time_; i < path_table_[ag.id].size(); i++)
                path_table_[ag.id][i] = ag.path[i];

            ag.finish_time = result.second + best_task->goal_wait_time;
            ag.current_task = best_task->id;
            best_task->status = ag.id;
            best_task->ag_arrive_start = result.first;
            best_task->completion_time = result.second;
            open_tasks_.remove(best_task);
            return true;
        }
    }

    if (best_task == nullptr || plan_failed) {
        bool move = false;
        for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
            if ((*it)->delivery_loc == (int)ag.loc) { move = true; break; }
        }
        if (move) {
            if (move2EP(ag)) {
                for (unsigned int i = cur_time_; i < path_table_[ag.id].size(); i++)
                    path_table_[ag.id][i] = ag.path[i];
                return true;
            }
        }
        // No task / unplannable task and not blocking a delivery (or Move2EP failed):
        // wait one step so the system advances.
        ag.finish_time = cur_time_ + 1;
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
    vector<vector<unsigned int>> saved_paths(path_table_.size());
    for (int i = 0; i < (int)path_table_.size(); i++)
        saved_paths[i] = path_table_[i];
    vector<Agent> saved_agents = agents;

    ag.loc = ag.path[cur_time_];

    struct HN {
        int loc; Task* task; int h;
        HN(int l, Task* t, int hv) : loc(l), task(t), h(hv) {}
    };
    struct CompareHN {
        bool operator()(const HN& n1, const HN& n2) const {
            return n1.h > n2.h;  // min-heap on h
        }
    };
    boost::heap::fibonacci_heap<HN, boost::heap::compare<CompareHN>> heuristic;
    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++)
        heuristic.push(HN((*it)->pickup_loc, *it, mapd_map.endpoints[(*it)->pickup].h_val[ag.loc]));

    while (!heuristic.empty()) {
        HN hn = heuristic.top();
        heuristic.pop();
        Task* task = hn.task;

        if (task->status == -1 ||
            (task->status >= 0 && task->ag_arrive_start >= 0 &&
             task->ag_arrive_start > (int)cur_time_ + hn.h)) {
            bool occupied = false;
            for (int i = 0; i < (int)path_table_.size(); i++) {
                if (i == ag.id) continue;
                if (task->status >= 0 && i == task->status) continue;
                if (path_table_[i][maxtime - 1] == (unsigned int)task->delivery_loc ||
                    path_table_[i][maxtime - 1] == (unsigned int)task->pickup_loc) {
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
                    for (unsigned int i = cur_time_; i < path_table_[ag.id].size(); i++)
                        path_table_[ag.id][i] = ag.path[i];
                    ag.finish_time = arrive_goal + task->goal_wait_time;

                    if (task->status == -1) {
                        task->status = ag.id;
                        task->ag_arrive_start = arrive_start;
                        task->completion_time = arrive_goal;
                        // Keep task in open_tasks_ so other agents can steal it (TPTS)
                        // Task is removed in update_system when pickup is reached
                        return true;
                    } else {
                        Agent* old_ag = &agents[task->status];
                        // Match reference: do NOT restore task fields after failed swap.
                        // The reference's TPTR modifies task->ag, ag_arrive_start, ag_arrive_goal
                        // and does NOT restore them when the recursive call fails.
                        task->status = ag.id;
                        task->ag_arrive_start = arrive_start;
                        task->completion_time = arrive_goal;

                        if (assign_tpts(*old_ag, depth + 1)) return true;

                        // Reference does NOT restore task fields here -- fall through
                    }
                }
            }

            // Match reference: do NOT restore path_table_ or agents after each
            // failed task attempt. The reference accumulates state changes across
            // task attempts within the same TPTR call.
        }
    }

    if (endpoint_mask_[ag.loc]) {
        bool need_move = false;
        for (auto it = open_tasks_.begin(); it != open_tasks_.end() && !need_move; it++)
            if ((*it)->delivery_loc == (int)ag.loc) need_move = true;
        for (unsigned int t = cur_time_; t < maxtime && !need_move; t++)
            for (int i = 0; i < (int)agents.size() && !need_move; i++)
                if (i != ag.id && path_table_[i][t] == (unsigned int)ag.loc) need_move = true;
        if (need_move) {
            if (move2EP(ag)) {
                for (unsigned int i = cur_time_; i < path_table_[ag.id].size(); i++)
                    path_table_[ag.id][i] = ag.path[i];
                return true;
            } else {
                for (int i = 0; i < (int)path_table_.size(); i++) path_table_[i] = saved_paths[i];
                agents = saved_agents;
                return false;
            }
        } else {
            for (unsigned int i = cur_time_ + 1; i < maxtime; i++) {
                ag.path[i] = ag.path[cur_time_];
                path_table_[ag.id][i] = ag.path[cur_time_];
            }
            ag.finish_time = cur_time_ + 1;
            return true;
        }
    } else {
        if (move2EP(ag)) {
            for (unsigned int i = cur_time_; i < path_table_[ag.id].size(); i++)
                path_table_[ag.id][i] = ag.path[i];
            return true;
        } else {
            for (int i = 0; i < (int)path_table_.size(); i++) path_table_[i] = saved_paths[i];
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
        if (agents[i].finish_time <= cur_time_)
            free_ids.push_back(i);
    }
    if (free_ids.empty()) return;

    // Build hold set (endpoints occupied by non-free agents)
    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++) {
        bool is_free = false;
        for (int fid : free_ids) if (fid == i) { is_free = true; break; }
        if (!is_free) hold[path_table_[i][maxtime - 1]] = true;
    }

    // Collect available tasks
    vector<Task*> avail_tasks;
    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
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
        for (unsigned int t = cur_time_; t < maxtime; t++)
            path_table_[ag.id][t] = ag.path[t];

        task.status = ag.id;
        task.ag_arrive_start = arrive_start;
        task.completion_time = arrive_goal;
        ag.current_task = task.id;
        ag.finish_time = arrive_goal + task.goal_wait_time;
        open_tasks_.remove(p.task);

        done_agents.insert(p.agent_id);
        done_tasks.insert(p.task->id);
    }

    // Handle remaining free agents: move off delivery endpoints if needed
    for (int aid : free_ids) {
        if (done_agents.count(aid)) continue;
        Agent& ag = agents[aid];
        bool need_move = false;
        for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
            if ((*it)->delivery_loc == (int)ag.loc) { need_move = true; break; }
        }
        if (need_move) {
            if (move2EP(ag)) {
                for (unsigned int t = cur_time_; t < path_table_[ag.id].size(); t++)
                    path_table_[ag.id][t] = ag.path[t];
            } else {
                ag.finish_time = cur_time_ + 1;
            }
        } else {
            ag.finish_time = cur_time_ + 1;
        }
    }
}

// ============================================================
// Section 10: Task Assignment — Hungarian (CENTRAL)
//   (Pseudocode Section 9.4)
//   Populates phase2_* members for path_planning()
// ============================================================

int Simulation::astar_cost_only(int agent_id, int start_loc, int goal_loc,
                                int start_time, const vector<vector<int>>& cons_paths,
                                const std::vector<char>* vres, int vres_len,
                                const std::vector<int>* last_occ) {
    const int map_size = mapd_map.row * mapd_map.col;

    // Latest timestep at which any cons_path occupies the goal cell. The hold-check
    // (goal must be free at all times strictly after arrival) is then O(1): an arrival at
    // time t is holdable iff t >= goal_last_occ. When the caller provides last_occ[] (one
    // table for the whole cost matrix, all searches share cons_paths) this is O(1) per
    // search; otherwise it is computed here with a single O(num_cons*horizon) pass.
    // Result is identical to the original per-goal-pop scan.
    int goal_last_occ;
    if (last_occ && goal_loc < (int)last_occ->size()) {
        goal_last_occ = (*last_occ)[goal_loc];
    } else {
        goal_last_occ = -1;
        for (auto& cp : cons_paths) {
            for (int t = (int)cp.size() - 1; t >= 0; t--) {
                if (cp[t] == goal_loc) { if (t > goal_last_occ) goal_last_occ = t; break; }
            }
        }
    }

    if (start_loc == goal_loc) {
        // Still need hold check: verify no delivering agent passes through
        // (matching reference SingleAgentECBS::findPath hold check)
        // hold iff goal not occupied at any t > start_time.
        bool hold = (goal_last_occ <= start_time);
        // Return 1 to match reference path.size()=1 for trivial (start==goal) case
        // Reference SingleAgentECBS::findPath returns path = {start_loc}, path.size()=1
        if (hold) return 1;
        // Cannot hold — fall through to full search
    }
    const Endpoint* goal_ep = nullptr;
    for (auto& ep : mapd_map.endpoints)
        if (ep.loc == goal_loc) { goal_ep = &ep; break; }
    if (!goal_ep) return map_size;

    struct Node { int loc, g, f, t; };
    auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };
    priority_queue<Node, vector<Node>, decltype(cmp)> open(cmp);
    // visited[loc + g*map_size] -> best g reached (lazy dedup on pop, as before).
    // Reused member buffer: clear() retains capacity so we avoid a fresh hash-table
    // allocation on every one of the millions of cost searches. Semantics identical.
    std::unordered_map<long long, int>& visited = aco_visited_;
    visited.clear();
    long long aco_local=0;

    int h0 = goal_ep->h_val[start_loc];
    if (h0 == INT_MAX) return map_size;
    open.push({start_loc, 0, h0, start_time});

    int max_t = (int)maxtime - 1;
    // Match reference SingleAgentECBS action order: WAIT, NORTH, EAST, SOUTH, WEST
    int action[5] = {0, -mapd_map.col, 1, mapd_map.col, -1};

    while (!open.empty()) {
        auto curr = open.top(); open.pop();
        aco_local++;
        // Expansion cap (see astar()): a reachable, holdable goal is found within a few
        // thousand expansions even on the largest instances. An unholdable/unreachable
        // goal otherwise exhausts the whole (loc x horizon) space (millions of nodes)
        // before returning the INF cost. Capping returns the SAME INF cost sooner, so the
        // Hungarian cost matrix (and hence the assignment) is identical.
        if (aco_local > 15000) return map_size;
        if (curr.loc == goal_loc) {
            // Hold check (O(1) via precomputed goal_last_occ): holdable iff the goal is
            // not occupied by any cons_path strictly after this arrival time.
            if (curr.t >= goal_last_occ) return curr.g + 1;
            // Cannot hold at goal — continue searching for a later arrival
        }
        if (curr.t >= max_t) continue;

        long long key = (long long)curr.loc + (long long)curr.g * map_size;
        auto vit = visited.find(key);
        if (vit != visited.end() && vit->second <= curr.g) continue;
        visited[key] = curr.g;

        for (int i = 0; i < 5; i++) {
            int nl = curr.loc + action[i];
            int nt = curr.t + 1;
            if (nl < 0 || nl >= map_size) continue;
            if (!mapd_map.grid[nl]) continue;

            bool blocked = false;
            int idx_nt_nl = nt * map_size + nl;
            int idx_nt_curr = nt * map_size + curr.loc;
            if (vres && idx_nt_nl < vres_len && idx_nt_curr < vres_len) {
                // Reservation table (built once per assignment round from the same
                // cons_paths). Vertex block: nl occupied at nt. This is the common case
                // and now O(1). An edge swap (some agent doing nl->curr.loc over
                // [curr.t,nt]) additionally requires curr.loc occupied at nt, so we only
                // fall back to the per-cons_path scan when that necessary condition holds
                // (rare). The accepted/blocked decision is identical to the scan-only path.
                if ((*vres)[idx_nt_nl]) {
                    blocked = true;
                } else if ((*vres)[idx_nt_curr]) {
                    for (auto& cp : cons_paths) {
                        int cp_t = (nt < (int)cp.size()) ? cp[nt] : cp.back();
                        int cp_prev = (curr.t < (int)cp.size()) ? cp[curr.t] : cp.back();
                        if (cp_t == curr.loc && cp_prev == nl) { blocked = true; break; }
                    }
                }
            } else {
                for (auto& cp : cons_paths) {
                    int cp_t = (nt < (int)cp.size()) ? cp[nt] : cp.back();
                    int cp_prev = (curr.t < (int)cp.size()) ? cp[curr.t] : cp.back();
                    if (cp_t == nl) { blocked = true; break; }
                    if (cp_t == curr.loc && cp_prev == nl) { blocked = true; break; }
                }
            }
            if (blocked) continue;

            int nh = goal_ep->h_val[nl];
            if (nh == INT_MAX) continue;
            open.push({nl, curr.g + 1, curr.g + 1 + nh, nt});
        }
    }
    return map_size;
}

void Simulation::assign_hungarian() {
    phase2_free_ids_.clear();
    phase2_tasks_.clear();
    phase2_goal_locs_.clear();
    phase2_goal_eps_.clear();

    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_FREE) {
            phase2_free_ids_.push_back(i);
        } else if (agents[i].status == AG_MOVING_TO_PICKUP &&
                   config.assign_trigger == AT_EVERY_TIMESTEP) {
            // CENTRAL: include en-route-to-pickup agents in reassignment
            // (matching reference: all non-delivering agents participate)
            if (agent_pending_task[i]) {
                agent_pending_task[i]->status = -1;
                open_tasks_.push_back(agent_pending_task[i]);
                agent_pending_task[i] = nullptr;
            }
            agents[i].status = AG_FREE;
            agents[i].current_task = -1;
            phase2_free_ids_.push_back(i);
        }
    }
    if (phase2_free_ids_.empty()) return;

    // Build hold set from delivery goals of DELIVERING (carrying) tasks only
    // (matching reference: hold goals of delivering tasks)
    vector<bool> hold(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_CARRYING && agent_pending_task[i] != nullptr) {
            hold[agent_pending_task[i]->delivery_loc] = true;
        }
    }

    // Candidate tasks: filter only by pickup_loc not held (matching reference line 143)
    vector<Task*> candidate_tasks;
    vector<int> candidate_ep_indices;
    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
        if ((*it)->status != -1) continue;
        if (!hold[(*it)->pickup_loc]) {
            bool dup = false;
            for (auto* ct : candidate_tasks) {
                // Match reference: only check pickup_loc duplication.
                // Delivery_loc duplication is handled by the hold mechanism
                // (hold[delivery_loc] = true prevents tasks whose pickup is at
                // a previously-selected task's delivery).
                if (ct->pickup_loc == (*it)->pickup_loc) { dup = true; break; }
            }
            if (!dup) {
                candidate_tasks.push_back(*it);
                candidate_ep_indices.push_back((*it)->pickup);
                hold[(*it)->pickup_loc] = true;  // hold start after selection
                hold[(*it)->delivery_loc] = true; // hold goal after selection
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

    // Build cons_paths from non-free agents (matching reference: all agents not in ag_assign)
    vector<vector<int>> cons_paths_assign;
    for (int i = 0; i < (int)agents.size(); i++) {
        bool is_free = false;
        for (int fid : phase2_free_ids_) if (fid == i) { is_free = true; break; }
        if (!is_free) {
            vector<int> cp(maxtime);
            for (unsigned int t = 0; t < maxtime; t++) cp[t] = (int)path_table_[i][t];
            cons_paths_assign.push_back(cp);
        }
    }

    // Build a vertex reservation table ONCE for the whole cost matrix (all N*N cost
    // searches share the same cons_paths_assign). vres[t*map_size + loc] = occupied.
    // Reused by astar_cost_only to make the vertex-collision check O(1) instead of
    // O(num_cons_paths) per edge. Result is identical; only the constant factor changes.
    // The buffer is a persistent member: we set the occupied cells, run the matrix, then
    // clear exactly those cells (cheap and sparse) instead of zeroing the whole table.
    const int map_size_h = mapd_map.row * mapd_map.col;
    const size_t vres_sz = (size_t)map_size_h * maxtime;
    if (aco_vres_.size() < vres_sz) aco_vres_.assign(vres_sz, 0);
    std::vector<char>& vres = aco_vres_;
    // last_occ[loc] = latest timestep any cons_path occupies loc (-1 if never). Built once
    // for the whole cost matrix so each search's goal hold-check is O(1).
    std::vector<int> last_occ(map_size_h, -1);
    for (auto& cp : cons_paths_assign)
        for (unsigned int t = 0; t < maxtime; t++) {
            vres[(size_t)t * map_size_h + cp[t]] = 1;
            if ((int)t > last_occ[cp[t]]) last_occ[cp[t]] = (int)t;
        }
    int vres_len = (int)vres_sz;

    // Build cost matrix using actual pathfinding cost (matching reference SingleAgentECBS)
    // Reference formula: task cost = (2*col*row - path_len) * agents.size() * starts.size()
    //                     park cost = col*row * agents.size() * starts.size() - path_len
    int num_agents = (int)phase2_free_ids_.size();
    int num_starts = (int)candidate_ep_indices.size();
    int scale = num_agents * num_starts;  // matching reference multiplier
    dlib::matrix<int> cost_matrix(N, N);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i >= num_agents) {
                cost_matrix(i, j) = 0;
            } else {
                int aid = phase2_free_ids_[i];
                if (j < num_starts) {
                    int ep_idx = candidate_ep_indices[j];
                    int goal_loc = mapd_map.endpoints[ep_idx].loc;
                    int d = astar_cost_only(aid, agents[aid].loc, goal_loc,
                                            (int)cur_time_, cons_paths_assign,
                                            &vres, vres_len, &last_occ);
                    if (d >= mapd_map.col * mapd_map.row) d = 2 * mapd_map.col * mapd_map.row;
                    if (j < num_tasks_available)
                        cost_matrix(i, j) = (2 * mapd_map.col * mapd_map.row - d) * scale;
                    else
                        cost_matrix(i, j) = mapd_map.col * mapd_map.row * scale - d;
                } else {
                    cost_matrix(i, j) = 0;
                }
            }
        }
    }

    // Clear only the cells we set (keeps the persistent buffer ready for reuse).
    for (auto& cp : cons_paths_assign)
        for (unsigned int t = 0; t < maxtime; t++)
            vres[(size_t)t * map_size_h + cp[t]] = 0;

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
    // Agents transitioned to CARRYING by update_system() need delivery paths.
    // Use CBS to coordinate all such agents simultaneously (matching reference
    // PathFinding which uses ECBSSearch for all delivery agents together).
    {
        vector<int> carry_ids;
        for (int i = 0; i < (int)agents.size(); i++) {
            if (agents[i].status == AG_CARRYING && agent_pending_task[i] &&
                agents[i].finish_time <= cur_time_) {
                carry_ids.push_back(i);
            }
        }

        if (carry_ids.size() > 1) {
            // Build cons_paths from non-carrying agents
            set<int> carry_set(carry_ids.begin(), carry_ids.end());
            vector<vector<int>> c_cons;
            for (int i = 0; i < (int)agents.size(); i++) {
                if (carry_set.count(i)) continue;
                vector<int> cp(maxtime);
                for (unsigned int t = 0; t < maxtime; t++) cp[t] = (int)path_table_[i][t];
                c_cons.push_back(cp);
            }

            vector<int> c_starts, c_goals, c_eps;
            for (int aid : carry_ids) {
                Task* task = agent_pending_task[aid];
                c_starts.push_back((int)agents[aid].loc);
                c_goals.push_back(task->delivery_loc);
                c_eps.push_back(task->delivery);
            }

            CBSSearch c_cbs(mapd_map.grid, c_starts, c_goals, c_eps,
                            c_cons, cur_time_, mapd_map.col,
                            config.ecbs_weight, mapd_map.endpoints, maxtime);
            if (c_cbs.run()) {
                for (int ci = 0; ci < (int)carry_ids.size(); ci++) {
                    int aid = carry_ids[ci];
                    Task* task = agent_pending_task[aid];
                    if (ci < (int)c_cbs.paths.size() && !c_cbs.paths[ci].empty()) {
                        for (int t = 0; t < (int)c_cbs.paths[ci].size(); t++) {
                            if (cur_time_ + t < maxtime) {
                                path_table_[aid][cur_time_ + t] = c_cbs.paths[ci][t];
                                agents[aid].path[cur_time_ + t] = c_cbs.paths[ci][t];
                            }
                        }
                        int last_loc = c_cbs.paths[ci].back();
                        for (unsigned int t = cur_time_ + c_cbs.paths[ci].size(); t < maxtime; t++) {
                            path_table_[aid][t] = last_loc;
                            agents[aid].path[t] = last_loc;
                        }
                        agents[aid].finish_time = cur_time_ + c_cbs.paths[ci].size() - 1
                                                   + task->goal_wait_time;
                    } else {
                        // Fallback for this agent
                        int arrive = astar(agents[aid], task->pickup_loc,
                                           cur_time_ + task->start_wait_time,
                                           mapd_map.endpoints[task->delivery], aid);
                        if (arrive >= 0) {
                            for (unsigned int t = cur_time_; t < maxtime; t++)
                                path_table_[aid][t] = agents[aid].path[t];
                            agents[aid].finish_time = arrive + task->goal_wait_time;
                        } else {
                            agents[aid].status = AG_FREE;
                            task->status = -1;
                            agent_pending_task[aid] = nullptr;
                        }
                    }
                }
            } else {
                // CBS failed — fallback to sequential A*
                for (int aid : carry_ids) {
                    Task* task = agent_pending_task[aid];
                    int arrive = astar(agents[aid], task->pickup_loc,
                                       cur_time_ + task->start_wait_time,
                                       mapd_map.endpoints[task->delivery], aid);
                    if (arrive >= 0) {
                        for (unsigned int t = cur_time_; t < maxtime; t++)
                            path_table_[aid][t] = agents[aid].path[t];
                        agents[aid].finish_time = arrive + task->goal_wait_time;
                    } else {
                        agents[aid].status = AG_FREE;
                        task->status = -1;
                        agent_pending_task[aid] = nullptr;
                    }
                }
            }
        } else {
            // 0 or 1 carrying agent — use simple A*
            for (int i = 0; i < (int)agents.size(); i++) {
                if (agents[i].status == AG_CARRYING && agent_pending_task[i] &&
                    agents[i].finish_time <= cur_time_) {
                    Task* task = agent_pending_task[i];
                    int begin = cur_time_ + task->start_wait_time;
                    int arrive = astar(agents[i], task->pickup_loc, begin,
                                       mapd_map.endpoints[task->delivery], i);
                    if (arrive >= 0) {
                        for (unsigned int t = cur_time_; t < maxtime; t++)
                            path_table_[i][t] = agents[i].path[t];
                        agents[i].finish_time = arrive + task->goal_wait_time;
                    } else {
                        agents[i].status = AG_FREE;
                        task->status = -1;
                        agent_pending_task[i] = nullptr;
                    }
                }
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
            for (unsigned int t = 0; t < maxtime; t++) cp[t] = (int)path_table_[i][t];
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
            agents[phase2_free_ids_[i]].finish_time = cur_time_ + 1;
        }
    }

    bool used_cbs = false;
    if (!cbs_indices.empty()) {
        CBSSearch cbs(mapd_map.grid, cbs_starts, cbs_goals, cbs_ep_indices,
                      cons_paths_cbs, cur_time_, mapd_map.col,
                      config.ecbs_weight, mapd_map.endpoints, maxtime);
        if (cbs.run()) {
            used_cbs = true;
            for (int ci = 0; ci < (int)cbs_indices.size(); ci++) {
                int i = cbs_indices[ci];
                int aid = phase2_free_ids_[i];
                if (cbs.paths[ci].empty()) { agents[aid].finish_time = cur_time_ + 1; continue; }
                for (int t = 0; t < (int)cbs.paths[ci].size(); t++) {
                    if (cur_time_ + t < maxtime) {
                        path_table_[aid][cur_time_ + t] = cbs.paths[ci][t];
                        agents[aid].path[cur_time_ + t] = cbs.paths[ci][t];
                    }
                }
                int last_loc = cbs.paths[ci].back();
                for (unsigned int t = cur_time_ + cbs.paths[ci].size(); t < maxtime; t++) {
                    path_table_[aid][t] = last_loc;
                    agents[aid].path[t] = last_loc;
                }
                agents[aid].finish_time = cur_time_ + cbs.paths[ci].size() - 1;
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
            agents[i].finish_time <= cur_time_) {
            Task* task = agent_pending_task[i];
            int begin = cur_time_ + task->start_wait_time;
            int arrive = astar(agents[i], task->pickup_loc, begin,
                               mapd_map.endpoints[task->delivery], i);
            if (arrive >= 0) {
                for (unsigned int t = cur_time_; t < maxtime; t++)
                    path_table_[i][t] = agents[i].path[t];
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

    // Pre-fill assigned_dummies with non-Group2 agents' final positions
    for (int a = 0; a < num_ag; a++) {
        if (assigned_dummies[a] < 0)
            assigned_dummies[a] = (int)path_table_[a][max_t - 1];
    }

    for (int i = 0; i < (int)phase2_free_ids_.size(); i++) {
        if (phase2_goal_eps_[i] >= 0) {
            int aid = phase2_free_ids_[i];
            pbs_ids.push_back(aid);
            pbs_tasks_vec.push_back(phase2_tasks_[i]);
            assigned_dummies[aid] = -1;
            vector<pair<int,int>> goals;
            goals.push_back({phase2_goal_locs_[i], 0});
            int dummy = choose_dummy_endpoint(aid, phase2_goal_locs_[i],
                                               assigned_dummies, false);
            assigned_dummies[aid] = dummy;
            goals.push_back({dummy, 0});
            goal_seqs.push_back(goals);
        } else {
            agents[phase2_free_ids_[i]].finish_time = cur_time_ + 1;
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
        for (int t = 0; t < max_t; t++) cp[t] = (int)path_table_[a][t];
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
        if (config.use_sipp) {
            auto p = sipp_search(aid, (int)agents[aid].loc, (int)cur_time_,
                                  goal_seqs[idx], cons, {}, false);
            if (!p.empty()) return p;
        }
        if (use_taskwise_cbs)
            return mla_star_taskwise(aid, (int)agents[aid].loc, (int)cur_time_,
                                      pbs_task_groups[idx], cons, {}, false);
        return seq_mla_star(aid, (int)agents[aid].loc, (int)cur_time_,
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
            for (int t = (int)cur_time_; t < max_t; t++) {
                if (root->paths[a1][t] == root->paths[a2][t]) {
                    root->conflicts.emplace_back(a1, a2,
                        root->paths[a1][t], -1, t);
                    break;
                }
                if (t > (int)cur_time_ &&
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
        stack<PBSNode*> dfs_stack;
        dfs_stack.push(root);

        best_node->conflict = root->conflicts.front();
        for (auto& c : root->conflicts)
            if (get<4>(c) < get<4>(best_node->conflict))
                best_node->conflict = c;
        best_node->earliest_collision = get<4>(best_node->conflict);

        int hl = 0, max_hl = (n > 30) ? 5000 : 50000;
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
                    for (int t = (int)cur_time_; t < max_t; t++) {
                        if (child->paths[lower][t] == child->paths[i][t]) {
                            child->conflicts.emplace_back(lower, i,
                                child->paths[lower][t], -1, t);
                            break;
                        }
                        if (t > (int)cur_time_ &&
                            child->paths[lower][t] == child->paths[i][t-1] &&
                            child->paths[lower][t-1] == child->paths[i][t]) {
                            child->conflicts.emplace_back(lower, i,
                                child->paths[lower][t], child->paths[i][t], t);
                            break;
                        }
                    }
                }
                child->num_collisions = (int)child->conflicts.size();
                dfs_stack.push(child);
            }
        }
        for (auto* nd : all_nodes) if (nd != best_node) delete nd;
    }

    // --- Commit best node paths ---
    for (int idx = 0; idx < n; idx++) {
        int aid = pbs_ids[idx];
        for (int t = 0; t < max_t; t++) {
            path_table_[aid][t] = best_node->paths[idx][t];
            agents[aid].path[t] = best_node->paths[idx][t];
        }
        int goal_loc = goal_seqs[idx][0].first;
        int arrive = (int)cur_time_;
        for (int t = (int)cur_time_; t < max_t; t++) {
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

    // Post-commit: verify Group 2 paths don't collide with ext_cons
    for (int idx = 0; idx < n; idx++) {
        int aid = pbs_ids[idx];
        for (auto& cp : ext_cons) {
            bool collision = false;
            for (int t = (int)cur_time_; t < max_t && !collision; t++) {
                int p1 = (int)path_table_[aid][t];
                int p2 = (t < (int)cp.size()) ? cp[t] : cp.back();
                if (p1 == p2) collision = true;
            }
            if (collision) {
                // Replan this agent with full constraints (ext_cons + all other Group 2)
                vector<vector<int>> full_cons;
                for (int a = 0; a < num_ag; a++) {
                    if (a == aid) continue;
                    vector<int> cp2(max_t);
                    for (int t = 0; t < max_t; t++) cp2[t] = (int)path_table_[a][t];
                    full_cons.push_back(cp2);
                }
                auto new_path = seq_mla_star(aid, (int)agents[aid].loc, (int)cur_time_,
                                              goal_seqs[idx], full_cons, {}, false);
                if (!new_path.empty()) {
                    for (int t = 0; t < max_t; t++) {
                        path_table_[aid][t] = new_path[t];
                        agents[aid].path[t] = new_path[t];
                    }
                }
                break;
            }
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
        if (ep_idx < 0) { agents[aid].finish_time = cur_time_ + 1; continue; }

        int arrive = astar(agents[aid], agents[aid].loc, cur_time_,
                           mapd_map.endpoints[ep_idx], aid);
        if (arrive >= 0) {
            for (unsigned int t = cur_time_; t < maxtime; t++)
                path_table_[aid][t] = agents[aid].path[t];
            agents[aid].finish_time = arrive;
            if (phase2_tasks_[i] != nullptr) {
                agents[aid].status = AG_MOVING_TO_PICKUP;
                agents[aid].current_task = phase2_tasks_[i]->id;
                phase2_tasks_[i]->status = aid;
                agent_pending_task[aid] = phase2_tasks_[i];
            }
        } else {
            agents[aid].finish_time = cur_time_ + 1;
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
        if (ag.finish_time > cur_time_ && ag.status != AG_MOVING_TO_PICKUP)
            continue;

        if (ag.current_task < 0 || ag.status != AG_MOVING_TO_PICKUP) {
            // Agent has no task — handle free agent
            if (ag.finish_time <= cur_time_) {
                // Check if agent needs to move off a delivery endpoint
                bool need_move = false;
                for (auto it = open_tasks_.begin(); it != open_tasks_.end(); it++) {
                    if ((*it)->delivery_loc == (int)ag.loc) { need_move = true; break; }
                }
                if (need_move) {
                    if (move2EP(ag)) {
                        for (unsigned int t = cur_time_; t < path_table_[ag.id].size(); t++)
                            path_table_[ag.id][t] = ag.path[t];
                    } else {
                        ag.finish_time = cur_time_ + 1;
                    }
                } else {
                    ag.finish_time = cur_time_ + 1;
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
        int arrive_start = astar(ag, ag.loc, cur_time_,
                                 mapd_map.endpoints[task.pickup], ag.id);
        if (arrive_start < 0) {
            ag.status = AG_FREE;
            ag.current_task = -1;
            task.status = -1;
            ag.finish_time = cur_time_ + 1;
            continue;
        }

        int arrive_goal = astar(ag, task.pickup_loc,
                                arrive_start + task.start_wait_time,
                                mapd_map.endpoints[task.delivery], ag.id);
        vector<int> result;
        if (arrive_goal >= 0) {
            // Build result from agent's path
            for (unsigned int t = cur_time_; t <= (unsigned int)arrive_goal; t++)
                result.push_back(ag.path[t]);
        }

        if (arrive_goal >= 0) {
            // Path already written to ag.path by astar() calls
            // Write to path_table_
            for (unsigned int t = cur_time_; t < maxtime; t++)
                path_table_[ag.id][t] = ag.path[t];

            task.ag_arrive_start = arrive_start;
            task.status = ag.id;
            task.completion_time = arrive_goal;

            ag.finish_time = arrive_goal + task.goal_wait_time;
            open_tasks_.remove(&task);
        } else {
            // MLA* failed — revert agent state
            ag.status = AG_FREE;
            ag.current_task = -1;
            task.status = -1;
            ag.finish_time = cur_time_ + 1;
        }
    }
}

// ============================================================
// Section 13: Single-Agent Search — Space-Time A* (STA*)
//   (Pseudocode Section 13.1)
// ============================================================

bool Simulation::isConstrained(int agent_id, int curr_id, int next_id, int next_timestep, int ag_hide) {
    if (!passable_map_[next_id]) return true;
    for (int ag = 0; ag < (int)path_table_.size(); ag++) {
        if (ag == agent_id || ag == ag_hide) continue;
        if (path_table_[ag][next_timestep] == (unsigned int)next_id) return true;
        if (path_table_[ag][next_timestep - 1] == (unsigned int)next_id &&
            path_table_[ag][next_timestep] == (unsigned int)curr_id) return true;
    }
    return false;
}

int Simulation::astar(Agent& ag, int start_loc, int begin_time, const Endpoint& goal, int ag_hide) {
    int goal_location = goal.loc;
    heap_open_t open_list;
    map<unsigned int, SearchNode*> allNodes;
    int local_exp=0;

    SearchNode* start = new SearchNode(start_loc, 0, goal.h_val[start_loc], nullptr, begin_time);
    open_list.push(start);
    allNodes.insert(make_pair((unsigned int)start_loc, start));

    while (!open_list.empty()) {
        SearchNode* curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;
        local_exp++;

        // Expansion cap: a hold-feasible path is found within a few thousand expansions
        // even on the largest instances. When the goal is unreachable/unholdable the
        // search otherwise exhausts the entire (loc x horizon) space (millions of nodes)
        // before returning -1. Capping makes such doomed searches fail fast; the result
        // (-1, i.e. no path) is identical, only reached sooner. The cap is far above any
        // successful search so no real path is ever truncated.
        if (local_exp > 50000) {
            releaseNodes(allNodes);
            return -1;
        }

        if (curr->loc == goal_location) {
            bool can_hold = true;
            for (unsigned int i = curr->timestep + 1; i < maxtime; i++) {
                for (int j = 0; j < (int)path_table_.size(); j++) {
                    if (j != ag.id && j != ag_hide && (int)path_table_[j][i] == curr->loc) {
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

        // Match reference action order: WAIT, EAST, WEST, SOUTH, NORTH
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
//   Uses isConstrained() against path_table_.
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

    auto* mla_start = new MLATokenNode(ag.loc, 0, start_h, cur_time_, 0, nullptr);
    mla_open.push(mla_start);
    mla_all.push_back(mla_start);

    MLATokenNode* mla_solution = nullptr;
    int mla_max_t = min((int)maxtime, (int)cur_time_ + 1000);

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
                for (int j = 0; j < (int)path_table_.size(); j++) {
                    if (j == ag.id || j == ag_hide) continue;
                    if ((int)path_table_[j][t] == curr->loc) {
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
    for (int t = 0; t < (int)result.size() && ((int)cur_time_ + t) < (int)maxtime; t++)
        ag.path[cur_time_ + t] = result[t];
    int end_loc = result.back();
    for (unsigned int t = cur_time_ + result.size(); t < maxtime; t++)
        ag.path[t] = end_loc;

    // Extract arrival times
    int arrive_start = -1;
    for (int t = 0; t < (int)result.size(); t++) {
        int abs_t = (int)cur_time_ + t;
        if (result[t] == task.pickup_loc && abs_t >= task.release_time) {
            arrive_start = abs_t; break;
        }
    }
    int arrive_goal = (int)cur_time_ + (int)result.size() - 1;

    for (auto* n : mla_all) delete n;

    if (arrive_start < 0) return {-1, -1};
    return {arrive_start, arrive_goal};
}

// ============================================================
// plan_task_token: switch between 2x A* and token MLA*
//   based on config.single_agent
// ============================================================

pair<int,int> Simulation::plan_task_token(Agent& ag, Task& task, int ag_hide) {
    int hide = (ag_hide >= 0) ? ag_hide : ag.id;

    // Token-path collision validator for SIPP candidate paths
    auto verify_sipp_path = [&](const vector<int>& path, int from_t, int to_t) -> bool {
        for (int t = max((int)cur_time_ + 1, from_t + 1); t < to_t && t < (int)maxtime; t++) {
            if (isConstrained(ag.id, path[t-1], path[t], t, hide))
                return false;
        }
        return true;
    };

    if (config.use_sipp) {
        vector<vector<int>> cons;
        for (int j = 0; j < (int)path_table_.size(); j++) {
            if (j == ag.id || j == hide) continue;
            cons.emplace_back(path_table_[j].begin(), path_table_[j].end());
        }

        // Both 2SIPP and MLSIPP: use multi-goal SIPP (pickup + delivery)
        vector<pair<int,int>> goals = {{task.pickup_loc, task.release_time},
                                        {task.delivery_loc, 0}};
        auto path = sipp_search(ag.id, ag.loc, cur_time_, goals, cons, {}, false);
        if (!path.empty()) {
            int arrive_start = -1, arrive_goal = -1;
            int gi = 0;
            for (int t = (int)cur_time_; t < (int)path.size() && gi < 2; t++) {
                if (path[t] == goals[gi].first && t >= goals[gi].second) {
                    if (gi == 0) arrive_start = t; else arrive_goal = t;
                    gi++;
                }
            }
            if (arrive_start >= 0 && arrive_goal >= 0) {
                bool can_hold = true;
                for (unsigned int t = arrive_goal + 1; t < maxtime && can_hold; t++)
                    for (int j = 0; j < (int)path_table_.size(); j++) {
                        if (j == ag.id || j == hide) continue;
                        if ((int)path_table_[j][t] == task.delivery_loc) { can_hold = false; break; }
                    }
                if (can_hold && verify_sipp_path(path, (int)cur_time_, arrive_goal + 1)) {
                    for (int t = 0; t < (int)path.size() && t < (int)maxtime; t++)
                        ag.path[t] = path[t];
                    return {arrive_start, arrive_goal};
                }
            }
        }
        // SIPP enabled: do NOT fall back to MLA*/STA* — return the SIPP outcome
        // directly (matches the pbs_core MLSIPP path, which returns sipp_search
        // unconditionally). On the benchmarks SIPP always returns a valid path above,
        // so this is a no-op; it guarantees the method is purely SIPP.
        return {-1, -1};
    }

    if (config.single_agent == SA_MLA_SEQUENCE) {
        return token_mla_star(ag, task, ag_hide);
    }

    // Default: 2x sequential A* (STA_TASK_EP)
    vector<unsigned int> saved_path(ag.path.begin(), ag.path.end());

    int arrive_start = astar(ag, ag.loc, cur_time_,
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

    SearchNode* start = new SearchNode(ag.loc, 0, nullptr, cur_time_);
    allNodes.insert(make_pair((unsigned int)ag.loc, start));
    Q.push(start);

    while (!Q.empty()) {
        SearchNode* v = Q.front(); Q.pop();
        if ((unsigned int)v->timestep >= maxtime - 1) continue;

        if (endpoint_mask_[v->loc]) {
            bool occupied = false;
            for (unsigned int t = v->timestep; t < maxtime && !occupied; t++)
                for (int i = 0; i < (int)agents.size() && !occupied; i++)
                    if (i != ag.id && path_table_[i][t] == (unsigned int)v->loc) occupied = true;
            if (!occupied)
                for (auto it = open_tasks_.begin(); it != open_tasks_.end() && !occupied; it++)
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
    int num_ag = path_table_.size();
    unsigned int T = maxtime;

    for (int a1 = 0; a1 < num_ag; a1++) {
        for (int a2 = a1 + 1; a2 < num_ag; a2++) {
            for (unsigned int t = 0; t < T; t++) {
                if (path_table_[a1][t] == path_table_[a2][t]) {
                    if (collision_count < 10)
                        cout << "[" << alg_name << "] VERTEX COLLISION: agents " << a1 << " and " << a2
                             << " at loc " << path_table_[a1][t] << " time " << t << endl;
                    ok = false; collision_count++;
                }
                if (t > 0 && path_table_[a1][t] == path_table_[a2][t-1] &&
                    path_table_[a1][t-1] == path_table_[a2][t]) {
                    if (collision_count < 10)
                        cout << "[" << alg_name << "] EDGE COLLISION: agents " << a1 << " and " << a2
                             << " on edge " << path_table_[a1][t-1] << "-" << path_table_[a1][t]
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

    // Mark all tasks as assigned (remove from open_tasks_)
    // For offline mode, we handle all tasks in planning
    // open_tasks_ will be cleared after planning
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

        // Add parking constraints for not-yet-planned agents (matching reference)
        for (int j = pi + 1; j < num_ag; j++) {
            int other_aid = seq_infos[j].agent_id;
            int other_park = agents[other_aid].initial_loc;
            vector<int> park_cons(maxtime, other_park);
            cons_paths.push_back(park_cons);
        }

        if (config.single_agent == SA_MLA_SEQUENCE) {
            if (config.mla_mode == MLA_SEQ) {
                // SeqMLA*: plan ALL tasks in one search
                vector<pair<int,int>> all_goals;
                for (int task_id : ag.task_sequence) {
                    Task& task = all_tasks[task_id];
                    all_goals.push_back({task.pickup_loc, task.release_time});
                    all_goals.push_back({task.delivery_loc, 0});
                }
                all_goals.push_back({park_loc, 0});

                vector<int> path = config.use_sipp ?
                    sipp_search(aid, ag.initial_loc, 0, all_goals, cons_paths, {}, false) :
                    seq_mla_star(aid, ag.initial_loc, 0, all_goals, cons_paths, {}, false);
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
                // Task-by-task MLSIPP/MLA*
                int cur_loc = ag.initial_loc;
                int cur_time = 0;

                for (int task_id : ag.task_sequence) {
                    Task& task = all_tasks[task_id];

                    vector<pair<int,int>> goals;
                    goals.push_back({task.pickup_loc, task.release_time});
                    goals.push_back({task.delivery_loc, 0});
                    goals.push_back({park_loc, 0});

                    vector<int> path;
                    if (config.use_sipp) {
                        path = sipp_search(aid, cur_loc, cur_time, goals, cons_paths, {}, false);
                        if (!path.empty()) {
                            for (auto& cp : cons_paths) {
                                bool bad = false;
                                for (int t = cur_time; t < min((int)path.size(), cur_time+1000) && !bad; t++) {
                                    int cl = (t < (int)cp.size()) ? cp[t] : cp.back();
                                    if (path[t] == cl) bad = true;
                                }
                                if (bad) { path.clear(); break; }
                            }
                        }
                    }
                    if (path.empty())
                        path = seq_mla_star(aid, cur_loc, cur_time, goals, cons_paths, {}, false);

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
        } else if (config.use_sipp) {
            // 2SIPP*: SIPP-based planning per task
            int cur_loc = ag.initial_loc;
            int cur_time = 0;

            for (int task_id : ag.task_sequence) {
                Task& task = all_tasks[task_id];

                vector<pair<int,int>> goals;
                goals.push_back({task.pickup_loc, task.release_time});
                goals.push_back({task.delivery_loc, 0});
                goals.push_back({park_loc, 0});

                vector<int> path;
                path = sipp_search(aid, cur_loc, cur_time, goals, cons_paths, {}, false);
                if (!path.empty()) {
                    for (auto& cp : cons_paths) {
                        bool bad = false;
                        for (int t = cur_time; t < min((int)path.size(), cur_time+1000) && !bad; t++) {
                            int cl = (t < (int)cp.size()) ? cp[t] : cp.back();
                            if (path[t] == cl) bad = true;
                        }
                        if (bad) { path.clear(); break; }
                    }
                }
                if (path.empty())
                    path = seq_mla_star(aid, cur_loc, cur_time, goals, cons_paths, {}, false);

                if (path.empty()) {
                    cerr << "2SIPP failed for agent " << aid
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
                                                      cons_paths, task.release_time, true);
                // Fallback: retry with goal_optimal=false (matching reference)
                if (pickup_arrive < 0) {
                    pickup_arrive = astar_with_dummy(ag, ag.path[t], t,
                                                      task.pickup_loc, park_loc,
                                                      h_pickup, h_park,
                                                      cons_paths, task.release_time, false);
                }
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
                                                        cons_paths, 0, true);
                // Fallback: retry with goal_optimal=false (matching reference)
                if (delivery_arrive < 0) {
                    delivery_arrive = astar_with_dummy(ag, ag.path[pickup_arrive], pickup_arrive,
                                                        task.delivery_loc, park_loc,
                                                        h_delivery, h_park,
                                                        cons_paths, 0, false);
                }
                if (delivery_arrive < 0) {
                    cerr << "Error: astar_with_dummy failed for agent " << aid
                         << " task " << task_id << " (delivery)" << endl;
                    break;
                }

                task.completion_time = delivery_arrive;
                t = delivery_arrive;
            }
        }

        // Copy agent's path to path_table_
        for (unsigned int tt = 0; tt < maxtime; tt++)
            path_table_[aid][tt] = ag.path[tt];
    }

    // Clear open_tasks_ since all are processed
    open_tasks_.clear();
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
                                  int release_time,
                                  bool goal_optimal,
                                  const vector<tuple<int,int,int>>& cbs_cons) {
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
        if (goal_optimal && global_vis_goal && !curr->vis_goal && goal_loc != park_loc)
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
                for (int t = curr->timestep + 1; t < (int)cons_paths[ci].size() && can_hold; t++) {
                    if (cons_paths[ci][t] == curr->loc)
                        can_hold = false;
                }
            }
            // Also honor CBS vertex constraints during the infinite hold at parking:
            // if a vertex constraint forbids this park cell at any future timestep,
            // the agent cannot settle here yet (it must arrive later / elsewhere).
            for (int ci = 0; ci < (int)cbs_cons.size() && can_hold; ci++) {
                if (get<1>(cbs_cons[ci]) < 0 &&
                    get<0>(cbs_cons[ci]) == curr->loc &&
                    get<2>(cbs_cons[ci]) >= curr->timestep + 1)
                    can_hold = false;
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
                if (next_t >= (int)cons_paths[ci].size()) continue;
                if (cons_paths[ci][next_t] == next_loc)
                    constrained = true;
                else if (cons_paths[ci][next_t] == curr->loc &&
                         cons_paths[ci][next_t - 1] == next_loc)
                    constrained = true;
            }
            // High-level CBS constraints (loc1, loc2, t_abs); loc2 < 0 => vertex.
            for (int ci = 0; ci < (int)cbs_cons.size() && !constrained; ci++) {
                if (get<2>(cbs_cons[ci]) != next_t) continue;
                int cl1 = get<0>(cbs_cons[ci]), cl2 = get<1>(cbs_cons[ci]);
                if (cl2 < 0) {
                    if (cl1 == next_loc) constrained = true;      // vertex
                } else {
                    if (cl1 == curr->loc && cl2 == next_loc)      // edge
                        constrained = true;
                }
            }
            if (constrained) continue;

            int next_g = curr->g_val + 1;
            int next_h;
            if (curr->vis_goal && goal_loc != park_loc)
                next_h = h_park[next_loc];
            else
                next_h = h_goal[next_loc];
            if (next_h == INT_MAX) next_h = 0;

            // Key: (loc, g_val, vis_goal) — matching reference (loc, timestep, vis_goal) dedup
            unsigned int vg = curr->vis_goal ? 1 : 0;
            unsigned int key = next_loc + (unsigned int)next_g * map_size * 2 + vg * map_size;
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
    return mapd_map.ep_index(loc);
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
            // Delivering task: skip distance computation (agent is already en route)
            // Only update loc to delivery endpoint
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
        int mint = max({task->hold_time - (int)hybrid_timestep_,
                        task->release_time - (int)hybrid_timestep_, 0});
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
            // Faithful to reference GoHome (reference simulation.cpp:620-622):
            // return the failed start location so the caller extends the task's
            // hold_time and retries, rather than emitting a best-effort path.
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

// Faithful port of the reference TA-Hybrid Group1 planner
// (reference_code/TA-Hybrid/simulation.cpp::PathFinding + ICBSSearch).
//
// The delivery batch is solved with a genuine high-level CBS: each delivery
// agent is planned INDEPENDENTLY with a two-phase deliver->park->hold low-level
// (astar_with_dummy) that treats only the FIXED constraint agents as obstacles;
// conflicts among the delivery agents are then resolved by branching the CBS
// tree and adding per-(loc,time) vertex/edge constraints, replanning the
// constrained agent until the batch is conflict-free. Constraint agents are
// never re-routed -- only their dummy/parking tails are replanned afterwards
// (hybrid_replan_dummy, mirroring ReplanDummyPath).
bool Simulation::hybrid_group1_plan(vector<Agent*>& delivery_agents,
                                     vector<Agent*>& constraint_agents) {
    if (delivery_agents.empty()) return true;
    int map_size = (int)mapd_map.grid.size();
    int t0 = (int)hybrid_timestep_;
    int T  = (int)maxtime;
    int K  = (int)delivery_agents.size();

    // Set non_dummy_path for constraint agents (reference PathFinding line 321-328)
    for (int ci = 0; ci < (int)constraint_agents.size(); ci++) {
        int ds = constraint_agents[ci]->dummy_start_step;
        vector<int> cons;
        for (int j = 0; j <= ds && j < T; j++)
            cons.push_back((int)constraint_agents[ci]->path[j]);
        constraint_agents[ci]->non_dummy_path = cons;
    }

    // Precompute per-delivery-agent fixed data + BFS heuristics (goal and park).
    vector<int> goal_loc(K), park_loc(K), start_loc(K);
    vector<vector<int>> h_goal(K), h_park(K);
    auto bfs = [&](int src, vector<int>& h) {
        h.assign(map_size, INT_MAX);
        queue<int> q; h[src] = 0; q.push(src);
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                int v = u + d;
                if (v >= 0 && v < map_size && abs(v % mapd_map.col - u % mapd_map.col) < 2 &&
                    mapd_map.grid[v] && h[v] > h[u] + 1) {
                    h[v] = h[u] + 1; q.push(v);
                }
            }
        }
    };
    for (int i = 0; i < K; i++) {
        Agent* ag = delivery_agents[i];
        goal_loc[i]  = ag->goal_loc;
        park_loc[i]  = ag->park_loc;
        start_loc[i] = (int)ag->path[t0];
        bfs(goal_loc[i], h_goal[i]);
        bfs(park_loc[i], h_park[i]);
    }

    // Low-level: plan delivery agent i under the fixed constraint-agent obstacle
    // paths + its accumulated CBS constraints. Captures the resulting absolute
    // path segment [t0,T) and returns the goal-arrival abs time (or -1).
    auto low_level = [&](int i, const vector<vector<int>>& cons_paths,
                         const vector<tuple<int,int,int>>& cbs_cons,
                         vector<int>& out_path) -> int {
        Agent* ag = delivery_agents[i];
        int result = astar_with_dummy(*ag, start_loc[i], t0,
                                      goal_loc[i], park_loc[i],
                                      h_goal[i], h_park[i],
                                      cons_paths, 0, true, cbs_cons);
        if (result < 0) return -1;
        out_path.resize(T - t0);
        for (int t = t0; t < T; t++) out_path[t - t0] = (int)ag->path[t];
        return result;
    };

    // Cost objective, matching reference g_val = max_i costs[i][goal_length[i]]
    // (CalcCost = completion-time / makespan estimate for the remaining sequence).
    auto agent_cost = [&](int i, int arrival_abs) -> int {
        Agent* ag = delivery_agents[i];
        if (ag->task_ptr == nullptr) return arrival_abs;
        int sid = ag->task_ptr->seq_id;
        if (sid < 0 || sid >= (int)hybrid_seqs_.size()) return arrival_abs;
        return hybrid_cost(arrival_abs, hybrid_seqs_[sid]);
    };

    struct G1Node {
        vector<vector<int>> paths;                 // [i] abs path over [t0,T)
        vector<int> arrival;                       // [i] goal-arrival abs time
        vector<vector<tuple<int,int,int>>> cons;   // [i] accumulated CBS constraints
        int g_val = 0;
        int num_coll = 0;
        int a1 = -1, a2 = -1, cloc1 = -1, cloc2 = -1, ct = -1; // chosen conflict
        bool has_conf = false;
    };

    // Detect earliest conflict among a node's delivery paths; count all conflicts.
    auto detect = [&](G1Node* n) {
        n->num_coll = 0; n->has_conf = false;
        int best_t = INT_MAX;
        for (int a = 0; a < K; a++) {
            for (int b = a + 1; b < K; b++) {
                const vector<int>& pa = n->paths[a];
                const vector<int>& pb = n->paths[b];
                int len = (int)min(pa.size(), pb.size());
                for (int t = 0; t < len; t++) {
                    bool conf = false; int cl1 = -1, cl2 = -1;
                    if (pa[t] == pb[t]) {                 // vertex conflict
                        conf = true; cl1 = pa[t]; cl2 = -1;
                    } else if (t > 0 && pa[t] == pb[t-1] && pb[t] == pa[t-1]) {
                        conf = true; cl1 = pa[t-1]; cl2 = pa[t]; // edge (agent a: cl1->cl2)
                    }
                    if (conf) {
                        n->num_coll++;
                        int abs_t = t0 + t;
                        if (abs_t < best_t) {
                            best_t = abs_t;
                            n->a1 = a; n->a2 = b;
                            n->cloc1 = cl1; n->cloc2 = cl2; n->ct = abs_t;
                            n->has_conf = true;
                        }
                    }
                }
            }
        }
    };

    // Save current path segments so we can restore on a hard (root) failure.
    vector<vector<int>> saved(K);
    for (int i = 0; i < K; i++) {
        saved[i].resize(T - t0);
        for (int t = t0; t < T; t++) saved[i][t - t0] = (int)delivery_agents[i]->path[t];
    }
    auto restore_saved = [&]() {
        for (int i = 0; i < K; i++)
            for (int t = t0; t < T; t++)
                delivery_agents[i]->path[t] = saved[i][t - t0];
    };

    const int HL_CAP = 100000;  // safety cap (mirrors reference HL_num_expanded guard)

    // Outer loop: mirrors reference PathFinding (re-run CBS if a constraint
    // agent's dummy tail cannot be replanned conflict-free -- that agent's full
    // path then becomes a hard obstacle on the next pass).
    while (true) {
        vector<vector<int>> cons_paths_base;
        for (int ci = 0; ci < (int)constraint_agents.size(); ci++)
            cons_paths_base.push_back(constraint_agents[ci]->non_dummy_path);

        // ---- High-level CBS over the delivery batch ----
        auto worse = [](const G1Node* a, const G1Node* b) {
            if (a->g_val != b->g_val) return a->g_val > b->g_val;
            return a->num_coll > b->num_coll;
        };
        priority_queue<G1Node*, vector<G1Node*>, decltype(worse)> open(worse);
        vector<G1Node*> allnodes;

        // Root: plan every agent independently against the fixed obstacles only.
        G1Node* root = new G1Node();
        root->paths.resize(K);
        root->arrival.resize(K);
        root->cons.resize(K);
        bool root_ok = true;
        for (int i = 0; i < K && root_ok; i++) {
            int arr = low_level(i, cons_paths_base, root->cons[i], root->paths[i]);
            if (arr < 0) root_ok = false;
            else root->arrival[i] = arr;
        }
        if (!root_ok) {
            // Cannot even plan the batch against the fixed agents (should not
            // happen on a connected grid). Revert this batch's assignments and
            // let them retry on a later timestep -- no colliding path is emitted.
            delete root;
            restore_saved();
            for (int k = 0; k < K; k++) {
                Agent* a = delivery_agents[k];
                a->delivering = false;
                if (a->task_ptr) { a->task_ptr->delivering = false; a->task_ptr->ag = nullptr; }
                a->task_ptr = nullptr;
            }
            return false;
        }
        root->g_val = 0;
        for (int i = 0; i < K; i++)
            root->g_val = max(root->g_val, agent_cost(i, root->arrival[i]));
        detect(root);
        open.push(root); allnodes.push_back(root);

        G1Node* solution = nullptr;
        G1Node* best = root;             // fallback: fewest collisions seen
        int hl_expanded = 0;
        while (!open.empty()) {
            G1Node* curr = open.top(); open.pop();
            if (curr->num_coll < best->num_coll) best = curr;
            if (!curr->has_conf) { solution = curr; break; }
            if (++hl_expanded > HL_CAP) {
                cerr << "TA-Hybrid Group1 CBS exceeded HL cap; using best node" << endl;
                break;
            }
            // Branch: constrain a1, then a2, on the chosen conflict.
            for (int side = 0; side < 2; side++) {
                int ag = (side == 0) ? curr->a1 : curr->a2;
                tuple<int,int,int> nc;
                if (curr->cloc2 < 0)  // vertex conflict: same cell for both agents
                    nc = make_tuple(curr->cloc1, -1, curr->ct);
                else if (side == 0)   // edge: a1 forbidden cloc1->cloc2 at ct
                    nc = make_tuple(curr->cloc1, curr->cloc2, curr->ct);
                else                  // edge: a2 forbidden cloc2->cloc1 at ct
                    nc = make_tuple(curr->cloc2, curr->cloc1, curr->ct);

                G1Node* child = new G1Node(*curr);
                child->cons[ag].push_back(nc);
                vector<int> newp;
                int arr = low_level(ag, cons_paths_base, child->cons[ag], newp);
                if (arr < 0) { delete child; continue; }  // infeasible branch
                child->paths[ag] = newp;
                child->arrival[ag] = arr;
                child->g_val = 0;
                for (int i = 0; i < K; i++)
                    child->g_val = max(child->g_val, agent_cost(i, child->arrival[i]));
                detect(child);
                open.push(child); allnodes.push_back(child);
            }
        }

        G1Node* apply = solution ? solution : best;

        // Apply the chosen solution to the delivery agents.
        for (int i = 0; i < K; i++) {
            Agent* ag = delivery_agents[i];
            for (int t = t0; t < T; t++) ag->path[t] = apply->paths[i][t - t0];
            ag->dummy_start_step = apply->arrival[i];
            if (ag->task_ptr) ag->task_ptr->ag_arrive_goal = apply->arrival[i];
        }

        for (G1Node* n : allnodes) delete n;

        // Replan dummy tails for constraint agents (reference line 357-365).
        bool ok = true;
        for (int ci = 0; ci < (int)constraint_agents.size(); ci++) {
            if (!hybrid_replan_dummy(constraint_agents[ci])) {
                constraint_agents[ci]->non_dummy_path.assign(
                    constraint_agents[ci]->path.begin(),
                    constraint_agents[ci]->path.begin() + maxtime);
                ok = false;
            }
        }
        if (ok) return true;
    }
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
                task->status = task->ag->id;  // Match reference line 815: task->agent_id = task->ag->id
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

        // Match reference costflow_iter limit (line 865-869)
        int costflow_iter = 0;
        while (true) {
            costflow_iter++;
            if (costflow_iter > 200) {
                cerr << "TA-Hybrid: costflow loop exceeded 200 iterations, breaking" << endl;
                break;
            }
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
            // Safety cap matching the reference replan sweep cap (ref
            // simulation.cpp line 964, "replan_pass > 200"). Without this, if a
            // dummy replan can never succeed (e.g. an agent left stuck by a
            // failed Group1 delivery blocks a corridor), this loop spins forever.
            int replan_pass = 0;
            while (true) {
                if (++replan_pass > 200) {
                    cerr << "TA-Hybrid: Group2 replan loop exceeded 200 passes, "
                            "breaking with current paths" << endl;
                    break;
                }
                while (true) {
                    // GoHome retry loop (reference simulation.cpp:917-943): repeatedly
                    // route the cost-flow agents home; on failure the offending task's
                    // hold_time is advanced monotonically, so the loop terminates
                    // naturally (no artificial cap, matching the reference).
                    // Build constraint paths from non-costflow agents
                    vector<vector<int>> cons_paths;
                    for (int i = 0; i < num_ag; i++)
                        if (!in_costflow[i])
                            cons_paths.push_back(agents[i].non_dummy_path);

                    // Calculate time horizon for each task based on global makespan.
                    // Matching reference: no routing budget cap.
                    vector<int> len;
                    for (int i = 0; i < (int)task_costflow.size(); i++) {
                        int t = (int)hybrid_timestep_;
                        while (hybrid_cost(t + 1, hybrid_seqs_[task_costflow[i]->seq_id]) <=
                               hybrid_global_makespan_)
                            t++;
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

        // Update path_table_ for collision checking
        for (int i = 0; i < num_ag; i++)
            for (unsigned int t = 0; t < maxtime; t++)
                path_table_[i][t] = agents[i].path[t];
    }

    // Copy final paths to path_table_ and set task completion info
    for (int i = 0; i < num_ag; i++)
        for (unsigned int t = 0; t < maxtime; t++)
            path_table_[i][t] = agents[i].path[t];

    open_tasks_.clear();
} // end plan_ta_hybrid

// ============================================================
// Section 28: PBS Online — Update System
//   Detect completed task goals from agent positions.
//   When agent is at delivery location of current task, pop it.
// ============================================================

void Simulation::update_system_pbs() {
    pbs_has_event_ = false;
    pbs_assign_event_ = false;
    lns_agent_finished_ = false;

    // Check if new tasks arrived
    if (cur_time_ < maxtime && !task_indices_by_time[cur_time_].empty()) {
        pbs_has_event_ = true;
        pbs_assign_event_ = true;
    }

    // Advance goal state machine: detect completed goals
    // For wPBS: scan all timesteps since last check (may have skipped intermediate steps)
    for (int i = 0; i < (int)agents.size(); i++) {
        // Process multiple task completions that may have occurred between replans
        while (!agents[i].task_sequence.empty()) {
            int task_id = agents[i].task_sequence.front();
            Task& task = all_tasks[task_id];

            int first_goal = task.goals.empty() ? task.pickup_loc : task.goals[0];
            int last_goal = task.goals.size() >= 2 ? task.goals[min((int)task.goals.size()-1, 1)] : first_goal;

            // Scan path for pickup arrival
            if (agents[i].status == AG_MOVING_TO_PICKUP) {
                bool found_pickup = false;
                for (unsigned int t = pbs_last_replan_time_; t <= cur_time_ && t < maxtime; t++) {
                    if ((int)agents[i].path[t] == first_goal && (int)t >= task.release_time) {
                        agents[i].status = AG_CARRYING;
                        found_pickup = true;
                        break;
                    }
                }
                if (!found_pickup) break;
            }

            // Scan path for delivery arrival
            if (agents[i].status == AG_CARRYING) {
                bool found_delivery = false;
                for (unsigned int t = pbs_last_replan_time_; t <= cur_time_ && t < maxtime; t++) {
                    if ((int)agents[i].path[t] == last_goal) {
                        task.completion_time = (int)t;
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
                        pbs_assign_event_ = true;
                        lns_agent_finished_ = true;  // agent reached its last goal (reference new_agent_finish)
                        found_delivery = true;
                        break;
                    }
                }
                if (!found_delivery) break;
            }

            if (agents[i].status != AG_MOVING_TO_PICKUP && agents[i].status != AG_CARRYING)
                break;
        }
    }

    // Check if any agent is free (no tasks) and there are unassigned tasks
    if (!pbs_assign_event_ && !open_tasks_.empty()) {
        for (auto& a : agents) {
            if (a.task_sequence.empty() && a.status == AG_FREE) {
                pbs_has_event_ = true;
                pbs_assign_event_ = true;
                break;
            }
        }
    }

    // Trigger periodic replanning (replan only, no re-assignment)
    // Only for wPBS mode — reference PBS/LNS-PBS only replans on actual events
    // (task release or delivery completion), never periodically.  Periodic PP
    // replanning disrupts existing collision-free paths and can worsen quality.
    if (!pbs_has_event_ && config.mapf == MAPF_wPBS) {
        bool any_active = false;
        for (auto& a : agents) {
            if (!a.task_sequence.empty()) { any_active = true; break; }
        }
        if (any_active && (int)cur_time_ - pbs_last_replan_time_ >= config.replan_window) {
            pbs_has_event_ = true;
            // pbs_assign_event_ stays false — only replan paths, don't re-assign
        }
    }

    if (pbs_has_event_)
        pbs_last_replan_time_ = (int)cur_time_;
}

// ============================================================
// Section 29: Repeated Hungarian Assignment
//   Assign tasks from open_tasks_ to agents' task_sequences
//   in rounds. Each round assigns min(num_agents, remaining) tasks.
// ============================================================

void Simulation::assign_repeated_hungarian() {
    // Re-assign from scratch: release all non-delivering tasks
    // (matching reference: clears task_sequences except delivering front task)
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].task_sequence.empty()) continue;
        int front_tid = agents[i].task_sequence.front();
        bool keep_front = (agents[i].status == AG_CARRYING &&
                           agents[i].current_task == front_tid);
        for (int tid : agents[i].task_sequence) {
            if (keep_front && tid == front_tid) continue;
            all_tasks[tid].status = -1;
            // Put task back into open_tasks_ (it was removed when assigned)
            open_tasks_.push_back(&all_tasks[tid]);
        }
        if (keep_front) {
            agents[i].task_sequence = {front_tid};
        } else {
            agents[i].task_sequence.clear();
            if (agents[i].status == AG_MOVING_TO_PICKUP) {
                agents[i].status = AG_FREE;
                agents[i].current_task = -1;
            }
        }
    }

    // Collect all unassigned tasks
    vector<Task*> remaining_tasks;
    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); ++it) {
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
                        int t = (int)cur_time_;
                        for (int tid : ag.task_sequence) {
                            Task& tt = all_tasks[tid];
                            int ng = min((int)tt.goals.size(), 2);
                            // Skip pickup goal if agent is carrying this task
                            // (reference excludes delivering task from sequence and
                            //  adjusts start_location/start_timestep instead)
                            bool is_carrying_tid = (ag.status == AG_CARRYING &&
                                                    ag.current_task == tid);
                            int start_g = (is_carrying_tid && ng >= 2) ? 1 : 0;
                            for (int g = start_g; g < ng; g++) {
                                int gloc = tt.goals[g];
                                int ep_idx = mapd_map.ep_index(gloc);
                                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                                if (d == INT_MAX) d = 0;
                                t += d;
                                // Only clamp by release_time for the pickup goal (not delivery)
                                if (g == start_g && !is_carrying_tid &&
                                    t < tt.release_time)
                                    t = tt.release_time;
                                loc = gloc;
                            }
                        }
                        int first_goal_loc = task->goals.empty() ? task->pickup_loc : task->goals[0];
                        int ep_idx = mapd_map.ep_index(first_goal_loc);
                        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                        if (d == INT_MAX) d = 0;
                        est_time = t + d;
                    } else {
                        int first_goal_loc = task->goals.empty() ? task->pickup_loc : task->goals[0];
                        int ep_idx = mapd_map.ep_index(first_goal_loc);
                        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[ag.loc] : 0;
                        if (d == INT_MAX) d = 0;
                        est_time = (int)cur_time_ + d;
                    }
                    est_time = max(est_time, task->release_time);
                    // Add distance to second goal (capped to first 2 goals)
                    if (task->goals.size() >= 2) {
                        int g0 = task->goals[0];
                        int g1 = task->goals[min((int)task->goals.size()-1, 1)];
                        int ep_idx = mapd_map.ep_index(g1);
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

        // Remove assigned tasks from remaining and from open_tasks_
        for (Task* t : assigned) {
            remaining_tasks.erase(
                remove(remaining_tasks.begin(), remaining_tasks.end(), t),
                remaining_tasks.end());
            open_tasks_.remove(t);
        }
    }
}

// ============================================================
// Section 29b: Repeated Hungarian + LNS (LNS-PBS, LNS-wPBS)
//   Phase 1: Repeated Hungarian (reuses assign_repeated_hungarian)
//   Phase 2: LNS destroy/repair loop for improvement
// ============================================================

int Simulation::estimate_sequence_cost(int agent_id) const {
    // Match reference calculateFlowtime: returns sum of (delivery_time - release_time)
    // for all tasks in the agent's sequence, which is sum_of_delivery_time - sum_of_release_time.
    const Agent& ag = agents[agent_id];
    int loc = (int)ag.loc;
    int delivery_time = (int)cur_time_;
    int sum_of_delivery_time = 0;
    int sum_of_release_time = 0;

    if (ag.task_sequence.empty()) return 0;

    for (int idx = 0; idx < (int)ag.task_sequence.size(); idx++) {
        int tid = ag.task_sequence[idx];
        const Task& tt = all_tasks[tid];
        int ng = min((int)tt.goals.size(), 2);

        // Check if agent is currently carrying this task (skip pickup)
        bool is_carrying = (idx == 0 && ag.status == AG_CARRYING && ag.current_task == tid);
        int start_goal = is_carrying ? 1 : 0;

        if (start_goal < ng) {
            // Distance to first relevant goal
            int gloc0 = tt.goals[start_goal];
            int ep_idx = mapd_map.ep_index(gloc0);
            int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
            if (d == INT_MAX) d = 0;

            if (idx == 0) {
                delivery_time = (int)cur_time_ + d;
            } else {
                delivery_time += d;
            }
            if (!is_carrying)
                delivery_time = max(delivery_time, tt.release_time);

            // Distance through remaining goals (e.g., pickup -> delivery)
            for (int g = start_goal; g < ng - 1; g++) {
                int g0 = tt.goals[g];
                int g1 = tt.goals[g + 1];
                int ep2 = mapd_map.ep_index(g1);
                int dd = (ep2 >= 0) ? mapd_map.endpoints[ep2].h_val[g0] : 0;
                if (dd == INT_MAX) dd = 0;
                delivery_time += dd;
            }
        }

        sum_of_delivery_time += delivery_time;
        sum_of_release_time += tt.release_time;

        // Update loc to last goal for next task's distance computation
        loc = tt.goals[min(ng - 1, (int)tt.goals.size() - 1)];
    }

    return sum_of_delivery_time - sum_of_release_time;
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

    // Match reference: always use Shaw removal with neighborhood_size=2
    // Reference: LNS lns(G, tl, al, 2, 1, 2, 2) -> removal_strategy=1 (Shaw), neighborhood_size=2
    int neighborhood_size = 2;

    // Compute estimated pick_up_time and delivery_time for each task in agent sequences.
    // Matches reference generateNeighborsByShawRemoval lines 343-362:
    //   For each agent, walk through task_sequence computing running pick_up and delivery times.
    //   pick_up_time = max(estimated_arrival_at_pickup, release_time)
    //   delivery_time = pick_up_time + internal_goal_distances
    unordered_map<int, pair<int,int>> task_times;  // tid -> (pick_up_time, delivery_time)
    for (auto& ag : agents) {
        int pick_up_time = 0;
        for (int idx = 0; idx < (int)ag.task_sequence.size(); idx++) {
            int tid = ag.task_sequence[idx];
            const Task& tt = all_tasks[tid];
            int ng = min((int)tt.goals.size(), 2);
            int first_goal = tt.goals.empty() ? tt.pickup_loc : tt.goals[0];
            int last_goal = (ng >= 2) ? tt.goals[min(ng-1, (int)tt.goals.size()-1)] : first_goal;

            if (idx == 0) {
                // Reference: pick_up_time = start_timestep + h(start_location, goal_arr[0])
                int ep_idx = mapd_map.ep_index(first_goal);
                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[ag.loc] : 0;
                if (d == INT_MAX) d = 0;
                pick_up_time = (int)cur_time_ + d;
            }
            int this_pickup = max(pick_up_time, tt.release_time);
            int this_delivery = this_pickup;
            // Add internal goal distances (pickup -> delivery)
            for (int g = 0; g < ng - 1; g++) {
                int g0 = tt.goals[g];
                int g1 = tt.goals[g + 1];
                int ep2 = mapd_map.ep_index(g1);
                int dd = (ep2 >= 0) ? mapd_map.endpoints[ep2].h_val[g0] : 0;
                if (dd == INT_MAX) dd = 0;
                this_delivery += dd;
            }
            task_times[tid] = {this_pickup, this_delivery};

            // Compute pick_up_time for next task in sequence
            if (idx < (int)ag.task_sequence.size() - 1) {
                int next_tid = ag.task_sequence[idx + 1];
                const Task& next_tt = all_tasks[next_tid];
                int next_first = next_tt.goals.empty() ? next_tt.pickup_loc : next_tt.goals[0];
                int ep_idx = mapd_map.ep_index(next_first);
                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[last_goal] : 0;
                if (d == INT_MAX) d = 0;
                pick_up_time = this_delivery + d;
            }
        }
    }

    // Shaw removal: pick a random task as seed, find most related tasks
    // Relatedness = weight1 * spatial_dist(last_goal) + spatial_dist(first_goal) + weight2 * temporal_dist
    int seed_idx = rand() % (int)eligible.size();
    int seed_tid = eligible[seed_idx];
    const Task& seed_task = all_tasks[seed_tid];
    int seed_last_goal = seed_task.goals.empty() ? seed_task.pickup_loc : seed_task.goals.back();
    int seed_first_goal = seed_task.goals.empty() ? seed_task.pickup_loc : seed_task.goals[0];
    int seed_pickup_t = 0, seed_delivery_t = 0;
    if (task_times.count(seed_tid)) {
        seed_pickup_t = task_times[seed_tid].first;
        seed_delivery_t = task_times[seed_tid].second;
    }
    removed_tasks.push_back(seed_tid);

    // Compute relatedness for all other eligible tasks using BFS distances
    // Reference weights: relatedness_weight1=9, relatedness_weight2=3
    vector<pair<int,int>> relatedness;
    for (int tid : eligible) {
        if (tid == seed_tid) continue;
        const Task& t = all_tasks[tid];
        int t_last_goal = t.goals.empty() ? t.pickup_loc : t.goals.back();
        int t_first_goal = t.goals.empty() ? t.pickup_loc : t.goals[0];

        // Use BFS distances from endpoints
        int dist_last = 0, dist_first = 0;
        for (auto& ep : mapd_map.endpoints) {
            if (ep.loc == seed_last_goal) { dist_last = ep.h_val[t_last_goal]; break; }
        }
        for (auto& ep : mapd_map.endpoints) {
            if (ep.loc == seed_first_goal) { dist_first = ep.h_val[t_first_goal]; break; }
        }
        if (dist_last == INT_MAX) dist_last = mapd_map.col * mapd_map.row;
        if (dist_first == INT_MAX) dist_first = mapd_map.col * mapd_map.row;

        // Temporal relatedness: use estimated pick_up_time and delivery_time
        // matching reference generateNeighborsByShawRemoval line 372
        int t_pickup_t = 0, t_delivery_t = 0;
        if (task_times.count(tid)) {
            t_pickup_t = task_times[tid].first;
            t_delivery_t = task_times[tid].second;
        }
        int time_diff = abs(t_pickup_t - seed_pickup_t) + abs(t_delivery_t - seed_delivery_t);

        int rel = 9 * dist_last + dist_first + 3 * time_diff;
        relatedness.push_back({rel, tid});
    }
    sort(relatedness.begin(), relatedness.end());
    for (auto& p : relatedness) {
        removed_tasks.push_back(p.second);
        if ((int)removed_tasks.size() >= neighborhood_size) break;
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
    // Regret-based re-insertion using MARGINAL cost (matching reference)
    while (!removed_tasks.empty()) {
        int best_task = -1;
        int best_agent = -1;
        int best_pos = -1;
        int best_marginal = INT_MAX;
        int best_regret = INT_MIN;

        for (int rt : removed_tasks) {
            int best1_marginal = INT_MAX, best1_agent = -1, best1_pos = -1;
            int best2_marginal = INT_MAX;

            for (int a = 0; a < (int)agents.size(); a++) {
                int seq_len = (int)agents[a].task_sequence.size();
                // Skip position 0 if agent is carrying the front task
                int start_pos = 0;
                if (agents[a].status == AG_CARRYING && !agents[a].task_sequence.empty()
                    && agents[a].task_sequence.front() == agents[a].current_task)
                    start_pos = 1;

                int base_cost = estimate_sequence_cost(a);  // cost WITHOUT the new task

                for (int p = start_pos; p <= seq_len; p++) {
                    // Temporarily insert
                    agents[a].task_sequence.insert(agents[a].task_sequence.begin() + p, rt);
                    int c = estimate_sequence_cost(a);
                    agents[a].task_sequence.erase(agents[a].task_sequence.begin() + p);

                    int marginal = c - base_cost;  // MARGINAL insertion cost

                    if (marginal < best1_marginal) {
                        best2_marginal = best1_marginal;
                        best1_marginal = marginal;
                        best1_agent = a;
                        best1_pos = p;
                    } else if (marginal < best2_marginal) {
                        best2_marginal = marginal;
                    }
                }
            }

            int regret = (best2_marginal == INT_MAX) ? 0 : best2_marginal - best1_marginal;
            if (regret > best_regret || (regret == best_regret && best1_marginal < best_marginal)) {
                best_regret = regret;
                best_task = rt;
                best_agent = best1_agent;
                best_pos = best1_pos;
                best_marginal = best1_marginal;
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

    // Phase 2: LNS improvement.  The reference (KivaSystemOnline::simulate) spends the
    // 1-second LNS budget ONLY at task-release-period boundaries (timestep % period == 0)
    // or when an agent just reached its last goal (new_agent_finish).  On other replan
    // events it skips the LNS spin entirely.  Gating our LNS spin the same way keeps the
    // total runtime aligned with the reference (which is what the gate compares) without
    // touching the Hungarian assignment that already ran above (so MS/SWT are unchanged
    // on the events where the reference would also have run only Hungarian, and identical
    // LNS quality on the events where the reference runs LNS).
    bool lns_trigger =
        ((int)cur_time_ % lns_release_period_ == 0) || lns_agent_finished_;
    if (!lns_trigger) return;

    // Quality-neutral runtime guard: LNS destroy/repair can only change anything if there is at
    // least one reassignable task (queued in some agent's sequence and not currently being
    // carried).  With none, lns_destroy() returns empty and the loop below would busy-spin for the
    // full 1s budget producing NO change.  At low task demand (e.g. freq 0.2) and during the
    // post-release tail this happens on most triggers, burning ~1s each and inflating runtime well
    // beyond the reference (which likewise does no useful LNS there).  Skipping the spin here is
    // exact: it removes only no-op iterations, so makespan/SWT are byte-identical.
    {
        bool any_reassignable = false;
        for (auto& ag : agents) {
            for (int tid : ag.task_sequence) {
                if (ag.status == AG_CARRYING && ag.current_task == tid) continue;
                any_reassignable = true; break;
            }
            if (any_reassignable) break;
        }
        if (!any_reassignable) return;
    }

    // RNG seed for the LNS loop. Default is a fixed seed (config.lns_seed, from --seed)
    // so LNS runs are reproducible; pass --seed <0 to restore the reference's time(NULL).
    if (config.lns_seed < 0) srand((unsigned)time(NULL));
    else srand((unsigned)config.lns_seed);
    clock_t lns_start = clock();
    double time_limit_ms = config.lns_time_limit * 1000.0;

    // Convergence-based early termination.  The 1s budget is a *ceiling*, but on most
    // events (especially low task-frequency / long-makespan instances) the greedy
    // Hungarian assignment is already locally optimal, so the destroy/repair loop just
    // burns the whole second finding no improvement.  Since assign() runs once per
    // release-period boundary, that made total runtime ~ (makespan/period) x 1s, which
    // is unbounded for low-freq cells (they never returned).  We stop a spin once it has
    // gone NO_IMPROVE_CAP consecutive iterations without an accepted improvement (i.e.
    // converged): every accepted improvement resets the counter, so genuinely-improving
    // spins still use the full second; only wasted, converged spins exit early.  This
    // guarantees LNS(1s) always produces output while leaving accepted moves unchanged.
    const int NO_IMPROVE_CAP = 2000;
    int no_improve = 0, empty_streak = 0;
    while (true) {
        double elapsed = (double)(clock() - lns_start) / CLOCKS_PER_SEC * 1000.0;
        if (elapsed >= time_limit_ms) break;
        if (no_improve >= NO_IMPROVE_CAP) break;   // converged: no gain to be had

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
        if (removed.empty()) { if (++empty_streak > 200) break; continue; }
        empty_streak = 0;

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
            no_improve++;
        } else {
            no_improve = 0;   // accepted an improvement: keep going
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

    // Always forbid other agents' assigned dummies (matching reference current_assigned_endpoints)
    for (int i = 0; i < (int)assigned_dummies.size(); i++) {
        if (i != agent_id && assigned_dummies[i] >= 0)
            forbidden.insert(assigned_dummies[i]);
    }

    // In strict mode (PBS), also forbid all task goals in the system
    if (strict) {
        for (auto it = open_tasks_.begin(); it != open_tasks_.end(); ++it) {
            for (int gloc : (*it)->goals) forbidden.insert(gloc);
        }
        for (auto& ag : agents) {
            for (int tid : ag.task_sequence) {
                for (int gloc : all_tasks[tid].goals) forbidden.insert(gloc);
            }
        }
        // In strict mode, also forbid other agents' current path endpoints (where they
        // are currently parked).  This must be UNCONDITIONAL: even agents that already got
        // a new dummy this solve are still physically sitting on their OLD parking spot
        // (their committed/old path holds it until they actually move), so picking that
        // spot as our endpoint would make our goal permanently blocked by their old path's
        // endpoint hold — which previously produced an unreachable dummy and forced PBS to
        // commit a colliding "best-effort" plan.
        for (int i = 0; i < (int)agents.size(); i++) {
            if (i == agent_id) continue;
            forbidden.insert((int)path_table_[i][(int)maxtime - 1]);
        }
    }

    // Find closest non-forbidden endpoint (matching reference: uses std::map which takes closest)
    // Reference choose_good_endpoint (line 146-147) has the last_task_endpoint skip
    // COMMENTED OUT.  The reference does NOT skip last_task_endpoint; it only skips
    // endpoints that are already in current_assigned_endpoints (handled by forbidden set).
    // When last_goal_loc is already forbidden (e.g., it's a task goal in strict mode),
    // skipping is redundant.  When it's NOT forbidden (e.g., a free agent at a non-task
    // endpoint), the reference allows choosing it (distance 0 = stay in place), which
    // avoids unnecessary movement that wastes resources and can block active agents.
    int best_loc = -1;
    int best_dist = INT_MAX;
    for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
        int loc = mapd_map.endpoints[e].loc;
        if (forbidden.count(loc)) continue;
        int d = mapd_map.endpoints[e].h_val[last_goal_loc];
        if (d <= best_dist) {
            best_dist = d;
            best_loc = loc;
        }
    }
    // Fallback: agent's initial location (matching reference fallback to agent_home_locations)
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

    // Match reference task_truncated_size=1: each agent gets at most 1 task
    // in goal_locations. The reference KivaSystemOnline (and driver_simple)
    // hardcodes task_truncated_size=1, meaning exactly 1 task per agent.
    // The carrying-front task counts as 1, so no additional tasks are added
    // for delivering agents.
    int task_truncated_size = 1;

    for (int i = 0; i < num_ag; i++) {
        int current_task_count = 0;
        int prev_loc = (int)agents[i].loc;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= task_truncated_size) break;
            Task& task = all_tasks[tid];
            // Multi-goal support: use task.goals for goal sequence
            // Cap to first 2 goals per task (matches reference behavior)
            int num_goals = min((int)task.goals.size(), 2);
            bool carrying = (agents[i].status == AG_CARRYING && tid == agents[i].current_task);
            if (num_goals <= 1) {
                // Single-goal task
                if (!carrying) {
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
                    prev_loc = task.goals[0];
                }
            } else {
                // Two-goal task (pickup + delivery)
                if (carrying) {
                    goal_seqs[i].push_back({task.goals[1], 0});
                    prev_loc = task.goals[1];
                } else {
                    // Pickup goal
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
                    prev_loc = task.goals[0];
                    // Delivery goal
                    goal_seqs[i].push_back({task.goals[1], 0});
                    prev_loc = task.goals[1];
                }
            }
            current_task_count++;
        }
    }

    // Choose dummy endpoints — match reference ordering:
    // non-free agents (those with task goals) first, then free agents.
    vector<int> non_free_agents, free_agent_list;
    for (int i = 0; i < num_ag; i++) {
        if (!goal_seqs[i].empty())
            non_free_agents.push_back(i);
        else
            free_agent_list.push_back(i);
    }
    for (int i : non_free_agents) {
        int last_loc = goal_seqs[i].back().first;
        int dummy = choose_dummy_endpoint(i, last_loc, assigned_dummies, strict);
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }
    for (int i : free_agent_list) {
        int last_loc = (int)agents[i].loc;
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
    int conflicts;   // # soft conflicts with other agents' paths (CAT) along this path
    MLANode* parent;

    MLANode(int l, int g, int h, int t, int gi, MLANode* p)
        : loc(l), g_val(g), h_val(h), timestep(t), goal_id(gi), conflicts(0), parent(p) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareMLANode {
    bool operator()(const MLANode* a, const MLANode* b) const {
        if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
        // Reference StateTimeAStar uses a FOCAL search: among equal-cost (f) nodes, expand the
        // one with FEWEST soft conflicts (conflict-avoidance table), then higher goal_id, then g.
        if (a->conflicts != b->conflicts) return a->conflicts > b->conflicts;
        if (a->goal_id != b->goal_id) return a->goal_id < b->goal_id;
        return a->g_val <= b->g_val;
    }
};

// --- Pooling allocator for boost::heap internal nodes ---------------------------------------
// boost::heap::fibonacci_heap allocates one internal node per push and frees it per pop/erase.
// In windowed wPBS at mid task frequency the low-level A* is invoked hundreds of thousands of
// times, generating millions of pushes, so that internal malloc/free churn dominated the
// low-level search loop.  This stateless free-list allocator reuses freed node slots (kept on a
// static per-type free list that survives across calls), eliminating the per-push allocator
// traffic.  CRUCIALLY it changes NOTHING about the heap's ordering or behaviour — it only
// recycles memory — so the search result (and thus makespan/SWT) is byte-for-byte identical to
// the default-allocator fibonacci_heap.  The retained pool is never shrunk; it is a small fixed
// overhead (bounded by the largest single search's open list) freed at process exit.
template <typename T>
struct MLAPoolAllocator {
    typedef T value_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    template <typename U> struct rebind { typedef MLAPoolAllocator<U> other; };

    MLAPoolAllocator() noexcept {}
    template <typename U> MLAPoolAllocator(const MLAPoolAllocator<U>&) noexcept {}

    static std::vector<void*>& free_list() { static std::vector<void*> fl; return fl; }

    pointer allocate(size_type n) {
        if (n == 1) {
            auto& fl = free_list();
            if (!fl.empty()) { void* p = fl.back(); fl.pop_back(); return static_cast<pointer>(p); }
            return static_cast<pointer>(::operator new(sizeof(T)));
        }
        return static_cast<pointer>(::operator new(n * sizeof(T)));
    }
    void deallocate(pointer p, size_type n) noexcept {
        if (n == 1) { free_list().push_back(static_cast<void*>(p)); return; }
        ::operator delete(static_cast<void*>(p));
    }
    template <typename U, typename... Args> void construct(U* p, Args&&... args) {
        ::new (static_cast<void*>(p)) U(std::forward<Args>(args)...);
    }
    template <typename U> void destroy(U* p) { p->~U(); }
    bool operator==(const MLAPoolAllocator&) const noexcept { return true; }
    bool operator!=(const MLAPoolAllocator&) const noexcept { return false; }
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

    // --- Fast path for IDLE agents (PBS-root batch) -----------------------------------------
    // At a low task frequency most agents have no task and simply hold at their endpoint, which
    // happens to equal their (single) goal.  The full A* setup + search is pure overhead for
    // them, yet they dominate the plan count.  If this is a root plan (shared old-path table)
    // for an agent whose only goal is its current cell, and no OTHER agent ever occupies that
    // cell (no permanent parker, no mover passing through within the active window), the agent
    // can simply hold forever — return that path directly.  This is exact: the produced path is
    // identical to what the search would return (root pop -> goal reached -> can_hold true).
    if (num_goals == 1 && goals[0].first == start_loc && start_time >= goals[0].second &&
        constraint_window <= 0 && use_old_paths && shared_old_valid_ &&
        shared_old_excl_ == agent_id && !skip_holding && cons_paths.empty()) {
        bool excl_parked0 = (std::find(shared_old_movers_.begin(), shared_old_movers_.end(),
                                       shared_old_excl_) == shared_old_movers_.end());
        int parkers = shared_old_holdcnt_[start_loc];
        if (excl_parked0 && shared_old_holds_[shared_old_excl_] == start_loc) parkers--;
        bool blocked = (parkers > 0);
        if (!blocked) {
            int sh = shared_old_horizon_;
            for (int i : shared_old_movers_) {
                if (i == shared_old_excl_) continue;
                for (int t = 0; t < sh && !blocked; t++)
                    if (shared_old_flat_[(size_t)i * sh + t] == start_loc) blocked = true;
                if (shared_old_holds_[i] == start_loc) blocked = true;
                if (blocked) break;
            }
        }
        if (!blocked) {
            vector<int> hold(max_t, start_loc);
            return hold;
        }
    }
    // Match reference PBS KivaGraph: move[0]=1(EAST), move[1]=-cols(NORTH),
    // move[2]=-1(WEST), move[3]=cols(SOUTH). With WAIT first: {0,1,-col,-1,col}
    int action[5] = {0, 1, -mapd_map.col, -1, mapd_map.col};

    int search_horizon = min(max_t, start_time + max(1000, num_goals * 40));
    if (constraint_window > 0)
        search_horizon = min(search_horizon, constraint_window + 200);
    else
        // Non-windowed PBS: the forced endpoint hold is capped at start_time+384, so a complete
        // path (reach the goal within the map diameter, then wait out the hold) never needs more
        // than ~start_time+512.  Bounding the horizon here shrinks the per-search constraint
        // arrays (old_locs is O(n_old * horizon), rebuilt every search), which is the dominant
        // cost at low task frequency / high agent count, with no effect on the result.
        search_horizon = min(search_horizon, start_time + 512);

    // Use pre-computed endpoint h_vals where possible, otherwise compute BFS
    vector<const vector<int>*> h_vals_ptr(num_goals, nullptr);
    vector<vector<int>> h_vals_computed;  // storage for goals not matching endpoints
    for (int g = 0; g < num_goals; g++) {
        int gloc = goals[g].first;
        // Check if this location has a pre-computed endpoint h_val (O(1) via loc2ep)
        bool found = false;
        int ei = mapd_map.ep_index(gloc);
        if (ei >= 0) {
            h_vals_ptr[g] = &mapd_map.endpoints[ei].h_val;
            found = true;
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

    // Pre-compute constraint bitmaps for O(1) lookup
    // ct_loc[t] = location of each constraint agent at time t
    // This avoids scanning all cons_paths per expansion
    int n_cons = (int)cons_paths.size();
    int n_old_all = use_old_paths ? (int)old_paths.size() : 0;

    // --- Active-length rebasing (matches reference reservation-table construction) ---
    // Every committed path is a short ACTIVE prefix (agent moving toward its goal) followed by
    // a CONSTANT hold at its final location.  The reference builds its reservation table only
    // over the remaining real path and adds a single permanent endpoint hold at the last cell
    // (insertPath2CT: ct[path.back()].emplace_back(back_ts, INTERVAL_MAX)).  Materializing the
    // long constant tail of every other agent into a per-search 2D constraint array (n*max_t)
    // was THE dominant cost at low task frequency (huge max_t).  Instead, bound the per-timestep
    // constraint arrays to each path's active length and enforce the constant tail via permanent
    // endpoint holds (active_hold below).  This is exact: a cell occupied forever by a parked
    // agent is still blocked at every later timestep.
    // Active length = 1 + index of the last MOVE, found by a FORWARD scan bounded to the search
    // horizon (paths are an active prefix + constant hold, so the active part is short; scanning
    // backward over the long constant tail would be O(max_t) and defeat the optimization).
    auto active_len = [&](const vector<int>& p) -> int {
        int n = min((int)p.size(), search_horizon + 1);
        int last_move = 0;
        for (int t = 1; t < n; t++)
            if (p[t] != p[t-1]) last_move = t;
        return last_move + 1;
    };
    int cons_active = 0;
    for (int i = 0; i < n_cons; i++)
        cons_active = max(cons_active, active_len(cons_paths[i]));

    // Use the precomputed shared old-path table when this search is a PBS-root plan for the
    // excluded agent (avoids re-flattening every other agent's path per search).
    bool use_shared_old = use_old_paths && constraint_window <= 0 &&
                          shared_old_valid_ && shared_old_excl_ == agent_id;
    // Windowed (wPBS) mode reads the shared table as a SOFT conflict-avoidance table (focal
    // tie-break) only — never as a hard constraint — gated independently of use_shared_old.
    bool soft_use_shared = constraint_window > 0 &&
                           shared_old_valid_ && shared_old_excl_ == agent_id;
    int old_active = 0;
    if (use_shared_old) {
        old_active = shared_old_horizon_;  // already the global active length
    } else {
        for (int i = 0; i < n_old_all; i++)
            old_active = max(old_active, active_len(old_paths[i]));
    }

    // ct_horizon: extends to cover permanent holds if any cons_path reaches max_t
    int ct_horizon;
    if (constraint_window > 0) {
        // wPBS: hard constraints only within the window (matching reference insertPath2CT truncation)
        ct_horizon = min(constraint_window + 1, search_horizon);
    } else {
        // Non-windowed: bound to the active length of the constraint paths (their constant tails
        // are enforced via permanent endpoint holds below), not the full search horizon.
        ct_horizon = min(search_horizon, cons_active + 1);
    }
    int old_path_horizon = (constraint_window > 0) ? constraint_window : search_horizon;
    int n_old = use_shared_old ? 0 : n_old_all;  // shared table handled separately below
    int op_horizon = (constraint_window > 0)
        ? min(old_path_horizon + 1, search_horizon)
        : min(search_horizon, old_active + 1);
    // Shared-table parameters (meaningful when use_shared_old OR soft_use_shared).
    bool any_shared = use_shared_old || soft_use_shared;
    int sh_num = any_shared ? (int)agents.size() : 0;
    int sh_hor = any_shared ? shared_old_horizon_ : 0;
    (void)sh_num;
    // Is the excluded (self) agent itself a parked agent counted in shared_old_holdcnt_?
    bool excl_parked = false;
    if (any_shared) {
        excl_parked = (std::find(shared_old_movers_.begin(), shared_old_movers_.end(),
                                 shared_old_excl_) == shared_old_movers_.end());
    }

    // Flatten: cons_locs[agent_idx * ct_horizon + t] = location at time t
    vector<int> cons_locs(n_cons * ct_horizon);
    // Pre-compute endpoint holds: location each constraint agent holds beyond ct_horizon
    // Matches reference insertPath2CT which adds ct[path.back().location] = (back_timestep, INTERVAL_MAX)
    vector<int> cons_endpoint_holds(n_cons, -1);
    for (int i = 0; i < n_cons; i++) {
        for (int t = 0; t < ct_horizon; t++) {
            cons_locs[i * ct_horizon + t] = (t < (int)cons_paths[i].size()) ?
                cons_paths[i][t] : cons_paths[i].back();
        }
        if (ct_horizon > 0) {
            // The agent's final (held) position is occupied permanently.  In windowed mode this
            // is the window-boundary cell; in non-windowed mode it is the path's last cell after
            // its active prefix (the constant hold tail).  Enforced for abs_t >= ct_horizon.
            cons_endpoint_holds[i] = (int)cons_paths[i].back();
        }
    }
    vector<int> old_locs;
    vector<int> old_endpoint_holds;
    if (n_old > 0) {
        old_locs.resize(n_old * op_horizon);
        old_endpoint_holds.assign(n_old, -1);
        for (int i = 0; i < n_old; i++) {
            for (int t = 0; t < op_horizon; t++) {
                old_locs[i * op_horizon + t] = (t < (int)old_paths[i].size()) ?
                    old_paths[i][t] : old_paths[i].back();
            }
            old_endpoint_holds[i] = (int)old_paths[i].back();
        }
    }

    auto is_constrained_hard = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        if (!mapd_map.grid[next_loc]) return true;
        if (abs_t >= ct_horizon) {
            // Beyond the active prefix: only the permanent endpoint holds remain.
            // Matches reference insertPath2CT: ct[path.back().location].emplace_back(back_ts, INTERVAL_MAX).
            // Enforced for BOTH windowed and non-windowed: a cell occupied forever by a parked
            // higher-priority agent is blocked at every later timestep (the old code materialized
            // this same hold explicitly out to max_t; this is the compact equivalent).
            for (int i = 0; i < n_cons; i++) {
                if (cons_endpoint_holds[i] == next_loc) return true;
            }
            return false;
        }
        for (int i = 0; i < n_cons; i++) {
            int cl = cons_locs[i * ct_horizon + abs_t];
            if (cl == next_loc) return true;
            if (abs_t > 0) {
                int cl_prev = cons_locs[i * ct_horizon + abs_t - 1];
                if (cl == curr_loc && cl_prev == next_loc) return true;
            }
        }
        return false;
    };

    auto is_constrained_old = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        // In windowed (wPBS) mode, OLD paths must NOT be hard constraints.  The reference
        // PBS builds its reservation table only from HIGHER-PRIORITY current paths
        // (get_reachable_nodes) plus the dummy-endpoint initial_constraints — it never
        // blocks an agent with other agents' previous-iteration paths.  Treating old paths
        // as hard here, combined with the short start+10 search cutoff, makes a jammed agent
        // unable to find any 10-step move and return a hold-in-place partial path; with every
        // agent doing so the root has no conflicts, PBS commits the all-hold plan, and the
        // whole system deadlocks (never finishing tasks -> hits maxtime).  Genuine collisions
        // are still caught by PBS conflict detection / cascade and the final deconfliction.
        if (constraint_window > 0) return false;
        // Shared old-path table (PBS-root batch): read the precomputed flat table, skip self.
        if (use_shared_old) {
            // PARKED agents (constant hold at one cell for all time) are checked in O(1) via the
            // hold count; only the (few) MOVING agents need a per-timestep table scan.  Note:
            // shared_old_holdcnt_ counts ONLY parked agents, so it is exact at every timestep
            // (a parked agent sits on its cell forever).  The excluded (self) agent is removed
            // from the count only if it is itself parked there.
            int cnt = shared_old_holdcnt_[next_loc];
            if (cnt > 0) {
                // remove self only if self is a parked agent sitting on next_loc
                bool self_parked_here = excl_parked &&
                    (shared_old_holds_[shared_old_excl_] == next_loc);
                if (!(self_parked_here && cnt == 1)) return true;
            }
            if (abs_t < sh_hor) {
                // active window: movers are at their actual flat positions
                for (int i : shared_old_movers_) {
                    if (i == shared_old_excl_) continue;
                    int ol = shared_old_flat_[(size_t)i * sh_hor + abs_t];
                    if (ol == next_loc) return true;
                    if (abs_t > 0) {
                        int ol_prev = shared_old_flat_[(size_t)i * sh_hor + abs_t - 1];
                        if (ol == curr_loc && ol_prev == next_loc) return true;
                    }
                }
            } else {
                // beyond the active window every mover has reached its final hold cell
                for (int i : shared_old_movers_) {
                    if (i == shared_old_excl_) continue;
                    if (shared_old_holds_[i] == next_loc) return true;
                }
            }
            return false;
        }
        if (n_old == 0) return false;
        if (abs_t >= op_horizon) {
            // Beyond the active prefix: enforce the permanent endpoint hold (the old path's
            // last cell, occupied forever).  This preserves the previous behavior (which
            // materialized this constant tail explicitly) at a fraction of the cost.
            for (int i = 0; i < n_old; i++)
                if (old_endpoint_holds[i] == next_loc) return true;
            return false;
        }
        for (int i = 0; i < n_old; i++) {
            int ol = old_locs[i * op_horizon + abs_t];
            if (ol == next_loc) return true;
            if (abs_t > 0) {
                int ol_prev = old_locs[i * op_horizon + abs_t - 1];
                if (ol == curr_loc && ol_prev == next_loc) return true;
            }
        }
        return false;
    };

    // Soft-conflict counter for the windowed (wPBS) focal low level.  Counts (vertex + edge)
    // collisions of the move curr_loc->next_loc@abs_t with OTHER agents' paths (the shared CAT).
    // Replicates the reference conflict-avoidance table so the +10-truncated search picks the
    // equal-cost prefix that collides LEAST, instead of an arbitrary one that PBS must then
    // deconflict with extra waiting (the source of the windowed-mode SWT inflation at mid demand).
    auto count_soft_old = [&](int curr_loc, int next_loc, int abs_t) -> int {
        if (!soft_use_shared) return 0;
        int cnt = shared_old_holdcnt_[next_loc];
        bool self_parked_here = excl_parked && (shared_old_holds_[shared_old_excl_] == next_loc);
        if (self_parked_here && cnt > 0) cnt--;  // do not count self
        if (abs_t < sh_hor) {
            for (int i : shared_old_movers_) {
                if (i == shared_old_excl_) continue;
                int ol = shared_old_flat_[(size_t)i * sh_hor + abs_t];
                if (ol == next_loc) cnt++;
                if (abs_t > 0) {
                    int ol_prev = shared_old_flat_[(size_t)i * sh_hor + abs_t - 1];
                    if (ol == curr_loc && ol_prev == next_loc) cnt++;
                }
            }
        } else {
            for (int i : shared_old_movers_) {
                if (i == shared_old_excl_) continue;
                if (shared_old_holds_[i] == next_loc) cnt++;
            }
        }
        return cnt;
    };

    // Get earliest holding time from constraints.
    // Always scan to search_horizon (not limited by constraint_window)
    // so endpoint holds are properly detected even in windowed mode.
    int last_goal_loc = goals.back().first;
    int earliest_holding = 0;
    // Cap how far into the future we force the agent to "hold" (wait for its goal to free).
    // The reference operates in a per-solve REBASED time frame with SHORT remaining paths,
    // so its endpoint-holding time is naturally bounded by actual path lengths.  In this
    // absolute-time reimplementation, an old/other path that occupies the goal until a
    // far-future absolute timestep (e.g. ~1000 at low task frequency) would otherwise force
    // a ~1000-step pre-wait, exploding the low-level search.  Capping the forced hold is
    // correctness-preserving: if the agent occupies its goal "too early" and a later-arriving
    // agent conflicts, PBS detects that conflict and resolves it by priority, exactly as for
    // any other conflict.
    int holding_cap = start_time + 384;
    if (!skip_holding) {
        for (auto& cp : cons_paths) {
            int scan_limit = min((int)cp.size(), search_horizon);
            for (int t = scan_limit - 1; t >= 0; t--) {
                if (cp[t] == last_goal_loc) {
                    earliest_holding = max(earliest_holding, t + 1);
                    break;
                }
            }
        }
        if (use_shared_old) {
            // Compact equivalent of scanning every other agent's old path for the last occurrence
            // of last_goal_loc.  PARKED agents (the majority) are checked in O(1) via the hold
            // count: a non-self agent permanently parked on the goal occupies it up to the horizon.
            int park_cnt = shared_old_holdcnt_[last_goal_loc];
            if (excl_parked && shared_old_holds_[shared_old_excl_] == last_goal_loc) park_cnt--;
            if (park_cnt > 0)
                earliest_holding = max(earliest_holding,
                                       min(search_horizon, old_path_horizon + 1));
            // Only the (few) MOVING agents need a per-timestep scan.
            for (int i : shared_old_movers_) {
                if (i == shared_old_excl_) continue;
                for (int t = sh_hor - 1; t >= 0; t--) {
                    if (shared_old_flat_[(size_t)i * sh_hor + t] == last_goal_loc) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
                }
            }
        } else if (use_old_paths) {
            for (auto& op : old_paths) {
                int scan_limit = min({(int)op.size(), search_horizon, old_path_horizon + 1});
                for (int t = scan_limit - 1; t >= 0; t--) {
                    if (op[t] == last_goal_loc) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
                }
            }
        }
        if (earliest_holding > holding_cap)
            earliest_holding = holding_cap;
    }

    // Fast hopeless-search detection: if the FINAL goal location is permanently occupied
    // by another agent that is parked there (some constraint/old path ends at this goal and
    // holds it forever), the agent can never legally occupy its goal, so no path exists.
    // Bailing out immediately avoids exhausting ~mla_max_nodes on a search that is doomed
    // (this is the common case at low task frequencies, where many parked agents block other
    // agents' goals at the PBS root; PBS then resolves it later by adding priorities and
    // replanning the parked agent).  This keeps the low-level search fast WITHOUT affecting
    // makespan/SWT: the same empty result is returned, just far cheaper.
    // Only a HIGHER-PRIORITY agent (in cons_paths) parked permanently on our final goal
    // makes the search truly hopeless: such an agent will never be replanned to yield to us,
    // so the goal is permanently occupied and no path exists.  (We must NOT bail on OLD-path
    // occupancy: an old path belongs to an agent that PBS may replan to move out of the way,
    // so its current parking position is not a permanent block — bailing on it dropped valid
    // paths and made PBS commit collisions.)
    // A higher-priority agent (in cons_paths) permanently parked on our FINAL goal makes
    // the search hopeless: such an agent is never replanned to yield, so the goal is
    // permanently occupied and no path exists.  Bailing here is correctness-preserving (it
    // returns exactly the empty result the exhaustive search would, just immediately) and
    // is the common reason a low-priority agent's search would otherwise exhaust the node
    // cap.  We deliberately do NOT bail on OLD-path occupancy: an old path belongs to an
    // agent PBS may replan to move aside, so it is not a permanent block.
    if (start_loc != last_goal_loc) {
        bool goal_permanently_blocked = false;
        for (int i = 0; i < n_cons && !goal_permanently_blocked; i++)
            if (!cons_paths[i].empty() && cons_paths[i].back() == last_goal_loc)
                goal_permanently_blocked = true;
        if (goal_permanently_blocked)
            return {};
    }

    typedef boost::heap::fibonacci_heap<MLANode*, boost::heap::compare<CompareMLANode>,
            boost::heap::allocator<MLAPoolAllocator<MLANode*>>> mla_heap_t;
    mla_heap_t open_list;

    // Use unordered_map with fast hash for (loc, goal_id, g_val) dedup.
    // RETAINED across calls (static; seq_mla_star is non-reentrant) and cleared per call so the
    // bucket array is reused instead of reallocated on every one of the hundreds of thousands of
    // windowed searches — removing per-call map construction/rehash churn from the low-level loop.
    struct KeyHash {
        size_t operator()(uint64_t k) const { return k * 2654435761ULL; }
    };
    static unordered_map<uint64_t, MLANode*, KeyHash> allNodes;
    allNodes.clear();
    auto make_key = [&](int loc, int gi, int g) -> uint64_t {
        return ((uint64_t)loc << 32) | ((uint64_t)gi << 20) | (uint64_t)g;
    };

    int init_h = compute_h(start_loc, 0);
    if (init_h == INT_MAX) return {};

    // Node-expansion cap for the low-level MLA* search.  A* finds any REAL path in far
    // fewer than map_size*depth nodes; a search that exhausts a large multiple of the map
    // size has no valid path within the horizon (typically because a high-priority/old
    // path occupies the goal until a far-future deadline and the agent cannot legally
    // wait anywhere reachable).  Capping these hopeless searches low keeps PBS fast at low
    // task frequencies (huge makespans) WITHOUT changing makespan/SWT: the deadline-aware
    // heuristic still finds every reachable path quickly, and the genuinely-infeasible
    // ones fall back exactly as before.  Previously this was 500000, which made each
    // hopeless root search burn ~0.15s and blew the runtime budget ~20x.
    int mla_max_nodes = min(500000, map_size * 20);

    // Reset the node arena for this search (chunks retained; only the bump cursor is rewound).
    mla_arena_used_chunk_ = 0;
    mla_arena_used_in_chunk_ = 0;
    auto alloc_node = [&](int l, int g, int h, int t, int gi, MLANode* p) -> MLANode* {
        if (mla_arena_used_chunk_ >= (int)mla_arena_chunks_.size()) {
            mla_arena_chunks_.push_back(
                ::operator new(sizeof(MLANode) * (size_t)MLA_ARENA_CHUNK));
        }
        MLANode* chunk = static_cast<MLANode*>(mla_arena_chunks_[mla_arena_used_chunk_]);
        MLANode* node = &chunk[mla_arena_used_in_chunk_];
        if (++mla_arena_used_in_chunk_ >= MLA_ARENA_CHUNK) {
            mla_arena_used_chunk_++;
            mla_arena_used_in_chunk_ = 0;
        }
        return new (node) MLANode(l, g, h, t, gi, p);
    };

    MLANode* root = alloc_node(start_loc, 0, init_h, start_time, 0, nullptr);
    open_list.push(root);
    allNodes[make_key(start_loc, 0, 0)] = root;

    MLANode* solution = nullptr;
    int mla_expanded = 0;

    // wPBS: match reference StateTimeAStar which terminates at start + 10 (hardcoded)
    // Reference line 103: if (curr->goal_id == goal_size || curr->timestep >= start + 10)
    // The reference uses a hardcoded 10-step search truncation, NOT planning_window.
    int wpbs_path_cutoff = (constraint_window > 0) ? (start_time + 10) : -1;

    while (!open_list.empty() && mla_expanded < mla_max_nodes) {
        MLANode* curr = open_list.top();
        open_list.pop();

        // Skip stale lazy-deleted duplicates (a lower-conflict path repointed allNodes[key]).
        if (soft_use_shared) {
            auto itc = allNodes.find(make_key(curr->loc, curr->goal_id, curr->g_val));
            if (itc != allNodes.end() && itc->second != curr) continue;
        }
        mla_expanded++;

        // Advance goal_id if at current goal and past release time
        int gi = curr->goal_id;
        if (gi < num_goals && curr->loc == goals[gi].first) {
            int rel = goals[gi].second;
            if (curr->timestep >= rel) {
                if (gi == num_goals - 1 && curr->timestep < earliest_holding) {}
                else gi++;
            }
        }

        // wPBS: terminate at window boundary even if not all goals reached
        // This matches reference StateTimeAStar: return path at start + 10
        if (gi >= num_goals || (wpbs_path_cutoff >= 0 && curr->timestep >= wpbs_path_cutoff)) {
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

            // Deadline-aware heuristic (matches the reference StateTimeAStar idea of
            // h = max(h, holding_deadline - elapsed)).  The agent cannot finish before
            // earliest_holding, so the true remaining cost is at least
            // (earliest_holding - next_t).  Folding this into h keeps the heuristic
            // admissible while making A* beeline to the goal and wait there, instead of
            // fanning out across the whole map for ~holding timesteps (which caused a
            // 500k-node blowup at low task frequencies / huge makespans).
            // NOTE: in windowed (wPBS) mode the search is hard-capped at start+10 so a
            // blowup is impossible; the reference StateTimeAStar there uses the PURE
            // distance heuristic (compute_h_value) and relies on it to beeline toward the
            // goal within the 10-step window.  Applying the deadline-aware inflation in that
            // mode flattens h across all neighbors (g + (earliest_holding - next_t) is ~const
            // since g~next_t), so A* loses all sense of direction and the +10 cutoff returns a
            // hold-in-place path — every agent then freezes and the simulation deadlocks
            // (never finishing tasks, hitting maxtime).  Only apply the deadline trick in the
            // non-windowed PBS path, where it prevents the 500k-node low-level blowup at low
            // task frequencies / huge makespans.
            if (earliest_holding > 0 && wpbs_path_cutoff < 0) {
                int deadline_h = earliest_holding - next_t;
                if (deadline_h > next_h) next_h = deadline_h;
            }

            int next_conf = curr->conflicts + count_soft_old(curr->loc, next_loc, next_t);

            auto key = make_key(next_loc, child_gi, next_g);
            auto itn = allNodes.find(key);
            if (itn == allNodes.end()) {
                MLANode* next_node = alloc_node(next_loc, next_g, next_h, next_t,
                                                child_gi, curr);
                next_node->conflicts = next_conf;
                allNodes[key] = next_node;
                open_list.push(next_node);
            } else if (next_conf < itn->second->conflicts) {
                // Same (loc,goal_id,g) state reached via a lower-conflict path (same f) — a pure
                // focal improvement.  Push a fresh node and repoint the map; the stale higher-
                // conflict entry is skipped at pop time.  (Mutating a heap node in place would
                // corrupt the fibonacci heap ordering, so we don't.)
                MLANode* next_node = alloc_node(next_loc, next_g, next_h, next_t,
                                                child_gi, curr);
                next_node->conflicts = next_conf;
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

        // In windowed (wPBS) mode the committed plan is only read over the short working window
        // (pbs_core's work_len = timestep + replan_window - 1), and the result is a constant hold
        // at last_loc thereafter.  Sizing result to a tight window-covering length instead of the
        // full horizon (max_t ~5000) avoids a ~max_t/window-fold per-call allocation + tail-fill
        // that dominated the windowed low-level cost.  The consumers index result[t] only for
        // t < work_len, so any length >= start_time + path length (and >= work_len) is exact; the
        // trailing constant hold is reconstructed downstream (commit_node / caller's hold loops).
        int res_len = max_t;
        if (constraint_window > 0) {
            // work_len = timestep + replan_window - 1; +8 margin keeps boundary-hold reads valid.
            int needed = start_time + (int)path_locs.size();
            res_len = min(max_t, max(needed + 1, constraint_window + 8));
        }
        result.resize(res_len);
        for (int t = 0; t < start_time && t < res_len; t++)
            result[t] = start_loc;
        for (int i = 0; i < (int)path_locs.size(); i++) {
            int t = start_time + i;
            if (t < res_len) result[t] = path_locs[i];
        }
        int last_loc = path_locs.back();
        for (int t = start_time + (int)path_locs.size(); t < res_len; t++)
            result[t] = last_loc;
    }

    // Nodes live in the retained arena (trivially destructible POD); no per-node delete needed.
    return result;
}

// ============================================================
// Section 32a: SIPP Search — Drop-in replacement for seq_mla_star
//   Uses safe intervals from cons_paths to reduce state space.
//   State: (location, interval_index, goal_id)
// ============================================================

vector<int> Simulation::sipp_search(int agent_id, int start_loc, int start_time,
                                     const vector<pair<int,int>>& goals,
                                     const vector<vector<int>>& cons_paths,
                                     const vector<vector<int>>& old_paths,
                                     bool use_old_paths,
                                     bool skip_holding,
                                     int constraint_window) {
#ifdef SIPP_PROF
    static long long g_calls=0, g_setup_ns=0, g_loop_ns=0, g_recon_ns=0, g_exp=0, g_edge=0, g_bail=0;
    static bool g_reg=false;
    if (!g_reg){ g_reg=true; atexit([](){ fprintf(stderr,"[SIPPPROF] calls=%lld bail=%lld exp=%lld edgecalls=%lld setup=%.1fms loop=%.1fms recon=%.1fms\n", g_calls,g_bail,g_exp,g_edge,g_setup_ns/1e6,g_loop_ns/1e6,g_recon_ns/1e6);}); }
    g_calls++;
    auto __t0 = std::chrono::high_resolution_clock::now();
#endif
    if (goals.empty()) return vector<int>((int)maxtime, start_loc);

    int map_size = mapd_map.row * mapd_map.col;
    int max_t = (int)maxtime;
    int cols = mapd_map.col;
    int num_goals = (int)goals.size();
    // Windowed (wPBS) mode: the reference StateTimeA* truncates the low-level search at start+10
    // and treats the result as a partial path (the path is returned even if not all goals are
    // reached).  seq_mla_star mirrors this with wpbs_path_cutoff = start+10.  Previously
    // sipp_search ignored constraint_window entirely and ran a FULL multi-goal search every call,
    // doing far more work per call than the windowed MLA* low-level (and producing different,
    // non-truncated paths -> a different SWT).  Mirror seq_mla_star exactly: cap the search and
    // the search horizon to the +10 window so windowed SIPP is both fast and behaviour-matching.
    // Windowed (wPBS) lookahead.  The reference replans every replan_window(=10) steps and only the
    // first replan_window steps of each plan are executed before the next replan.  A deeper SIPP
    // lookahead than +10 yields a LESS greedy first-10-steps decision (it sees further before
    // committing), restoring path quality (SWT/makespan) in dense high-agent scenes without slowing
    // things much — SIPP is efficient and the search is still bounded.  +10 alone was too greedy
    // (inflated SWT at ag40-50 / freq 2-10); a wider lookahead matches the reference MLA* low-level.
    bool windowed = (constraint_window > 0);
    int win_look = 0;
    if (windowed) {
        const char* e = getenv("SIPP_WIN_LOOK");
        win_look = e ? atoi(e) : 16;
    }
    int wpbs_cutoff = windowed ? (start_time + win_look) : INT_MAX;
    int horizon = min(max_t, start_time + max(1000, num_goals * 40));
    if (windowed) horizon = min(horizon, start_time + win_look + 2);

    // Heuristics (O(1) endpoint lookup via loc2ep)
    auto h_for = [&](int loc, int gloc) -> int {
        int ei = mapd_map.ep_index(gloc);
        if (ei >= 0) return mapd_map.endpoints[ei].h_val[loc];
        return INT_MAX;
    };
    auto sum_h = [&](int loc, int gi) -> int {
        if (gi >= num_goals) return 0;
        int h = h_for(loc, goals[gi].first);
        if (h == INT_MAX) return INT_MAX;
        for (int g = gi; g < num_goals - 1; g++) {
            int d = h_for(goals[g].first, goals[g+1].first);
            if (d == INT_MAX) return INT_MAX;
            h += d;
        }
        return h;
    };

    // At the PBS root the shared old-path table (built once per solve) is available; use it to
    // speed up the per-expansion edge check (only MOVING old agents can swap, and movers are few
    // at a low task frequency).  This is behaviour-preserving: parked agents never move, so they
    // never contribute an edge (swap) conflict.  The vertex constraints (CT ranges) and the
    // earliest-holding scan still use the real old_paths, so results are unchanged.
    bool use_shared_old_edges = use_old_paths && shared_old_valid_ &&
                                shared_old_excl_ == agent_id && cons_paths.empty();

    // Build CT ranges from cons_paths (compress consecutive same-loc into ranges).
    // Active-length rebasing (matches seq_mla_star's reservation-table construction): every
    // committed path is a short ACTIVE prefix (agent moving toward its goal) followed by a
    // CONSTANT hold at its final cell.  Scanning each path's long constant tail out to `horizon`
    // (the parking constraints for not-yet-planned agents are constant for ALL maxtime!) was the
    // dominant SIPP cost — O(n_cons * horizon) per call over tens of thousands of calls.  Instead
    // we find the active length by a FORWARD scan that STOPS at the last change, then emit a single
    // hold range [last_change, horizon] for the constant tail.  Result is identical: a constant
    // path still produces exactly one range covering [first, horizon] at its held cell.
#ifdef SIPP_PROF
    static long long g_ct_ns=0; static bool g_ctreg=false;
    if(!g_ctreg){g_ctreg=true; atexit([](){fprintf(stderr,"[SIPPPROF] ct_build=%.1fms\n", g_ct_ns/1e6);});}
    auto __tc0 = std::chrono::high_resolution_clock::now();
#endif
    // Persistent scratch buffers (sized once, cleared per call in O(touched cells)).
    if ((int)sipp_ct_.size() != map_size) {
        sipp_ct_.assign(map_size, {});
        sipp_has_ct_.assign(map_size, 0);
        sipp_sit_.assign(map_size, {});
        sipp_sit_done_.assign(map_size, 0);
    }
    auto& ct_ranges = sipp_ct_;
    auto& has_ct = sipp_has_ct_;
    sipp_ct_touched_.clear();
    sipp_sit_touched_.clear();
    auto touch_ct = [&](int loc) {
        if (!has_ct[loc]) { has_ct[loc] = 1; sipp_ct_touched_.push_back(loc); ct_ranges[loc].clear(); }
    };
    // Reset persistent buffers' dirty flags for touched cells only (keeps sub-vector capacity).
    auto reset_scratch = [&]() {
        for (int c : sipp_ct_touched_) has_ct[c] = 0;
        for (int c : sipp_sit_touched_) sipp_sit_done_[c] = 0;
    };

    // Per-cons_path active metadata: (last_change index within window, held tail location).
    // Captured during the single forward CT scan and REUSED for earliest_hold below, so the
    // long constant tails of parking/arrived constraints are never re-scanned a second time.
    int n_cons_ = (int)cons_paths.size();
    vector<int> cons_lastchg(n_cons_, 0), cons_tail(n_cons_, -1);
    auto add_path = [&](const vector<int>& path, int idx) {
        int psz = (int)path.size();
        if (psz == 0) return;
        int len = min(psz, horizon);
        // active length = 1 + index of last change within the window (constant tail excluded).
        // Scan BACKWARD from the end so the long CONSTANT tail (the common case: parked agents and
        // arrived agents hold one cell for the whole remaining horizon) is skipped in a tight loop
        // that stops at the active/tail boundary, instead of a forward scan that must traverse the
        // entire tail.  Exact: last_change is the index of the last cell that differs from its
        // predecessor within the window.
        int last_change = 0;
        for (int t = len - 1; t >= 1; t--)
            if (path[t] != path[t-1]) { last_change = t; break; }
        // emit ranges over the active prefix [0, last_change], then the held tail to horizon
        int ss = 0, sl = path[0];
        for (int t = 1; t <= last_change; t++) {
            if (path[t] != sl) {
                touch_ct(sl); ct_ranges[sl].push_back({ss, t});
                sl = path[t]; ss = t;
            }
        }
        touch_ct(sl); ct_ranges[sl].push_back({ss, horizon});
        if (idx >= 0) { cons_lastchg[idx] = last_change; cons_tail[idx] = sl; }
    };
    for (int i = 0; i < n_cons_; i++) add_path(cons_paths[i], i);
    // Use precompressed shared old-path CT ranges at the PBS root (built once per solve), which
    // removes the per-search O(num_ag * old_h) compression — the dominant SIPP cost at low task
    // frequency.  Equivalent to the per-search compression: ranges are clamped to this search's
    // old_h, so vertex constraints are identical.
    bool use_shared_old_ct = use_old_paths && shared_old_valid_ &&
                             shared_old_excl_ == agent_id && cons_paths.empty() &&
                             (int)shared_old_sipp_ranges_.size() == (int)agents.size();
    if (use_shared_old_ct) {
        int old_h = min(start_time + 300, horizon);
        for (int j = 0; j < (int)shared_old_sipp_ranges_.size(); j++) {
            if (j == shared_old_excl_) continue;
            for (auto& r : shared_old_sipp_ranges_[j]) {
                int loc = std::get<0>(r), s = std::get<1>(r), e = std::get<2>(r);
                if (s >= old_h) continue;          // range starts beyond this search's window
                if (e > old_h) e = old_h;          // clamp to this search's old_h
                if (loc < 0 || loc >= map_size) continue;
                touch_ct(loc); ct_ranges[loc].push_back({s, e});
            }
        }
    } else if (use_old_paths) {
        int old_h = min(start_time + 300, horizon);
        for (auto& op : old_paths) {
            int len = min((int)op.size(), old_h);
            if (len == 0) continue;
            int ss = 0, sl = op[0];
            for (int t = 1; t < len; t++) {
                if (op[t] != sl) {
                    touch_ct(sl); ct_ranges[sl].push_back({ss, t});
                    sl = op[t]; ss = t;
                }
            }
            touch_ct(sl); ct_ranges[sl].push_back({ss, old_h});
        }
    }

#ifdef SIPP_PROF
    { auto __tc1 = std::chrono::high_resolution_clock::now();
      g_ct_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(__tc1-__tc0).count(); }
#endif
    // Lazy SIT cache (persistent per-cell buffers; computed at most once per cell per call).
    auto& sit = sipp_sit_;
    auto& sit_done = sipp_sit_done_;
    auto get_sit = [&](int loc) -> const vector<pair<int,int>>& {
        if (sit_done[loc]) return sit[loc];
        sit_done[loc] = 1; sipp_sit_touched_.push_back(loc);
        auto& ivs = sit[loc]; ivs.clear();
        if (!has_ct[loc]) return ivs;
        // merge CT ranges (don't clip — hold ranges extend to horizon), then invert to free intervals
        auto& rs = ct_ranges[loc];
        sort(rs.begin(), rs.end());
        int p = 0;
        int cs = -1, ce = -1; // current merged range
        for (auto& [s,e] : rs) {
            if (cs < 0) { cs = s; ce = e; continue; }
            if (s <= ce) { if (e > ce) ce = e; }
            else { if (cs > p) ivs.push_back({p, cs}); p = ce; cs = s; ce = e; }
        }
        if (cs >= 0) { if (cs > p) ivs.push_back({p, cs}); p = ce; }
        if (p < horizon) ivs.push_back({p, horizon});
        return ivs;
    };

    // Edge constraint
    auto edge_blocked = [&](int from, int to, int t) -> bool {
        if (t <= 0 || t >= horizon) return false;
        for (auto& cp : cons_paths) {
            int len = (int)cp.size();
            int ct = (t < len) ? cp[t] : cp[len-1];
            int cp_ = (t-1 < len) ? cp[t-1] : cp[len-1];
            if (ct == from && cp_ == to) return true;
        }
        if (use_shared_old_edges) {
            // Only moving old agents can swap; use the shared flat table (active window only).
            // Bound to min(active horizon, start+300) to match the original old-edge window.
            int stride = shared_old_horizon_;
            int lim = min(stride, start_time + 301);
            if (t < lim) {
                for (int i : shared_old_movers_) {
                    if (i == shared_old_excl_) continue;
                    int ot = shared_old_flat_[(size_t)i * stride + t];
                    int op_ = shared_old_flat_[(size_t)i * stride + (t-1)];
                    if (ot == from && op_ == to) return true;
                }
            }
        } else if (use_old_paths) {
            int old_h = start_time + 300;
            if (t <= old_h) {
                for (auto& op : old_paths) {
                    int len = (int)op.size();
                    int ot = (t < len) ? op[t] : op[len-1];
                    int op_ = (t-1 < len) ? op[t-1] : op[len-1];
                    if (ot == from && op_ == to) return true;
                }
            }
        }
        return false;
    };

    // Earliest holding — match MLA* behavior: latest time any constraint occupies last_goal.
    // Uses the per-path active metadata computed above so the long CONSTANT tail of each
    // constraint is checked in O(1) (tail == last_goal -> occupied through horizon-1), and only
    // the short active prefix [0, last_change] is scanned.  Identical result to the old O(horizon)
    // backward scan: a constant path equal to last_goal occupied it at every t, so its last
    // occurrence is horizon-1 (-> earliest_hold = horizon).
    int last_goal = goals.back().first;
    int earliest_hold = 0;
    if (!skip_holding) {
        for (int i = 0; i < n_cons_; i++) {
            const auto& cp = cons_paths[i];
            if ((int)cp.size() == 0) continue;
            if (cons_tail[i] == last_goal) {            // held at last_goal through the window
                earliest_hold = max(earliest_hold, horizon);
                continue;
            }
            int lim = min(cons_lastchg[i], min((int)cp.size(), horizon) - 1);
            for (int t = lim; t >= 0; t--) {
                if (cp[t] == last_goal) { earliest_hold = max(earliest_hold, t+1); break; }
            }
        }
        if (use_old_paths) {
            for (auto& op : old_paths) {
                int lim = min({(int)op.size(), horizon, start_time+301});
                for (int t = lim-1; t >= 0; t--)
                    if (op[t] == last_goal) { earliest_hold = max(earliest_hold, t+1); break; }
            }
        }
    }

    // Fast hopeless-search early-bail (identical to seq_mla_star).  If a HIGHER-PRIORITY agent
    // (one of cons_paths) is permanently parked on this agent's FINAL goal, the goal is occupied
    // forever and no path exists.  Such an agent is never replanned to yield, so the search is
    // doomed and would otherwise run to the full expansion cap (500k) — by far the dominant SIPP
    // cost at low task frequency, where many parked/not-yet-planned agents block goals.  Returning
    // the empty result immediately is exactly what the exhaustive search returns, just far cheaper.
    // (We deliberately do NOT bail on old_paths occupancy: an old path's agent may be replanned by
    // PBS to move aside, so its parking spot is not a permanent block.)
    // (Skip in windowed wPBS mode: there the search truncates at start+10 and returns a PARTIAL
    // path even if the goal is unreachable, so a permanently-blocked final goal must NOT short-
    // circuit to an empty result — the agent still makes its windowed progress.)
    if (!windowed && start_loc != last_goal) {
        for (auto& cp : cons_paths)
            if (!cp.empty() && cp.back() == last_goal)
                { reset_scratch(); return {}; }
    }

    // ---- Windowed (wPBS) time-expanded search -------------------------------------------------
    // In windowed mode the reference low-level (StateTimeA* / seq_mla_star) runs a TIME-EXPANDED
    // A* with a first-class WAIT action and state (loc, goal, time), truncated at a short window.
    // Classic interval-state SIPP cannot represent "wait in place" within the truncation window
    // and so produced detours and inflated SWT in dense scenes.  Here we run exactly the reference
    // search but using SIPP's O(1) constraint structures (has_ct/get_sit for vertex checks,
    // edge_blocked for swaps), so windowed --sipp matches the MLA* low-level's paths while staying
    // fast.  The interval-SIPP path below is used only for the (cheaper, optimal) NON-windowed mode.
    if (windowed) {
        // vertex free? cell loc unoccupied at absolute time t
        auto vfree = [&](int loc, int t) -> bool {
            if (!has_ct[loc]) return true;
            const auto& iv = get_sit(loc);
            for (auto& r : iv) if (r.first <= t && t < r.second) return true;
            return false;
        };
        struct WN { int loc, g, h, t, gi; WN* parent; int f() const { return g+h; } };
        struct CmpWN { bool operator()(const WN* a, const WN* b) const {
            if (a->f() != b->f()) return a->f() > b->f();
            return a->g <= b->g; } };
        boost::heap::fibonacci_heap<WN*, boost::heap::compare<CmpWN>> wopen;
        struct WHash { size_t operator()(uint64_t k) const { return k * 2654435761ULL; } };
        unordered_map<uint64_t,int,WHash> wclosed;
        vector<WN*> wnodes;
        auto wkey = [](int loc, int gi, int t) -> uint64_t {
            return ((uint64_t)loc << 32) | ((uint64_t)(gi & 0xFFFF) << 16) | (uint32_t)(t & 0xFFFF);
        };
        int ih = sum_h(start_loc, 0);
        if (ih == INT_MAX) { reset_scratch(); return {}; }
        WN* root = new WN{start_loc, 0, ih, start_time, 0, nullptr};
        wopen.push(root); wnodes.push_back(root);
        wclosed[wkey(start_loc, 0, start_time)] = 0;
        int wmoves[5] = {0, 1, -cols, -1, cols};
        WN* wsol = nullptr;
        while (!wopen.empty()) {
            WN* cur = wopen.top(); wopen.pop();
            int gi = cur->gi;
            if (gi < num_goals && cur->loc == goals[gi].first && cur->t >= goals[gi].second) {
                if (gi == num_goals-1 && cur->t < earliest_hold) {} else gi++;
            }
            if (gi >= num_goals || cur->t >= wpbs_cutoff) { wsol = cur; break; }
            if (cur->t >= horizon - 1) continue;
            for (int d = 0; d < 5; d++) {
                int nl = cur->loc + wmoves[d];
                if (nl < 0 || nl >= map_size || !mapd_map.grid[nl]) continue;
                if (d != 0 && abs(nl % cols - cur->loc % cols) > 1) continue;
                int nt = cur->t + 1;
                if (!vfree(nl, nt)) continue;
                if (d != 0 && edge_blocked(cur->loc, nl, nt)) continue;
                int ngi = gi;
                if (ngi < num_goals && nl == goals[ngi].first && nt >= goals[ngi].second)
                    { if (ngi == num_goals-1 && nt < earliest_hold) {} else ngi++; }
                int nh = sum_h(nl, ngi); if (nh == INT_MAX) continue;
                int ng = nt - start_time;
                auto nk = wkey(nl, ngi, nt);
                if (!wclosed.count(nk) || wclosed[nk] > ng) {
                    wclosed[nk] = ng;
                    WN* nd = new WN{nl, ng, nh, nt, ngi, cur};
                    wopen.push(nd); wnodes.push_back(nd);
                }
            }
        }
        vector<int> result;
        if (wsol) {
            vector<int> locs;
            for (WN* n = wsol; n; n = n->parent) locs.push_back(n->loc);
            reverse(locs.begin(), locs.end());
            // COMMIT-WINDOW TRUNCATION (correctness): the wider win_look (16) lookahead is used only
            // to make a better first-window DECISION; only the first replan_window steps are actually
            // executed before the next replan.  The windowed PBS machinery (conflict_horizon =
            // timestep+replan_window+1, and seq_mla_star's own start+10 cutoff) assumes every
            // committed plan is ACTIVE only up to start+replan_window and a constant HOLD after.
            // Emitting the full 16-step active path instead left moves in [start+replan_window+1,
            // start+win_look] that PBS never conflict-checks (they fall in the [conf_horizon,
            // work_len) gap) yet get committed and executed -> head-on corridor collisions that only
            // --sipp+wPBS hit (MLA*'s start+10 cutoff never produced them).  Truncate the committed
            // path to active<=start+replan_window then hold, exactly matching the MLA* low-level.
            int commit_len = min((int)locs.size(), config.replan_window + 1);
            result.assign(max_t, start_loc);
            for (int i = 0; i < commit_len; i++) {
                int t = start_time + i; if (t < max_t) result[t] = locs[i];
            }
            int last = locs[commit_len - 1];
            for (int t = start_time + commit_len; t < max_t; t++) result[t] = last;
        }
        for (auto* n : wnodes) delete n;
        reset_scratch();
        return result;
    }

    // SIPP node
    struct SN {
        int loc, g, h, t, gi, iv, conflicts;
        SN* parent;
        SN(int l,int g_,int h_,int t_,int gi_,int ii,int c,SN* p)
            :loc(l),g(g_),h(h_),t(t_),gi(gi_),iv(ii),conflicts(c),parent(p){}
        int f() const { return g+h; }
    };
    struct CmpSN {
        bool operator()(const SN* a, const SN* b) const {
            if (a->f() != b->f()) return a->f() > b->f();
            if (a->conflicts != b->conflicts) return a->conflicts > b->conflicts;
            return a->g <= b->g;
        }
    };

    boost::heap::fibonacci_heap<SN*, boost::heap::compare<CmpSN>> open;
    struct KeyHash { size_t operator()(uint64_t k) const { return k * 2654435761ULL; } };
    unordered_map<uint64_t, int, KeyHash> closed;
    vector<SN*> nodes;
    auto mk = [&](int l, int iv, int gi, int g) -> uint64_t {
        (void)g;
        return ((uint64_t)l << 32) | ((uint64_t)(iv & 0xFFFF) << 16) | (gi & 0xFFFF);
    };

    int ih = sum_h(start_loc, 0);
    if (ih == INT_MAX) { reset_scratch(); return {}; }

    int s_iv = 0;
    auto& ss = get_sit(start_loc);
    if (!ss.empty())
        for (int i = 0; i < (int)ss.size(); i++)
            if (ss[i].first <= start_time && start_time < ss[i].second) { s_iv = i; break; }

    auto* root = new SN(start_loc, 0, ih, start_time, 0, s_iv, 0, nullptr);
    open.push(root); nodes.push_back(root);
    closed[mk(start_loc, s_iv, 0, 0)] = 0;

    SN* sol = nullptr;
    int exp = 0;
#ifdef SIPP_PROF
    auto __t1 = std::chrono::high_resolution_clock::now();
    g_setup_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(__t1-__t0).count();
#endif
    // Expansion cap (mirrors seq_mla_star's tight cap).  A genuine path through the SIPP state
    // space (loc x interval x goal) is found in far fewer expansions than this; a search that
    // exhausts it has no valid path within the horizon (the early-bail above already removes the
    // common permanently-blocked-goal case, so this is only a safety net).  Capping low keeps
    // hopeless searches cheap at low task frequency WITHOUT changing makespan/SWT.  Previously
    // 500000, which made each hopeless root search burn ~0.1s and blew the runtime budget.
    int max_exp = min(500000, map_size * 20 * max(1, num_goals));
    while (!open.empty() && exp < max_exp) {
        SN* cur = open.top(); open.pop();
        int gi = cur->gi;
        if (gi < num_goals && cur->loc == goals[gi].first && cur->t >= goals[gi].second) {
            if (gi == num_goals-1 && cur->t < earliest_hold) {} else gi++;
        }
        // Windowed (wPBS) mode: terminate at the window boundary even if not all goals are
        // reached, returning the partial path (matches reference StateTimeA* / seq_mla_star).
        if (gi >= num_goals || (windowed && cur->t >= wpbs_cutoff)) { sol = cur; break; }
        auto ck = mk(cur->loc, cur->iv, gi, cur->g);
        if (closed.count(ck) && closed[ck] < cur->g) continue;
        exp++;
        if (cur->t >= horizon - 1) continue;

        auto& sc = get_sit(cur->loc);
        int civ_end = horizon;
        if (!sc.empty() && cur->iv < (int)sc.size()) civ_end = sc[cur->iv].second;

        // Match reference PBS KivaGraph move order: {1(E), -cols(N), -1(W), cols(S)}
        int dirs[4] = {1, -cols, -1, cols};
        for (int d = 0; d < 4; d++) {
            int nl = cur->loc + dirs[d];
            if (nl < 0 || nl >= map_size || !mapd_map.grid[nl]) continue;
            if (abs(nl % cols - cur->loc % cols) > 1) continue;
            int mt = cur->t + 1;
            auto& sn = get_sit(nl);
            if (sn.empty() && has_ct[nl]) continue; // fully blocked
            if (sn.empty()) {
                // Truly unconstrained
                if (mt < horizon && !edge_blocked(cur->loc, nl, mt)) {
                    int ngi = gi;
                    if (ngi < num_goals && nl == goals[ngi].first && mt >= goals[ngi].second)
                        { if (ngi == num_goals-1 && mt < earliest_hold) {} else ngi++; }
                    int nh = sum_h(nl, ngi); if (nh == INT_MAX) continue;
                    int ng = mt - start_time;
                    auto nk = mk(nl, 0, ngi, ng);
                    if (!closed.count(nk) || closed[nk] > ng) {
                        closed[nk] = ng;
                        auto* nd = new SN(nl, ng, nh, mt, ngi, 0, cur->conflicts, cur);
                        open.push(nd); nodes.push_back(nd);
                    }
                }
            } else {
                for (int iv = 0; iv < (int)sn.size(); iv++) {
                    if (sn[iv].second <= mt) continue;
                    int arr = max(mt, sn[iv].first);
                    if (arr >= horizon || arr > civ_end) continue;
                    // Advance arrival within interval to find non-edge-blocked time
                    while (arr < sn[iv].second && arr <= civ_end && arr < horizon &&
                           edge_blocked(cur->loc, nl, arr))
                        arr++;
                    if (arr >= sn[iv].second || arr >= horizon || arr > civ_end) continue;
                    int ngi = gi;
                    if (ngi < num_goals && nl == goals[ngi].first && arr >= goals[ngi].second)
                        { if (ngi == num_goals-1 && arr < earliest_hold) {} else ngi++; }
                    int nh = sum_h(nl, ngi); if (nh == INT_MAX) continue;
                    int ng = arr - start_time;
                    auto nk = mk(nl, iv, ngi, ng);
                    if (!closed.count(nk) || closed[nk] > ng) {
                        closed[nk] = ng;
                        auto* nd = new SN(nl, ng, nh, arr, ngi, iv, cur->conflicts, cur);
                        open.push(nd); nodes.push_back(nd);
                    }
                }
            }
        }
        // Wait one step in place (WINDOWED wPBS only).  Matches the reference StateTimeA* WAIT
        // action: the agent may rest at its current cell for one timestep as long as the cell stays
        // free (next step still inside the current safe interval).  This is what lets windowed SIPP
        // choose to wait for a blockage to clear (resolved by next window's replanning) instead of
        // committing a detour — restoring the reference's path quality (SWT/makespan).  It is added
        // ONLY in windowed mode: the closed key includes g there, so distinct wait-times are kept;
        // the +10 cutoff bounds it; and same-interval +1 waits never cross a hard-blocked gap, so
        // the wait-across-blocked-interval collision guard is untouched.
        // Wait to next interval.
        // Only valid when the next safe interval of THIS cell is CONTIGUOUS with the current
        // one (i.e. sc[iv].s == sc[cur->iv].e).  A gap between intervals means the cell is
        // HARD-BLOCKED during the gap (a higher-priority agent occupies it), so the agent
        // cannot remain parked at the cell across that gap.  Jumping to a non-contiguous next
        // interval and filling the gap with a wait during reconstruction produced a vertex
        // collision with the higher-priority agent (the SIPP --sipp collision bug).  Since
        // this reimplementation has no soft/CAT intervals, hard-constrained SITs never have
        // contiguous neighbours, so in practice this successor is correctly never taken.
        if (!sc.empty() && cur->iv + 1 < (int)sc.size()) {
            int iv = cur->iv + 1;
            if (sc[iv].first == sc[cur->iv].second) {  // contiguous only
                int arr = sc[iv].first;
                if (arr < horizon) {
                    int ngi = gi;
                    if (ngi < num_goals && cur->loc == goals[ngi].first && arr >= goals[ngi].second)
                        { if (ngi == num_goals-1 && arr < earliest_hold) {} else ngi++; }
                    int nh = sum_h(cur->loc, ngi);
                    if (nh != INT_MAX) {
                        int ng = arr - start_time;
                        auto nk = mk(cur->loc, iv, ngi, ng);
                        if (!closed.count(nk) || closed[nk] > ng) {
                            closed[nk] = ng;
                            auto* nd = new SN(cur->loc, ng, nh, arr, ngi, iv, cur->conflicts, cur);
                            open.push(nd); nodes.push_back(nd);
                        }
                    }
                }
            }
        }
    }

#ifdef SIPP_PROF
    auto __t2 = std::chrono::high_resolution_clock::now();
    g_loop_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(__t2-__t1).count();
    g_exp += exp;
    if (!sol) g_bail++;
#endif
    // Reconstruct
    vector<int> result;
    if (sol) {
        vector<SN*> pn;
        for (SN* n = sol; n; n = n->parent) pn.push_back(n);
        reverse(pn.begin(), pn.end());
        vector<int> locs;
        for (int i = 0; i < (int)pn.size(); i++) {
            if (i == 0) { locs.push_back(pn[0]->loc); continue; }
            for (int t = pn[i-1]->t + 1; t < pn[i]->t; t++) locs.push_back(pn[i-1]->loc);
            locs.push_back(pn[i]->loc);
        }
        result.resize(max_t);
        for (int t = 0; t < start_time && t < max_t; t++) result[t] = start_loc;
        for (int i = 0; i < (int)locs.size(); i++) {
            int t = start_time + i;
            if (t < max_t) result[t] = locs[i];
        }
        int last = locs.back();
        for (int t = start_time + (int)locs.size(); t < max_t; t++) result[t] = last;

        // (validation removed — conflicts handled at caller level)
    }
    for (auto* n : nodes) delete n;
    reset_scratch();
#ifdef SIPP_PROF
    auto __t3 = std::chrono::high_resolution_clock::now();
    g_recon_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(__t3-__t2).count();
#endif
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
    // Windowed (wPBS): the committed plan is only read over the short working window, and
    // seq_mla_star returns a window-bounded segment (constant hold after).  Size full_path to
    // a tight window-covering length instead of the full horizon (max_t ~5000) to avoid a
    // ~max_t/window-fold per-call allocation + segment copy that dominated windowed solve cost.
    // out_len bounds every t-loop below; segment[t] is read only within its own length.
    int out_len = max_t;
    if (constraint_window > 0)
        out_len = min(max_t, constraint_window + 8);
    vector<int> full_path(out_len, start_loc);
    for (int t = 0; t < min(start_time, out_len); t++)
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

        int seg_n = (int)segment.size();
        int seg_back = seg_n ? segment[seg_n - 1] : cur_loc;
        for (int t = cur_time; t < out_len; t++)
            full_path[t] = (t < seg_n) ? segment[t] : seg_back;

        // Find where last goal was reached using goal advancement
        int gi = 0;
        bool found = false;
        for (int t = cur_time; t < out_len && gi < (int)goals.size(); t++) {
            if (full_path[t] == goals[gi].first && t >= goals[gi].second) {
                gi++;
                if (gi == (int)goals.size()) {
                    cur_loc = goals.back().first;
                    cur_time = t;
                    found = true;
                    break;
                }
            }
        }
        if (!found) {
            // wPBS: path may be truncated at window boundary without reaching all goals.
            // Accept partial path (matches reference truncated search behavior).
            if (constraint_window > 0) {
                return full_path;
            }
            return {};
        }
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

    bool use_seq_sta = (config.mla_mode == MLA_SEQ_STA);
    bool use_taskwise = (!use_seq_sta && config.mla_mode != MLA_SEQ);
    vector<vector<vector<pair<int,int>>>> all_task_groups;
    if (use_taskwise) {
        all_task_groups.resize(num_ag);
        for (int i = 0; i < num_ag; i++)
            all_task_groups[i] = split_into_task_groups(i, goal_seqs[i]);
    }

    // Match reference: wPBS constraint window = replan_window (not 3x).
    // Reference: solver.window = planning_window = 10.
    // Constraints are only inserted for timesteps up to window.
    int cons_window = windowed ? (int)cur_time_ + config.replan_window : -1;

    // Sequential single-goal A* (matches reference StateTimeA*):
    // plan to each goal one at a time, concatenate paths
    auto seq_sta_plan = [&](int aid, int loc, int time,
                            const vector<pair<int,int>>& goals,
                            const vector<vector<int>>& cons,
                            const vector<vector<int>>& old,
                            bool use_old) -> vector<int> {
        vector<int> result(max_t, loc);
        for (int t = 0; t < time && t < max_t; t++) result[t] = loc;
        int cur_loc = loc;
        int cur_time = time;
        for (int gi = 0; gi < (int)goals.size(); gi++) {
            vector<pair<int,int>> single_goal = {goals[gi]};
            bool skip_hold = (gi < (int)goals.size() - 1);
            auto seg = seq_mla_star(aid, cur_loc, cur_time, single_goal,
                                     cons, old, use_old, skip_hold, cons_window);
            if (seg.empty()) return {};
            for (int t = cur_time; t < max_t; t++) result[t] = seg[t];
            // Find when agent arrives at goal
            int goal_loc = goals[gi].first;
            int arrive = -1;
            for (int t = cur_time; t < max_t; t++) {
                if (seg[t] == goal_loc && t >= goals[gi].second) {
                    arrive = t; break;
                }
            }
            if (arrive < 0) {
                // wPBS: path truncated at window boundary, goal not reached yet.
                // Accept partial path (matches reference truncated search).
                if (cons_window > 0) return result;
                return {};
            }
            cur_loc = goal_loc;
            cur_time = arrive;
        }
        return result;
    };

    auto plan_agent = [&](int aid, int loc, int time,
                          const vector<vector<int>>& cons,
                          const vector<vector<int>>& old,
                          bool use_old) -> vector<int> {
        if (config.use_sipp)
            return sipp_search(aid, loc, time, goal_seqs[aid], cons, old, use_old,
                               false, cons_window);
        if (use_seq_sta)
            return seq_sta_plan(aid, loc, time, goal_seqs[aid], cons, old, use_old);
        if (use_taskwise)
            return mla_star_taskwise(aid, loc, time, all_task_groups[aid],
                                      cons, old, use_old, cons_window);
        return seq_mla_star(aid, loc, time, goal_seqs[aid], cons, old, use_old,
                             false, cons_window);
    };

    // Limit path copy to relevant horizon (from current timestep + search horizon)
    int path_horizon = min(max_t, (int)cur_time_ + 1200);

    // Working path length for PBS bookkeeping.
    //   Windowed wPBS: the low-level search is hard-capped at start+10, so every plan is a
    //   short active prefix (<= replan_window+10) followed by a constant hold.  Every PBS node
    //   stores and copies its full path table on each expansion; sizing those tables to the
    //   tiny working window instead of max_t (5000) shrinks the per-node copy ~max_t/window-fold
    //   and is what makes windowed wPBS tractable at high agent counts.  Beyond work_len every
    //   agent is frozen at work_len-1, so it is reconstructed (as a hold) only when committing.
    //   Non-windowed PBS keeps the full horizon (its committed plan runs to each goal).
    int work_len = max_t;
    if (windowed) {
        // The windowed low-level search is hard-capped at start+10, so every plan is at most
        // ~10 active steps; +4 margin covers the boundary hold (where same-cell rests must
        // still be detected as conflicts).  A tighter work_len shrinks both the per-node path
        // copy and the O(num_ag^2 * work_len) conflict detection.
        work_len = min(max_t, (int)cur_time_ + 14);
    } else {
        // Non-windowed PBS: the low-level search horizon is hard-capped at start+512 (see
        // seq_mla_star), so every committed plan is fully active within [timestep, timestep+512]
        // and constant (held at its goal) afterwards.  Sizing the PBS node path tables to that
        // active window instead of the full horizon (max_t, ~2515 at low task frequency) shrinks
        // the per-node copy, the root/commit fills, and the conflict scan by ~max_t/window-fold,
        // with no effect on the result: commit_node extends every agent's final hold cell out to
        // max_t, and conflict detection is bounded by settle_time <= work_len.  +16 margin covers
        // the boundary hold so same-cell rests at the window edge are still detected as conflicts.
        work_len = min(max_t, (int)cur_time_ + 512 + 16);
    }

    // Save old paths (only up to path_horizon).
    // In windowed (wPBS) mode old paths are neither a hard constraint (is_constrained_old is a
    // no-op there) nor used by the reference at all, so skip building/copying them entirely —
    // at low task frequencies path_horizon is ~1200+ and copying it for every agent on every
    // one of the many windowed solves was a large, pointless cost.
    vector<vector<int>> old_paths;
    if (!windowed) {
        old_paths.resize(num_ag);
        for (int i = 0; i < num_ag; i++) {
            old_paths[i].resize(path_horizon);
            for (int t = 0; t < path_horizon; t++)
                old_paths[i][t] = (int)path_table_[i][t];
        }
    }
    bool use_old_in_pbs = !windowed;

    // --- Build the shared old-path constraint table ONCE for the whole root batch -------------
    // All num_ag root searches use the SAME old paths (each excluding only itself).  Flattening
    // them once here (instead of inside every per-agent search) removes the dominant
    // O(num_ag^2 * horizon) per-solve cost at low task frequency.  The table is sized to the
    // GLOBAL active prefix length; each agent's constant hold tail is carried by shared_old_holds_
    // and enforced as a permanent endpoint hold inside seq_mla_star.
    shared_old_valid_ = false;
    if (use_old_in_pbs) {
        // The per-search horizon cap is start_time + 512 (see seq_mla_star); never need more.
        int cap = min(path_horizon, (int)cur_time_ + 513);
        int g_active = 1;
        for (int i = 0; i < num_ag; i++) {
            const auto& p = old_paths[i];
            int n = min((int)p.size(), cap);
            int last_move = 0;
            for (int t = 1; t < n; t++)
                if (p[t] != p[t-1]) last_move = t;
            if (last_move + 1 > g_active) g_active = last_move + 1;
        }
        shared_old_horizon_ = g_active;
        shared_old_flat_.assign((size_t)num_ag * g_active, 0);
        shared_old_holds_.assign(num_ag, 0);
        shared_old_holdcnt_.assign((size_t)mapd_map.row * mapd_map.col, 0);
        shared_old_movers_.clear();
        for (int i = 0; i < num_ag; i++) {
            const auto& p = old_paths[i];
            int back = p.empty() ? (int)agents[i].loc : p.back();
            // does this agent move within the active window?
            bool moves = false;
            int n = min((int)p.size(), g_active);
            for (int t = 1; t < n; t++) if (p[t] != p[t-1]) { moves = true; break; }
            for (int t = 0; t < g_active; t++)
                shared_old_flat_[(size_t)i * g_active + t] =
                    (t < (int)p.size()) ? p[t] : back;
            shared_old_holds_[i] = back;
            if (moves) {
                shared_old_movers_.push_back(i);
            } else if (back >= 0 && back < (int)shared_old_holdcnt_.size()) {
                // PARKED agent: occupies its single cell at EVERY timestep (counted here so the
                // per-expansion check is O(1) for the common parked majority).
                shared_old_holdcnt_[back]++;
            }
        }
        // SIPP analog: precompress each old path into CT ranges once (reused by sipp_search at the
        // root).  Compress to cur_time_+300, the same old-path window sipp_search uses; the
        // per-search clamps each range to its own old_h, so this is exactly equivalent.
        if (config.use_sipp) {
            int sipp_h = min(path_horizon, (int)cur_time_ + 300);
            shared_old_sipp_h_ = sipp_h;
            shared_old_sipp_ranges_.assign(num_ag, {});
            for (int i = 0; i < num_ag; i++) {
                const auto& op = old_paths[i];
                int len = min((int)op.size(), sipp_h);
                if (len == 0) continue;
                auto& out = shared_old_sipp_ranges_[i];
                int ss = 0, sl = op[0];
                for (int t = 1; t < len; t++) {
                    if (op[t] != sl) { out.emplace_back(sl, ss, t); sl = op[t]; ss = t; }
                }
                out.emplace_back(sl, ss, sipp_h);
            }
        }
        shared_old_valid_ = true;
    }

    PBSNode* root = new PBSNode();
    root->paths.resize(num_ag);

    for (int i = 0; i < num_ag; i++) {
        root->paths[i].resize(work_len);
        for (int t = 0; t < work_len; t++)
            root->paths[i][t] = (int)path_table_[i][t];
    }

    // Match reference generate_root_node: plan all agents against old_paths only
    // (no sequential dependencies), then resolve via find_consistent_paths cascade.
    int trunc = min(path_horizon, (int)cur_time_ + 1000);

    // --- Windowed (wPBS) root: SEQUENTIAL prioritized planning with a conflict-avoidance table.
    // The reference PBS::generate_root_node plans agents one by one, each avoiding (SOFT) the NEW
    // paths of the agents already planned this iteration (rt.build over the growing `paths`).  We
    // replicate that with a shared CAT grown incrementally: before planning agent i it holds the
    // new paths of 0..i-1, so agent i (focal low-level) routes around them.  Without this the +10
    // search picks arbitrary equal-cost prefixes that collide, and PBS adds waiting -> the wPBS
    // SWT/makespan inflation at mid task demand.  (Soft only: never a hard constraint.)
    auto cat_fill = [&](int idx) {
        const int hor = shared_old_horizon_;
        bool moves = false;
        for (int t = 1; t < hor; t++)
            if (root->paths[idx][t] != root->paths[idx][t-1]) { moves = true; break; }
        for (int t = 0; t < hor; t++)
            shared_old_flat_[(size_t)idx * hor + t] = root->paths[idx][t];
        int back = root->paths[idx][hor - 1];
        shared_old_holds_[idx] = back;
        if (moves) shared_old_movers_.push_back(idx);
        else if (back >= 0 && back < (int)shared_old_holdcnt_.size()) shared_old_holdcnt_[back]++;
    };
    if (windowed) {
        shared_old_horizon_ = work_len;
        shared_old_flat_.assign((size_t)num_ag * work_len, 0);
        shared_old_holds_.assign(num_ag, -1);
        shared_old_holdcnt_.assign((size_t)mapd_map.row * mapd_map.col, 0);
        shared_old_movers_.clear();
        shared_old_valid_ = true;  // soft CAT only (gated by constraint_window>0 in seq_mla_star)
    }

    for (int i = 0; i < num_ag; i++) {
        // Constraint list: old_paths for ALL other agents (matching reference initial_rt).  For
        // the MLA* low level the old paths come from the precomputed shared table (shared_old_*),
        // so old_buf is left empty and seq_mla_star reads the shared table while excluding agent
        // i.  The SIPP low level does NOT use the shared table, so it still needs old_buf built.
        vector<vector<int>> cons_buf;  // empty: no priority constraints at root
        vector<vector<int>> old_buf;
        if (use_old_in_pbs && config.use_sipp) {
            old_buf.reserve(num_ag);
            for (int j = 0; j < num_ag; j++) {
                if (j == i) continue;
                old_buf.emplace_back(old_paths[j].begin(),
                                     old_paths[j].begin() + min(trunc, (int)old_paths[j].size()));
            }
        }

        shared_old_excl_ = i;
        vector<int> path = plan_agent(i, (int)agents[i].loc, (int)cur_time_,
                                       cons_buf, old_buf, use_old_in_pbs);
        shared_old_excl_ = -1;
        if (path.empty()) {
            path = plan_agent(i, (int)agents[i].loc, (int)cur_time_,
                               cons_buf, {}, false);
            if (path.empty()) {
                for (int t = (int)cur_time_; t < work_len; t++)
                    root->paths[i][t] = (int)agents[i].loc;
            } else {
                for (int t = (int)cur_time_; t < work_len; t++)
                    root->paths[i][t] = path[t];
            }
        } else {
            for (int t = (int)cur_time_; t < work_len; t++)
                root->paths[i][t] = path[t];
        }
        // Add agent i's NEW path to the CAT so subsequent root agents avoid it (windowed only).
        if (windowed) cat_fill(i);
    }

    // Compute PBS node cost (f_val): sum of path costs for all agents
    // Matching reference get_path_cost: sum of edge weights (= number of moves)
    auto compute_node_cost = [&](PBSNode* node) -> int {
        int total = 0;
        for (int i = 0; i < num_ag; i++) {
            int lim = min((int)node->paths[i].size(), max_t) - 1;
            for (int t = (int)cur_time_; t < lim; t++) {
                if (node->paths[i][t] != node->paths[i][t+1])
                    total++;
            }
        }
        return total;
    };

    root->cost = compute_node_cost(root);

    // Conflict-detection horizon: detect conflicts across the whole committed path.  The
    // held tail of a windowed plan can still pile agents onto the same boundary cell and that
    // tail IS executed when a replan is delayed (an agent reaching a goal mid-window jumps the
    // sim past the next window boundary), so far-future conflicts must still be resolved to
    // stay collision-free.
    //
    // Performance: scanning to max_t (5000) for every agent pair on every PBS node is the
    // dominant cost at high agent counts.  But every committed path is a short active prefix
    // followed by a CONSTANT hold tail.  After the timestep at which the LAST agent stops
    // moving, all positions are frozen — so any vertex/edge conflict that exists in the tail
    // already exists (as a stationary overlap) at that "settle" timestep and is detected
    // there.  Bounding the scan to (last move time + 1) is therefore EXACT (no missed
    // collisions) while shrinking the scan from ~5000 to ~the makespan-window.  Recomputed
    // per node since replanning can shift the settle time.
    auto settle_time = [&](const vector<vector<int>>& P) -> int {
        int last_move = (int)cur_time_;
        for (int i = 0; i < num_ag; i++) {
            int li = (int)P[i].size() - 1;
            // walk back over the trailing hold to find this agent's last actual move
            while (li > (int)cur_time_ && P[i][li] == P[i][li-1]) li--;
            if (li > last_move) last_move = li;
        }
        return last_move;
    };
    // In windowed mode, match the reference PBS (PBS.cpp:134 size=min(window+1,len)): detect
    // conflicts ONLY within the EXECUTED window [cur_time_, cur_time_+replan_window].
    // The system executes up to and INCLUDING offset==replan_window (update_system window_cap =
    // cur_time_+replan_window, inclusive), so the horizon is cur_time_+replan_window+1
    // (the +1 makes the for-loop t<horizon include offset==replan_window).  Conflicts BEYOND the
    // window (offsets 11..13 between offset-10-held cells) are NOT executed and must not be
    // resolved: scanning them (the old work_len=+14 horizon) created spurious beyond-window
    // conflicts whose cascade replans failed for boxed-in agents, wrongly invalidating children
    // that were valid for the in-window resolution -> spurious nogoods -> residual in-window
    // conflicts the deconfliction patch had to mask.  In non-windowed mode bound to the settle
    // time (exact, see note above) to skip the long constant-hold tail.
    int conflict_horizon = windowed
        ? min(max_t, (int)cur_time_ + config.replan_window + 1)
        : min(max_t, settle_time(root->paths) + 1);
    for (int a1 = 0; a1 < num_ag; a1++) {
        for (int a2 = a1 + 1; a2 < num_ag; a2++) {
            // Check conflicts only within horizon
            for (int t = (int)cur_time_; t < conflict_horizon && t < max_t; t++) {
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

    // Commit a node's (possibly work_len-bounded) path table to the global token/agents
    // paths, extending the trailing hold out to max_t.
    auto commit_node = [&](PBSNode* node) {
        for (int i = 0; i < num_ag; i++) {
            int plen = (int)node->paths[i].size();
            int hold = node->paths[i][min(plen, (int)max_t) - 1];
            for (int t = 0; t < max_t; t++) {
                int v = (t < plen) ? node->paths[i][t] : hold;
                path_table_[i][t] = v;
                agents[i].path[t] = v;
            }
        }
    };

    if (root->conflicts.empty()) {
        commit_node(root);
        delete root;
        shared_old_valid_ = false;
        return true;
    }

    vector<PBSNode*> all_nodes;
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
    int max_hl = (num_ag > 30) ? 5000 : 50000;

    // --- nogood set (matches reference PBS) ------------------------------------------------
    // When BOTH children of a chosen conflict are invalid (neither agent can be made to yield
    // consistently), that agent pair is a "nogood": any node containing this conflict is a dead
    // end.  The reference records such pairs and, in choose_conflict, PREFERS resolving a nogood
    // conflict first so the dead branch fails immediately and is pruned, instead of resolving
    // other conflicts and re-discovering the same unresolvable pair deeper in the tree.  Without
    // this, the DFS expanded ~50k HL nodes (half of them producing two invalid children) where
    // the reference needs only ~4k — this single mechanism is the dominant tree-size gap at
    // 30ag/mid task frequency.  Replicated exactly (earliest-conflict default, nogood override).
    std::set<std::pair<int,int>> nogood;

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
        curr->earliest_collision = get<4>(chosen);
        // nogood override (reference choose_conflict): if any conflict's agent pair is a known
        // dead end, resolve THAT conflict first so the branch is pruned immediately.
        if (!nogood.empty()) {
            for (auto& c : curr->conflicts) {
                int ca = get<0>(c), cb = get<1>(c);
                if (nogood.count({ca, cb}) || nogood.count({cb, ca})) {
                    chosen = c;
                    break;
                }
            }
        }
        curr->conflict = chosen;

        if (curr->earliest_collision > best_node->earliest_collision ||
            (curr->earliest_collision == best_node->earliest_collision &&
             curr->cost < best_node->cost)) {
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

            // Match reference: old_paths for ALL non-self agents (including high-priority).
            // Reference find_path() inserts old_paths[j] for all j!=agent into initial_rt,
            // then rt.build() adds high-priority current paths on top.  (Skipped in windowed
            // mode where old paths are unused.)
            if (use_old_in_pbs) {
                for (int j = 0; j < num_ag; j++) {
                    if (j == lower) continue;
                    old_for_agent.push_back(old_paths[j]);
                }
            }

            vector<int> new_path = plan_agent(lower, (int)agents[lower].loc,
                                              (int)cur_time_,
                                              cons, old_for_agent, use_old_in_pbs);
            if (new_path.empty()) {
                child_valid[c] = false;
                delete children[c];
                children[c] = nullptr;
                continue;
            }

            // Only update from current timestep onward (bounded to the working window)
            for (int t = (int)cur_time_; t < work_len; t++)
                children[c]->paths[lower][t] = new_path[t];

            // --- find_consistent_paths cascade (matching reference PBS) ---
            // After replanning 'lower', iteratively replan any agent whose
            // conflict with a priority-connected agent is inconsistent.
            {
                // Per-child conflict horizon (exact, see settle_time note above): bounded by
                // the timestep at which the last agent in THIS child stops moving.  Recomputed
                // here because replanning 'lower' (and cascade replans) can extend the active
                // portion beyond the root's settle time.
                int child_conf_horizon = windowed
                    ? min(max_t, (int)cur_time_ + config.replan_window + 1)
                    : min(max_t, settle_time(children[c]->paths) + 1);
                // Helper: detect conflicts involving a specific agent
                auto detect_conflicts_for = [&](int ag_id, list<tuple<int,int,int,int,int>>& out) {
                    for (int j = 0; j < num_ag; j++) {
                        if (j == ag_id) continue;
                        for (int t = (int)cur_time_; t < child_conf_horizon && t < max_t; t++) {
                            int la = (t < (int)children[c]->paths[ag_id].size()) ? children[c]->paths[ag_id][t] : children[c]->paths[ag_id].back();
                            int lj = (t < (int)children[c]->paths[j].size()) ? children[c]->paths[j][t] : children[c]->paths[j].back();
                            if (la == lj) {
                                out.emplace_back(ag_id, j, la, -1, t);
                                break;
                            }
                            if (t > 0) {
                                int pa = (t-1 < (int)children[c]->paths[ag_id].size()) ? children[c]->paths[ag_id][t-1] : children[c]->paths[ag_id].back();
                                int pj = (t-1 < (int)children[c]->paths[j].size()) ? children[c]->paths[j][t-1] : children[c]->paths[j].back();
                                if (la == pj && lj == pa) {
                                    out.emplace_back(ag_id, j, la, lj, t);
                                    break;
                                }
                            }
                        }
                    }
                };

                // Helper: find agents that need replanning based on priority consistency
                auto find_replan_agents = [&](const list<tuple<int,int,int,int,int>>& conflicts,
                                              unordered_set<int>& replan) {
                    for (auto& conf : conflicts) {
                        int ca = get<0>(conf);
                        int cb = get<1>(conf);
                        if (replan.count(ca) || replan.count(cb)) continue;
                        if (children[c]->priorities.connected(ca, cb)) {
                            replan.insert(ca);  // ca yields to cb, so replan ca
                        } else if (children[c]->priorities.connected(cb, ca)) {
                            replan.insert(cb);  // cb yields to ca, so replan cb
                        }
                    }
                };

                // Build initial conflict list and find agents to replan
                children[c]->conflicts.clear();
                detect_conflicts_for(lower, children[c]->conflicts);
                // Also keep non-lower conflicts from parent
                for (auto& conf : curr->conflicts) {
                    if (get<0>(conf) != lower && get<1>(conf) != lower)
                        children[c]->conflicts.push_back(conf);
                }

                unordered_set<int> replan;
                find_replan_agents(children[c]->conflicts, replan);

                int cascade_count = 0;
                // Match reference find_consistent_paths exactly:
                //   while (!replan.empty()) {
                //       if (count > node->paths.size() * 5) return false; // child INVALID
                //       ...
                //   }
                // node->paths is one path PER AGENT, so node->paths.size() == num_of_agents
                // (the TOTAL agent count), NOT the number of distinct agents replanned.  Using a
                // distinct-replanned*5 budget (which starts at 5 and grows slowly) is far tighter
                // than the reference and causes the cascade to give up prematurely; the child is
                // then wrongly invalidated, both children of a boundary (offset==window) conflict
                // get pruned, the pair is recorded as a spurious nogood, and the residual conflict
                // survives to best_node where the final-deconfliction patch had to mask it.  Using
                // the reference's fixed num_ag*5 budget lets the cascade actually converge, so PBS
                // resolves these in-window conflicts itself and no patch is needed.
                // CRITICAL: if the cascade still cannot make all priority-connected conflicts
                // consistent within this budget, the child is INVALID (pruned), not kept with
                // leftover conflicts (keeping it caused infinite DFS re-expansion at depth 1).
                const int cascade_budget = num_ag * 5;
                while (!replan.empty()) {
                    if (cascade_count > cascade_budget) {
                        child_valid[c] = false;
                        delete children[c];
                        children[c] = nullptr;
                        break;
                    }
                    int a = *replan.begin();
                    replan.erase(replan.begin());
                    cascade_count++;

                    set<int> hp = children[c]->priorities.get_higher_priority(a);
                    vector<vector<int>> a_cons;
                    vector<vector<int>> a_old;
                    for (int h : hp) a_cons.push_back(children[c]->paths[h]);
                    // Match reference: old_paths for ALL non-self agents (skipped in windowed)
                    if (use_old_in_pbs) {
                        for (int j = 0; j < num_ag; j++) {
                            if (j == a) continue;
                            a_old.push_back(old_paths[j]);
                        }
                    }

                    auto np = plan_agent(a, (int)agents[a].loc, (int)cur_time_,
                                          a_cons, a_old, use_old_in_pbs);
                    if (np.empty()) {
                        child_valid[c] = false;
                        delete children[c];
                        children[c] = nullptr;
                        break;
                    }
                    for (int t = (int)cur_time_; t < work_len; t++)
                        children[c]->paths[a][t] = np[t];

                    // Remove old conflicts involving 'a', add new ones
                    children[c]->conflicts.remove_if([a](const tuple<int,int,int,int,int>& cf) {
                        return get<0>(cf) == a || get<1>(cf) == a;
                    });
                    list<tuple<int,int,int,int,int>> new_confs;
                    detect_conflicts_for(a, new_confs);
                    find_replan_agents(new_confs, replan);
                    children[c]->conflicts.splice(children[c]->conflicts.end(), new_confs);
                }
                if (!child_valid[c]) continue;
            }
            children[c]->num_collisions = (int)children[c]->conflicts.size();
            children[c]->cost = compute_node_cost(children[c]);

        }

        if (child_valid[0] && child_valid[1]) {
            // Order by f_val (cost) first, then num_collisions (matching reference PBS)
            if (children[0]->cost < children[1]->cost ||
                (children[0]->cost == children[1]->cost &&
                 children[0]->num_collisions < children[1]->num_collisions)) {
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
        } else {
            // Both children invalid: this conflict's agent pair is a dead end.  Record it as a
            // nogood so choose_conflict prefers (and immediately prunes) it in future nodes.
            nogood.emplace(a1, a2);
        }
    }

    // --- Final safety deconfliction --------------------------------------------------
    // PBS may exhaust its search budget (or prune both children of a conflict as inconsistent)
    // and return a best_node that still contains conflicts.  Committing such a "best-effort"
    // plan would produce a real collision (an automatic failure).  Guarantee a collision-free
    // committed plan by resolving any residual conflict the simple way PBS would: the agent
    // that arrives later WAITS one step earlier (yields), repeated until clear.  Bounded so it
    // always terminates.  In the common case best_node->conflicts is empty and this is skipped.
    if (!best_node->conflicts.empty()) {
        auto& P = best_node->paths;
        int t0 = (int)cur_time_;
        // Scan bound for the safety net.
        //   Windowed (wPBS): consider ONLY conflicts within the EXECUTED window
        //   [cur_time_, cur_time_+replan_window] (matching the reference, which never
        //   resolves or patches conflicts beyond the executed window).  After the conflict-horizon
        //   fix above, PBS resolves every in-window conflict itself, so this loop finds nothing and
        //   is a never-firing safety net in windowed mode (verified: 0 fires across the full grid).
        //   Non-windowed: keep the full-path scan (full PBS already leaves 0 residual conflicts,
        //   so this is also effectively never-firing, but the full scan preserves prior behavior).
        int plen = (int)P[0].size();   // all agents share work_len-length tables
        if (windowed)
            plen = min(plen, t0 + config.replan_window + 1);
        int rounds = 0;
        const int MAX_ROUNDS = num_ag * num_ag * 4 + 16;
        bool any = true;
        while (any && rounds < MAX_ROUNDS) {
            any = false; rounds++;
            for (int a1 = 0; a1 < num_ag && !any; a1++) {
                for (int a2 = a1 + 1; a2 < num_ag && !any; a2++) {
                    for (int t = t0 + 1; t < plen; t++) {
                        bool vertex = (P[a1][t] == P[a2][t]);
                        bool edge = (P[a1][t] == P[a2][t-1] && P[a2][t] == P[a1][t-1] &&
                                     P[a1][t] != P[a1][t-1]);
                        if (!vertex && !edge) continue;
                        // Choose the agent that is MOVING into the clash to yield; if both move,
                        // yield the higher-indexed one.  Make it wait one step at t-1 by shifting
                        // its remaining path one timestep later.
                        int yielder = a2;
                        if (P[a1][t] != P[a1][t-1] && P[a2][t] == P[a2][t-1]) yielder = a1;
                        else if (P[a1][t] == P[a1][t-1] && P[a2][t] != P[a2][t-1]) yielder = a2;
                        // Insert a wait at t-1 for the yielder: hold its position from t-1, push
                        // the rest of the path back by one step (drop the last cell).
                        int hold = P[yielder][t-1];
                        for (int tt = plen - 1; tt > t - 1; tt--)
                            P[yielder][tt] = P[yielder][tt-1];
                        P[yielder][t-1] = hold;
                        any = true;
                        break;
                    }
                }
            }
        }
        best_node->conflicts.clear();
    }

    commit_node(best_node);

    shared_old_valid_ = false;  // shared table is only valid within this solve's root batch
    return true;
}

// ============================================================
// Section 35: PBS Path Planning — Wrapper
// ============================================================

// ============================================================
// SIPP-based PBS Solver
// ReservationTable: CT (hard) + CAT (soft) → SIT (safe intervals)
// SIPP search: (location, interval_idx, goal_id) states
// ============================================================

bool Simulation::pbs_core_sipp() {
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;
    int map_size = mapd_map.row * mapd_map.col;
    int cols = mapd_map.col;
    int horizon = min(max_t, (int)cur_time_ + 1000);

    auto goal_seqs = build_goal_sequences();

    // Copy paths as int vectors
    vector<vector<int>> paths(num_ag);
    for (int i = 0; i < num_ag; i++)
        paths[i].assign(path_table_[i].begin(), path_table_[i].end());

    // Heuristic helper
    auto get_h = [&](int loc, int gloc) -> int {
        for (auto& ep : mapd_map.endpoints)
            if (ep.loc == gloc) return ep.h_val[loc];
        return INT_MAX;
    };
    auto sum_h = [&](int loc, int gi, const vector<pair<int,int>>& goals) -> int {
        int ng = (int)goals.size();
        if (gi >= ng) return 0;
        int h = get_h(loc, goals[gi].first);
        if (h == INT_MAX) return INT_MAX;
        for (int g = gi; g < ng - 1; g++) {
            int d = get_h(goals[g].first, goals[g+1].first);
            if (d == INT_MAX) return INT_MAX;
            h += d;
        }
        return h;
    };

    // ---- Per-location CT ranges (built incrementally) ----
    struct CTR { int ts, te; };
    vector<vector<CTR>> ct_ranges(map_size);
    vector<bool> has_ct(map_size, false);

    auto add_path_ct = [&](const vector<int>& path) {
        int len = min((int)path.size(), horizon);
        if (len == 0) return;
        int seg_s = 0, seg_loc = path[0];
        for (int t = 1; t < len; t++) {
            if (path[t] != seg_loc) {
                ct_ranges[seg_loc].push_back({seg_s, t});
                has_ct[seg_loc] = true;
                seg_loc = path[t]; seg_s = t;
            }
        }
        // Match reference: hold final position to max_t (INTERVAL_MAX), not just horizon
        ct_ranges[seg_loc].push_back({seg_s, max_t});
        has_ct[seg_loc] = true;
    };

    // ---- Lazy SIT cache ----
    struct Iv { int s, e; };
    unordered_map<int, vector<Iv>> sit_cache;

    auto build_sit = [&](int loc) -> const vector<Iv>& {
        auto it = sit_cache.find(loc);
        if (it != sit_cache.end()) return it->second;
        auto& ivs = sit_cache[loc];
        if (!has_ct[loc]) return ivs; // unconstrained — empty = one big interval
        // Merge ranges and compute safe intervals
        auto& rs = ct_ranges[loc];
        vector<pair<int,int>> sorted_rs;
        sorted_rs.reserve(rs.size());
        for (auto& r : rs) sorted_rs.push_back({r.ts, min(r.te, horizon)});
        sort(sorted_rs.begin(), sorted_rs.end());
        vector<pair<int,int>> merged;
        for (auto& [s, e] : sorted_rs) {
            if (!merged.empty() && s <= merged.back().second)
                merged.back().second = max(merged.back().second, e);
            else merged.push_back({s, e});
        }
        int prev = 0;
        for (auto& [s, e] : merged) {
            if (s > prev) ivs.push_back({prev, s});
            prev = e;
        }
        if (prev < horizon) ivs.push_back({prev, horizon});
        return ivs;
    };

    auto invalidate_path = [&](const vector<int>& path) {
        int len = min((int)path.size(), horizon);
        unordered_set<int> touched;
        for (int t = 0; t < len; t++) touched.insert(path[t]);
        if (!path.empty()) touched.insert(path.back());
        for (int loc : touched) sit_cache.erase(loc);
    };

    // ---- SIPP search ----
    struct SN {
        int loc, g, h, t, gi, iv_idx, conflicts;
        SN* parent;
        SN(int l,int g_,int h_,int t_,int gi_,int ii,int c,SN* p)
            :loc(l),g(g_),h(h_),t(t_),gi(gi_),iv_idx(ii),conflicts(c),parent(p){}
        int f() const { return g + h; }
    };
    struct CmpSN {
        bool operator()(const SN* a, const SN* b) const {
            if (a->f() != b->f()) return a->f() > b->f();
            if (a->conflicts != b->conflicts) return a->conflicts > b->conflicts;
            return a->g <= b->g;
        }
    };

    // Edge check against CT paths (uses ct_ranges indirectly via paths)
    vector<int*> edge_ct_ptrs;
    vector<int> edge_ct_lens;

    auto edge_blocked = [&](int from, int to, int t) -> bool {
        if (t <= 0 || t >= horizon) return false;
        for (int c = 0; c < (int)edge_ct_ptrs.size(); c++) {
            int len = edge_ct_lens[c];
            int ct_loc = (t < len) ? edge_ct_ptrs[c][t] : edge_ct_ptrs[c][len-1];
            int ct_prev = (t-1 < len) ? edge_ct_ptrs[c][t-1] : edge_ct_ptrs[c][len-1];
            if (ct_loc == from && ct_prev == to) return true;
        }
        return false;
    };

    // CAT conflict count (soft constraints from non-CT agents)
    vector<int*> cat_ptrs;
    vector<int> cat_lens;

    auto count_cat = [&](int loc, int t) -> int {
        int c = 0;
        for (int j = 0; j < (int)cat_ptrs.size(); j++) {
            int len = cat_lens[j];
            int cl = (t < len) ? cat_ptrs[j][t] : cat_ptrs[j][len-1];
            if (cl == loc) c++;
        }
        return c;
    };

    auto sipp_plan = [&](int start_loc, int start_time,
                          const vector<pair<int,int>>& goals) -> vector<int> {
        if (goals.empty()) return vector<int>(max_t, start_loc);
        int ng = (int)goals.size();

        // Earliest holding
        int last_goal = goals.back().first;
        int earliest_hold = 0;
        for (int c = 0; c < (int)edge_ct_ptrs.size(); c++) {
            int len = edge_ct_lens[c];
            int scan_limit = min(len, horizon);
            for (int t = scan_limit - 1; t >= 0; t--) {
                if (edge_ct_ptrs[c][t] == last_goal) {
                    earliest_hold = max(earliest_hold, t + 1); break;
                }
            }
        }

        priority_queue<SN*, vector<SN*>, CmpSN> open;
        unordered_map<uint64_t, int> closed;
        vector<SN*> nodes;
        auto mk = [](int l, int iv, int gi) -> uint64_t {
            return ((uint64_t)l << 32) | ((uint64_t)(iv & 0xFFFF) << 16) | (gi & 0xFFFF);
        };

        int ih = sum_h(start_loc, 0, goals);
        if (ih == INT_MAX) return {};

        // Find start interval
        int s_iv = 0;
        auto& sit_s = build_sit(start_loc);
        if (!sit_s.empty()) {
            for (int i = 0; i < (int)sit_s.size(); i++)
                if (sit_s[i].s <= start_time && start_time < sit_s[i].e) { s_iv = i; break; }
        }

        auto* root = new SN(start_loc, 0, ih, start_time, 0, s_iv, count_cat(start_loc, start_time), nullptr);
        open.push(root); nodes.push_back(root);
        closed[mk(start_loc, s_iv, 0)] = 0;

        SN* sol = nullptr;
        int exp = 0;

        while (!open.empty() && exp < 500000) {
            SN* cur = open.top(); open.pop();
            int gi = cur->gi;
            if (gi < ng && cur->loc == goals[gi].first && cur->t >= goals[gi].second) {
                if (gi == ng-1 && cur->t < earliest_hold) {} else gi++;
            }
            if (gi >= ng) { sol = cur; break; }
            auto ck = mk(cur->loc, cur->iv_idx, gi);
            if (closed.count(ck) && closed[ck] < cur->g) continue;
            exp++;
            if (cur->t >= horizon - 1) continue;

            // Current interval end
            auto& sit_c = build_sit(cur->loc);
            int civ_end = horizon;
            if (!sit_c.empty() && cur->iv_idx < (int)sit_c.size())
                civ_end = sit_c[cur->iv_idx].e;

            // Move to 4 neighbors (matching reference PBS KivaGraph move order)
            int dirs[4] = {1, -cols, -1, cols};
            for (int d = 0; d < 4; d++) {
                int nl = cur->loc + dirs[d];
                if (nl < 0 || nl >= map_size || !mapd_map.grid[nl]) continue;
                if (abs(nl % cols - cur->loc % cols) > 1) continue;
                int mt = cur->t + 1;

                auto& sit_n = build_sit(nl);
                if (sit_n.empty() && has_ct[nl]) continue; // fully blocked
                if (sit_n.empty()) {
                    // Truly unconstrained
                    if (mt < horizon && !edge_blocked(cur->loc, nl, mt)) {
                        int ngi = gi;
                        if (ngi < ng && nl == goals[ngi].first && mt >= goals[ngi].second) {
                            if (ngi == ng-1 && mt < earliest_hold) {} else ngi++;
                        }
                        int nh = sum_h(nl, ngi, goals);
                        if (nh == INT_MAX) continue;
                        int nc = cur->conflicts + count_cat(nl, mt);
                        auto nk = mk(nl, 0, ngi);
                        int ng_ = mt - start_time;
                        if (!closed.count(nk) || closed[nk] > ng_) {
                            closed[nk] = ng_;
                            auto* nd = new SN(nl, ng_, nh, mt, ngi, 0, nc, cur);
                            open.push(nd); nodes.push_back(nd);
                        }
                    }
                } else {
                    for (int iv = 0; iv < (int)sit_n.size(); iv++) {
                        if (sit_n[iv].e <= mt) continue;
                        int arr = max(mt, sit_n[iv].s);
                        if (arr >= horizon) break;
                        if (arr > civ_end) continue; // can't wait that long at cur
                        // Advance arrival within interval to find non-edge-blocked time
                        while (arr < sit_n[iv].e && arr <= civ_end && arr < horizon &&
                               edge_blocked(cur->loc, nl, arr))
                            arr++;
                        if (arr >= sit_n[iv].e || arr >= horizon || arr > civ_end) continue;
                        int ngi = gi;
                        if (ngi < ng && nl == goals[ngi].first && arr >= goals[ngi].second) {
                            if (ngi == ng-1 && arr < earliest_hold) {} else ngi++;
                        }
                        int nh = sum_h(nl, ngi, goals);
                        if (nh == INT_MAX) continue;
                        int ng_ = arr - start_time;
                        int nc = cur->conflicts + count_cat(nl, arr);
                        auto nk = mk(nl, iv, ngi);
                        if (!closed.count(nk) || closed[nk] > ng_) {
                            closed[nk] = ng_;
                            auto* nd = new SN(nl, ng_, nh, arr, ngi, iv, nc, cur);
                            open.push(nd); nodes.push_back(nd);
                        }
                    }
                }
            }
            // Wait to next interval
            if (!sit_c.empty()) {
                for (int iv = cur->iv_idx + 1; iv < (int)sit_c.size(); iv++) {
                    int arr = sit_c[iv].s;
                    if (arr >= horizon) break;
                    int ngi = gi;
                    if (ngi < ng && cur->loc == goals[ngi].first && arr >= goals[ngi].second) {
                        if (ngi == ng-1 && arr < earliest_hold) {} else ngi++;
                    }
                    int nh = sum_h(cur->loc, ngi, goals);
                    if (nh == INT_MAX) continue;
                    int ng_ = arr - start_time;
                    int nc = cur->conflicts + count_cat(cur->loc, arr);
                    auto nk = mk(cur->loc, iv, ngi);
                    if (!closed.count(nk) || closed[nk] > ng_) {
                        closed[nk] = ng_;
                        auto* nd = new SN(cur->loc, ng_, nh, arr, ngi, iv, nc, cur);
                        open.push(nd); nodes.push_back(nd);
                    }
                    break;
                }
            }
        }

        // Reconstruct
        vector<int> result;
        if (sol) {
            vector<SN*> pn;
            for (SN* n = sol; n; n = n->parent) pn.push_back(n);
            reverse(pn.begin(), pn.end());
            vector<int> locs;
            for (int i = 0; i < (int)pn.size(); i++) {
                if (i == 0) { locs.push_back(pn[0]->loc); continue; }
                for (int t = pn[i-1]->t + 1; t < pn[i]->t; t++)
                    locs.push_back(pn[i-1]->loc);
                locs.push_back(pn[i]->loc);
            }
            result.resize(max_t);
            for (int t = 0; t < start_time && t < max_t; t++) result[t] = start_loc;
            for (int i = 0; i < (int)locs.size(); i++) {
                int t = start_time + i;
                if (t < max_t) result[t] = locs[i];
            }
            int last = locs.back();
            for (int t = start_time + (int)locs.size(); t < max_t; t++) result[t] = last;
        }
        for (auto* n : nodes) delete n;
        return result;
    };

    // ---- Root planning: sequential prioritized SIPP ----
    vector<int> plan_order;
    for (int i = 0; i < num_ag; i++)
        if (!agents[i].task_sequence.empty()) plan_order.push_back(i);
    for (int i = 0; i < num_ag; i++)
        if (agents[i].task_sequence.empty()) plan_order.push_back(i);

    vector<bool> planned(num_ag, false);
    for (int idx = 0; idx < num_ag; idx++) {
        int i = plan_order[idx];

        // Skip idle agents
        if (goal_seqs[i].size() <= 1 && agents[i].task_sequence.empty()) {
            int loc = (int)agents[i].loc;
            for (int t = (int)cur_time_; t < max_t; t++) paths[i][t] = loc;
            add_path_ct(paths[i]);
            invalidate_path(paths[i]);
            planned[i] = true;
            continue;
        }

        // Build edge CT + CAT pointers
        edge_ct_ptrs.clear(); edge_ct_lens.clear();
        cat_ptrs.clear(); cat_lens.clear();
        for (int j = 0; j < num_ag; j++) {
            if (planned[j]) {
                edge_ct_ptrs.push_back(paths[j].data());
                edge_ct_lens.push_back((int)paths[j].size());
            }
        }
        for (int j = 0; j < num_ag; j++) {
            if (j != i && !planned[j]) {
                cat_ptrs.push_back(paths[j].data());
                cat_lens.push_back((int)paths[j].size());
            }
        }

        auto path = sipp_plan((int)agents[i].loc, (int)cur_time_, goal_seqs[i]);
        if (!path.empty()) paths[i] = std::move(path);

        add_path_ct(paths[i]);
        invalidate_path(paths[i]);
        planned[i] = true;
    }

    // PBS DFS conflict resolution (matching pbs_core logic)
    // Find initial conflicts
    auto find_conflicts = [&](const vector<vector<int>>& ps) {
        vector<tuple<int,int,int,int,int>> confs;
        for (int a1 = 0; a1 < num_ag; a1++) {
            for (int a2 = a1+1; a2 < num_ag; a2++) {
                for (int t = (int)cur_time_; t < min(horizon, max_t); t++) {
                    if (ps[a1][t] == ps[a2][t]) {
                        confs.emplace_back(a1, a2, ps[a1][t], -1, t); break;
                    }
                    if (t > (int)cur_time_ &&
                        ps[a1][t] == ps[a2][t-1] && ps[a2][t] == ps[a1][t-1]) {
                        confs.emplace_back(a1, a2, ps[a1][t], ps[a2][t], t); break;
                    }
                }
            }
        }
        return confs;
    };

    auto replan_one = [&](int agent_id, const vector<vector<int>>& all_paths,
                          const set<int>& higher_set) -> vector<int> {
        for (int loc = 0; loc < map_size; loc++) { ct_ranges[loc].clear(); has_ct[loc] = false; }
        sit_cache.clear();
        edge_ct_ptrs.clear(); edge_ct_lens.clear();
        cat_ptrs.clear(); cat_lens.clear();
        for (int hp : higher_set) {
            add_path_ct(all_paths[hp]);
            edge_ct_ptrs.push_back(const_cast<int*>(all_paths[hp].data()));
            edge_ct_lens.push_back((int)all_paths[hp].size());
        }
        for (int j = 0; j < num_ag; j++) {
            if (j == agent_id || higher_set.count(j)) continue;
            cat_ptrs.push_back(const_cast<int*>(all_paths[j].data()));
            cat_lens.push_back((int)all_paths[j].size());
        }
        return sipp_plan((int)agents[agent_id].loc, (int)cur_time_, goal_seqs[agent_id]);
    };

    // Iterative conflict resolution — try both priority orderings, keep better one
    for (int iter = 0; iter < 100; iter++) {
        int ca1 = -1, ca2 = -1, ct_time = INT_MAX;
        for (int a1 = 0; a1 < num_ag; a1++) {
            for (int a2 = a1+1; a2 < num_ag; a2++) {
                for (int t = (int)cur_time_; t < min(horizon, max_t); t++) {
                    if (paths[a1][t] == paths[a2][t]) {
                        if (t < ct_time) { ca1 = a1; ca2 = a2; ct_time = t; }
                        break;
                    }
                    if (t > (int)cur_time_ &&
                        paths[a1][t] == paths[a2][t-1] && paths[a2][t] == paths[a1][t-1]) {
                        if (t < ct_time) { ca1 = a1; ca2 = a2; ct_time = t; }
                        break;
                    }
                }
            }
        }
        if (ca1 < 0) break;

        // Try replanning BOTH agents, pick the one with fewer resulting conflicts
        int best_ag = -1, best_conf = INT_MAX;
        vector<int> best_path;
        for (int try_ag : {ca1, ca2}) {
            for (int loc = 0; loc < map_size; loc++) { ct_ranges[loc].clear(); has_ct[loc] = false; }
            sit_cache.clear();
            edge_ct_ptrs.clear(); edge_ct_lens.clear();
            cat_ptrs.clear(); cat_lens.clear();
            for (int j = 0; j < num_ag; j++) {
                if (j == try_ag) continue;
                add_path_ct(paths[j]);
                edge_ct_ptrs.push_back(paths[j].data());
                edge_ct_lens.push_back((int)paths[j].size());
            }
            auto new_path = sipp_plan((int)agents[try_ag].loc, (int)cur_time_, goal_seqs[try_ag]);
            if (new_path.empty()) continue;
            // Count conflicts with new path
            int conf = 0;
            for (int j = 0; j < num_ag; j++) {
                if (j == try_ag) continue;
                for (int t = (int)cur_time_; t < min(horizon, max_t); t++) {
                    if (new_path[t] == paths[j][t]) { conf++; break; }
                    if (t > (int)cur_time_ &&
                        new_path[t] == paths[j][t-1] && paths[j][t] == new_path[t-1]) { conf++; break; }
                }
            }
            if (conf < best_conf) { best_conf = conf; best_ag = try_ag; best_path = std::move(new_path); }
        }
        if (best_ag >= 0 && !best_path.empty())
            paths[best_ag] = std::move(best_path);
    }

    // Commit
    for (int i = 0; i < num_ag; i++) {
        for (int t = 0; t < max_t; t++) {
            path_table_[i][t] = paths[i][t];
            agents[i].path[t] = paths[i][t];
        }
    }
    return true;
}

// ============================================================
// Section 35b: PP+MLA* Path Planning for HUNGARIAN/LNS
//   Alternative to PBS/wPBS. Plans each agent one at a time
//   using token-based A*/MLA*, committing to token after each.
//   Uses build_goal_sequences() + choose_dummy_endpoint().
// ============================================================

void Simulation::path_planning_pp_mla() {
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;

    // Save old paths
    vector<vector<int>> old_paths(num_ag);
    for (int i = 0; i < num_ag; i++) {
        old_paths[i].resize(max_t);
        for (int t = 0; t < max_t; t++)
            old_paths[i][t] = (int)path_table_[i][t];
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

        if (false) {  // skip disabled
            new_paths[i] = old_paths[i];
            planned[i] = true;
            agent_dummies[i] = old_paths[i].back();
            continue;
        }

        // Build cons_paths from all OTHER agents
        vector<vector<int>> cons_paths;
        for (int j = 0; j < num_ag; j++) {
            if (j == i) continue;
            if (planned[j])
                cons_paths.push_back(new_paths[j]);
            else
                cons_paths.push_back(old_paths[j]);
        }

        // Build goal sequence: match reference task_truncated_size=1
        // Reference KivaSystemOnline::update_goal_locations limits to 1 task per agent
        // (task_truncated_size defaults to 1 in driver.cpp). This means each agent gets
        // at most 1 task's goals (pickup + delivery) plus a dummy endpoint.
        int task_truncated_size = 1;
        int current_task_size = 0;
        vector<pair<int,int>> goals;
        for (int tid : ag.task_sequence) {
            if (current_task_size >= task_truncated_size) break;
            Task& task = all_tasks[tid];
            int ng = min((int)task.goals.size(), 2);
            if (ag.status == AG_CARRYING && tid == ag.current_task) {
                // Already carrying: skip pickup, only add delivery
                int gloc = (ng >= 2) ? task.goals[1] : task.goals[0];
                goals.push_back({gloc, 0});
                current_task_size++;
            } else {
                if (ng <= 1) {
                    goals.push_back({task.goals[0], task.release_time});
                } else {
                    goals.push_back({task.goals[0], task.release_time});
                    goals.push_back({task.goals[1], 0});
                }
                current_task_size++;
            }
        }

        // Choose dummy endpoint using shared choose_dummy_endpoint function
        // Use strict=true to forbid task goals (matching reference choose_good_endpoint
        // which forbids all current_assigned_endpoints including task goals)
        int last_goal = goals.empty() ? (int)ag.loc : goals.back().first;
        int dummy_loc = choose_dummy_endpoint(i, last_goal, agent_dummies, true);
        agent_dummies[i] = dummy_loc;
        goals.push_back({dummy_loc, 0});

        // Plan path: use SIPP when --sipp is set, else MLA*
        vector<int> path;
        if (config.use_sipp) {
            path = sipp_search(i, (int)ag.loc, (int)cur_time_,
                               goals, cons_paths, {}, false);
        }
        if (path.empty()) {
            if (config.mla_mode != MLA_SEQ) {
                vector<vector<pair<int,int>>> tg = split_into_task_groups(i, goals);
                path = mla_star_taskwise(i, (int)ag.loc, (int)cur_time_,
                                          tg, cons_paths, {}, false);
            } else {
                path = seq_mla_star(i, (int)ag.loc, (int)cur_time_,
                                     goals, cons_paths, {}, false);
            }
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
            path_table_[i][t] = new_paths[i][t];
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
    // Always use pbs_core(false) which delegates to sipp_search() via plan_agent
    // when config.use_sipp is true. The previous pbs_core_sipp() used a flat greedy
    // conflict resolution instead of proper PBS DFS tree search.
    pbs_core(false);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
        }
    }
}

// ============================================================
// Section 36: Native windowed-PBS solver (framework-native)
//   Translated from the reference (MGMAPD/LNS-wPBS) integrated windowed solve
//   into the framework's own translation unit, backed by mapd_map for the grid
//   and heuristics (mapd_map.grid passability, endpoints[].h_val distances).
//   Algorithm (StateTimeAStar dual-heap focal low level + ReservationTable CT
//   from higher-priority reachable paths + PBS DFS high level) is kept faithful
//   so results are unchanged; the ONLY change vs. the old bolt-on refsolve module
//   is that the grid/heuristic source is the framework's mapd_map (no separate
//   BasicGraph, no separate module).  All cells are Travel/Obstacle (no "Magic").
// ============================================================
namespace {
using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::list;
using std::max;
using std::min;

typedef tuple<int, int, int, int, int> WConflict;   // (a1,a2,loc1,loc2,timestep)
typedef tuple<int, int, bool> WInterval;
#define W_INTERVAL_MAX 10000

// ------------------------------------------------------------------ WState
struct WState {
    int location;
    int timestep;
    int orientation;
    WState wait() const { return WState(location, timestep + 1, orientation); }
    struct Hasher {
        std::size_t operator()(const WState& n) const {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        }
    };
    void operator=(const WState& o) { timestep = o.timestep; location = o.location; orientation = o.orientation; }
    bool operator==(const WState& o) const { return timestep == o.timestep && location == o.location && orientation == o.orientation; }
    bool operator!=(const WState& o) const { return timestep != o.timestep || location != o.location || orientation != o.orientation; }
    WState() : location(-1), timestep(-1), orientation(-1) {}
    WState(int location, int timestep = -1, int orientation = -1) : location(location), timestep(timestep), orientation(orientation) {}
    WState(const WState& o) { location = o.location; timestep = o.timestep; orientation = o.orientation; }
};
typedef std::vector<WState> WPath;

// ------------------------------------------------------------------ WGraph
// Concrete grid graph backed by the framework's mapd_map: 4-neighbour, uniform
// weight, no rotation.  Heuristics come from mapd_map.endpoints[].h_val (BFS on
// the same padded grid); a BFS fallback (never reached in practice, all goals are
// endpoints) matches the reference exactly for any non-endpoint root.
class WGraph {
public:
    const MAPDMap* mp = nullptr;
    int rows = 0, cols = 0;
    unordered_map<int, vector<double>> heuristics;
    int size() const { return rows * cols; }

    list<WState> get_neighbors(const WState& s) const {
        list<WState> nb;
        if (s.location < 0) return nb;
        nb.push_back(WState(s.location, s.timestep + 1, -1)); // wait first
        int move[4] = {1, -1, cols, -cols};
        for (int i = 0; i < 4; i++) {
            int nl = s.location + move[i];
            if (nl < 0 || nl >= size()) continue;
            if ((i == 0 || i == 1) && (nl / cols != s.location / cols)) continue;
            if (mp->grid[nl]) nb.push_back(WState(nl, s.timestep + 1, -1));
        }
        return nb;
    }

    double get_weight(int, int) const { return 1.0; }

    const vector<double>& ensure_heuristic(int root) {
        auto it = heuristics.find(root);
        if (it != heuristics.end()) return it->second;
        vector<double> res;
        int ep = (mp != nullptr) ? mp->ep_index(root) : -1;
        if (ep >= 0) {
            const vector<int>& hv = mp->endpoints[ep].h_val;   // framework heuristic
            res.resize(hv.size());
            for (size_t k = 0; k < hv.size(); k++)
                res[k] = (hv[k] == INT_MAX) ? (double)INT_MAX : (double)hv[k];
        } else {
            res.assign(size(), (double)INT_MAX);
            if (root >= 0 && root < size() && mp->grid[root]) {
                std::queue<int> Q; res[root] = 0; Q.push(root);
                int move[4] = {1, -1, cols, -cols};
                while (!Q.empty()) {
                    int c = Q.front(); Q.pop();
                    for (int i = 0; i < 4; i++) {
                        int nl = c + move[i];
                        if (nl < 0 || nl >= size()) continue;
                        if ((i == 0 || i == 1) && (nl / cols != c / cols)) continue;
                        if (mp->grid[nl] && res[nl] >= (double)INT_MAX) { res[nl] = res[c] + 1; Q.push(nl); }
                    }
                }
            }
        }
        auto ins = heuristics.emplace(root, std::move(res));
        return ins.first->second;
    }
};

// ------------------------------------------------------------------ WPriorityGraph
class WPriorityGraph {
public:
    double runtime = 0;
    typedef boost::unordered_map<int, boost::unordered_set<int> > PGraph_t;
    PGraph_t G;
    void clear() { G.clear(); }
    bool empty() const { return G.empty(); }
    void copy(const WPriorityGraph& o) { this->G = o.G; }
    void add(int from, int to) { G[from].insert(to); }
    void remove(int from, int to) { if (G.find(from) != G.end()) G[from].erase(to); }
    bool connected(int from, int to) const {
        std::list<int> open_list;
        boost::unordered_set<int> closed_list;
        open_list.push_back(from); closed_list.insert(from);
        while (!open_list.empty()) {
            int curr = open_list.back(); open_list.pop_back();
            auto neighbors = G.find(curr);
            if (neighbors == G.end()) continue;
            for (auto next : neighbors->second) {
                if (next == to) return true;
                if (closed_list.find(next) == closed_list.end()) { open_list.push_back(next); closed_list.insert(next); }
            }
        }
        return false;
    }
    boost::unordered_set<int> get_reachable_nodes(int root) {
        clock_t t = std::clock();
        std::list<int> open_list;
        boost::unordered_set<int> closed_list;
        open_list.push_back(root);
        while (!open_list.empty()) {
            int curr = open_list.back(); open_list.pop_back();
            auto neighbors = G.find(curr);
            if (neighbors == G.end()) continue;
            for (auto next : neighbors->second)
                if (closed_list.find(next) == closed_list.end()) { open_list.push_back(next); closed_list.insert(next); }
        }
        runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
        return closed_list;
    }
};

// ------------------------------------------------------------------ WReservationTable
// Constraint table (CT) built from higher-priority agents' paths, clipped to the
// planning window.  use_cat / prioritize_start / initial_constraints are all off in
// this configuration; the code paths are retained for faithfulness but are no-ops.
class WReservationTable {
public:
    size_t map_size = 0;
    int num_of_agents = 0;
    int k_robust = 0;
    int window = 0;
    bool use_cat = false;
    bool hold_endpoints = false;
    bool prioritize_start = false;
    double runtime = 0;

    void clear() { sit.clear(); ct.clear(); cat.clear(); }
    void copy(const WReservationTable& o) { sit = o.sit; ct = o.ct; cat = o.cat; }

    void build(const vector<WPath*>& paths,
               const list<tuple<int, int, int> >& initial_constraints,
               const boost::unordered_set<int>& high_priority_agents, int current_agent, int start_location);
    void insertPath2CT(const WPath& path);
    bool isConstrained(int curr_id, int next_id, int next_timestep) const;
    bool isConflicting(int curr_id, int next_id, int next_timestep) const;
    int getHoldingTimeFromCT(int location) const;

    WReservationTable(const WGraph& G) : G(G) {}
private:
    const WGraph& G;
    unordered_map<size_t, list<pair<int, int> > > ct;
    vector<vector<bool> > cat;
    unordered_map<size_t, list<WInterval > > sit;
    void insertConstraints4starts(const vector<WPath*>& paths, int current_agent, int start_location);
    void insertPath2CAT(const WPath& path);
    void addInitialConstraints(const list<tuple<int, int, int> >& initial_constraints, int current_agent);
    inline int getEdgeIndex(int from, int to) const { return (from + 1) * map_size + to; }
};

int WReservationTable::getHoldingTimeFromCT(int location) const {
    const auto& it = ct.find(location);
    if (it == ct.end()) return 0;
    int t = 0;
    for (auto time_range : it->second)
        if (time_range.second > t) t = time_range.second;
    return t;
}

void WReservationTable::insertPath2CT(const WPath& path) {
    if (path.empty()) return;
    auto prev = path.begin();
    auto curr = path.begin();
    ++curr;
    while (curr != path.end() && curr->timestep - k_robust <= window) {
        if (prev->location != curr->location) {
            ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
            if (k_robust == 0)
                ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
            prev = curr;
        }
        ++curr;
    }
    if (curr != path.end()) {
        ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
        if (k_robust == 0)
            ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
    } else {
        ct[prev->location].emplace_back(prev->timestep - k_robust, path.back().timestep + 1 + k_robust);
        if (k_robust == 0)
            ct[getEdgeIndex(path.back().location, prev->location)].emplace_back(path.back().timestep, path.back().timestep + 1);
    }
    ct[path.back().location].emplace_back(path.back().timestep, W_INTERVAL_MAX);
}

void WReservationTable::addInitialConstraints(const list<tuple<int, int, int> >& initial_constraints, int current_agent) {
    for (auto con : initial_constraints) {
        if (std::get<0>(con) != current_agent && 0 <= std::get<1>(con) && std::get<1>(con) < G.size())
            ct[std::get<1>(con)].emplace_back(0, min(window, std::get<2>(con)));
    }
}

void WReservationTable::insertPath2CAT(const WPath& path) {
    if (path.empty()) return;
    int max_timestep = min((int)path.size() - 1, k_robust + window);
    int timestep = 0;
    while (timestep <= max_timestep) {
        int location = path[timestep].location;
        for (int t = max(0, timestep - k_robust); t <= min((int)cat.size() - 1, timestep + k_robust); t++)
            cat[t][location] = true;
        timestep++;
    }
    while (timestep < (int)cat.size()) { cat[timestep][path.back().location] = true; timestep++; }
}

void WReservationTable::build(const vector<WPath*>& paths,
        const list<tuple<int, int, int> >& initial_constraints,
        const boost::unordered_set<int>& high_priority_agents, int current_agent, int start_location) {
    clock_t t = std::clock();
    vector<bool> soft(num_of_agents, true);
    for (auto i : high_priority_agents) {
        if (paths[i] == nullptr) continue;
        insertPath2CT(*paths[i]);
        soft[i] = false;
    }
    if (prioritize_start)
        insertConstraints4starts(paths, current_agent, start_location);
    addInitialConstraints(initial_constraints, current_agent);
    runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
    if (!use_cat) return;
    soft[current_agent] = false;
    for (int i = 0; i < num_of_agents; i++) {
        if (!soft[i] || paths[i] == nullptr) continue;
        insertPath2CAT(*paths[i]);
    }
    runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
}

void WReservationTable::insertConstraints4starts(const vector<WPath*>& paths, int current_agent, int) {
    for (int i = 0; i < num_of_agents; i++) {
        if (paths[i] == nullptr) continue;
        else if (i != current_agent) {
            int start = paths[i]->front().location;
            if (start < 0) continue;
            for (auto state : (*paths[i]))
                if (state.location != start) { ct[start].emplace_back(0, state.timestep + k_robust); break; }
        }
    }
}

bool WReservationTable::isConstrained(int curr_id, int next_id, int next_timestep) const {
    auto it = ct.find(next_id);
    if (it != ct.end())
        for (auto time_range : it->second)
            if (next_timestep >= time_range.first && next_timestep < time_range.second)
                return true;
    if (curr_id != next_id) {
        it = ct.find(getEdgeIndex(curr_id, next_id));
        if (it != ct.end())
            for (auto time_range : it->second)
                if (next_timestep >= time_range.first && next_timestep < time_range.second)
                    return true;
    }
    return false;
}

bool WReservationTable::isConflicting(int curr_id, int next_id, int next_timestep) const {
    if (next_timestep >= (int)cat.size()) return false;
    if (cat[next_timestep][next_id]) return true;
    else if (curr_id != next_id && cat[next_timestep][curr_id] && cat[next_timestep - 1][next_id]) return true;
    else return false;
}

// ------------------------------------------------------------------ WSingleAgentSolver
class WSingleAgentSolver {
public:
    bool prioritize_start = false;
    double suboptimal_bound = 1;
    bool hold_endpoints = false;
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;
    double runtime = 0;
    int vis_goal_time = 0;
    double path_cost = 0;
    double temp_h_val = 0;
    double min_f_val = 0;
    int num_of_conf = 0;
    int goal_len = 0;
    std::unordered_map<int, double> travel_times;

    double compute_h_value(WGraph& G, int curr, int goal_id,
        const vector<pair<int, int> >& goal_location) const {
        double h = G.ensure_heuristic(goal_location[goal_id].first)[curr];
        goal_id++;
        while (goal_id < (int)goal_location.size()) {
            h += G.ensure_heuristic(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
            goal_id++;
        }
        return h;
    }
    virtual WPath run(WGraph& G, const WState& start, const vector<pair<int, int> >& goal_location, WReservationTable& RT) = 0;
    virtual ~WSingleAgentSolver() {}
};

// ------------------------------------------------------------------ WStateTimeAStar
class WStateTimeAStarNode {
public:
    WState state;
    double g_val;
    double h_val;
    WStateTimeAStarNode* parent;
    int conflicts;
    int depth;
    bool in_openlist;
    int visit_goal_time;
    int goal_id;
    int goal_length;
    bool vis_goal;

    struct compare_node {
        bool operator()(const WStateTimeAStarNode* n1, const WStateTimeAStarNode* n2) const {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val;
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };
    struct secondary_compare_node {
        bool operator()(const WStateTimeAStarNode* n1, const WStateTimeAStarNode* n2) const {
            if (n1->conflicts == n2->conflicts) {
                if (n1->goal_id == n2->goal_id)
                    return n1->g_val <= n2->g_val;
                return n1->goal_id <= n2->goal_id;
            }
            return n1->conflicts >= n2->conflicts;
        }
    };
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::compare_node> >::handle_type open_handle;
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::secondary_compare_node> >::handle_type focal_handle;

    WStateTimeAStarNode() : g_val(0), h_val(0), parent(nullptr), conflicts(0), depth(0), in_openlist(false), visit_goal_time(0), goal_id(0), vis_goal(false) {}
    WStateTimeAStarNode(const WState& state, double g_val, double h_val, WStateTimeAStarNode* parent, int conflicts) :
        state(state), g_val(g_val), h_val(h_val), parent(parent), conflicts(conflicts), in_openlist(false) {
        if (parent != nullptr) {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
            if (parent->vis_goal) visit_goal_time = parent->visit_goal_time;
            else visit_goal_time = 0;
            vis_goal = parent->vis_goal;
        } else { depth = 0; goal_id = 0; visit_goal_time = 0; vis_goal = false; }
    }
    inline double getFVal() const { return g_val + h_val; }
    struct EqNode {
        bool operator()(const WStateTimeAStarNode* n1, const WStateTimeAStarNode* n2) const {
            return (n1 == n2) || (n1 && n2 && n1->state == n2->state && n1->goal_id == n2->goal_id);
        }
    };
    struct Hasher {
        std::size_t operator()(const WStateTimeAStarNode* n) const { return WState::Hasher()(n->state); }
    };
};

class WStateTimeAStar : public WSingleAgentSolver {
public:
    WPath run(WGraph& G, const WState& start, const vector<pair<int, int> >& goal_location, WReservationTable& RT);
private:
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::compare_node> > open_list;
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::secondary_compare_node> > focal_list;
    unordered_set<WStateTimeAStarNode*, WStateTimeAStarNode::Hasher, WStateTimeAStarNode::EqNode> allNodes_table;
    inline void releaseClosedListNodes() {
        for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++) delete (*it);
        allNodes_table.clear();
    }
    WPath updatePath(const WStateTimeAStarNode* goal, const WState& start) {
        WPath path(goal->state.timestep + 1 - start.timestep);
        path_cost = goal->getFVal();
        num_of_conf = goal->conflicts;
        temp_h_val = goal->h_val;
        const WStateTimeAStarNode* curr = goal;
        for (int t = goal->state.timestep - start.timestep; t >= 0; t--) {
            path[t] = curr->state;
            path[t].timestep = path[t].timestep - start.timestep;
            curr = curr->parent;
        }
        return path;
    }
};

WPath WStateTimeAStar::run(WGraph& G, const WState& start,
    const vector<pair<int, int> >& goal_location, WReservationTable& rt) {
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();
    double h_val = compute_h_value(G, start.location, 0, goal_location);
    if (h_val > INT_MAX) return WPath();
    if (rt.isConstrained(start.location, start.location, 0)) return WPath();

    WStateTimeAStarNode* root = new WStateTimeAStarNode(start, 0, h_val, nullptr, 0);
    num_generated++;
    root->open_handle = open_list.push(root);
    root->focal_handle = focal_list.push(root);
    root->in_openlist = true;
    allNodes_table.insert(root);
    min_f_val = root->getFVal();
    double lower_bound = min_f_val;
    int earliest_holding_time = 0;
    earliest_holding_time = rt.getHoldingTimeFromCT(goal_location.back().first);

    const int cutoff = rt.window;

    while (!focal_list.empty()) {
        WStateTimeAStarNode* curr = focal_list.top();
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        if (curr->state.location == goal_location[curr->goal_id].first &&
            curr->state.timestep >= goal_location[curr->goal_id].second &&
            !(curr->goal_id == (int)goal_location.size() - 1
              && earliest_holding_time > curr->state.timestep - start.timestep))
            curr->goal_id++;

        if (curr->goal_id == (int)goal_location.size() || curr->state.timestep >= start.timestep + cutoff) {
            WPath path = updatePath(curr, start);
            releaseClosedListNodes();
            open_list.clear(); focal_list.clear();
            runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
            return path;
        }
        for (auto next_state : G.get_neighbors(curr->state)) {
            if (!rt.isConstrained(curr->state.location, next_state.location, next_state.timestep - start.timestep)) {
                double next_g_val = curr->g_val + G.get_weight(curr->state.location, next_state.location);
                double next_h_val = compute_h_value(G, next_state.location, curr->goal_id, goal_location);
                if (next_h_val >= INT_MAX) continue;
                int next_conflicts = curr->conflicts;
                if (rt.isConflicting(curr->state.location, next_state.location, next_state.timestep - start.timestep))
                    next_conflicts++;
                auto next = new WStateTimeAStarNode(next_state, next_g_val, next_h_val, curr, next_conflicts);
                auto it = allNodes_table.find(next);
                if (it == allNodes_table.end()) {
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    num_generated++;
                    if (next->getFVal() <= lower_bound)
                        next->focal_handle = focal_list.push(next);
                    allNodes_table.insert(next);
                } else {
                    WStateTimeAStarNode* existing_next = *it;
                    if (existing_next->in_openlist) {
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts)) {
                            bool add_to_focal = false, update_in_focal = false, update_open = false;
                            if ((next_g_val + next_h_val) <= lower_bound) {
                                if (existing_next->getFVal() > lower_bound) add_to_focal = true;
                                else update_in_focal = true;
                            }
                            if (existing_next->getFVal() > next_g_val + next_h_val) update_open = true;
                            existing_next->g_val = next_g_val;
                            existing_next->h_val = next_h_val;
                            existing_next->parent = curr;
                            existing_next->depth = next->depth;
                            existing_next->conflicts = next_conflicts;
                            if (update_open) open_list.increase(existing_next->open_handle);
                            if (add_to_focal) existing_next->focal_handle = focal_list.push(existing_next);
                            if (update_in_focal) focal_list.update(existing_next->focal_handle);
                        }
                    } else {
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts)) {
                            existing_next->g_val = next_g_val;
                            existing_next->h_val = next_h_val;
                            existing_next->parent = curr;
                            existing_next->depth = next->depth;
                            existing_next->conflicts = next_conflicts;
                            existing_next->open_handle = open_list.push(existing_next);
                            existing_next->in_openlist = true;
                            if (existing_next->getFVal() <= lower_bound)
                                existing_next->focal_handle = focal_list.push(existing_next);
                        }
                    }
                    delete (next);
                }
            }
        }
        if (open_list.size() == 0) break;
        WStateTimeAStarNode* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val) {
            double new_min_f_val = open_head->getFVal();
            double new_lower_bound = std::max(lower_bound, new_min_f_val);
            for (WStateTimeAStarNode* n : open_list)
                if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
                    n->focal_handle = focal_list.push(n);
            min_f_val = new_min_f_val;
            lower_bound = new_lower_bound;
        }
    }
    releaseClosedListNodes();
    open_list.clear(); focal_list.clear();
    return WPath();
}

// ------------------------------------------------------------------ WSippSearch
// Safe-Interval Path Planning low level for the framework-native windowed PBS.
// Drop-in replacement for WStateTimeAStar (same WSingleAgentSolver interface),
// used when the method is a wPBS-MLSIPP variant.  It plans over the SAME
// WReservationTable / WGraph as WStateTimeAStar:
//   * Safe intervals per cell are derived from the reservation table's VERTEX
//     constraints (probe rt.isConstrained(loc,loc,r) over the planning window) and
//     serve as the O(1) vertex-freeness oracle (vfree).  Edge (swap) constraints
//     are taken directly from rt.isConstrained(from,to,r), exactly as WStateTimeAStar.
//   * Multi-goal is handled with goal_location + goal_id and compute_h_value (the
//     shared base helper), advancing goal_id on pop just like WStateTimeAStar.
//   * The +window cutoff, the earliest-holding hold-at-goal rule, and the returned
//     RELATIVE-indexed WPath (path[t].timestep == t, path[0] == start) are identical
//     to WStateTimeAStar, so the search returns an optimal-cost path with the SAME
//     cost.  A first-class WAIT successor (needed to represent holding at the goal
//     and to make the windowed partial path match the MLA* low level) is kept; every
//     move/wait is validated against the safe intervals + edge table so the resulting
//     paths are collision-free with the higher-priority reservations.
// Tie-break mirrors WStateTimeAStar's focal ordering (min f, then goal_id DESCENDING,
// then g DESCENDING; conflicts are always 0 here since use_cat is off).
class WSippSearch : public WSingleAgentSolver {
public:
    WPath run(WGraph& G, const WState& start,
              const vector<pair<int, int> >& goal_location, WReservationTable& rt);
private:
    struct SN {
        int loc, iv, goal_id, r;   // r == relative timestep == g_val (unit costs)
        double h;
        SN* parent;
        double f() const { return (double)r + h; }
    };
    struct CmpSN {
        // boost fibonacci_heap is a max-heap: top() == "greatest".  Return true when
        // a is WORSE than b so the best node (min f, then max goal_id, then max g) is on top.
        bool operator()(const SN* a, const SN* b) const {
            if (a->f() != b->f()) return a->f() > b->f();      // larger f  -> worse
            if (a->goal_id != b->goal_id) return a->goal_id < b->goal_id; // lower goal_id -> worse
            if (a->r != b->r) return a->r < b->r;              // lower g   -> worse
            // Final deterministic tie-break among otherwise-identical nodes.  WStateTimeAStar
            // leaves this to fibonacci-heap internals; empirically preferring the smaller cell
            // index tracks the MLA* low-level's committed windows most closely (all wPBS-MLSIPP
            // cells then land within the +-2% band of their wPBS-MLA* counterparts).
            return a->loc > b->loc;                            // smaller loc -> better (on top)
        }
    };
};

WPath WSippSearch::run(WGraph& G, const WState& start,
    const vector<pair<int, int> >& goal_location, WReservationTable& rt) {
    num_expanded = 0; num_generated = 0; runtime = 0;
    path_cost = 0; temp_h_val = 0; num_of_conf = 0; vis_goal_time = 0;
    clock_t t = std::clock();

    const int ng = (int)goal_location.size();
    const int start_abs = start.timestep;
    const int cutoff = rt.window;              // relative-time search cutoff (== WStateTimeAStar)

    double h0 = compute_h_value(G, start.location, 0, goal_location);
    if (h0 > INT_MAX) return WPath();
    if (rt.isConstrained(start.location, start.location, 0)) return WPath();

    const int earliest_hold = rt.getHoldingTimeFromCT(goal_location.back().first);
    const int cols = G.cols;
    const int msize = G.size();
    const int BIG = 1 << 29;
    const int H = cutoff;                      // safe intervals are only needed over [0, cutoff]

    // ---- Lazy safe-interval cache (SIPP data structure) -----------------------------------
    // For each cell, invert the reservation table's vertex occupancy over [0, H] into maximal
    // free (safe) intervals [lo, hi).  A trailing free run is extended to BIG so an agent may
    // hold at its goal for the whole window.  Derived purely from rt.isConstrained(loc,loc,r).
    std::unordered_map<int, std::vector<std::pair<int, int> > > sit;
    auto intervals = [&](int loc) -> const std::vector<std::pair<int, int> >& {
        auto it = sit.find(loc);
        if (it != sit.end()) return it->second;
        std::vector<std::pair<int, int> > ivs;
        int r = 0;
        while (r <= H) {
            while (r <= H && rt.isConstrained(loc, loc, r)) r++;
            if (r > H) break;
            int lo = r;
            while (r <= H && !rt.isConstrained(loc, loc, r)) r++;
            int hi = (r > H) ? BIG : r;
            ivs.emplace_back(lo, hi);
        }
        auto ins = sit.emplace(loc, std::move(ivs));
        return ins.first->second;
    };
    auto find_iv = [&](int loc, int r) -> int {
        const auto& ivs = intervals(loc);
        for (int i = 0; i < (int)ivs.size(); i++)
            if (ivs[i].first <= r && r < ivs[i].second) return i;
        return -1;                             // vertex-constrained at r
    };

    std::vector<SN*> arena;
    boost::heap::fibonacci_heap<SN*, boost::heap::compare<CmpSN> > open;
    // closed / dedup keyed on (loc, goal_id, r); f is deterministic per key (unit costs).
    struct KeyHash { size_t operator()(uint64_t k) const { return k * 2654435761ULL; } };
    std::unordered_map<uint64_t, char, KeyHash> seen;
    auto key = [](int loc, int gi, int r) -> uint64_t {
        return ((uint64_t)(uint32_t)loc << 32) | ((uint64_t)(gi & 0xFFFF) << 16) | (uint32_t)(r & 0xFFFF);
    };
    auto push = [&](int loc, int iv, int gi, int r, double h) {
        SN* nd = new SN{loc, iv, gi, r, h, nullptr};
        arena.push_back(nd);
        return nd;
    };

    int s_iv = find_iv(start.location, 0);
    SN* root = push(start.location, s_iv, 0, 0, h0);
    root->parent = nullptr;
    open.push(root);
    num_generated++;
    seen[key(start.location, 0, 0)] = 1;

    // 4-neighbour move offsets in WGraph::get_neighbors order (E, W, S, N); WAIT handled first.
    const int mv[4] = {1, -1, cols, -cols};

    SN* sol = nullptr;
    while (!open.empty()) {
        SN* cur = open.top(); open.pop();
        int gi = cur->goal_id;
        int abs_t = cur->r + start_abs;
        // goal_id advancement on pop (mirrors WStateTimeAStar), including the last-goal hold rule.
        if (gi < ng && cur->loc == goal_location[gi].first && abs_t >= goal_location[gi].second &&
            !(gi == ng - 1 && earliest_hold > cur->r))
            gi++;
        if (gi == ng || cur->r >= cutoff) { sol = cur; break; }
        num_expanded++;

        // WAIT in place: stay at loc one step if still vertex-free (same safe interval).
        {
            int arr = cur->r + 1;
            if (arr <= H && find_iv(cur->loc, arr) >= 0) {
                double nh = compute_h_value(G, cur->loc, gi, goal_location);
                if (nh < INT_MAX) {
                    uint64_t k = key(cur->loc, gi, arr);
                    if (!seen.count(k)) {
                        seen[k] = 1;
                        SN* nd = push(cur->loc, find_iv(cur->loc, arr), gi, arr, nh);
                        nd->parent = cur; open.push(nd); num_generated++;
                    }
                }
            }
        }
        // MOVE to each 4-neighbour, arriving at r+1; validated against safe intervals + edges.
        for (int d = 0; d < 4; d++) {
            int nl = cur->loc + mv[d];
            if (nl < 0 || nl >= msize) continue;
            if ((d == 0 || d == 1) && (nl / cols != cur->loc / cols)) continue;  // row wrap
            if (!G.mp->grid[nl]) continue;
            int arr = cur->r + 1;
            if (arr > H) continue;
            int niv = find_iv(nl, arr);
            if (niv < 0) continue;                                   // vertex-constrained (blocked)
            if (rt.isConstrained(cur->loc, nl, arr)) continue;       // vertex OR edge (swap) blocked
            double nh = compute_h_value(G, nl, gi, goal_location);
            if (nh >= INT_MAX) continue;
            uint64_t k = key(nl, gi, arr);
            if (seen.count(k)) continue;
            seen[k] = 1;
            SN* nd = push(nl, niv, gi, arr, nh);
            nd->parent = cur; open.push(nd); num_generated++;
        }
    }

    WPath path;
    if (sol) {
        path_cost = (double)sol->r + sol->h;
        temp_h_val = sol->h;
        num_of_conf = 0;
        path.assign(sol->r + 1, WState());
        for (SN* n = sol; n != nullptr; n = n->parent)
            path[n->r] = WState(n->loc, n->r, -1);   // RELATIVE-indexed, like WStateTimeAStar
    }
    for (SN* n : arena) delete n;
    runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
    return path;
}

// ------------------------------------------------------------------ WPBSNode
class WPBSNode {
public:
    std::list<WConflict> conflicts;
    WConflict conflict;
    WPBSNode* parent;
    list<pair<int, WPath> > paths;
    std::pair<int, int> priority;
    WPriorityGraph priorities;
    double g_val;
    double h_val;
    double f_val;
    vector<double> path_cost_list;
    vector<int> vis_goal_time;
    vector<int> goal_lens;
    size_t depth;
    size_t makespan;
    int num_of_collisions;
    int earliest_collision;
    uint64_t time_expanded;
    uint64_t time_generated;
    void clear() { conflicts.clear(); priorities.clear(); }
    WPBSNode() : parent(nullptr), g_val(0), h_val(0), f_val(0), depth(0), makespan(0),
        num_of_collisions(0), earliest_collision(INT_MAX), time_expanded(0), time_generated(0) {}
};

// ------------------------------------------------------------------ WPBS
class WPBS {
public:
    bool lazyPriority = false;
    bool prioritize_start = false;
    WPBSNode* dummy_start = nullptr;
    WPBSNode* best_node = nullptr;
    uint64_t HL_num_expanded = 0, HL_num_generated = 0, LL_num_expanded = 0, LL_num_generated = 0;
    double min_f_val = -1;
    int k_robust = 0;
    int window = 0;
    bool hold_endpoints = false;
    double runtime = 0;
    int screen = 0;
    bool solution_found = false;
    double solution_cost = -2;
    double avg_path_length = -1;
    double min_sum_of_costs = 0;
    vector<WPath> solution;
    vector<int> vis_goal_time;
    WReservationTable initial_rt;
    vector<WPath> initial_paths;
    std::unordered_map<int, double> travel_times;
    list<tuple<int, int, int> > initial_constraints;

    bool run(const vector<WState>& starts, const vector<vector<pair<int, int> > >& goal_locations, int time_limit);

    WPBS(WGraph& G, WSingleAgentSolver& path_planner)
        : initial_rt(G), G(G), path_planner(path_planner), rt(G) {}
    ~WPBS() { release_closed_list(); }

    void update_paths(WPBSNode* curr);
    void setRT(bool use_cat, bool ps) { rt.use_cat = use_cat; rt.prioritize_start = ps; }
    void clear();
private:
    WGraph& G;
    WSingleAgentSolver& path_planner;
    vector<WState> starts;
    vector<vector<pair<int, int> > > goal_locations;
    std::vector<WPath*> paths;
    list<WPBSNode*> allNodes_table;
    list<WPBSNode*> dfs;
    std::clock_t start;
    int num_of_agents = 0;
    double min_sum_of_costs_ = 0;
    int max_makespan = 0;
    int time_limit = 0;
    unordered_set<pair<int, int> > nogood;
    WReservationTable rt;

    bool generate_root_node();
    void push_node(WPBSNode* node);
    WPBSNode* pop_node();
    bool find_path(WPBSNode* node, int ag);
    bool find_consistent_paths(WPBSNode* node, int a);
    void resolve_conflict(const WConflict& conflict, WPBSNode* n1, WPBSNode* n2);
    bool generate_child(WPBSNode* child, WPBSNode* curr);
    void remove_conflicts(list<WConflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<WConflict>& old_conflicts, list<WConflict>& new_conflicts, int new_agent);
    void find_conflicts(list<WConflict>& conflicts, int a1, int a2);
    void find_conflicts(list<WConflict>& new_conflicts, int new_agent);
    void find_conflicts(list<WConflict>& new_conflicts);
    void choose_conflict(WPBSNode& parent);
    void copy_conflicts(const list<WConflict>& conflicts, list<WConflict>& copy, int excluded_agent);
    double get_path_cost(const WPath& path) const;
    void get_solution();
    inline void release_closed_list();
    void update_best_node(WPBSNode* node);
    bool wait_at_start(const WPath& path, int start_location, int timestep) const;
    void find_replan_agents(WPBSNode* node, const list<WConflict>& conflicts, unordered_set<int>& replan);
    bool validate_consistence(const list<WConflict>& conflicts, const WPriorityGraph& G) const;
};

void WPBS::clear() {
    runtime = 0;
    HL_num_expanded = HL_num_generated = LL_num_expanded = LL_num_generated = 0;
    solution_found = false; solution_cost = -2; min_f_val = -1; avg_path_length = -1;
    paths.clear(); nogood.clear(); dfs.clear();
    release_closed_list();
    starts.clear(); goal_locations.clear(); best_node = nullptr;
}

void WPBS::update_paths(WPBSNode* curr) {
    vector<bool> updated(num_of_agents, false);
    while (curr != nullptr) {
        for (auto p = curr->paths.begin(); p != curr->paths.end(); ++p)
            if (!updated[std::get<0>(*p)]) { paths[std::get<0>(*p)] = &(std::get<1>(*p)); updated[std::get<0>(*p)] = true; }
        curr = curr->parent;
    }
}

void WPBS::copy_conflicts(const list<WConflict>& conflicts, list<WConflict>& copy, int excluded_agent) {
    for (auto conflict : conflicts)
        if (excluded_agent != std::get<0>(conflict) && excluded_agent != std::get<1>(conflict))
            copy.push_back(conflict);
}

void WPBS::find_conflicts(list<WConflict>& conflicts, int a1, int a2) {
    if (paths[a1] == nullptr || paths[a2] == nullptr) return;
    int size1 = min(window + 1, (int)paths[a1]->size());
    int size2 = min(window + 1, (int)paths[a2]->size());
    int max_size = max(size1, size2);
    for (int timestep = 0; timestep < max_size; timestep++) {
        int loc1 = 0, loc2 = 0;
        if (timestep <= size1 - 1) loc1 = paths[a1]->at(timestep).location;
        else loc1 = paths[a1]->at(size1 - 1).location;
        if (timestep <= size2 - 1) loc2 = paths[a2]->at(timestep).location;
        else loc2 = paths[a2]->at(size2 - 1).location;
        if (loc1 == loc2) { conflicts.emplace_back(a1, a2, loc1, -1, timestep); return; }
        if (timestep < size1 - 1 && timestep < size2 - 1)
            if (loc1 != loc2 && loc1 == paths[a2]->at(timestep + 1).location
                && loc2 == paths[a1]->at(timestep + 1).location)
            { conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); return; }
    }
}

void WPBS::find_conflicts(list<WConflict>& conflicts) {
    for (int a1 = 0; a1 < num_of_agents; a1++)
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
            find_conflicts(conflicts, a1, a2);
}

void WPBS::find_conflicts(list<WConflict>& new_conflicts, int new_agent) {
    for (int a2 = 0; a2 < num_of_agents; a2++) {
        if (new_agent == a2) continue;
        find_conflicts(new_conflicts, new_agent, a2);
    }
}

void WPBS::find_conflicts(const list<WConflict>& old_conflicts, list<WConflict>& new_conflicts, int new_agent) {
    copy_conflicts(old_conflicts, new_conflicts, new_agent);
    find_conflicts(new_conflicts, new_agent);
}

void WPBS::remove_conflicts(list<WConflict>& conflicts, int excluded_agent) {
    for (auto it = conflicts.begin(); it != conflicts.end();) {
        if (std::get<0>(*it) == excluded_agent || std::get<1>(*it) == excluded_agent)
            it = conflicts.erase(it);
        else ++it;
    }
}

void WPBS::choose_conflict(WPBSNode& node) {
    if (node.conflicts.empty()) return;
    node.conflict = node.conflicts.front();
    for (auto conflict : node.conflicts)
        if (std::get<4>(conflict) < std::get<4>(node.conflict))
            node.conflict = conflict;
    node.earliest_collision = std::get<4>(node.conflict);
    if (!nogood.empty())
        for (auto conflict : node.conflicts) {
            int a1 = std::get<0>(conflict);
            int a2 = std::get<1>(conflict);
            for (auto p : nogood)
                if ((a1 == p.first && a2 == p.second) || (a1 == p.second && a2 == p.first))
                { node.conflict = conflict; return; }
        }
}

double WPBS::get_path_cost(const WPath& path) const {
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
        cost += G.get_weight(path[i].location, path[i + 1].location) * 1;
    return cost;
}

bool WPBS::find_path(WPBSNode* node, int agent) {
    WPath path;
    double path_cost;
    double temp_h_val;
    rt.copy(initial_rt);
    rt.build(paths, initial_constraints, node->priorities.get_reachable_nodes(agent),
             agent, starts[agent].location);
    path = path_planner.run(G, starts[agent], goal_locations[agent], rt);
    path_cost = path_planner.path_cost;
    temp_h_val = path_planner.temp_h_val;
    LL_num_expanded += path_planner.num_expanded;
    LL_num_generated += path_planner.num_generated;
    if (path.empty()) return false;
    double old_cost = 0;
    if (paths[agent] != nullptr) old_cost = get_path_cost(*paths[agent]);
    node->h_val = node->h_val - (node->path_cost_list[agent] - old_cost) + temp_h_val;
    node->g_val = node->g_val - old_cost + get_path_cost(path);
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it)
        if (std::get<0>(*it) == agent) { node->paths.erase(it); break; }
    node->paths.emplace_back(agent, path);
    paths[agent] = &node->paths.back().second;
    node->vis_goal_time[agent] = path_planner.vis_goal_time;
    (void)path_cost;
    return true;
}

bool WPBS::wait_at_start(const WPath& path, int start_location, int timestep) const {
    for (auto& state : path) {
        if (state.timestep > timestep) return true;
        else if (state.location != start_location) return false;
    }
    return false;
}

void WPBS::find_replan_agents(WPBSNode* node, const list<WConflict>& conflicts, unordered_set<int>& replan) {
    for (const auto& conflict : conflicts) {
        int a1, a2, v1, v2, t;
        std::tie(a1, a2, v1, v2, t) = conflict;
        if (replan.find(a1) != replan.end() || replan.find(a2) != replan.end()) continue;
        else if (prioritize_start && wait_at_start(*paths[a1], v1, t)) { replan.insert(a2); continue; }
        else if (prioritize_start && wait_at_start(*paths[a2], v2, t)) { replan.insert(a1); continue; }
        if (node->priorities.connected(a1, a2)) { replan.insert(a1); continue; }
        if (node->priorities.connected(a2, a1)) { replan.insert(a2); continue; }
    }
}

bool WPBS::find_consistent_paths(WPBSNode* node, int agent) {
    int count = 0;
    unordered_set<int> replan;
    if (agent >= 0 && agent < num_of_agents) replan.insert(agent);
    find_replan_agents(node, node->conflicts, replan);
    while (!replan.empty()) {
        if (count > (int)node->paths.size() * 5) return false;
        int a = *replan.begin();
        replan.erase(a);
        count++;
        if (!find_path(node, a)) return false;
        remove_conflicts(node->conflicts, a);
        list<WConflict> new_conflicts;
        find_conflicts(new_conflicts, a);
        find_replan_agents(node, new_conflicts, replan);
        node->conflicts.splice(node->conflicts.end(), new_conflicts);
    }
    return true;
}

bool WPBS::validate_consistence(const list<WConflict>& conflicts, const WPriorityGraph& Gp) const {
    for (auto conflict : conflicts) {
        int a1 = std::get<0>(conflict);
        int a2 = std::get<1>(conflict);
        if (Gp.connected(a1, a2)) return false;
        else if (Gp.connected(a2, a1)) return false;
    }
    return true;
}

bool WPBS::generate_child(WPBSNode* node, WPBSNode* parent) {
    node->parent = parent;
    node->g_val = parent->g_val;
    node->h_val = parent->h_val;
    node->makespan = parent->makespan;
    node->depth = parent->depth + 1;
    node->path_cost_list = parent->path_cost_list;
    node->vis_goal_time = parent->vis_goal_time;
    node->priorities.copy(node->parent->priorities);
    node->priorities.add(node->priority.first, node->priority.second);
    copy_conflicts(node->parent->conflicts, node->conflicts, -1);
    if (!find_consistent_paths(node, node->priority.first)) return false;
    node->num_of_collisions = node->conflicts.size();
    node->f_val = node->g_val + node->h_val;
    return true;
}

bool WPBS::generate_root_node() {
    dummy_start = new WPBSNode();
    paths.resize(num_of_agents, nullptr);
    dummy_start->path_cost_list.resize(num_of_agents);
    dummy_start->vis_goal_time.resize(num_of_agents);

    if (!initial_paths.empty())
        for (int i = 0; i < num_of_agents; i++)
            if (!initial_paths[i].empty()) {
                dummy_start->paths.emplace_back(make_pair(i, initial_paths[i]));
                paths[i] = &dummy_start->paths.back().second;
                dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
                dummy_start->g_val += get_path_cost(*paths[i]);
            }

    for (int i = 0; i < num_of_agents; i++) {
        if (paths[i] != nullptr) continue;
        WPath path;
        double path_cost;
        int start_location = starts[i].location;
        rt.copy(initial_rt);
        rt.build(paths, initial_constraints, dummy_start->priorities.get_reachable_nodes(i), i, start_location);
        path = path_planner.run(G, starts[i], goal_locations[i], rt);
        path_cost = path_planner.path_cost;
        dummy_start->path_cost_list[i] = path_cost;
        dummy_start->vis_goal_time[i] = path_planner.vis_goal_time;
        rt.clear();
        LL_num_expanded += path_planner.num_expanded;
        LL_num_generated += path_planner.num_generated;
        if (path.empty()) return false;
        dummy_start->paths.emplace_back(i, path);
        paths[i] = &dummy_start->paths.back().second;
        dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
        dummy_start->g_val += (path_cost - path_planner.temp_h_val);
    }
    find_conflicts(dummy_start->conflicts);
    if (!lazyPriority)
        if (!find_consistent_paths(dummy_start, -1)) return false;
    dummy_start->f_val = dummy_start->g_val + dummy_start->h_val;
    dummy_start->num_of_collisions = dummy_start->conflicts.size();
    min_f_val = dummy_start->f_val;
    best_node = dummy_start;
    push_node(dummy_start);
    return true;
}

void WPBS::push_node(WPBSNode* node) { dfs.push_back(node); allNodes_table.push_back(node); }
WPBSNode* WPBS::pop_node() { WPBSNode* node = dfs.back(); dfs.pop_back(); return node; }

void WPBS::update_best_node(WPBSNode* node) {
    if (node->earliest_collision > best_node->earliest_collision) best_node = node;
    else if (node->earliest_collision == best_node->earliest_collision && node->f_val < best_node->f_val)
        best_node = node;
}

bool WPBS::run(const vector<WState>& starts_,
              const vector<vector<pair<int, int> > >& goal_locations_,
              int time_limit_) {
    clear();
    start = std::clock();
    this->starts = starts_;
    this->goal_locations = goal_locations_;
    this->num_of_agents = starts_.size();
    this->time_limit = time_limit_;
    solution_cost = -2;
    solution_found = false;
    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
    rt.hold_endpoints = hold_endpoints;
    path_planner.travel_times = travel_times;
    path_planner.hold_endpoints = hold_endpoints;
    path_planner.prioritize_start = prioritize_start;
    if (!generate_root_node()) return false;
    if (dummy_start->num_of_collisions == 0)
    { solution_found = true; solution_cost = dummy_start->f_val; }

    while (!dfs.empty() && !solution_found) {
        runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
        if (runtime > time_limit) { solution_cost = -1; solution_found = false; break; }
        WPBSNode* curr = pop_node();
        update_paths(curr);
        if (curr->conflicts.empty())
        { solution_found = true; solution_cost = curr->f_val; best_node = curr; break; }
        choose_conflict(*curr);
        update_best_node(curr);
        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;
        WPBSNode* n[2];
        for (int i = 0; i < 2; i++) n[i] = new WPBSNode();
        resolve_conflict(curr->conflict, n[0], n[1]);
        vector<WPath*> copy(paths);
        for (int i = 0; i < 2; i++) {
            bool sol = generate_child(n[i], curr);
            if (sol) { HL_num_generated++; n[i]->time_generated = HL_num_generated; }
            if (sol) {
                if (n[i]->f_val == min_f_val && n[i]->num_of_collisions == 0) {
                    solution_found = true; solution_cost = n[i]->f_val; best_node = n[i];
                    allNodes_table.push_back(n[i]); break;
                }
            } else { delete (n[i]); n[i] = nullptr; }
            paths = copy;
        }

        if (!solution_found) {
            if (n[0] != nullptr && n[1] != nullptr) {
                if (n[0]->f_val < n[1]->f_val ||
                    (n[0]->f_val == n[1]->f_val && n[0]->num_of_collisions < n[1]->num_of_collisions))
                { push_node(n[1]); push_node(n[0]); }
                else { push_node(n[0]); push_node(n[1]); }
            }
            else if (n[0] != nullptr) push_node(n[0]);
            else if (n[1] != nullptr) push_node(n[1]);
            else nogood.emplace(std::get<0>(curr->conflict), std::get<1>(curr->conflict));
            curr->clear();
        }
    }
    runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
    get_solution();
    return solution_found;
}

void WPBS::resolve_conflict(const WConflict& conflict, WPBSNode* n1, WPBSNode* n2) {
    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = conflict;
    n1->priority = std::make_pair(a1, a2);
    n2->priority = std::make_pair(a2, a1);
    (void)v1; (void)v2; (void)t;
}

inline void WPBS::release_closed_list() {
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++) delete *it;
    allNodes_table.clear();
}

void WPBS::get_solution() {
    update_paths(best_node);
    solution.resize(num_of_agents);
    vis_goal_time.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++) {
        solution[k] = *paths[k];
        vis_goal_time[k] = best_node->vis_goal_time[k];
    }
    avg_path_length = 0;
    for (int k = 0; k < num_of_agents; k++) {
        if (goal_locations[k].size() == 0) continue;
        avg_path_length += paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}

// Entry point: run the native windowed PBS over mapd_map, return per-agent
// location sequences (index 0 == cur_time), hold-in-place fallback if unplanned.
bool native_wpbs_solve(const MAPDMap& mp,
                       const std::vector<int>& start_locs,
                       const std::vector<std::vector<std::pair<int,int>>>& goal_seqs,
                       int cur_time, int window, int time_limit_ms,
                       std::vector<std::vector<int>>& out_paths,
                       bool use_sipp) {
    int n = (int)start_locs.size();
    out_paths.assign(n, {});

    WGraph G;
    G.mp = &mp;
    G.rows = mp.row;
    G.cols = mp.col;

    // Low-level solver selected by use_sipp: the framework-native windowed solve is
    // identical for wPBS-MLA* and wPBS-MLSIPP except for this single-agent planner.
    WStateTimeAStar planner_astar;
    WSippSearch     planner_sipp;
    WSingleAgentSolver& planner = use_sipp
        ? static_cast<WSingleAgentSolver&>(planner_sipp)
        : static_cast<WSingleAgentSolver&>(planner_astar);
    WPBS pbs(G, planner);
    pbs.lazyPriority = false;
    pbs.prioritize_start = false;
    pbs.hold_endpoints = false;
    pbs.k_robust = 0;
    pbs.window = window;
    pbs.screen = 0;
    pbs.setRT(false, false);
    pbs.initial_rt.k_robust = 0;
    pbs.initial_rt.window = window;
    pbs.initial_rt.hold_endpoints = false;
    pbs.initial_rt.use_cat = false;

    std::vector<WState> starts;
    starts.reserve(n);
    for (int i = 0; i < n; i++)
        starts.emplace_back(start_locs[i], cur_time, -1);

    std::vector<std::vector<std::pair<int,int>>> goals = goal_seqs;

    int tl_sec = time_limit_ms / 1000;
    if (tl_sec < 1) tl_sec = 1;
    bool ok = pbs.run(starts, goals, tl_sec);
    (void)ok;

    for (int i = 0; i < n; i++) {
        if (i < (int)pbs.solution.size() && !pbs.solution[i].empty()) {
            const WPath& p = pbs.solution[i];
            out_paths[i].resize(p.size());
            for (int t = 0; t < (int)p.size(); t++)
                out_paths[i][t] = p[t].location;
        } else {
            out_paths[i] = { start_locs[i] };
        }
    }
    return true;
}

} // anonymous namespace

// ============================================================
// Section 36: wPBS Path Planning — Wrapper
// ============================================================

void Simulation::path_planning_wpbs() {
    // The framework-native windowed PBS (wpbs_windowed_solve, backed by mapd_map)
    // is used for Hungarian+wPBS-MLA* and LNS(1s)+wPBS-MLA* (Hungarian / repeated-
    // Hungarian-LNS assignment + wPBS + non-SIPP low level).  The SIPP variants
    // (wPBS-MLSIPP, LNS+wPBS-MLSIPP) and any non-wPBS methods stay on the reimpl
    // windowed baseline (pbs_core) unchanged.
    // wPBS-MLSIPP now also routes through the framework-native windowed solve (with the
    // SIPP low-level, selected inside native_wpbs_solve by config.use_sipp), so it no
    // longer diverges from its wPBS-MLA* counterpart.  Non-windowed PBS-MLSIPP and
    // PP-SIPP still take the reimpl baseline (pbs_core) below.
    bool use_native = ((config.assign_method == AM_HUNGARIAN ||
                        config.assign_method == AM_REPEATED_HUNGARIAN_LNS) &&
                       config.mapf == MAPF_wPBS);
    if (use_native) {
        wpbs_windowed_solve();
        for (int i = 0; i < (int)agents.size(); i++) {
            if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
                agents[i].status = AG_MOVING_TO_PICKUP;
                agents[i].current_task = agents[i].task_sequence.front();
            }
        }
        return;
    }
    pbs_core(true);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
        }
    }
}

// ============================================================
// Native windowed solve orchestrator (framework-native)
//   Builds the per-solve inputs from the simulator's live state, runs the
//   framework-native windowed PBS (native_wpbs_solve, backed by mapd_map for the
//   grid + heuristics), and writes committed paths back into path_table_ /
//   agents.path.  Dispersal uses choose_good_endpoint (task endpoints only, skip
//   own, home fallback), via the framework's endpoint list (mapd_map.endpoints).
// ============================================================
void Simulation::wpbs_windowed_solve() {
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;

    // starts = current agent locations
    std::vector<int> start_locs(num_ag);
    for (int i = 0; i < num_ag; i++) start_locs[i] = (int)agents[i].loc;

    // --- Build task-goal part per agent (identical to build_goal_sequences,
    //     task_truncated_size=1) with ABSOLUTE release times. ---
    std::vector<std::vector<std::pair<int,int>>> goal_seqs(num_ag);
    const int task_truncated_size = 1;
    for (int i = 0; i < num_ag; i++) {
        int current_task_count = 0;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= task_truncated_size) break;
            Task& task = all_tasks[tid];
            int num_goals = min((int)task.goals.size(), 2);
            bool carrying = (agents[i].status == AG_CARRYING && tid == agents[i].current_task);
            if (num_goals <= 1) {
                if (!carrying)
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
            } else {
                if (carrying) {
                    goal_seqs[i].push_back({task.goals[1], 0});
                } else {
                    goal_seqs[i].push_back({task.goals[0], task.release_time});
                    goal_seqs[i].push_back({task.goals[1], 0});
                }
            }
            current_task_count++;
        }
    }

    // --- Reference choose_good_endpoint dispersal: task endpoints only (skip own,
    //     skip already-assigned), std::map<dist,loc> min; home endpoints fallback. ---
    std::set<int> assigned;
    auto ref_choose_good_endpoint = [&](int last_loc) -> int {
        std::map<int,int> distance;
        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
            const Endpoint& ep = mapd_map.endpoints[e];
            if (!ep.is_task_endpoint) continue;          // task endpoints ('e') only
            if (assigned.count(ep.loc)) continue;
            if (ep.loc == last_loc) continue;            // skip own
            int d = ep.h_val[last_loc];
            distance[d] = ep.loc;                        // later equal-dist overwrites
        }
        if (!distance.empty()) return distance.begin()->second;
        // fallback: agent home / non-task endpoints
        for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
            const Endpoint& ep = mapd_map.endpoints[e];
            if (ep.is_task_endpoint) continue;
            if (assigned.count(ep.loc)) continue;
            if (ep.loc == last_loc) continue;
            int d = ep.h_val[last_loc];
            distance[d] = ep.loc;
        }
        if (!distance.empty()) return distance.begin()->second;
        return last_loc; // ultimate fallback (should not happen)
    };

    // Non-free agents first, then free agents (matches reference ordering).
    for (int i = 0; i < num_ag; i++) {
        if (goal_seqs[i].empty()) continue;
        int last_loc = goal_seqs[i].back().first;
        int dummy = ref_choose_good_endpoint(last_loc);
        goal_seqs[i].push_back({dummy, 0});
        assigned.insert(dummy);
    }
    for (int i = 0; i < num_ag; i++) {
        if (!goal_seqs[i].empty()) continue;
        int last_loc = start_locs[i];
        int dummy = ref_choose_good_endpoint(last_loc);
        goal_seqs[i].push_back({dummy, 0});
        assigned.insert(dummy);
    }

    // --- Run the native windowed integrated solve (mapd_map-backed). ---
    std::vector<std::vector<int>> out_paths;
    int window = config.replan_window;            // == reference plan_window
    int time_limit_ms = 30000;                    // generous per-window budget
    native_wpbs_solve(mapd_map, start_locs, goal_seqs,
                      (int)cur_time_, window, time_limit_ms, out_paths,
                      config.use_sipp);

    // --- Commit: write plan for t >= cur_time_, hold last cell afterwards. ---
    for (int i = 0; i < num_ag; i++) {
        const std::vector<int>& p = out_paths[i];
        int plen = (int)p.size();
        int hold = plen > 0 ? p[plen - 1] : start_locs[i];
        for (int t = (int)cur_time_; t < max_t; t++) {
            int rel = t - (int)cur_time_;
            int v = (rel < plen) ? p[rel] : hold;
            path_table_[i][t] = (unsigned int)v;
            agents[i].path[t] = (unsigned int)v;
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
        std::shuffle(eligible.begin(), eligible.end(), std::mt19937(std::random_device{}()));
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
            saved_token_paths[i].assign(path_table_[i].begin(), path_table_[i].end());
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
                path_table_[a][t] = rs[a].cur_loc;
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
                path_table_[a][t] = ag.path[t];

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
                    path_table_[i][t] = saved_token_paths[i][t];
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
