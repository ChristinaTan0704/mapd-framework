// ============================================================================
//  Unified MAPD Framework — TP/TPTS, CENTRAL, TA, Hungarian, or LNS assignment
//                          — TP/TPTS with sequential STA* or MLSIPP
//                          — CENTRAL/CENTRAL-fixed with segment-by-segment CBS
//                            through arbitrary ordered task goals
//                          — TA-Prioritized with LKH3 assignment and STA*
//                          — PBS with MLA* or MLSIPP
//                          — wPBS with MLA* or MLSIPP
//                          — PP with MLSIPP
//
//  Preset (config.h : "LNS_PBS"):
//      mode              = ONLINE / OFFLINE / SEMI_ONLINE
//      assign_method     = DECOUPLED_GREEDY / DECOUPLED_GREEDY_SWAPS /
//                          CENTRALIZED_GREEDY / CENTRAL_HUNGARIAN /
//                          REPEATED_HUNGARIAN /
//                          LKH3_TSP / REPEATED_HUNGARIAN_LNS
//      assign_trigger    = ON_FREE_WAITS / EVERY_TIMESTEP /
//                          ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE
//      mapf              = CBS / PBS / wPBS / PP_PER_TASK / PP_TASK_SEQUENCE
//      single_agent      = STA_TASK_EP / SEQ_STA / MLA_SEQUENCE / MLSIPP_SEQUENCE
//      dummy_path       = true / false
//      endpoint_strategy = NEAREST_WITH_STRICT_EXCLUSIONS
//      task_sequence_limit = maximum tasks included in each PBS/wPBS plan
//
//  Layout follows the unified pseudocode:
//      Section 0   Initialisation                       (Algorithm 1, lines 1-6)
//      Section 1   Main loop                            (Algorithm 1, lines 7-13)
//      Section 2   Computation dispatchers              (Algorithm 1, line 12)
//      Section 3   Task assignment  TP / TPTS / HUNGARIAN / LKH3_TSP / LNS
//      Section 4   Task sequences -> goal sequences
//      Section 5   MAPF             PBS
//      Section 6   Single agent     STA* / MLA* / MLSIPP
//      Section 7   System update    (Algorithm 1, line 13)
//      Section 8   Reporting
//
//  Every dispatcher switches on a config axis and rejects the values that are
//  not implemented yet, so another algorithm is added by filling in one case.
// ============================================================================
#include "simulation.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <new>
#include <random>
#include <stdexcept>
#include <dlib/optimization/max_cost_assignment.h>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

// ============================================================================
//  Section 0 — Initialisation                            (Algorithm 1, 1-6)
// ============================================================================

void Simulation::init(const string& map_file, const string& task_file,
                      const MAPDConfig& cfg, const string& tour_file) {
    config = cfg;
    RandomTieBreaker::seed(config.seed);
    tour_file_ = tour_file;

    // --- instance ---
    mapd_map.load(map_file);
    maxtime = mapd_map.maxtime;
    all_tasks = load_tasks(task_file, mapd_map.endpoints);

    // Index tasks by release time so release_tasks() is O(#released).
    task_indices_by_time.resize(maxtime);
    t_task = 0;
    for (int i = 0; i < (int)all_tasks.size(); i++) {
        int rt = all_tasks[i].release_time;
        if (rt >= 0 && rt < (int)maxtime) {
            task_indices_by_time[rt].push_back(i);
            if (rt > t_task) t_task = rt;
        }
    }

    // --- Algorithm 1 line 1-2: t <- 0, T <- {} ---
    cur_time_ = 0;
    last_released_time_ = -1;   // release_tasks() still has to run for t = 0
    open_tasks_.clear();

    // --- Algorithm 1 line 5-6: empty task sequences, trivial paths pi_i[0] = v_init_i ---
    agents.resize(mapd_map.num_agents);
    path_table_.resize(mapd_map.num_agents);
    for (int i = 0; i < mapd_map.num_agents; i++) {
        agents[i].init(i, mapd_map.agent_starts[i], mapd_map.col, mapd_map.row, maxtime);
        path_table_[i].assign(maxtime, mapd_map.agent_starts[i]);
    }

    new_or_deferred_task_event_ = false;
    new_available_agent_ = false;
    last_path_planning_time_ = 0;
}

// ============================================================================
//  Section 1 — Algorithm 1: Unified MAPD Framework
//
//      while not End():
//          T <- T u {newly released tasks}          release_tasks()
//          (re)assign tasks and (re)plan paths      task_assignment_and_path_planning()
//          agents move along their paths, t <- t+1  advance_time()
// ============================================================================

void Simulation::run() {
    while (!end()) {
        release_tasks();                       // Algorithm 1, lines 8-11
        task_assignment_and_path_planning();   // Algorithm 1, line 12
        advance_time();                        // Algorithm 1, line 13
    }
}

// --- End() ------------------------------------------------------------------
// Done once nothing is pending anywhere: no released-but-unassigned task, no
// future release, no queued task in any agent's sequence, nobody still moving.
bool Simulation::end() const {
    if (!open_tasks_.empty()) return false;
    if ((int)cur_time_ <= t_task) return false;
    if (!all_task_sequences_empty()) return false;
    if (any_agent_busy()) return false;
    return true;
}

bool Simulation::all_task_sequences_empty() const {
    for (auto& a : agents)
        if (!a.task_sequence.empty()) return false;
    return true;
}

// --- GetReleasedTasks() -----------------------------------------------------
// Algorithm 1 lines 8-11, switching on config.mode.  advance_time() may jump
// several timesteps at once, so this releases the whole skipped interval.
void Simulation::release_tasks() {
    int from = last_released_time_ + 1;
    int reveal_through = (int)cur_time_;
    bool revealed_new_task = false;

    switch (config.mode) {
    case MODE_ONLINE:
        // Online tasks become known only at their actual release timestep.
        for (int t = from; t <= reveal_through && t < (int)maxtime; t++)
            for (int idx : task_indices_by_time[t]) {
                open_tasks_.push_back(&all_tasks[idx]);
                revealed_new_task = true;
            }
        break;

    case MODE_SEMI_ONLINE: {
        // The paper defines the look-ahead horizon in release batches, not in
        // raw timesteps. L=1 therefore reveals the current batch and the next
        // distinct future batch. At each actual release, the horizon advances
        // and reveals one more future batch.
        int future_batches = 0;
        if (config.semi_online_lookahead_batches > 0) {
            for (int t = (int)cur_time_ + 1; t < (int)maxtime; t++) {
                if (task_indices_by_time[t].empty()) continue;
                reveal_through = t;
                if (++future_batches >= config.semi_online_lookahead_batches)
                    break;
            }
        }

        for (int t = from; t <= reveal_through && t < (int)maxtime; t++)
            for (int idx : task_indices_by_time[t]) {
                open_tasks_.push_back(&all_tasks[idx]);
                revealed_new_task = true;
            }
        break;
    }

    case MODE_OFFLINE:
        // All tasks are known up front.
        if (last_released_time_ < 0)
            for (int i = 0; i < (int)all_tasks.size(); i++) {
                open_tasks_.push_back(&all_tasks[i]);
                revealed_new_task = true;
            }
        break;
    }

    last_released_time_ = max(last_released_time_, reveal_through);
    if (revealed_new_task &&
        (config.assign_trigger == AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE ||
         config.assign_trigger ==
            AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE))
        new_or_deferred_task_event_ = true;
}

// ============================================================================
//  Section 2 — Computation                               (Algorithm 1, line 12)
//
//  The two halves of the framework: decide WHO does WHAT (task assignment),
//  then decide HOW they get there (path planning).  Each half has its own
//  trigger so an algorithm can re-plan without re-assigning.
// ============================================================================

void Simulation::task_assignment_and_path_planning() {
    const bool assignment_triggered = should_assign();
    if (assignment_triggered)
        task_assignment();      // assignment half

    if (should_replan(assignment_triggered)) {
        path_planning(assignment_triggered); // path-planning half
        last_path_planning_time_ = cur_time_;
    }
}

// --- ShouldAssign() : switch on assign_trigger ---
bool Simulation::should_assign() {
    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        // TP processes one available agent per framework iteration. This stays
        // true even when no task is open so the selected agent can be advanced
        // by one wait step, matching token-passing's no-task branch.
        for (const auto& agent : agents)
            if (agent.finish_time <= cur_time_) return true;
        return false;
    case AT_EVERY_TIMESTEP:
        // CENTRAL assigns endpoints to every non-carrying agent at every
        // timestep, exactly as described by the centralized algorithm.
        return true;
    case AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE: {
        // CENTRAL-fixed changes the free-agent assignment only when new work
        // arrives or an occupied agent completes its delivery.
        const bool assignment_event =
            new_or_deferred_task_event_ || new_available_agent_;
        if (assignment_event) {
            new_or_deferred_task_event_ = false;
            new_available_agent_ = false;
        }
        return assignment_event;
    }
    case AT_ONCE:
        return cur_time_ == 0 && !open_tasks_.empty();
    case AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE: {
        const bool assignment_event =
            new_or_deferred_task_event_ || new_available_agent_;
        if (assignment_event) { // reset variable
            new_or_deferred_task_event_ = false;
            new_available_agent_ = false;
        }
        return assignment_event;
    }
    default:
        cerr << "should_assign: assign_trigger not implemented in this build" << endl;
        return false;
    }
}

// --- AssignTask() : switch on assign_method ---
void Simulation::task_assignment() {
    switch (config.assign_method) {
    case AM_DECOUPLED_GREEDY:
        assign_decoupled_greedy_step();
        break;
    case AM_DECOUPLED_GREEDY_SWAPS:
        assign_tpts_step();
        break;
    case AM_CENTRALIZED_GREEDY:
        assign_hbh_mla();
        break;
    case AM_CENTRAL_HUNGARIAN:
        assign_central_hungarian();
        break;
    case AM_REPEATED_HUNGARIAN:
        assign_repeated_hungarian();
        truncate_online_task_sequences();
        break;
    case AM_REPEATED_HUNGARIAN_LNS:
        assign_repeated_hungarian_lns();
        truncate_online_task_sequences();
        break;
    case AM_LKH3_TSP:
        assign_ta_tsp();
        break;
    case AM_LKH3_TSP_REASSIGN:
        assign_ta_hybrid();
        break;
    default:
        cerr << "task_assignment: assign_method not implemented in this build" << endl;
        break;
    }
}

// --- ShouldReplan() : switch on assign_trigger ---
bool Simulation::should_replan(bool assignment_triggered) const {
    switch (config.assign_trigger) {
    case AT_ON_FREE_WAITS:
        // TP's STA* path is planned and committed inside task assignment.
        return false;
    case AT_EVERY_TIMESTEP:
        return true;
    case AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE:
        if (assignment_triggered) return true;
        // Reaching a task goal does not rerun Hungarian assignment. It only
        // invokes the first CBS group for agents that need their next segment.
        for (const Agent& agent : agents)
            if (agent.status == AG_CARRYING &&
                agent.current_task >= 0 &&
                agent.current_task < (int)all_tasks.size() &&
                agent.current_goal_index >= 1 &&
                agent.current_goal_index <
                    (int)all_tasks[agent.current_task].goals.size() &&
                (all_tasks[agent.current_task].ag_arrive_start ==
                    (int)cur_time_ || agent.finish_time <= cur_time_))
                return true;
        return false;
    case AT_ONCE:
        return assignment_triggered;
    case AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE:
        // PBS/PP plan after assignment. wPBS additionally refreshes paths at
        // fixed window boundaries without rerunning task assignment.
        if (assignment_triggered) return true;

        // TA-Hybrid invokes Group 1 whenever a task agent reaches an ordered
        // goal and needs the next segment of that same multi-goal task.
        if (config.mapf == MAPF_TA_HYBRID_TWO_GROUP) {
            for (const Agent& agent : agents)
                if (agent.status == AG_CARRYING &&
                    agent.current_task >= 0 &&
                    agent.current_task < (int)all_tasks.size() &&
                    agent.current_goal_index <
                        (int)all_tasks[agent.current_task].goals.size() &&
                    agent.finish_time <= cur_time_)
                    return true;
        }

        if (config.mapf == MAPF_wPBS && !all_task_sequences_empty())
            return (int)cur_time_ - (int)last_path_planning_time_ >=
                   config.wpbs_replan_window;
        return false;
    default:
        cerr << "should_replan: assign_trigger not implemented in this build" << endl;
        return false;
    }
}

// --- Path_Planning() : switch on mapf ---
void Simulation::path_planning(bool assignment_triggered) {
    switch (config.mapf) {
    case MAPF_PP_PER_TASK:
        path_planning_pp_per_task();
        break;
    case MAPF_PP_TASK_SEQUENCE:
        path_planning_pp_task_sequence();
        break;
    case MAPF_CBS:
        path_planning_ecbs(assignment_triggered);
        break;
    case MAPF_PBS:
        path_planning_pbs();
        break;
    case MAPF_wPBS:
        path_planning_wpbs();
        break;
    case MAPF_TA_HYBRID_TWO_GROUP:
        path_planning_ta_hybrid(assignment_triggered);
        break;
    default:
        cerr << "path_planning: mapf not implemented in this build" << endl;
        break;
    }
}

// ============================================================================
//  Section 3 — Task assignment
// ============================================================================

// --- TP: decoupled greedy assignment ---------------------------------------
// Select exactly one currently available agent. The ordering is the explicit,
// deterministic pair (finish_time, agent_id); no separate timestep-transition
// flag is needed.
void Simulation::assign_decoupled_greedy_step() {
    Agent* agent = &agents[0];
    for (int i = 1; i < (int)agents.size(); i++)
        if (agents[i].finish_time < agent->finish_time ||
            (agents[i].finish_time == agent->finish_time &&
             agents[i].id < agent->id))
            agent = &agents[i];
    if (agent->finish_time <= cur_time_)
        assign_decoupled_greedy(*agent);
}

bool Simulation::assign_decoupled_greedy(Agent& agent) {
    agent.loc = agent.path[cur_time_];

    // A TP task may be selected only when none of its ordered goals is
    // permanently held by another agent's currently committed path.
    vector<bool> held(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)path_table_.size(); i++)
        if (i != agent.id) held[path_table_[i][maxtime - 1]] = true;

    Task* best_task = nullptr;
    for (Task* task : open_tasks_) {
        bool goal_held = false;
        for (int goal_loc : task->goals)
            if (held[goal_loc]) {
                goal_held = true;
                break;
            }
        if (goal_held) continue;
        if (best_task == nullptr ||
            mapd_map.endpoints[task->pickup].h_val[agent.loc] <
            mapd_map.endpoints[best_task->pickup].h_val[agent.loc])
            best_task = task;
    }

    if (best_task != nullptr) {
        pair<int,int> arrival = plan_token_task(agent, *best_task);
        if (arrival.first >= 0) {
            for (unsigned int t = cur_time_; t < path_table_[agent.id].size(); t++)
                path_table_[agent.id][t] = agent.path[t];

            agent.finish_time = arrival.second + best_task->goal_wait_time;
            agent.current_task = best_task->id;
            agent.current_goal_index = 0;
            best_task->status = agent.id;
            best_task->ag_arrive_start = arrival.first;
            best_task->completion_time = arrival.second;
            open_tasks_.remove(best_task);
            return true;
        }
    }

    // WAIT_OR_NEAREST_SAFE returns the current location when waiting is safe, another
    // endpoint when relocation is required, and -1 if no safe endpoint exists.
    const vector<int> no_assigned_dummies;
    int endpoint = choose_dummy_endpoint(
        agent.id, agent.loc, no_assigned_dummies);
    if (endpoint >= 0 && endpoint != agent.loc &&
        plan_path2_to_endpoint(agent, endpoint)) {
        return true;
    }

    agent.finish_time = cur_time_ + 1;
    return true;
}

// --- TPTS: decoupled greedy assignment with task swaps ---------------------
void Simulation::tpts_purge_picked_up_tasks() {
    auto task = open_tasks_.begin();
    while (task != open_tasks_.end()) {
        if ((*task)->status >= 0 && (*task)->ag_arrive_start >= 0 &&
            (int)cur_time_ >= (*task)->ag_arrive_start)
            task = open_tasks_.erase(task);
        else
            ++task;
    }
}

void Simulation::assign_tpts_step() {
    Agent* agent = &agents[0];
    for (int i = 1; i < (int)agents.size(); i++)
        if (agents[i].finish_time < agent->finish_time ||
            (agents[i].finish_time == agent->finish_time &&
             agents[i].id < agent->id))
            agent = &agents[i];

    // A TPTS task remains in the token while its assigned agent is travelling
    // to its first goal, allowing another agent with an earlier arrival to
    // steal it. Once that first goal is reached, execution of the ordered
    // multi-goal task has begun and the task can no longer be swapped.
    tpts_purge_picked_up_tasks();
    if (agent->finish_time <= cur_time_)
        assign_tpts(*agent);
}

bool Simulation::assign_tpts(Agent& agent, int depth) {
    if (depth >= (int)agents.size()) return false;

    vector<vector<unsigned int>> saved_paths = path_table_;
    vector<Agent> saved_agents = agents;
    agent.loc = agent.path[cur_time_];

    struct Candidate {
        Task* task;
        int distance;
        Candidate(Task* candidate_task, int candidate_distance)
            : task(candidate_task), distance(candidate_distance) {}
    };
    struct CompareCandidate {
        bool operator()(const Candidate& lhs, const Candidate& rhs) const {
            return lhs.distance > rhs.distance;
        }
    };

    boost::heap::fibonacci_heap<Candidate,
        boost::heap::compare<CompareCandidate>> candidates;
    for (Task* task : open_tasks_)
        candidates.push(Candidate(
            task, mapd_map.endpoints[task->pickup].h_val[agent.loc]));

    while (!candidates.empty()) {
        Candidate candidate = candidates.top();
        candidates.pop();
        Task* task = candidate.task;

        const bool unassigned = task->status == -1;
        const bool can_arrive_earlier =
            task->status >= 0 && task->ag_arrive_start >= 0 &&
            task->ag_arrive_start > (int)cur_time_ + candidate.distance;
        if (!unassigned && !can_arrive_earlier) continue;

        bool endpoint_occupied = false;
        for (int other = 0; other < (int)path_table_.size(); other++) {
            if (other == agent.id) continue;
            if (task->status >= 0 && other == task->status) continue;
            for (int goal_loc : task->goals)
                if (path_table_[other][maxtime - 1] ==
                    (unsigned int)goal_loc) {
                    endpoint_occupied = true;
                    break;
                }
            if (endpoint_occupied) break;
        }
        if (endpoint_occupied) continue;

        // Algorithm 2 treats each possible swap as a transaction.  Path
        // planning and recursive reassignment temporarily mutate this agent's
        // path/reservation and the candidate task's ownership timestamps.  If
        // the candidate fails, restore exactly the state seen before it so the
        // next candidate is evaluated against an unmodified token.
        const Agent candidate_agent_before = agent;
        const vector<unsigned int> candidate_reservation_before =
            path_table_[agent.id];
        const int candidate_status_before = task->status;
        const int candidate_arrival_before = task->ag_arrive_start;
        const int candidate_completion_before = task->completion_time;
        auto rollback_candidate = [&]() {
            agent = candidate_agent_before;
            path_table_[agent.id] = candidate_reservation_before;
            task->status = candidate_status_before;
            task->ag_arrive_start = candidate_arrival_before;
            task->completion_time = candidate_completion_before;
        };

        int hidden_agent = task->status >= 0 ? task->status : -1;
        pair<int,int> arrival = plan_token_task(agent, *task, hidden_agent);
        if (arrival.first < 0 ||
            (!unassigned && arrival.first >= task->ag_arrive_start)) {
            rollback_candidate();
            continue;
        }

        for (unsigned int t = cur_time_; t < path_table_[agent.id].size(); t++)
            path_table_[agent.id][t] = agent.path[t];
        agent.finish_time = arrival.second + task->goal_wait_time;

        if (unassigned) {
            task->status = agent.id;
            task->ag_arrive_start = arrival.first;
            task->completion_time = arrival.second;
            return true;
        }

        Agent* displaced_agent = &agents[task->status];
        task->status = agent.id;
        task->ag_arrive_start = arrival.first;
        task->completion_time = arrival.second;
        if (assign_tpts(*displaced_agent, depth + 1)) return true;

        // The displaced agent could not be reassigned. Algorithm 2 requires
        // the tentative path, task owner, and timing fields to be restored
        // before another candidate is considered.
        rollback_candidate();
    }

    const vector<int> no_assigned_dummies;
    int endpoint = choose_dummy_endpoint(
        agent.id, agent.loc, no_assigned_dummies);
    if (endpoint == agent.loc) {
        for (unsigned int t = cur_time_ + 1; t < maxtime; t++) {
            agent.path[t] = agent.path[cur_time_];
            path_table_[agent.id][t] = agent.path[cur_time_];
        }
        agent.finish_time = cur_time_ + 1;
        return true;
    }
    if (endpoint >= 0 && plan_path2_to_endpoint(agent, endpoint)) {
        return true;
    }

    path_table_ = saved_paths;
    // Restore in place so Agent references held by enclosing recursive calls
    // remain valid.
    for (int i = 0; i < (int)agents.size(); i++)
        agents[i] = saved_agents[i];
    return false;
}

// --- HBH-MLA*: centralized h-value greedy assignment ----------------------
// Build all available-agent/open-task pairs, scan them by nondecreasing
// distance to the first task goal, and commit a pair only when the existing
// MLA* planner finds a collision-free path through every ordered task goal.
void Simulation::assign_hbh_mla() {
    if (config.single_agent != SA_MLA_SEQUENCE)
        throw invalid_argument("HBH currently requires MLA* low-level search");

    vector<int> available_agents;
    for (Agent& agent : agents) {
        if (agent.finish_time > cur_time_) continue;

        // A previously assigned HBH task has now finished. Its completion time
        // was recorded when the full MLA* path was committed.
        agent.status = AG_FREE;
        agent.current_task = -1;
        agent.current_goal_index = 0;
        available_agents.push_back(agent.id);
    }
    if (available_agents.empty()) return;

    struct AgentTaskPair {
        int agent_id;
        Task* task;
        int h_value;
    };

    vector<AgentTaskPair> candidates;
    for (int agent_id : available_agents) {
        for (Task* task : open_tasks_) {
            if (task->status != -1 || task->goals.empty()) continue;
            int first_endpoint = mapd_map.ep_index(task->goals.front());
            if (first_endpoint < 0) continue;
            int h_value =
                mapd_map.endpoints[first_endpoint].h_val[agents[agent_id].loc];
            if (h_value < INT_MAX)
                candidates.push_back({agent_id, task, h_value});
        }
    }
    stable_sort(candidates.begin(), candidates.end(),
        [](const AgentTaskPair& lhs, const AgentTaskPair& rhs) {
            return lhs.h_value < rhs.h_value;
        });

    set<int> assigned_agents;
    set<int> assigned_tasks;
    const vector<vector<int>> no_old_paths;

    for (const AgentTaskPair& candidate : candidates) {
        if (assigned_agents.count(candidate.agent_id) ||
            assigned_tasks.count(candidate.task->id) ||
            candidate.task->status != -1)
            continue;

        Agent& agent = agents[candidate.agent_id];
        Task& task = *candidate.task;

        vector<pair<int,int>> ordered_goals;
        ordered_goals.reserve(task.goals.size());
        for (int goal_index = 0;
             goal_index < (int)task.goals.size(); goal_index++)
            ordered_goals.push_back({
                task.goals[goal_index],
                goal_index == 0 ? task.release_time : 0});
        vector<vector<pair<int,int>>> task_groups(1, ordered_goals);

        vector<vector<int>> constraints;
        constraints.reserve(agents.size() - 1);
        for (int other = 0; other < (int)agents.size(); other++)
            if (other != agent.id)
                constraints.emplace_back(
                    path_table_[other].begin(), path_table_[other].end());

        MLAStarPlanner planner(*this);
        vector<int> path = planner.solve(MLAStarRequest(
            agent.id, agent.loc, (int)cur_time_, task_groups,
            constraints, no_old_paths, false));
        if (path.empty()) continue;

        int first_goal_time = -1;
        int final_goal_time = -1;
        int goal_index = 0;
        for (int timestep = (int)cur_time_;
             timestep < (int)path.size() &&
             goal_index < (int)ordered_goals.size(); timestep++) {
            while (goal_index < (int)ordered_goals.size() &&
                   path[timestep] == ordered_goals[goal_index].first &&
                   timestep >= ordered_goals[goal_index].second) {
                if (goal_index == 0) first_goal_time = timestep;
                if (goal_index == (int)ordered_goals.size() - 1)
                    final_goal_time = timestep;
                goal_index++;
            }
        }
        if (first_goal_time < 0 || final_goal_time < 0) continue;

        int hold_location = path.back();
        for (unsigned int timestep = cur_time_; timestep < maxtime; timestep++) {
            int location = timestep < path.size() ? path[timestep] : hold_location;
            agent.path[timestep] = location;
            path_table_[agent.id][timestep] = location;
        }

        task.status = agent.id;
        task.ag_arrive_start = first_goal_time;
        task.completion_time = final_goal_time;
        agent.status = AG_MOVING_TO_PICKUP;
        agent.current_task = task.id;
        agent.current_goal_index = 0;
        agent.finish_time = final_goal_time + task.goal_wait_time;
        agent.last_endpoint = task.goals.back();
        open_tasks_.remove(&task);
        assigned_agents.insert(agent.id);
        assigned_tasks.insert(task.id);
    }

    // Paper Algorithm 2: an unassigned available agent waits unless its
    // current location is needed by an open task; in that case it relocates to
    // the closest free non-task endpoint selected by the unified policy.
    const vector<int> no_reserved_endpoints;
    for (int agent_id : available_agents) {
        if (assigned_agents.count(agent_id)) continue;
        Agent& agent = agents[agent_id];
        int endpoint = choose_dummy_endpoint(
            agent_id, agent.loc, no_reserved_endpoints);
        if (endpoint >= 0 && endpoint != agent.loc &&
            plan_path2_to_endpoint(agent, endpoint)) {
            agent.last_endpoint = endpoint;
            continue;
        }

        for (unsigned int timestep = cur_time_ + 1;
             timestep < maxtime; timestep++) {
            agent.path[timestep] = agent.loc;
            path_table_[agent.id][timestep] = agent.loc;
        }
        agent.finish_time = cur_time_ + 1;
        agent.last_endpoint = agent.loc;
    }
}

// --- CENTRAL: event preparation and phase-1 instant pickups ----------------
Task* Simulation::central_current_task(int agent_id) {
    int task_id = agents[agent_id].current_task;
    if (task_id < 0 || task_id >= (int)all_tasks.size()) return nullptr;
    return &all_tasks[task_id];
}

void Simulation::central_phase1_instant_pickup() {
    // Reserve every remaining goal of an executing MG-MAPD task. For ordinary
    // two-goal MAPD this is exactly the task's delivery endpoint.
    vector<bool> occupied_goal_held(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++) {
        Task* task = central_current_task(i);
        if (agents[i].status != AG_CARRYING || !task) continue;
        int first_remaining = max(1, agents[i].current_goal_index);
        for (int goal_index = first_remaining;
             goal_index < (int)task->goals.size(); goal_index++)
            occupied_goal_held[task->goals[goal_index]] = true;
    }

    vector<int> agent_at_location(mapd_map.row * mapd_map.col, -1);
    vector<int> endpoint_hold_count(mapd_map.row * mapd_map.col, 0);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status != AG_CARRYING)
            agent_at_location[agents[i].loc] = i;
        endpoint_hold_count[path_table_[i][maxtime - 1]]++;
    }

    vector<Task*> completed_at_pickup;
    for (Task* task : open_tasks_) {
        if (task->status != -1) continue;
        if (task->goals.empty()) continue;
        int pickup_location = task->goals.front();
        int agent_id = agent_at_location[pickup_location];
        if (agent_id < 0) continue;

        bool remaining_goal_blocked = false;
        int own_held_location = path_table_[agent_id][maxtime - 1];
        for (int goal_index = 1;
             goal_index < (int)task->goals.size(); goal_index++) {
            int goal_location = task->goals[goal_index];
            int other_hold_count = endpoint_hold_count[goal_location] -
                (own_held_location == goal_location ? 1 : 0);
            if (other_hold_count > 0 || occupied_goal_held[goal_location]) {
                remaining_goal_blocked = true;
                break;
            }
        }
        if (remaining_goal_blocked)
            continue;

        agent_at_location[pickup_location] = -1;
        Task* previous_task = central_current_task(agent_id);
        if (agents[agent_id].status == AG_MOVING_TO_PICKUP && previous_task &&
            previous_task != task) {
            previous_task->status = -1;
        }

        task->status = agent_id;
        task->ag_arrive_start = (int)cur_time_;
        agents[agent_id].status = AG_CARRYING;
        agents[agent_id].current_task = task->id;
        agents[agent_id].current_goal_index = 1;
        if (task->goals.size() == 1) {
            task->completion_time = (int)cur_time_;
            task->status = INT_MAX;
            agents[agent_id].status = AG_FREE;
            agents[agent_id].current_task = -1;
            agents[agent_id].current_goal_index = 0;
            agents[agent_id].last_endpoint = pickup_location;
            completed_at_pickup.push_back(task);
            continue;
        }

        agents[agent_id].last_endpoint = task->goals[1];
        for (int goal_index = 1;
             goal_index < (int)task->goals.size(); goal_index++)
            occupied_goal_held[task->goals[goal_index]] = true;
    }
    for (Task* task : completed_at_pickup)
        open_tasks_.remove(task);
}

int Simulation::central_path_cost(
        int /*agent_id*/, int start_loc, int goal_loc, int start_time,
        const vector<vector<int>>& constraints,
        const vector<char>* vertex_reservations, int reservation_size,
        const vector<int>* last_occupation,
        unordered_map<long long, int>& visited) {
    const int map_size = mapd_map.row * mapd_map.col;
    int goal_last_occupation = -1;
    if (last_occupation && goal_loc < (int)last_occupation->size()) {
        goal_last_occupation = (*last_occupation)[goal_loc];
    } else {
        for (const vector<int>& path : constraints)
            for (int t = (int)path.size() - 1; t >= 0; t--)
                if (path[t] == goal_loc) {
                    goal_last_occupation = max(goal_last_occupation, t);
                    break;
                }
    }

    if (start_loc == goal_loc && goal_last_occupation <= start_time)
        return 1;

    int endpoint_index = mapd_map.ep_index(goal_loc);
    if (endpoint_index < 0) return map_size;
    const Endpoint& goal = mapd_map.endpoints[endpoint_index];

    struct CostNode { int loc, g, f, time; uint64_t tie_breaker; };
    auto compare = [](const CostNode& lhs, const CostNode& rhs) {
        if (lhs.f != rhs.f) return lhs.f > rhs.f;
        if (lhs.g != rhs.g) return lhs.g < rhs.g;
        return lhs.tie_breaker > rhs.tie_breaker;
    };
    priority_queue<CostNode, vector<CostNode>, decltype(compare)> open(compare);
    visited.clear();

    int initial_h = goal.h_val[start_loc];
    if (initial_h == INT_MAX) return map_size;
    open.push({start_loc, 0, initial_h, start_time,
               RandomTieBreaker::next()});

    const int max_time = (int)maxtime - 1;
    const int actions[5] = {0, -mapd_map.col, 1, mapd_map.col, -1};
    int expanded = 0;

    while (!open.empty()) {
        CostNode current = open.top();
        open.pop();
        if (++expanded > 15000) return map_size;
        if (current.loc == goal_loc && current.time >= goal_last_occupation)
            return current.g + 1;
        if (current.time >= max_time) continue;

        long long key = (long long)current.loc + (long long)current.g * map_size;
        auto previous = visited.find(key);
        if (previous != visited.end() && previous->second <= current.g)
            continue;
        visited[key] = current.g;

        for (int action : actions) {
            int next_loc = current.loc + action;
            int next_time = current.time + 1;
            if (next_loc < 0 || next_loc >= map_size || !mapd_map.grid[next_loc])
                continue;

            bool blocked = false;
            int next_vertex = next_time * map_size + next_loc;
            int swap_vertex = next_time * map_size + current.loc;
            if (vertex_reservations && next_vertex < reservation_size &&
                swap_vertex < reservation_size) {
                if ((*vertex_reservations)[next_vertex]) {
                    blocked = true;
                } else if ((*vertex_reservations)[swap_vertex]) {
                    for (const vector<int>& path : constraints) {
                        int at_next = next_time < (int)path.size() ?
                            path[next_time] : path.back();
                        int at_current = current.time < (int)path.size() ?
                            path[current.time] : path.back();
                        if (at_next == current.loc && at_current == next_loc) {
                            blocked = true;
                            break;
                        }
                    }
                }
            } else {
                for (const vector<int>& path : constraints) {
                    int at_next = next_time < (int)path.size() ?
                        path[next_time] : path.back();
                    int at_current = current.time < (int)path.size() ?
                        path[current.time] : path.back();
                    if (at_next == next_loc ||
                        (at_next == current.loc && at_current == next_loc)) {
                        blocked = true;
                        break;
                    }
                }
            }
            if (blocked) continue;

            int h = goal.h_val[next_loc];
            if (h == INT_MAX) continue;
            open.push({next_loc, current.g + 1, current.g + 1 + h, next_time,
                       RandomTieBreaker::next()});
        }
    }
    return map_size;
}

void Simulation::assign_central_hungarian() {
    // At a CENTRAL assignment event, Phase 1 first turns every non-carrying
    // agent already resting at an eligible first goal into a task agent. The
    // remaining agents are rematched to first goals or pairwise-distinct
    // parking endpoints.
    central_phase1_instant_pickup();

    vector<int> agent_ids;

    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status == AG_FREE) {
            agent_ids.push_back(i);
        } else if (agents[i].status == AG_MOVING_TO_PICKUP) {
            Task* task = central_current_task(i);
            if (task) task->status = -1;
            agents[i].status = AG_FREE;
            agents[i].current_task = -1;
            agents[i].current_goal_index = 0;
            agent_ids.push_back(i);
        }
    }
    if (agent_ids.empty()) return;

    vector<bool> held(mapd_map.row * mapd_map.col, false);
    for (int i = 0; i < (int)agents.size(); i++) {
        Task* task = central_current_task(i);
        if (agents[i].status != AG_CARRYING || !task) continue;
        int first_remaining = max(1, agents[i].current_goal_index);
        for (int goal_index = first_remaining;
             goal_index < (int)task->goals.size(); goal_index++)
            held[task->goals[goal_index]] = true;
    }

    vector<Task*> candidate_tasks;
    vector<int> candidate_endpoint_ids;
    for (Task* task : open_tasks_) {
        if (task->status != -1 || task->goals.empty()) continue;

        // Generalize the paper's pickup/delivery test: all ordered goals of a
        // candidate task must avoid goals already reserved by executing tasks
        // and earlier candidates in T'. Repeated goals inside this one task
        // remain valid because they do not create an inter-task conflict.
        bool goal_conflict = false;
        for (int goal_location : task->goals)
            if (held[goal_location]) {
                goal_conflict = true;
                break;
            }
        if (goal_conflict) continue;

        int pickup_endpoint = mapd_map.ep_index(task->goals.front());
        if (pickup_endpoint < 0)
            throw runtime_error("CENTRAL task goal is not an endpoint");
        candidate_tasks.push_back(task);
        candidate_endpoint_ids.push_back(pickup_endpoint);
        for (int goal_location : task->goals)
            held[goal_location] = true;
    }
    int task_destination_count = candidate_tasks.size();

    // The unified endpoint selector receives every location already reserved
    // by a carrying task or a task in T'. Each new parking endpoint is appended
    // to the same local list, making all destinations pairwise distinct.
    vector<int> reserved_endpoints;
    for (int location = 0; location < (int)held.size(); location++)
        if (held[location]) reserved_endpoints.push_back(location);

    if ((int)candidate_endpoint_ids.size() < (int)agent_ids.size()) {
        for (int agent_id : agent_ids) {
            int endpoint_location = choose_dummy_endpoint(
                agent_id, agents[agent_id].loc, reserved_endpoints);
            int endpoint_id = mapd_map.ep_index(endpoint_location);
            if (endpoint_id < 0)
                throw runtime_error(
                    "CENTRAL could not select a valid parking endpoint");
            reserved_endpoints.push_back(endpoint_location);
            candidate_endpoint_ids.push_back(endpoint_id);
        }
    }

    int matrix_size = max(
        (int)agent_ids.size(), (int)candidate_endpoint_ids.size());
    if (matrix_size == 0) return;

    vector<vector<int>> fixed_paths;
    for (int i = 0; i < (int)agents.size(); i++) {
        bool is_free = find(agent_ids.begin(), agent_ids.end(), i) !=
                       agent_ids.end();
        if (!is_free)
            fixed_paths.emplace_back(path_table_[i].begin(), path_table_[i].end());
    }

    const int map_size = mapd_map.row * mapd_map.col;
    size_t reservation_count = (size_t)map_size * maxtime;
    vector<char> vertex_reservations(reservation_count, 0);
    unordered_map<long long, int> cost_visited;
    vector<int> last_occupation(map_size, -1);
    for (const vector<int>& path : fixed_paths)
        for (unsigned int t = 0; t < maxtime; t++) {
            vertex_reservations[(size_t)t * map_size + path[t]] = 1;
            last_occupation[path[t]] = max(last_occupation[path[t]], (int)t);
        }

    int agent_count = agent_ids.size();
    int destination_count = candidate_endpoint_ids.size();
    int scale = agent_count * destination_count;
    dlib::matrix<int> costs(matrix_size, matrix_size);
    for (int row = 0; row < matrix_size; row++) {
        for (int column = 0; column < matrix_size; column++) {
            if (row >= agent_count || column >= destination_count) {
                costs(row, column) = 0;
                continue;
            }
            int agent_id = agent_ids[row];
            int endpoint_id = candidate_endpoint_ids[column];
            int path_cost = central_path_cost(
                agent_id, agents[agent_id].loc,
                mapd_map.endpoints[endpoint_id].loc, (int)cur_time_, fixed_paths,
                &vertex_reservations, (int)reservation_count,
                &last_occupation, cost_visited);
            if (path_cost >= map_size) path_cost = 2 * map_size;
            if (column < task_destination_count)
                costs(row, column) = (2 * map_size - path_cost) * scale;
            else
                costs(row, column) = map_size * scale - path_cost;
        }
    }

    vector<long> assignment = dlib::max_cost_assignment(costs);

    for (int i = 0; i < (int)agent_ids.size(); i++) {
        int agent_id = agent_ids[i];
        int destination = assignment[i];
        if (destination >= (int)candidate_endpoint_ids.size()) {
            agents[agent_id].last_endpoint = agents[agent_id].loc;
            agents[agent_id].status = AG_FREE;
            agents[agent_id].current_task = -1;
            agents[agent_id].current_goal_index = 0;
            continue;
        }

        int endpoint_id = candidate_endpoint_ids[destination];
        agents[agent_id].last_endpoint = mapd_map.endpoints[endpoint_id].loc;

        if (destination < task_destination_count) {
            Task* task = candidate_tasks[destination];
            // CENTRAL assigns the pickup endpoint, not ownership of the task.
            // The task becomes owned only when an agent actually reaches its
            // pickup, matching the paper and released implementation.
            agents[agent_id].status = AG_MOVING_TO_PICKUP;
            agents[agent_id].current_task = task->id;
            agents[agent_id].current_goal_index = 0;
        } else {
            agents[agent_id].status = AG_FREE;
            agents[agent_id].current_task = -1;
            agents[agent_id].current_goal_index = 0;
        }
    }
}

namespace {
struct TATourAssignment {
    vector<vector<int>> tasks_by_agent;
    vector<int> agent_order;
};

TATourAssignment read_ta_tour(const string& tour_file, int agent_count,
                              int task_count) {
    if (tour_file.empty()) {
        throw runtime_error("TA_PRIORITIZED requires --tour <file>");
    }

    ifstream input(tour_file);
    if (!input.is_open()) {
        throw runtime_error("Tour file not found: " + tour_file);
    }

    string line;
    bool found_tour = false;
    while (getline(input, line)) {
        if (line.find("TOUR_SECTION") != string::npos) {
            found_tour = true;
            break;
        }
    }
    if (!found_tour) {
        throw runtime_error("TOUR_SECTION not found in " + tour_file);
    }

    TATourAssignment assignment;
    assignment.tasks_by_agent.resize(agent_count);
    vector<bool> seen_agent(agent_count, false);
    int current_agent = -1;
    const int node_count = agent_count + task_count;
    for (int index = 0; index < node_count; index++) {
        int node = -1;
        if (!(input >> node) || node < 0) break;
        if (node >= 1 && node <= agent_count) {
            current_agent = node - 1;
            if (!seen_agent[current_agent]) {
                assignment.agent_order.push_back(current_agent);
                seen_agent[current_agent] = true;
            }
        } else {
            int task_id = node - agent_count - 1;
            if (current_agent < 0 || current_agent >= agent_count ||
                task_id < 0 || task_id >= task_count) {
                throw runtime_error(
                    "Invalid LKH3 tour node " + to_string(node));
            }
            assignment.tasks_by_agent[current_agent].push_back(task_id);
        }
    }

    // Keep deterministic coverage even if a hand-written tour omits an empty
    // depot from its explicit ordering.
    for (int agent_id = 0; agent_id < agent_count; agent_id++)
        if (!seen_agent[agent_id])
            assignment.agent_order.push_back(agent_id);
    return assignment;
}
} // namespace

// --- TA-Prioritized: read the complete offline LKH3 mTSP assignment -------
void Simulation::assign_ta_tsp() {
    TATourAssignment assignment = read_ta_tour(
        tour_file_, (int)agents.size(), (int)all_tasks.size());

    for (Agent& agent : agents) agent.task_sequence.clear();

    vector<bool> assigned(all_tasks.size(), false);
    for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++) {
        for (int task_id : assignment.tasks_by_agent[agent_id]) {
            Task& task = all_tasks[task_id];
            if (task.status == INT_MAX) continue;
            if (assigned[task_id])
                throw runtime_error(
                    "TA-Prioritized tour assigns task " +
                    to_string(task_id) + " more than once");

            agents[agent_id].task_sequence.push_back(task_id);
            task.status = agent_id;
            assigned[task_id] = true;
        }
    }

    for (Task* task : open_tasks_)
        if (task->status != INT_MAX && !assigned[task->id])
            throw runtime_error(
                "TA-Prioritized tour does not assign known task " +
                to_string(task->id));

    // Every task in the complete offline instance now belongs to a sequence.
    open_tasks_.clear();
}

// TA-Hybrid keeps the LKH3 task sequences but permits an idle agent to take
// the final not-yet-executing task from another sequence when that reduces the
// estimated completion time. The complete task set and tour are known at time
// zero; later calls only reassign unfinished tasks after an agent becomes
// available. All temporary lookup data remains local to this assignment call.
void Simulation::assign_ta_hybrid() {
    const int agent_count = (int)agents.size();
    TATourAssignment tour = read_ta_tour(
        tour_file_, agent_count, (int)all_tasks.size());

    vector<int> tour_agent(all_tasks.size(), -1);
    vector<int> tour_rank(all_tasks.size(), INT_MAX);
    for (int agent_id = 0; agent_id < agent_count; agent_id++)
        for (int rank = 0;
             rank < (int)tour.tasks_by_agent[agent_id].size(); rank++) {
            int task_id = tour.tasks_by_agent[agent_id][rank];
            tour_agent[task_id] = agent_id;
            tour_rank[task_id] = rank;
        }

    // Materialize tasks that have not yet entered a sequence. Existing
    // sequences may already have been permuted by Group 2 and therefore must
    // not be rebuilt from scratch.
    vector<vector<int>> newly_known(agent_count);
    for (Task* task : open_tasks_) {
        if (task->status == INT_MAX) continue;
        int owner = tour_agent[task->id];
        if (owner < 0)
            throw runtime_error(
                "TA-Hybrid tour does not assign known task " +
                to_string(task->id));
        newly_known[owner].push_back(task->id);
    }
    for (int owner = 0; owner < agent_count; owner++) {
        sort(newly_known[owner].begin(), newly_known[owner].end(),
             [&](int lhs, int rhs) {
                 return tour_rank[lhs] < tour_rank[rhs];
             });
        for (int task_id : newly_known[owner]) {
            agents[owner].task_sequence.push_back(task_id);
            all_tasks[task_id].seq_id = owner;
            all_tasks[task_id].status = owner;
        }
    }
    open_tasks_.clear();

    auto distance = [&](int from, int to) {
        int endpoint = mapd_map.ep_index(to);
        return endpoint < 0 ? INT_MAX :
            mapd_map.endpoints[endpoint].h_val[from];
    };
    auto estimate_sequence = [&](int owner) {
        int time = (int)cur_time_;
        int location = agents[owner].loc;
        for (int task_id : agents[owner].task_sequence) {
            const Task& task = all_tasks[task_id];
            int first_goal = task_id == agents[owner].current_task
                ? agents[owner].current_goal_index : 0;
            if (first_goal < 0) first_goal = 0;
            for (int goal = first_goal;
                 goal < (int)task.goals.size(); goal++) {
                int leg = distance(location, task.goals[goal]);
                if (leg == INT_MAX) return INT_MAX;
                time += leg;
                if (goal == 0) time = max(time, task.release_time);
                location = task.goals[goal];
            }
        }
        return time;
    };

    // Reference AssignNewTask(): an available empty-sequence agent can steal
    // the last unstarted task of the currently longest sequence.
    for (int receiver = 0; receiver < agent_count; receiver++) {
        if (agents[receiver].status != AG_FREE ||
            !agents[receiver].task_sequence.empty())
            continue;

        int donor = -1;
        int chosen_task = -1;
        int longest_finish = -1;
        for (int owner = 0; owner < agent_count; owner++) {
            if (owner == receiver || agents[owner].task_sequence.empty())
                continue;
            int task_id = agents[owner].task_sequence.back();
            if (task_id == agents[owner].current_task) continue;

            int donor_finish = estimate_sequence(owner);
            const Task& task = all_tasks[task_id];
            int pickup_distance =
                distance(agents[receiver].loc, task.goals.front());
            if (pickup_distance == INT_MAX || donor_finish == INT_MAX)
                continue;
            int arrival = (int)cur_time_ + pickup_distance;
            int receiver_finish = arrival;
            bool reachable = true;
            for (int goal = 1; goal < (int)task.goals.size(); goal++) {
                int leg = distance(task.goals[goal - 1], task.goals[goal]);
                if (leg == INT_MAX) {
                    reachable = false;
                    break;
                }
                receiver_finish += leg;
            }
            if (!reachable) continue;

            if (arrival > task.release_time - 10 &&
                receiver_finish < donor_finish &&
                donor_finish > longest_finish) {
                donor = owner;
                chosen_task = task_id;
                longest_finish = donor_finish;
            }
        }

        if (donor >= 0) {
            agents[donor].task_sequence.pop_back();
            agents[receiver].task_sequence.push_back(chosen_task);
            all_tasks[chosen_task].seq_id = receiver;
            all_tasks[chosen_task].status = receiver;
        }
    }
}

// --- Repeated Hungarian + LNS ----------------------------------------------
//
//  Phase 1  repeated Hungarian: every unfinished task is re-matched to an agent
//           from scratch (a task already picked up stays with its carrier).
//  Phase 2  LNS: destroy a small related neighbourhood of the assignment and
//           repair it greedily by regret; keep the result only if the estimated
//           cost improved.  Budgeted by config.lns_time_limit seconds.
// ============================================================================

void Simulation::assign_repeated_hungarian_lns() {
    // ---- Phase 1 ----
    assign_repeated_hungarian();

    if (config.lns_time_limit <= 0) return;

    // ---- Phase 2 gate ----
    // Nothing reassignable => destroy/repair cannot change anything and the
    // loop below would busy-wait for the whole budget producing no change.
    if (!lns_has_reassignable_task()) return;

    // LNS consumes the framework-wide RNG seed. Pass a negative seed to use
    // time(NULL), matching the reference's nondeterministic mode.
    if (config.seed < 0) srand((unsigned)time(NULL));
    else srand((unsigned)config.seed);

    clock_t lns_start = clock();
    double time_limit_ms = config.lns_time_limit * 1000.0;

    // The time limit is a ceiling, not a target. A positive configurable LNS
    // limit also stops after that many consecutive rejected moves. Set it to
    // zero to disable this early stop and run until the time budget expires.
    int no_improve = 0, empty_streak = 0;

    while (true) {
        double elapsed = (double)(clock() - lns_start) / CLOCKS_PER_SEC * 1000.0;
        if (elapsed >= time_limit_ms) break;
        if (config.lns_no_improvement_limit > 0 &&
            no_improve >= config.lns_no_improvement_limit)
            break;

        // --- snapshot (for rollback) ---
        vector<deque<int>> saved_seqs(agents.size());
        vector<int> saved_statuses(all_tasks.size());
        int saved_cost = 0;
        for (int i = 0; i < (int)agents.size(); i++) {
            saved_seqs[i] = agents[i].task_sequence;
            saved_cost += estimate_sequence_cost(i);
        }
        for (int i = 0; i < (int)all_tasks.size(); i++)
            saved_statuses[i] = all_tasks[i].status;

        // --- destroy ---
        vector<int> removed;
        lns_destroy(removed);
        if (removed.empty()) { if (++empty_streak > 200) break; continue; }
        empty_streak = 0;

        // --- repair ---
        lns_repair(removed);

        // --- accept / reject on the estimated objective ---
        int new_cost = 0;
        for (int i = 0; i < (int)agents.size(); i++)
            new_cost += estimate_sequence_cost(i);

        if (new_cost >= saved_cost) {
            for (int i = 0; i < (int)agents.size(); i++)
                agents[i].task_sequence = saved_seqs[i];
            for (int i = 0; i < (int)all_tasks.size(); i++)
                all_tasks[i].status = saved_statuses[i];
            no_improve++;
        } else {
            no_improve = 0;
        }
    }
}

bool Simulation::lns_has_reassignable_task() const {
    for (auto& ag : agents)
        for (int tid : ag.task_sequence) {
            if (ag.status == AG_CARRYING && ag.current_task == tid) continue;
            return true;
        }
    return false;
}

// --- Phase 1: repeated Hungarian -------------------------------------------
// Each round matches min(#agents, #eligible) tasks optimally; rounds repeat
// until every eligible task has an owner. Tasks conflicting with old dummy
// endpoints remain open for the next timestep.
void Simulation::assign_repeated_hungarian() {
    // The PBS reference excludes tasks whose ordered goals overlap an endpoint
    // held by the previous plan. Keep those tasks in open_tasks_ until the new
    // plan moves its dummy endpoints away, then reconsider them next timestep.
    // The old endpoints are encoded by the committed path tails; the event
    // flag only records that at least one task needs a retry.
    unordered_set<int> old_dummy_endpoints;
    const bool defer_old_dummy_conflicts =
        config.dummy_path &&
        config.endpoint_strategy == NEAREST_WITH_STRICT_EXCLUSIONS;
    if (defer_old_dummy_conflicts) {
        for (const auto& path : path_table_)
            if (!path.empty())
                old_dummy_endpoints.insert((int)path.back());
    }

    // Release everything except a task that is already being carried: those
    // agents keep their front task, everyone else starts from an empty sequence.
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].task_sequence.empty()) continue;
        int front_tid = agents[i].task_sequence.front();
        bool keep_front = (agents[i].status == AG_CARRYING &&
                           agents[i].current_task == front_tid);
        for (int tid : agents[i].task_sequence) {
            if (keep_front && tid == front_tid) continue;
            all_tasks[tid].status = -1;
            open_tasks_.push_back(&all_tasks[tid]);
        }
        if (keep_front) {
            agents[i].task_sequence = {front_tid};
        } else {
            agents[i].task_sequence.clear();
            if (agents[i].status == AG_MOVING_TO_PICKUP) {
                agents[i].status = AG_FREE;
                agents[i].current_task = -1;
                agents[i].current_goal_index = 0;
            }
        }
    }

    vector<Task*> remaining_tasks;
    for (Task* task : open_tasks_) {
        if (task->status != -1) continue;

        bool conflicts_with_old_dummy = false;
        if (defer_old_dummy_conflicts) {
            for (int goal : task->goals) {
                if (old_dummy_endpoints.count(goal)) {
                    conflicts_with_old_dummy = true;
                    new_or_deferred_task_event_ = true;
                    break;
                }
            }
        }
        if (!conflicts_with_old_dummy)
            remaining_tasks.push_back(task);
    }
    if (remaining_tasks.empty()) return;

    int num_ag = (int)agents.size();

    while (!remaining_tasks.empty()) {
        int remain = (int)remaining_tasks.size();
        int row = max(num_ag, remain);

        // Square cost matrix; padding cells are unusable (INT_MIN reward).
        dlib::matrix<int> cost_mat(row, row);
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < row; j++) {
                if (i >= num_ag || j >= remain) {
                    cost_mat(i, j) = INT_MIN;
                } else {
                    // max_cost_assignment maximises, so negate the arrival time.
                    cost_mat(i, j) = -hungarian_arrival_estimate(agents[i], *remaining_tasks[j]);
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

        for (Task* t : assigned) {
            remaining_tasks.erase(
                remove(remaining_tasks.begin(), remaining_tasks.end(), t),
                remaining_tasks.end());
            open_tasks_.remove(t);
        }
    }
}

// The assignment search may optimize every currently available task, but the
// paper materializes only the first C tasks of each agent's sequence. Tasks
// after that prefix become unassigned again and remain in the task pool for
// reconsideration at the next new-task/deferred-task/free-agent event. The
// event trigger is tracked separately, so merely waiting in open_tasks_ does
// not cause reassignment every timestep.
void Simulation::truncate_online_task_sequences() {
    // PP_PER_TASK deliberately materializes one task at a time. PBS and wPBS
    // materialize the paper's configurable C-task prefix.
    const int keep_count = (config.mapf == MAPF_PP_PER_TASK)
        ? 1 : max(1, config.task_sequence_limit);

    for (Agent& agent : agents) {
        while ((int)agent.task_sequence.size() > keep_count) {
            int task_id = agent.task_sequence.back();
            agent.task_sequence.pop_back();
            all_tasks[task_id].status = -1;
            open_tasks_.push_back(&all_tasks[task_id]);
        }
    }
}

// Estimated timestep at which `ag` would reach `task`'s last goal if the task
// were appended to its current sequence (endpoint BFS distances, no conflicts).
int Simulation::hungarian_arrival_estimate(const Agent& ag, const Task& task) const {
    int est_time = 0;

    if (!ag.task_sequence.empty()) {
        // Walk the existing sequence to find where/when the agent becomes free.
        int loc = (int)ag.loc;
        int t = (int)cur_time_;
        for (int tid : ag.task_sequence) {
            const Task& tt = all_tasks[tid];
            int ng = (int)tt.goals.size();
            // Resume the current task at its first unvisited goal. Queued tasks
            // always start at goal zero.
            bool is_current_task = (ag.current_task == tid);
            int start_g = is_current_task ? ag.current_goal_index : 0;
            for (int g = start_g; g < ng; g++) {
                int gloc = tt.goals[g];
                int ep_idx = mapd_map.ep_index(gloc);
                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                if (d == INT_MAX) d = 0;
                t += d;
                // Only the pickup can be blocked by the task's release time.
                if (g == 0 && t < tt.release_time)
                    t = tt.release_time;
                loc = gloc;
            }
        }
        int first_goal_loc = task.goals.empty() ? task.pickup_loc : task.goals[0];
        int ep_idx = mapd_map.ep_index(first_goal_loc);
        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
        if (d == INT_MAX) d = 0;
        est_time = t + d;
    } else {
        int first_goal_loc = task.goals.empty() ? task.pickup_loc : task.goals[0];
        int ep_idx = mapd_map.ep_index(first_goal_loc);
        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[ag.loc] : 0;
        if (d == INT_MAX) d = 0;
        est_time = (int)cur_time_ + d;
    }

    est_time = max(est_time, task.release_time);

    // Add every remaining leg through the task's ordered goal sequence.
    for (int g = 0; g + 1 < (int)task.goals.size(); g++) {
        int ep_idx = mapd_map.ep_index(task.goals[g + 1]);
        int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[task.goals[g]] : 0;
        if (d == INT_MAX) d = 0;
        est_time += d;
    }
    return est_time;
}

// --- The LNS objective ------------------------------------------------------
// Sum over the agent's sequence of (estimated delivery time - release time):
// the same flowtime the reference minimises.
int Simulation::estimate_sequence_cost(int agent_id) const {
    const Agent& ag = agents[agent_id];
    if (ag.task_sequence.empty()) return 0;

    int loc = (int)ag.loc;
    int delivery_time = (int)cur_time_;
    int sum_of_delivery_time = 0;
    int sum_of_release_time = 0;

    for (int idx = 0; idx < (int)ag.task_sequence.size(); idx++) {
        int tid = ag.task_sequence[idx];
        const Task& tt = all_tasks[tid];
        int ng = (int)tt.goals.size();

        bool is_current_task = (idx == 0 && ag.current_task == tid);
        int start_goal = is_current_task ? ag.current_goal_index : 0;

        if (start_goal < ng) {
            int gloc0 = tt.goals[start_goal];
            int ep_idx = mapd_map.ep_index(gloc0);
            int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
            if (d == INT_MAX) d = 0;

            if (idx == 0) delivery_time = (int)cur_time_ + d;
            else          delivery_time += d;

            if (start_goal == 0)
                delivery_time = max(delivery_time, tt.release_time);

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
        if (!tt.goals.empty()) loc = tt.goals.back();
    }

    return sum_of_delivery_time - sum_of_release_time;
}

// Estimated (first-goal time, final-goal time) of every queued task, used by
// the temporal term of Shaw relatedness.
void Simulation::lns_estimate_task_times(unordered_map<int, pair<int,int>>& task_times) const {
    for (auto& ag : agents) {
        int next_task_start_time = (int)cur_time_;
        int next_task_start_loc = (int)ag.loc;
        for (int idx = 0; idx < (int)ag.task_sequence.size(); idx++) {
            int tid = ag.task_sequence[idx];
            const Task& tt = all_tasks[tid];
            int ng = (int)tt.goals.size();
            if (ng == 0) continue;

            bool is_current_task = (idx == 0 && ag.current_task == tid);
            int start_goal = is_current_task ? ag.current_goal_index : 0;

            int this_pickup = (is_current_task && start_goal > 0 && tt.ag_arrive_start >= 0)
                ? tt.ag_arrive_start : -1;
            int this_delivery = next_task_start_time;
            int loc = next_task_start_loc;

            for (int g = start_goal; g < ng; g++) {
                int ep_idx = mapd_map.ep_index(tt.goals[g]);
                int d = (ep_idx >= 0) ? mapd_map.endpoints[ep_idx].h_val[loc] : 0;
                if (d == INT_MAX) d = 0;
                this_delivery += d;
                if (g == 0) {
                    this_delivery = max(this_delivery, tt.release_time);
                    this_pickup = this_delivery;
                }
                loc = tt.goals[g];
            }
            if (this_pickup < 0) this_pickup = (int)cur_time_;
            task_times[tid] = {this_pickup, this_delivery};
            next_task_start_time = this_delivery;
            next_task_start_loc = tt.goals.back();
        }
    }
}

// --- Destroy: Shaw removal --------------------------------------------------
// Pick a random queued task, then take the tasks most "related" to it, where
// relatedness mixes spatial distance between goals with temporal distance
// between estimated first/final-goal times. The paper applies spatial weight
// 9 to both distances and temporal weight 3 to the two time differences.
void Simulation::lns_destroy(vector<int>& removed_tasks) {
    vector<int> eligible;
    for (auto& ag : agents)
        for (int tid : ag.task_sequence) {
            if (ag.status == AG_CARRYING && ag.current_task == tid) continue;
            eligible.push_back(tid);
        }
    if (eligible.empty()) return;

    const int neighborhood_size = 2;   // reference LNS(..., 2, 1, 2, 2)

    unordered_map<int, pair<int,int>> task_times;
    lns_estimate_task_times(task_times);

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

    vector<pair<int,int>> relatedness;
    for (int tid : eligible) {
        if (tid == seed_tid) continue;
        const Task& t = all_tasks[tid];
        int t_last_goal = t.goals.empty() ? t.pickup_loc : t.goals.back();
        int t_first_goal = t.goals.empty() ? t.pickup_loc : t.goals[0];

        int dist_last = 0, dist_first = 0;
        for (auto& ep : mapd_map.endpoints)
            if (ep.loc == seed_last_goal) { dist_last = ep.h_val[t_last_goal]; break; }
        for (auto& ep : mapd_map.endpoints)
            if (ep.loc == seed_first_goal) { dist_first = ep.h_val[t_first_goal]; break; }
        if (dist_last == INT_MAX) dist_last = mapd_map.col * mapd_map.row;
        if (dist_first == INT_MAX) dist_first = mapd_map.col * mapd_map.row;

        int t_pickup_t = 0, t_delivery_t = 0;
        if (task_times.count(tid)) {
            t_pickup_t = task_times[tid].first;
            t_delivery_t = task_times[tid].second;
        }
        int time_diff = abs(t_pickup_t - seed_pickup_t) + abs(t_delivery_t - seed_delivery_t);

        relatedness.push_back({9 * (dist_last + dist_first) + 3 * time_diff, tid});
    }
    sort(relatedness.begin(), relatedness.end());
    for (auto& p : relatedness) {
        removed_tasks.push_back(p.second);
        if ((int)removed_tasks.size() >= neighborhood_size) break;
    }

    for (int tid : removed_tasks) {
        all_tasks[tid].status = -1;
        for (auto& ag : agents) {
            auto it = find(ag.task_sequence.begin(), ag.task_sequence.end(), tid);
            if (it != ag.task_sequence.end()) { ag.task_sequence.erase(it); break; }
        }
    }
}

// --- Repair: regret-2 insertion ---------------------------------------------
// Re-insert removed tasks one at a time, always taking the task whose second
// best insertion is worst relative to its best one (largest regret).
void Simulation::lns_repair(vector<int>& removed_tasks) {
    while (!removed_tasks.empty()) {
        int best_task = -1, best_agent = -1, best_pos = -1;
        int best_marginal = INT_MAX;
        int best_regret = INT_MIN;

        for (int rt : removed_tasks) {
            int best1_marginal = INT_MAX, best1_agent = -1, best1_pos = -1;
            int best2_marginal = INT_MAX;

            for (int a = 0; a < (int)agents.size(); a++) {
                int seq_len = (int)agents[a].task_sequence.size();
                // Position 0 is unavailable while the agent carries its front task.
                int start_pos = 0;
                if (agents[a].status == AG_CARRYING && !agents[a].task_sequence.empty()
                    && agents[a].task_sequence.front() == agents[a].current_task)
                    start_pos = 1;

                int base_cost = estimate_sequence_cost(a);

                for (int p = start_pos; p <= seq_len; p++) {
                    agents[a].task_sequence.insert(agents[a].task_sequence.begin() + p, rt);
                    int c = estimate_sequence_cost(a);
                    agents[a].task_sequence.erase(agents[a].task_sequence.begin() + p);

                    int marginal = c - base_cost;
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

// ============================================================================
//  Section 4 — Task sequences -> goal sequences
//
//  The planner never sees tasks, only an ordered list of (location, earliest
//  time) goals per agent. When dummy_path is enabled, the sequence ends with a
//  selected endpoint. If that endpoint equals the delivery location, the dummy
//  path has zero length; otherwise it is the post-delivery route to the endpoint.
// ============================================================================

vector<vector<pair<int,int>>> Simulation::build_goal_sequences() {
    int num_ag = (int)agents.size();
    vector<int> assigned_dummies(num_ag, -1);
    vector<vector<pair<int,int>>> goal_seqs(num_ag);

    // Match the paper's task-truncation parameter: plan only the first K tasks
    // in each assigned sequence; later tasks remain queued for a future replan.
    const int task_truncated_size = config.task_sequence_limit;

    for (int i = 0; i < num_ag; i++) {
        int current_task_count = 0;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= task_truncated_size) break;
            Task& task = all_tasks[tid];
            int start_goal = (tid == agents[i].current_task)
                ? agents[i].current_goal_index : 0;
            for (int g = start_goal; g < (int)task.goals.size(); g++)
                goal_seqs[i].push_back({task.goals[g], g == 0 ? task.release_time : 0});
            current_task_count++;
        }
    }

    if (!config.dummy_path) {
        // An idle agent still needs a stationary planning target, but active
        // agents stop at their final planned task goal with no dummy leg.
        for (int i = 0; i < num_ag; i++)
            if (goal_seqs[i].empty())
                goal_seqs[i].push_back({(int)agents[i].loc, 0});
        return goal_seqs;
    }

    // Dummy endpoints: agents that have work choose first, so idle agents are
    // the ones pushed out to the leftover endpoints. A selected endpoint equal
    // to the last task goal represents a zero-length dummy path.
    vector<int> busy_agents, free_agents;
    for (int i = 0; i < num_ag; i++) {
        if (!goal_seqs[i].empty()) busy_agents.push_back(i);
        else                       free_agents.push_back(i);
    }
    for (int i : busy_agents) {
        int dummy = choose_dummy_endpoint(i, goal_seqs[i].back().first, assigned_dummies);
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }
    for (int i : free_agents) {
        int dummy = choose_dummy_endpoint(i, (int)agents[i].loc, assigned_dummies);
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }

    return goal_seqs;
}

// Unified endpoint/parking selector. In the loaded map, task endpoints have
// Endpoint::is_task_endpoint=true and parking/home endpoints have false. Every
// supported algorithm delegates its explicit parking/relocation choice to this
// function, and every strategy admits parking endpoints according to its policy:
//   * TP/TPTS: nearest safe task or parking endpoint when waiting is unsafe;
//   * HBH: nearest safe parking endpoint when relocation is required;
//   * TA: the agent's own home parking endpoint;
//   * CENTRAL: nearest currently available task or parking endpoint;
//   * PBS/PP: nearest task or parking endpoint after strict exclusions;
//   * wPBS: task endpoints first, then any pairwise-free parking endpoint.
// Returning last_goal_loc means that no relocation path is required.
int Simulation::choose_dummy_endpoint(int agent_id, int last_goal_loc,
                                      const vector<int>& reserved_endpoints) {
    const bool nearest_available_path2 =
        config.endpoint_strategy == NEAREST_AVAILABLE &&
        (config.assign_method == AM_DECOUPLED_GREEDY ||
         config.assign_method == AM_DECOUPLED_GREEDY_SWAPS ||
         config.assign_method == AM_CENTRALIZED_GREEDY);
    if (config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK ||
        (nearest_available_path2 &&
         config.assign_method == AM_CENTRALIZED_GREEDY)) {
        // HBH Algorithm 2: wait unless this location is needed by an open
        // task. If it is, relocate to the nearest reachable endpoint that can
        // be held without colliding with another committed path. The native
        // HBH policy restricts the candidate to parking endpoints; the common
        // NEAREST_AVAILABLE override permits any otherwise-safe endpoint.
        bool need_move = !mapd_map.is_endpoint[last_goal_loc];
        for (Task* task : open_tasks_)
            for (int goal_location : task->goals)
                if (goal_location == last_goal_loc) {
                    need_move = true;
                    break;
                }

        // "Free endpoint" also means that no already committed path needs the
        // cell in the future; otherwise replacing this agent's old path with a
        // stationary wait could invalidate a path planned earlier.
        for (unsigned int timestep = cur_time_ + 1;
             timestep < maxtime && !need_move; timestep++)
            for (int other = 0; other < (int)agents.size(); other++)
                if (other != agent_id &&
                    path_table_[other][timestep] ==
                        (unsigned int)last_goal_loc) {
                    need_move = true;
                    break;
                }
        if (!need_move) return last_goal_loc;

        Agent probe = agents[agent_id];
        probe.loc = last_goal_loc;
        return search_path2_endpoint(probe, -1);
    }

    if (config.endpoint_strategy == WAIT_OR_NEAREST_SAFE ||
        (nearest_available_path2 &&
         config.assign_method != AM_CENTRALIZED_GREEDY)) {
        // TP/TPTS Path2 policy:
        //   * return the current endpoint when waiting there is safe;
        //   * otherwise choose the nearest reachable task/home endpoint that
        //     avoids open-task post-pickup goals and other agents' future paths.
        bool need_move = !mapd_map.is_endpoint[last_goal_loc];

        // TP and TPTS must vacate a location needed after pickup by an open
        // task. Index 0 keeps the original pickup semantics; every later goal
        // generalizes the original delivery-location Path2 condition.
        for (Task* task : open_tasks_)
            for (int goal_index = 1;
                 goal_index < (int)task->goals.size(); goal_index++)
                if (task->goals[goal_index] == last_goal_loc) {
                    need_move = true;
                    break;
                }

        // Algorithm 2 additionally moves a TPTS agent when another committed
        // path will cross its current endpoint in the future.
        if (config.assign_method == AM_DECOUPLED_GREEDY_SWAPS) {
            for (unsigned int t = cur_time_; t < maxtime && !need_move; t++)
                for (int other = 0; other < (int)agents.size(); other++)
                    if (other != agent_id &&
                        path_table_[other][t] ==
                            (unsigned int)last_goal_loc) {
                        need_move = true;
                        break;
                    }
        }

        if (!need_move) return last_goal_loc;

        // Selection returns only the endpoint. Use a temporary agent because
        // the time-space BFS also reconstructs the route to determine the
        // nearest reachable safe endpoint; the real path is planned afterward.
        Agent probe = agents[agent_id];
        probe.loc = last_goal_loc;
        return search_path2_endpoint(probe, -1);
    }

    if (config.endpoint_strategy == NEAREST_AVAILABLE) {
        // The caller supplies endpoints already selected in this planning
        // batch. CENTRAL additionally supplies occupied task goals and jointly
        // replans its free-agent batch, so its established behavior needs no
        // further exclusions here.
        set<int> forbidden;
        for (int location : reserved_endpoints)
            if (location >= 0) forbidden.insert(location);

        const bool offline_ta =
            config.assign_method == AM_LKH3_TSP ||
            config.assign_method == AM_LKH3_TSP_REASSIGN;
        if (offline_ta) {
            // TA planning treats every not-yet-planned agent's home as a
            // permanent reservation. Its own home is therefore the only
            // parking endpoint guaranteed to be available without changing
            // the offline prioritized/hybrid planning invariant.
            return agents[agent_id].initial_loc;
        }

        if (config.assign_method != AM_CENTRAL_HUNGARIAN) {
            // Other families may plan sequentially or invoke Path2 for only one
            // agent. In those contexts an endpoint is "available" only if it
            // is not still required by an open task, assigned to another
            // agent's pending task sequence, or permanently held by another
            // committed path. The requesting agent's own completed task goals
            // remain candidates, including its current delivery endpoint.
            for (Task* task : open_tasks_)
                for (int goal_location : task->goals)
                    forbidden.insert(goal_location);

            for (int other = 0; other < (int)agents.size(); other++) {
                if (other == agent_id) continue;
                for (int task_id : agents[other].task_sequence)
                    for (int goal_location : all_tasks[task_id].goals)
                        forbidden.insert(goal_location);
                forbidden.insert((int)path_table_[other][maxtime - 1]);
            }
        }

        if (mapd_map.is_endpoint[last_goal_loc] &&
            !forbidden.count(last_goal_loc))
            return last_goal_loc;

        int best_location = -1;
        int best_distance = INT_MAX;
        for (const Endpoint& endpoint : mapd_map.endpoints) {
            if (forbidden.count(endpoint.loc)) continue;
            int distance = endpoint.h_val[last_goal_loc];
            if (distance < best_distance) {
                best_distance = distance;
                best_location = endpoint.loc;
            }
        }
        return best_location;
    }

    if (config.endpoint_strategy == PAIRWISE_TASK_THEN_HOME) {
        // wPBS policy: only the newly selected endpoints must be pairwise
        // distinct. Prefer task endpoints, then home endpoints, while skipping
        // the agent's current/last goal.
        auto already_assigned = [&](int loc) {
            for (int i = 0; i < (int)reserved_endpoints.size(); i++)
                if (i != agent_id && reserved_endpoints[i] == loc)
                    return true;
            return false;
        };

        auto nearest_endpoint_of_type = [&](bool task_endpoint) {
            int best_loc = -1;
            int best_dist = INT_MAX;
            for (const Endpoint& endpoint : mapd_map.endpoints) {
                if (endpoint.is_task_endpoint != task_endpoint) continue;
                if (endpoint.loc == last_goal_loc) continue;
                if (already_assigned(endpoint.loc)) continue;

                int distance = endpoint.h_val[last_goal_loc];
                // Match the released implementation's std::map assignment:
                // the later endpoint wins when distances are equal.
                if (distance <= best_dist) {
                    best_dist = distance;
                    best_loc = endpoint.loc;
                }
            }
            return best_loc;
        };

        // wPBS uses pairwise-distinct task endpoints first, then any available
        // parking/home endpoint. Staying at the last goal is the final fallback.
        int endpoint = nearest_endpoint_of_type(true);
        if (endpoint >= 0) return endpoint;
        endpoint = nearest_endpoint_of_type(false);
        return endpoint >= 0 ? endpoint : last_goal_loc;
    }

    // Offline fixed-parking policy: every agent returns to its own home parking
    // endpoint. This is intentionally not a search over other agents' homes.
    if (config.endpoint_strategy == RETURN_TO_HOME)
        return agents[agent_id].initial_loc;

    if (config.endpoint_strategy != NEAREST_WITH_STRICT_EXCLUSIONS)
        throw logic_error("unsupported endpoint strategy");

    // PBS/PP strict policy: new dummy endpoints must be distinct and must avoid
    // unfinished-task goals plus the old parking endpoints of other agents.
    set<int> forbidden;

    for (int i = 0; i < (int)reserved_endpoints.size(); i++)
        if (i != agent_id && reserved_endpoints[i] >= 0)
            forbidden.insert(reserved_endpoints[i]);

    for (auto it = open_tasks_.begin(); it != open_tasks_.end(); ++it)
        for (int gloc : (*it)->goals) forbidden.insert(gloc);
    for (auto& ag : agents)
        for (int tid : ag.task_sequence)
            for (int gloc : all_tasks[tid].goals) forbidden.insert(gloc);
    // Other agents still physically occupy their OLD parking cell until they
    // actually move, so those cells are blocked too — otherwise our goal
    // could be permanently held by somebody else's committed path.
    for (int i = 0; i < (int)agents.size(); i++) {
        if (i == agent_id) continue;
        forbidden.insert((int)path_table_[i][(int)maxtime - 1]);
    }

    int best_loc = -1;
    int best_dist = INT_MAX;
    for (int e = 0; e < (int)mapd_map.endpoints.size(); e++) {
        int loc = mapd_map.endpoints[e].loc;
        if (forbidden.count(loc)) continue;
        int d = mapd_map.endpoints[e].h_val[last_goal_loc];
        if (d <= best_dist) { best_dist = d; best_loc = loc; }
    }
    if (best_loc < 0) best_loc = agents[agent_id].initial_loc;   // fall back home
    return best_loc;
}

// Regroup a flat goal sequence into per-task groups, so the low level can plan
// one task at a time (including all of that task's ordered goals).
vector<vector<pair<int,int>>> Simulation::split_into_task_groups(
    int agent_id, const vector<pair<int,int>>& goal_seq) const {
    vector<vector<pair<int,int>>> groups;
    int idx = 0;
    const int max_tasks = config.task_sequence_limit;
    int count = 0;

    for (int tid : agents[agent_id].task_sequence) {
        if (count >= max_tasks) break;
        const Task& task = all_tasks[tid];
        int start_goal = (tid == agents[agent_id].current_task)
            ? agents[agent_id].current_goal_index : 0;

        vector<pair<int,int>> group;
        for (int g = start_goal; g < (int)task.goals.size() && idx < (int)goal_seq.size(); g++)
            group.push_back(goal_seq[idx++]);
        if (!group.empty()) groups.push_back(group);
        count++;
    }

    if (idx < (int)goal_seq.size()) groups.push_back({goal_seq[idx]});   // the dummy
    return groups;
}

// ============================================================================
//  Section 5 — MAPF: Prioritized Planning / Priority-Based Search
//
//  High level: depth-first search over partial priority orders.  Each node
//  fixes "agent a yields to agent b"; the low level then re-plans the yielding
//  agent against the paths of everyone who outranks it.
// ============================================================================

// --- CENTRAL: CBS ----------------------------------------------------------
void Simulation::path_planning_ecbs(bool assignment_triggered) {
    // CENTRAL solves two smaller CBS instances each timestep, as described in
    // the paper: first agents that became occupied now, then all remaining
    // free/non-carrying agents. A focal weight of 1.0 gives optimal CBS;
    // values greater than 1.0 enable bounded-suboptimal ECBS.
    auto solve_batch = [&](const vector<int>& agent_ids,
                           const vector<int>& goal_locations,
                           const vector<int>& goal_endpoint_ids,
                           bool task_goal_batch) {
        if (agent_ids.empty()) return;

        set<int> batch_agents(agent_ids.begin(), agent_ids.end());
        vector<vector<int>> constraints;
        for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++)
            if (!batch_agents.count(agent_id))
                constraints.emplace_back(
                    path_table_[agent_id].begin(),
                    path_table_[agent_id].end());

        vector<int> starts;
        starts.reserve(agent_ids.size());
        for (int agent_id : agent_ids)
            starts.push_back(agents[agent_id].loc);

        ECBSPlanner planner;
        ECBSResult result = planner.solve(ECBSRequest(
            mapd_map.grid, starts, goal_locations, goal_endpoint_ids,
            constraints, (int)cur_time_, mapd_map.col,
            config.ecbs_focal_weight,
            config.cbs_high_level_expansion_limit,
            mapd_map.endpoints, (int)maxtime));
        if (!result.solution_found || result.paths.size() != agent_ids.size())
            throw runtime_error(
                "CENTRAL-CBS failed to find the required batch path");

        for (int index = 0; index < (int)agent_ids.size(); index++) {
            int agent_id = agent_ids[index];
            const vector<int>& path = result.paths[index];
            if (path.empty())
                throw runtime_error(
                    "CENTRAL-CBS returned an empty agent path");

            for (int t = 0;
                 t < (int)path.size() && cur_time_ + t < maxtime; t++) {
                path_table_[agent_id][cur_time_ + t] = path[t];
                agents[agent_id].path[cur_time_ + t] = path[t];
            }
            int hold_location = path.back();
            for (unsigned int t = cur_time_ + path.size(); t < maxtime; t++) {
                path_table_[agent_id][t] = hold_location;
                agents[agent_id].path[t] = hold_location;
            }

            int arrival_time = cur_time_ + path.size() - 1;
            agents[agent_id].finish_time = arrival_time;
            if (task_goal_batch) {
                Task* task = central_current_task(agent_id);
                if (!task)
                    throw runtime_error(
                        "CENTRAL-CBS task agent has no current task");
                // Completion and final-goal waiting apply only to the last
                // ordered goal. Intermediate goals trigger another Group-1
                // CBS segment when the agent reaches them.
                if (agents[agent_id].current_goal_index ==
                        (int)task->goals.size() - 1) {
                    task->completion_time = arrival_time;
                    agents[agent_id].finish_time += task->goal_wait_time;
                }
            }
        }
    };

    // Group 1: agents that became occupied or reached an intermediate goal at
    // this timestep. Plan one CBS segment to each task's next ordered goal.
    vector<int> task_agents;
    vector<int> task_goals;
    vector<int> task_endpoint_ids;
    for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++) {
        Task* task = central_current_task(agent_id);
        if (agents[agent_id].status != AG_CARRYING || !task)
            continue;
        int goal_index = agents[agent_id].current_goal_index;
        if (goal_index < 1 || goal_index >= (int)task->goals.size())
            continue;
        if (task->ag_arrive_start != (int)cur_time_ &&
            agents[agent_id].finish_time > cur_time_)
            continue;
        int endpoint_id = mapd_map.ep_index(task->goals[goal_index]);
        if (endpoint_id < 0)
            throw runtime_error("CENTRAL task goal is not an endpoint");
        task_agents.push_back(agent_id);
        task_goals.push_back(task->goals[goal_index]);
        task_endpoint_ids.push_back(endpoint_id);
    }
    solve_batch(task_agents, task_goals, task_endpoint_ids, true);

    // Group 2 is planned after task assignment. CENTRAL does this every
    // timestep; CENTRAL-fixed does it only after a new-task/free-agent event.
    if (!assignment_triggered) return;

    // All free/non-carrying agents are jointly planned to the pickup or
    // parking endpoint stored by assign_central_hungarian().
    vector<int> free_agents;
    vector<int> free_goals;
    vector<int> free_endpoint_ids;
    for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++) {
        if (agents[agent_id].status == AG_CARRYING) continue;
        int endpoint_id = mapd_map.ep_index(agents[agent_id].last_endpoint);
        if (endpoint_id < 0)
            throw runtime_error(
                "CENTRAL-CBS agent has an invalid assigned endpoint");
        free_agents.push_back(agent_id);
        free_goals.push_back(agents[agent_id].last_endpoint);
        free_endpoint_ids.push_back(endpoint_id);
    }
    solve_batch(free_agents, free_goals, free_endpoint_ids, false);
}

namespace {
struct TAFlowEdge {
    int from;
    int to;
    int capacity;
    int original_capacity;
    int cost;
    int location;
    int next;
};

// Local min-cost max-flow network used only by one TA-Hybrid Group-2 call.
// Keeping it local avoids adding persistent simulation state for temporary
// time-expanded graph data.
class TAFlowNetwork {
public:
    TAFlowNetwork(int node_count, int source, int sink)
        : head_(node_count, -1), previous_(node_count, -1),
          in_queue_(node_count, false), distance_(node_count, INT_MAX),
          source_(source), sink_(sink) {}

    void add_edges(int from, int to, int capacity, int cost, int location) {
        add_edge(from, to, capacity, cost, location);
        add_edge(to, from, 0, -cost, location);
    }

    void remove_edge(int from, int to) {
        for (int edge = head_[from]; edge >= 0; edge = edges_[edge].next)
            if (edges_[edge].to == to && edges_[edge].capacity > 0) {
                edges_[edge].capacity--;
                edges_[edge].original_capacity--;
                return;
            }
    }

    int solve() {
        int flow = 0;
        while (shortest_path()) {
            flow++;
            for (int node = sink_; node != source_;
                 node = edges_[previous_[node]].from) {
                edges_[previous_[node]].capacity--;
                edges_[previous_[node] ^ 1].capacity++;
            }
        }
        return flow;
    }

    vector<vector<int>> extract_paths() {
        vector<vector<int>> paths;
        for (int edge = head_[source_]; edge >= 0; edge = edges_[edge].next) {
            if (edges_[edge].original_capacity <= 0 ||
                edges_[edge].capacity >= edges_[edge].original_capacity)
                continue;
            vector<int> path = {edges_[edge].location};
            int node = edges_[edge].to;
            while (node != sink_) {
                int used = -1;
                for (int next = head_[node]; next >= 0;
                     next = edges_[next].next) {
                    if (edges_[next].original_capacity > 0 &&
                        edges_[next].capacity <
                            edges_[next].original_capacity) {
                        used = next;
                        break;
                    }
                }
                if (used < 0)
                    throw runtime_error(
                        "TA-Hybrid could not decode a min-cost flow path");
                edges_[used].capacity = edges_[used].original_capacity;
                if (edges_[used].location >= 0)
                    path.push_back(edges_[used].location);
                node = edges_[used].to;
            }
            paths.push_back(path);
        }
        return paths;
    }

private:
    vector<TAFlowEdge> edges_;
    vector<int> head_;
    vector<int> previous_;
    vector<bool> in_queue_;
    vector<int> distance_;
    queue<int> queue_;
    int source_;
    int sink_;

    void add_edge(int from, int to, int capacity, int cost, int location) {
        edges_.push_back({from, to, capacity, capacity, cost, location,
                          head_[from]});
        head_[from] = (int)edges_.size() - 1;
    }

    bool shortest_path() {
        fill(distance_.begin(), distance_.end(), INT_MAX);
        fill(previous_.begin(), previous_.end(), -1);
        fill(in_queue_.begin(), in_queue_.end(), false);
        while (!queue_.empty()) queue_.pop();
        distance_[source_] = 0;
        queue_.push(source_);
        in_queue_[source_] = true;
        while (!queue_.empty()) {
            int node = queue_.front();
            queue_.pop();
            in_queue_[node] = false;
            for (int edge = head_[node]; edge >= 0;
                 edge = edges_[edge].next) {
                const TAFlowEdge& candidate = edges_[edge];
                if (candidate.capacity <= 0 ||
                    distance_[node] == INT_MAX)
                    continue;
                int next_distance = distance_[node] + candidate.cost;
                if (next_distance >= distance_[candidate.to]) continue;
                distance_[candidate.to] = next_distance;
                previous_[candidate.to] = edge;
                if (!in_queue_[candidate.to]) {
                    queue_.push(candidate.to);
                    in_queue_[candidate.to] = true;
                }
            }
        }
        return distance_[sink_] != INT_MAX;
    }
};
} // namespace

// --- TA-Hybrid -------------------------------------------------------------
// Group 1 jointly plans newly carrying agents to their next ordered task goal
// with CBS and a reserved path to each agent's parking endpoint. Group 2 uses
// anonymous min-cost max-flow to match free agents to pairwise-distinct pickup
// goals, then reserves their parking paths. All batch state is local.
void Simulation::path_planning_ta_hybrid(bool assignment_triggered) {
    const int agent_count = (int)agents.size();
    const int map_size = (int)mapd_map.grid.size();
    const int now = (int)cur_time_;
    const int horizon = (int)maxtime;

    auto endpoint_distance = [&](int from, int to) {
        int endpoint = mapd_map.ep_index(to);
        return endpoint < 0 ? INT_MAX :
            mapd_map.endpoints[endpoint].h_val[from];
    };
    auto estimate_sequence = [&](int agent_id, int start,
                                 bool starts_at_front_pickup) {
        int time = start;
        int location = agents[agent_id].loc;
        bool first_task = true;
        for (int task_id : agents[agent_id].task_sequence) {
            const Task& task = all_tasks[task_id];
            int first_goal = task_id == agents[agent_id].current_task
                ? agents[agent_id].current_goal_index : 0;
            if (first_goal < 0) first_goal = 0;
            if (starts_at_front_pickup && first_task) {
                location = task.goals.front();
                first_goal = max(first_goal, 1);
                time = max(time, task.release_time);
            }
            for (int goal = first_goal;
                 goal < (int)task.goals.size(); goal++) {
                int leg = endpoint_distance(location, task.goals[goal]);
                if (leg == INT_MAX) return INT_MAX;
                time += leg;
                if (goal == 0) time = max(time, task.release_time);
                location = task.goals[goal];
            }
            first_task = false;
        }
        return time;
    };

    // ---------------- Group 1: task agents -> next ordered goal ---------
    vector<int> group1;
    for (int agent_id = 0; agent_id < agent_count; agent_id++) {
        const Agent& agent = agents[agent_id];
        if (agent.status == AG_CARRYING && agent.current_task >= 0 &&
            agent.current_task < (int)all_tasks.size() &&
            agent.current_goal_index <
                (int)all_tasks[agent.current_task].goals.size() &&
            agent.finish_time <= cur_time_)
            group1.push_back(agent_id);
    }

    if (!group1.empty()) {
        set<int> in_group(group1.begin(), group1.end());
        vector<vector<int>> fixed_paths;
        for (int other = 0; other < agent_count; other++)
            if (!in_group.count(other))
                fixed_paths.emplace_back(
                    agents[other].path.begin(), agents[other].path.end());

        struct HybridCBSNode {
            vector<vector<int>> paths;
            vector<int> arrivals;
            vector<vector<tuple<int,int,int>>> constraints;
            int cost = 0;
            int conflicts = 0;
            int first_agent = -1;
            int second_agent = -1;
            int first_location = -1;
            int second_location = -1;
            int conflict_time = -1;
        };

        auto low_level = [&](int group_index,
                             const vector<tuple<int,int,int>>& constraints,
                             vector<int>& path) {
            int agent_id = group1[group_index];
            Agent probe = agents[agent_id];
            const Task& task = all_tasks[probe.current_task];
            int goal = task.goals[probe.current_goal_index];
            int goal_endpoint = mapd_map.ep_index(goal);
            vector<int> reserved(agent_count, -1);
            int parking = choose_dummy_endpoint(agent_id, goal, reserved);
            int parking_endpoint = mapd_map.ep_index(parking);
            if (goal_endpoint < 0 || parking_endpoint < 0)
                throw runtime_error(
                    "TA-Hybrid Group 1 received a non-endpoint goal");

            int arrival = astar_with_dummy(
                probe, probe.loc, now, goal, parking,
                mapd_map.endpoints[goal_endpoint].h_val,
                mapd_map.endpoints[parking_endpoint].h_val,
                fixed_paths, 0, true, constraints);
            if (arrival < 0)
                arrival = astar_with_dummy(
                    probe, probe.loc, now, goal, parking,
                    mapd_map.endpoints[goal_endpoint].h_val,
                    mapd_map.endpoints[parking_endpoint].h_val,
                    fixed_paths, 0, false, constraints);
            if (arrival < 0) return -1;
            path.assign(probe.path.begin(), probe.path.end());
            return arrival;
        };

        auto detect_conflict = [&](HybridCBSNode& node) {
            node.conflicts = 0;
            node.first_agent = -1;
            int earliest = INT_MAX;
            for (int first = 0; first < (int)group1.size(); first++)
                for (int second = first + 1;
                     second < (int)group1.size(); second++)
                    for (int time = now; time < horizon; time++) {
                        bool vertex =
                            node.paths[first][time] == node.paths[second][time];
                        bool edge = time > now &&
                            node.paths[first][time] ==
                                node.paths[second][time - 1] &&
                            node.paths[first][time - 1] ==
                                node.paths[second][time];
                        if (!vertex && !edge) continue;
                        node.conflicts++;
                        if (time >= earliest) continue;
                        earliest = time;
                        node.first_agent = first;
                        node.second_agent = second;
                        node.conflict_time = time;
                        if (vertex) {
                            node.first_location = node.paths[first][time];
                            node.second_location = -1;
                        } else {
                            node.first_location = node.paths[first][time - 1];
                            node.second_location = node.paths[first][time];
                        }
                    }
        };

        auto worse = [](const HybridCBSNode* lhs,
                        const HybridCBSNode* rhs) {
            if (lhs->cost != rhs->cost) return lhs->cost > rhs->cost;
            return lhs->conflicts > rhs->conflicts;
        };
        priority_queue<HybridCBSNode*, vector<HybridCBSNode*>,
                       decltype(worse)> open(worse);
        vector<HybridCBSNode*> allocated;
        HybridCBSNode* root = new HybridCBSNode();
        root->paths.resize(group1.size());
        root->arrivals.resize(group1.size());
        root->constraints.resize(group1.size());
        for (int index = 0; index < (int)group1.size(); index++) {
            root->arrivals[index] = low_level(
                index, root->constraints[index], root->paths[index]);
            if (root->arrivals[index] < 0) {
                delete root;
                throw runtime_error(
                    "TA-Hybrid Group 1 low-level search failed");
            }
            root->cost = max(root->cost,
                estimate_sequence(group1[index], root->arrivals[index], false));
        }
        detect_conflict(*root);
        open.push(root);
        allocated.push_back(root);

        HybridCBSNode* solution = nullptr;
        int expanded = 0;
        while (!open.empty()) {
            HybridCBSNode* current = open.top();
            open.pop();
            if (current->first_agent < 0) {
                solution = current;
                break;
            }
            if (++expanded > 100000)
                break;

            for (int branch = 0; branch < 2; branch++) {
                int constrained = branch == 0
                    ? current->first_agent : current->second_agent;
                HybridCBSNode* child = new HybridCBSNode(*current);
                tuple<int,int,int> constraint;
                if (current->second_location < 0) {
                    constraint = make_tuple(
                        current->first_location, -1,
                        current->conflict_time);
                } else if (branch == 0) {
                    constraint = make_tuple(
                        current->first_location,
                        current->second_location,
                        current->conflict_time);
                } else {
                    constraint = make_tuple(
                        current->second_location,
                        current->first_location,
                        current->conflict_time);
                }
                child->constraints[constrained].push_back(constraint);
                int arrival = low_level(
                    constrained, child->constraints[constrained],
                    child->paths[constrained]);
                if (arrival < 0) {
                    delete child;
                    continue;
                }
                child->arrivals[constrained] = arrival;
                child->cost = 0;
                for (int index = 0; index < (int)group1.size(); index++)
                    child->cost = max(child->cost,
                        estimate_sequence(
                            group1[index], child->arrivals[index], false));
                detect_conflict(*child);
                open.push(child);
                allocated.push_back(child);
            }
        }

        if (!solution) {
            for (HybridCBSNode* node : allocated) delete node;
            throw runtime_error(
                "TA-Hybrid Group 1 CBS failed to find a collision-free batch");
        }
        for (int index = 0; index < (int)group1.size(); index++) {
            int agent_id = group1[index];
            for (int time = now; time < horizon; time++) {
                agents[agent_id].path[time] = solution->paths[index][time];
                path_table_[agent_id][time] = solution->paths[index][time];
            }
            agents[agent_id].finish_time = solution->arrivals[index];
            agents[agent_id].dummy_start_step = solution->arrivals[index];
        }
        for (HybridCBSNode* node : allocated) delete node;
    }

    // Group 2 is recomputed only after initial assignment, a new visible task,
    // or a sequence becoming free. Group 1 above remains independently event
    // driven for intermediate multi-goal transitions.
    if (!assignment_triggered) return;

    vector<int> remaining;
    for (int agent_id = 0; agent_id < agent_count; agent_id++)
        if (agents[agent_id].status != AG_CARRYING &&
            !agents[agent_id].task_sequence.empty())
            remaining.push_back(agent_id);

    int global_bound = now;
    for (int agent_id = 0; agent_id < agent_count; agent_id++)
        global_bound = max(
            global_bound, estimate_sequence(agent_id, now, false));

    while (!remaining.empty()) {
        vector<int> subgroup;
        set<int> used_pickups;
        vector<int> deferred;
        for (int owner : remaining) {
            int task_id = agents[owner].task_sequence.front();
            int pickup = all_tasks[task_id].goals.front();
            if (used_pickups.insert(pickup).second)
                subgroup.push_back(owner);
            else
                deferred.push_back(owner);
        }
        remaining.swap(deferred);

        set<int> subgroup_set(subgroup.begin(), subgroup.end());
        vector<vector<int>> fixed_paths;
        for (int other = 0; other < agent_count; other++)
            if (!subgroup_set.count(other))
                fixed_paths.emplace_back(
                    agents[other].path.begin(), agents[other].path.end());

        vector<Task*> tasks;
        for (int owner : subgroup)
            tasks.push_back(&all_tasks[agents[owner].task_sequence.front()]);

        vector<vector<int>> flow_paths;
        while (flow_paths.empty()) {
            vector<int> deadlines(tasks.size(), now);
            int maximum_deadline = now;
            for (int index = 0; index < (int)tasks.size(); index++) {
                int owner = tasks[index]->seq_id;
                int deadline = now;
                while (deadline + 1 < horizon &&
                       estimate_sequence(owner, deadline + 1, true) <=
                           global_bound)
                    deadline++;
                deadlines[index] = deadline;
                maximum_deadline = max(maximum_deadline, deadline);
            }

            int relative_horizon = maximum_deadline - now + 1;
            int layer_size = relative_horizon + 1;
            auto in_node = [&](int location, int time) {
                return 2 + (location * layer_size + time) * 2;
            };
            auto out_node = [&](int location, int time) {
                return in_node(location, time) + 1;
            };
            int task_base = 2 + map_size * layer_size * 2;
            int source = 0;
            int sink = 1;
            TAFlowNetwork flow(
                task_base + (int)tasks.size(), source, sink);

            for (int index = 0; index < (int)subgroup.size(); index++)
                flow.add_edges(
                    source, out_node(agents[subgroup[index]].loc, 0),
                    1, 0, index);

            for (int time = 0; time < relative_horizon; time++) {
                for (int location = 0; location < map_size; location++) {
                    if (!mapd_map.grid[location]) continue;
                    flow.add_edges(in_node(location, time),
                                   out_node(location, time), 1, 0, -1);
                    for (int action : {0, 1, -1, mapd_map.col,
                                       -mapd_map.col}) {
                        int next = location + action;
                        if (next < 0 || next >= map_size ||
                            !mapd_map.grid[next] ||
                            abs(next % mapd_map.col -
                                location % mapd_map.col) > 1)
                            continue;
                        flow.add_edges(out_node(location, time),
                                       in_node(next, time + 1),
                                       1, 1, next);
                    }
                }
            }

            for (int index = 0; index < (int)tasks.size(); index++) {
                // The framework consumes path events after advancing time.
                // Require one explicit path step even when the agent already
                // occupies the pickup so the pickup transition is observed.
                int first = max(now + 1, tasks[index]->release_time) - now;
                int last = deadlines[index] - now;
                for (int time = first; time <= last; time++)
                    flow.add_edges(
                        out_node(tasks[index]->goals.front(), time),
                        task_base + index, 1, 0, -1);
                flow.add_edges(task_base + index, sink, 1, 0, -1);
            }

            for (const vector<int>& path : fixed_paths) {
                for (int time = 1; time <= relative_horizon; time++) {
                    int absolute = now + time;
                    int location = absolute < (int)path.size()
                        ? path[absolute] : path.back();
                    flow.remove_edge(in_node(location, time),
                                     out_node(location, time));
                }
                for (int time = 0; time < relative_horizon; time++) {
                    int absolute = now + time;
                    int current = absolute < (int)path.size()
                        ? path[absolute] : path.back();
                    int next = absolute + 1 < (int)path.size()
                        ? path[absolute + 1] : path.back();
                    flow.remove_edge(out_node(next, time),
                                     in_node(current, time + 1));
                }
            }

            if (flow.solve() == (int)subgroup.size())
                flow_paths = flow.extract_paths();
            else if (++global_bound >= horizon)
                throw runtime_error(
                    "TA-Hybrid Group 2 min-cost flow exceeded the horizon");
        }

        vector<deque<int>> old_sequences(agent_count);
        for (int owner : subgroup)
            old_sequences[owner] = agents[owner].task_sequence;
        for (int owner : subgroup)
            agents[owner].task_sequence.clear();

        vector<int> arrival(agent_count, now);
        for (const vector<int>& encoded_path : flow_paths) {
            if (encoded_path.empty()) continue;
            int local_agent = encoded_path.front();
            if (local_agent < 0 || local_agent >= (int)subgroup.size())
                throw runtime_error("TA-Hybrid flow returned invalid agent");
            int agent_id = subgroup[local_agent];
            for (int offset = 1;
                 offset < (int)encoded_path.size(); offset++) {
                int time = now + offset;
                if (time < horizon)
                    agents[agent_id].path[time] = encoded_path[offset];
            }
            arrival[agent_id] = now + (int)encoded_path.size() - 1;
            int pickup = encoded_path.back();
            for (int time = arrival[agent_id]; time < horizon; time++)
                agents[agent_id].path[time] = pickup;
        }

        // The vertex-capacitated flow network prevents vertex conflicts. As
        // in the reference implementation, resolve any opposite-edge swaps by
        // exchanging the two path suffixes; this preserves both endpoints and
        // total flow cost while eliminating the edge collision.
        bool swapped = true;
        while (swapped) {
            swapped = false;
            for (int first = 0; first < (int)subgroup.size(); first++)
                for (int second = first + 1;
                     second < (int)subgroup.size(); second++) {
                    int a = subgroup[first];
                    int b = subgroup[second];
                    int stop = min(arrival[a], arrival[b]);
                    for (int time = now; time < stop; time++) {
                        if (agents[a].path[time] == agents[b].path[time + 1] &&
                            agents[b].path[time] == agents[a].path[time + 1]) {
                            for (int suffix = time + 1;
                                 suffix < horizon; suffix++)
                                swap(agents[a].path[suffix],
                                     agents[b].path[suffix]);
                            swap(arrival[a], arrival[b]);
                            swapped = true;
                            break;
                        }
                    }
                }
        }

        // A flow may send an agent to another sequence's pickup. Move that
        // complete sequence to the assigned agent, matching the anonymous
        // reassignment step in the paper and released implementation.
        for (int agent_id : subgroup) {
            int pickup = agents[agent_id].path[arrival[agent_id]];
            int source_owner = -1;
            for (Task* task : tasks)
                if (task->goals.front() == pickup) {
                    source_owner = task->seq_id;
                    break;
                }
            if (source_owner < 0)
                throw runtime_error(
                    "TA-Hybrid flow returned an unknown pickup endpoint");
            agents[agent_id].task_sequence = old_sequences[source_owner];
            for (int task_id : agents[agent_id].task_sequence) {
                all_tasks[task_id].seq_id = agent_id;
                all_tasks[task_id].status = agent_id;
            }
        }

        vector<vector<unsigned int>> flow_only_paths(agent_count);
        for (int agent_id : subgroup)
            flow_only_paths[agent_id] = agents[agent_id].path;

        vector<int> dummy_order = subgroup;
        bool dummy_success = false;
        int failed_agent = -1;
        // Try the flow order and one failed-agent-first order before invoking
        // the complete prioritized fallback below.
        for (int attempt = 0; attempt < 2 && !dummy_success; attempt++) {
            for (int agent_id : subgroup)
                agents[agent_id].path = flow_only_paths[agent_id];
            dummy_success = true;
            failed_agent = -1;

            for (int agent_id : dummy_order) {
                if (agents[agent_id].task_sequence.empty())
                    throw runtime_error(
                        "TA-Hybrid flow left an agent without a sequence");
                Task& task =
                    all_tasks[agents[agent_id].task_sequence.front()];
                int pickup = task.goals.front();
                int pickup_endpoint = mapd_map.ep_index(pickup);
                vector<int> reserved(agent_count, -1);
                int parking = choose_dummy_endpoint(
                    agent_id, pickup, reserved);
                int parking_endpoint = mapd_map.ep_index(parking);
                if (pickup_endpoint < 0 || parking_endpoint < 0)
                    throw runtime_error(
                        "TA-Hybrid Group 2 received a non-endpoint goal");

                vector<vector<int>> constraints;
                for (int other = 0; other < agent_count; other++)
                    if (other != agent_id)
                        constraints.emplace_back(
                            agents[other].path.begin(),
                            agents[other].path.end());
                int result = astar_with_dummy(
                    agents[agent_id], pickup, arrival[agent_id],
                    pickup, parking,
                    mapd_map.endpoints[pickup_endpoint].h_val,
                    mapd_map.endpoints[parking_endpoint].h_val,
                    constraints, task.release_time, false);
                if (result < 0) {
                    // Reference GoHome retries the flow with a later pickup
                    // hold time. The equivalent local fallback is to replan
                    // this agent's complete current->pickup->home path against
                    // the already fixed paths, allowing STA* to delay pickup
                    // arrival as much as necessary.
                    result = astar_with_dummy(
                        agents[agent_id], agents[agent_id].loc, now,
                        pickup, parking,
                        mapd_map.endpoints[pickup_endpoint].h_val,
                        mapd_map.endpoints[parking_endpoint].h_val,
                        constraints, task.release_time, false);
                    if (result >= 0) {
                        arrival[agent_id] = result;
                        continue;
                    }
                    dummy_success = false;
                    failed_agent = agent_id;
                    break;
                }
            }

            if (!dummy_success && failed_agent >= 0) {
                auto failed = find(
                    dummy_order.begin(), dummy_order.end(), failed_agent);
                if (failed != dummy_order.end()) {
                    dummy_order.erase(failed);
                    dummy_order.insert(dummy_order.begin(), failed_agent);
                }
            }
        }

        if (!dummy_success) {
            // If fixed flow arrivals leave no feasible ordering of dummy tails,
            // retain the anonymous flow assignment but jointly replace the
            // subgroup trajectories with complete prioritized
            // current->pickup->home paths. Later agents reserve their homes;
            // each planned path then becomes a hard constraint for the rest.
            vector<int> fallback_order = dummy_order;
            vector<vector<int>> planned_paths;
            vector<bool> planned(agent_count, false);
            dummy_success = true;
            for (int agent_id : fallback_order) {
                Task& task =
                    all_tasks[agents[agent_id].task_sequence.front()];
                int pickup = task.goals.front();
                int pickup_endpoint = mapd_map.ep_index(pickup);
                vector<int> reserved(agent_count, -1);
                int parking = choose_dummy_endpoint(
                    agent_id, pickup, reserved);
                int parking_endpoint = mapd_map.ep_index(parking);

                vector<vector<int>> constraints = fixed_paths;
                for (const vector<int>& path : planned_paths)
                    constraints.push_back(path);
                for (int other : subgroup)
                    if (other != agent_id && !planned[other])
                        constraints.emplace_back(
                            maxtime, agents[other].initial_loc);

                Agent probe = agents[agent_id];
                int result = astar_with_dummy(
                    probe, probe.loc, now, pickup, parking,
                    mapd_map.endpoints[pickup_endpoint].h_val,
                    mapd_map.endpoints[parking_endpoint].h_val,
                    constraints, task.release_time, false);
                if (result < 0) {
                    failed_agent = agent_id;
                    dummy_success = false;
                    break;
                }
                agents[agent_id].path = std::move(probe.path);
                arrival[agent_id] = result;
                planned[agent_id] = true;
                planned_paths.emplace_back(
                    agents[agent_id].path.begin(),
                    agents[agent_id].path.end());
            }
        }

        if (!dummy_success)
            throw runtime_error(
                "TA-Hybrid failed to reserve Group 2 fallback paths; agent " +
                to_string(failed_agent));

        for (int agent_id : subgroup) {
            Task& task = all_tasks[agents[agent_id].task_sequence.front()];
            task.ag = &agents[agent_id];
            agents[agent_id].status = AG_MOVING_TO_PICKUP;
            agents[agent_id].current_task = task.id;
            agents[agent_id].current_goal_index = 0;
            agents[agent_id].finish_time = arrival[agent_id];
            agents[agent_id].dummy_start_step = arrival[agent_id];
            for (int time = now; time < horizon; time++)
                path_table_[agent_id][time] = agents[agent_id].path[time];
        }
    }
}

// --- TA-Prioritized --------------------------------------------------------
// At every priority level, tentatively plan every remaining agent and select
// the one with the largest actual task-sequence completion time. This follows
// the paper's improved prioritized-planning rule; the released implementation
// uses the LKH tour order only as the deterministic tie breaker.
void Simulation::path_planning_pp_task_sequence() {
    const int agent_count = (int)agents.size();
    const int start_time = (int)cur_time_;
    TATourAssignment tour = read_ta_tour(
        tour_file_, agent_count, (int)all_tasks.size());

    vector<int> order_index(agent_count, agent_count);
    for (int i = 0; i < (int)tour.agent_order.size(); i++)
        order_index[tour.agent_order[i]] = i;

    struct PlannedTaskTime {
        int task_id;
        int pickup_time;
        int completion_time;
    };
    struct CandidatePlan {
        Agent agent;
        vector<PlannedTaskTime> task_times;
        int task_finish = 0;
    };

    vector<vector<int>> home_paths(
        agent_count, vector<int>(maxtime));
    for (int agent_id = 0; agent_id < agent_count; agent_id++)
        fill(home_paths[agent_id].begin(), home_paths[agent_id].end(),
             agents[agent_id].initial_loc);

    vector<int> selected_agents;
    vector<bool> selected(agent_count, false);
    vector<int> selected_endpoints(agent_count, -1);

    auto plan_candidate = [&](int agent_id,
                              const vector<vector<int>>& constraints,
                              CandidatePlan& candidate) {
        candidate.agent = agents[agent_id];
        candidate.task_times.clear();
        candidate.task_finish = start_time;

        int final_goal = (int)candidate.agent.path[start_time];
        for (int task_id : candidate.agent.task_sequence) {
            const Task& task = all_tasks[task_id];
            int first_goal = task_id == candidate.agent.current_task
                ? candidate.agent.current_goal_index : 0;
            if (first_goal < (int)task.goals.size())
                final_goal = task.goals.back();
        }

        int parking = config.dummy_path
            ? choose_dummy_endpoint(
                agent_id, final_goal, selected_endpoints)
            : final_goal;
        int parking_endpoint = mapd_map.ep_index(parking);
        if (parking_endpoint < 0)
            throw runtime_error(
                "TA-Prioritized parking location is not an endpoint for agent " +
                to_string(agent_id));
        const vector<int>& parking_h =
            mapd_map.endpoints[parking_endpoint].h_val;

        int time = start_time;
        bool planned_task_goal = false;
        for (int task_id : candidate.agent.task_sequence) {
            const Task& task = all_tasks[task_id];
            int first_goal = task_id == candidate.agent.current_task
                ? candidate.agent.current_goal_index : 0;
            if (first_goal < 0) first_goal = 0;
            if (first_goal >= (int)task.goals.size()) continue;

            PlannedTaskTime task_time = {
                task_id, first_goal > 0 ? task.ag_arrive_start : -1, -1};
            for (int goal_index = first_goal;
                 goal_index < (int)task.goals.size(); goal_index++) {
                int goal_location = task.goals[goal_index];
                int goal_endpoint = mapd_map.ep_index(goal_location);
                if (goal_endpoint < 0)
                    throw runtime_error(
                        "TA-Prioritized task goal is not an endpoint");
                const vector<int>& goal_h =
                    mapd_map.endpoints[goal_endpoint].h_val;
                int release_time = goal_index == 0
                    ? task.release_time : 0;

                int arrival = astar_with_dummy(
                    candidate.agent, candidate.agent.path[time], time,
                    goal_location, parking, goal_h, parking_h,
                    constraints, release_time, true);
                if (arrival < 0)
                    arrival = astar_with_dummy(
                        candidate.agent, candidate.agent.path[time], time,
                        goal_location, parking, goal_h, parking_h,
                        constraints, release_time, false);
                if (arrival < 0)
                    throw runtime_error(
                        "TA-Prioritized failed to plan agent " +
                        to_string(agent_id) + ", task " +
                        to_string(task_id) + ", goal " +
                        to_string(goal_index));

                if (goal_index == 0) task_time.pickup_time = arrival;
                if (goal_index + 1 == (int)task.goals.size()) {
                    task_time.completion_time = arrival;
                    candidate.task_finish = arrival;
                }
                time = arrival;
                planned_task_goal = true;
            }
            candidate.task_times.push_back(task_time);
        }

        // An empty sequence still needs a collision-free return to its fixed
        // parking endpoint. For a non-empty sequence the final sub-path above
        // has already retained exactly this dummy path.
        if (!planned_task_goal) {
            int current_location = candidate.agent.path[start_time];
            if (config.dummy_path) {
                int dummy_arrival = astar_with_dummy(
                    candidate.agent, current_location, start_time,
                    current_location, parking, parking_h, parking_h,
                    constraints, 0, false);
                if (dummy_arrival < 0)
                    throw runtime_error(
                        "TA-Prioritized failed to return idle agent " +
                        to_string(agent_id) + " to its parking endpoint");
            } else {
                for (int t = start_time; t < (int)maxtime; t++)
                    candidate.agent.path[t] = current_location;
            }
        }
        candidate.agent.finish_time = candidate.task_finish;
    };

    for (int priority = 0; priority < agent_count; priority++) {
        int best_agent = -1;
        CandidatePlan best_plan;

        for (int candidate_id = 0; candidate_id < agent_count;
             candidate_id++) {
            if (selected[candidate_id]) continue;

            vector<vector<int>> constraints;
            constraints.reserve(agent_count - 1);
            for (int selected_id : selected_agents)
                constraints.emplace_back(
                    agents[selected_id].path.begin(),
                    agents[selected_id].path.end());
            for (int other = 0; other < agent_count; other++)
                if (other != candidate_id && !selected[other])
                    constraints.push_back(home_paths[other]);

            CandidatePlan candidate;
            plan_candidate(candidate_id, constraints, candidate);
            if (best_agent < 0 ||
                candidate.task_finish > best_plan.task_finish ||
                (candidate.task_finish == best_plan.task_finish &&
                 order_index[candidate_id] < order_index[best_agent])) {
                best_agent = candidate_id;
                best_plan = std::move(candidate);
            }
        }

        if (best_agent < 0)
            throw runtime_error(
                "TA-Prioritized could not select the next agent");

        agents[best_agent].path = std::move(best_plan.agent.path);
        agents[best_agent].finish_time = best_plan.agent.finish_time;
        selected[best_agent] = true;
        selected_agents.push_back(best_agent);
        selected_endpoints[best_agent] = choose_dummy_endpoint(
            best_agent,
            agents[best_agent].task_sequence.empty()
                ? (int)agents[best_agent].loc
                : all_tasks[agents[best_agent].task_sequence.back()].goals.back(),
            selected_endpoints);

        for (const PlannedTaskTime& timing : best_plan.task_times) {
            Task& task = all_tasks[timing.task_id];
            if (timing.pickup_time >= 0)
                task.ag_arrive_start = timing.pickup_time;
            if (timing.completion_time >= 0)
                task.completion_time = timing.completion_time;
            task.status = best_agent;
        }
        for (unsigned int t = 0; t < maxtime; t++)
            path_table_[best_agent][t] = agents[best_agent].path[t];
    }

    for (Agent& agent : agents) {
        if (agent.task_sequence.empty()) {
            if (agent.status != AG_CARRYING) {
                agent.status = AG_FREE;
                agent.current_task = -1;
                agent.current_goal_index = 0;
            }
        } else if (agent.current_task < 0) {
            agent.status = AG_MOVING_TO_PICKUP;
            agent.current_task = agent.task_sequence.front();
            agent.current_goal_index = 0;
        }
    }
}

namespace {
struct TADummyNode {
    int location;
    int g;
    int h;
    int timestep;
    TADummyNode* parent;
    bool visited_goal;
    int goal_length;
    uint64_t tie_breaker;

    TADummyNode(int loc, int cost, int heuristic, int time,
                TADummyNode* previous, bool visited, int length)
        : location(loc), g(cost), h(heuristic), timestep(time),
          parent(previous), visited_goal(visited), goal_length(length),
          tie_breaker(RandomTieBreaker::next()) {}

    int f() const { return g + h; }
};

struct CompareTADummyNode {
    bool operator()(const TADummyNode* lhs, const TADummyNode* rhs) const {
        if (lhs->f() != rhs->f()) return lhs->f() > rhs->f();
        if (lhs->g != rhs->g) return lhs->g < rhs->g;
        return lhs->tie_breaker > rhs->tie_breaker;
    }
};
}

// Space-time A* whose accepted path reaches goal_loc and then the selected
// endpoint. The second leg is the dummy path and may have zero length.
int Simulation::astar_with_dummy(
        Agent& agent, int start_loc, int start_time,
        int goal_loc, int endpoint_loc,
        const vector<int>& goal_h, const vector<int>& endpoint_h,
        const vector<vector<int>>& constraint_paths,
        int release_time, bool goal_optimal,
        const vector<tuple<int,int,int>>& cbs_constraints) {
    const int map_size = mapd_map.row * mapd_map.col;
    const int max_time = (int)maxtime;
    const int relative_release = max(0, release_time - start_time);
    const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    priority_queue<TADummyNode*, vector<TADummyNode*>, CompareTADummyNode> open;
    unordered_map<unsigned long long, TADummyNode*> nodes;

    int initial_h = goal_h[start_loc];
    if (initial_h == INT_MAX) initial_h = 0;
    TADummyNode* start = new TADummyNode(
        start_loc, 0, initial_h, start_time, nullptr, false, 0);
    nodes[(unsigned int)start_loc] = start;
    open.push(start);

    bool a_goal_node_was_expanded = false;
    while (!open.empty()) {
        TADummyNode* current = open.top();
        open.pop();

        if (goal_optimal && a_goal_node_was_expanded &&
            !current->visited_goal && goal_loc != endpoint_loc)
            continue;

        if (!current->visited_goal && current->location == goal_loc &&
            current->g >= relative_release) {
            current->visited_goal = true;
            current->goal_length = current->g + 1;
            a_goal_node_was_expanded = true;
        }

        if (current->visited_goal && current->location == endpoint_loc) {
            bool can_hold = true;
            for (const vector<int>& constraint : constraint_paths) {
                for (int t = current->timestep + 1;
                     t < (int)constraint.size(); t++) {
                    if (constraint[t] == current->location) {
                        can_hold = false;
                        break;
                    }
                }
                if (!can_hold) break;
            }
            for (const auto& constraint : cbs_constraints) {
                if (get<1>(constraint) < 0 &&
                    get<0>(constraint) == current->location &&
                    get<2>(constraint) > current->timestep) {
                    can_hold = false;
                    break;
                }
            }

            if (can_hold) {
                int goal_arrival = start_time + current->goal_length - 1;
                vector<int> path;
                for (TADummyNode* node = current; node; node = node->parent)
                    path.push_back(node->location);
                reverse(path.begin(), path.end());

                for (int offset = 0; offset < (int)path.size(); offset++) {
                    int t = start_time + offset;
                    if (t < max_time) agent.path[t] = path[offset];
                }
                for (int t = start_time + path.size(); t < max_time; t++)
                    agent.path[t] = endpoint_loc;

                for (auto& entry : nodes) delete entry.second;
                return goal_arrival;
            }
        }

        if (current->timestep >= max_time - 1) continue;

        for (int action : actions) {
            int next_location = current->location + action;
            int next_time = current->timestep + 1;
            if (next_location < 0 || next_location >= map_size ||
                !mapd_map.grid[next_location] ||
                abs(next_location % mapd_map.col -
                    current->location % mapd_map.col) > 1)
                continue;

            bool constrained = false;
            for (const vector<int>& path : constraint_paths) {
                int next_other = next_time < (int)path.size()
                    ? path[next_time] : path.back();
                int current_other = current->timestep < (int)path.size()
                    ? path[current->timestep] : path.back();
                if (next_other == next_location ||
                    (next_other == current->location &&
                     current_other == next_location)) {
                    constrained = true;
                    break;
                }
            }
            for (const auto& constraint : cbs_constraints) {
                if (get<2>(constraint) != next_time) continue;
                int from = get<0>(constraint);
                int to = get<1>(constraint);
                if ((to < 0 && from == next_location) ||
                    (to >= 0 && from == current->location &&
                     to == next_location)) {
                    constrained = true;
                    break;
                }
            }
            if (constrained) continue;

            int next_g = current->g + 1;
            int next_h = current->visited_goal && goal_loc != endpoint_loc
                ? endpoint_h[next_location] : goal_h[next_location];
            if (next_h == INT_MAX) next_h = 0;

            unsigned long long key =
                (unsigned long long)next_location +
                (unsigned long long)next_g * map_size * 2ULL +
                (current->visited_goal ? (unsigned long long)map_size : 0ULL);
            if (nodes.find(key) != nodes.end() ||
                next_g >= max_time - start_time)
                continue;

            TADummyNode* next = new TADummyNode(
                next_location, next_g, next_h, next_time, current,
                current->visited_goal, current->goal_length);
            nodes[key] = next;
            open.push(next);
        }
    }

    for (auto& entry : nodes) delete entry.second;
    return -1;
}

// --- Prioritized planning ---------------------------------------------------
// Plan active agents before idle agents. Each new path becomes a hard
// constraint for the agents that follow; agents not yet planned retain their
// previously committed paths as constraints.
void Simulation::path_planning_pp_per_task() {
    const int num_agents = (int)agents.size();
    const int max_time = (int)maxtime;
    SIPPPlanner sipp(*this);
    MLAStarPlanner mla_star(*this);
    const vector<vector<int>> no_old_paths;

    vector<vector<int>> old_paths(num_agents, vector<int>(max_time));
    for (int i = 0; i < num_agents; i++)
        for (int t = 0; t < max_time; t++)
            old_paths[i][t] = (int)path_table_[i][t];

    vector<pair<int,int>> plan_order;
    for (int i = 0; i < num_agents; i++)
        plan_order.push_back({agents[i].task_sequence.empty() ? 1 : 0, i});
    sort(plan_order.begin(), plan_order.end());

    vector<vector<int>> new_paths(num_agents);
    vector<bool> planned(num_agents, false);
    vector<int> assigned_dummies(num_agents, -1);

    for (const auto& entry : plan_order) {
        int agent_id = entry.second;
        Agent& agent = agents[agent_id];

        vector<vector<int>> constraints;
        constraints.reserve(num_agents - 1);
        for (int other = 0; other < num_agents; other++) {
            if (other == agent_id) continue;
            constraints.push_back(planned[other] ? new_paths[other] : old_paths[other]);
        }

        // Match the reference PP configuration: plan at most the next task,
        // followed by a collision-safe dummy endpoint.
        vector<pair<int,int>> goals;
        int tasks_added = 0;
        for (int task_id : agent.task_sequence) {
            if (tasks_added >= 1) break;
            const Task& task = all_tasks[task_id];
            int start_goal = (task_id == agent.current_task)
                ? agent.current_goal_index : 0;
            for (int g = start_goal; g < (int)task.goals.size(); g++)
                goals.push_back({task.goals[g], g == 0 ? task.release_time : 0});
            tasks_added++;
        }

        int last_goal = goals.empty() ? (int)agent.loc : goals.back().first;
        if (config.dummy_path) {
            int dummy = choose_dummy_endpoint(agent_id, last_goal, assigned_dummies);
            assigned_dummies[agent_id] = dummy;
            goals.push_back({dummy, 0});
        } else if (goals.empty()) {
            goals.push_back({(int)agent.loc, 0});
        }

        vector<int> path;
        switch (config.single_agent) {
        case SA_MLSIPP_SEQUENCE:
            path = sipp.solve(SIPPRequest(
                agent_id, (int)agent.loc, (int)cur_time_, goals,
                constraints, no_old_paths, false));
            break;
        case SA_MLA_SEQUENCE: {
            vector<vector<pair<int,int>>> task_groups =
                split_into_task_groups(agent_id, goals);
            path = mla_star.solve(MLAStarRequest(
                agent_id, (int)agent.loc, (int)cur_time_, task_groups,
                constraints, no_old_paths, false));
            break;
        }
        default:
            cerr << "path_planning_pp_per_task: single_agent not implemented in this build" << endl;
            break;
        }

        // Preserve the archived PP behavior: if SIPP cannot find a path within
        // its search bound, retry with MLA* before falling back to waiting.
        if (path.empty() && config.single_agent == SA_MLSIPP_SEQUENCE) {
            vector<vector<pair<int,int>>> task_groups =
                split_into_task_groups(agent_id, goals);
            path = mla_star.solve(MLAStarRequest(
                agent_id, (int)agent.loc, (int)cur_time_, task_groups,
                constraints, no_old_paths, false));
        }

        new_paths[agent_id].resize(max_time);
        if (!path.empty()) {
            for (int t = 0; t < max_time; t++) new_paths[agent_id][t] = path[t];
        } else {
            for (int t = 0; t < max_time; t++) new_paths[agent_id][t] = (int)agent.loc;
        }

        planned[agent_id] = true;
        for (int t = 0; t < max_time; t++) {
            path_table_[agent_id][t] = new_paths[agent_id][t];
            agents[agent_id].path[t] = new_paths[agent_id][t];
        }
    }

    for (int i = 0; i < num_agents; i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
            agents[i].current_goal_index = 0;
        }
    }
}

void Simulation::path_planning_pbs() {
    PBSPlanner(*this).solve();

    // An agent that just received work starts moving toward its pickup.
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
            agents[i].current_goal_index = 0;
        }
    }
}

// --- per-solve context ------------------------------------------------------
void Simulation::pbs_build_context() {
    pbs_.num_ag = (int)agents.size();
    pbs_.max_t = (int)maxtime;
    pbs_.goal_seqs = build_goal_sequences();

    pbs_.task_groups.assign(pbs_.num_ag, {});
    for (int i = 0; i < pbs_.num_ag; i++)
        pbs_.task_groups[i] = split_into_task_groups(i, pbs_.goal_seqs[i]);

    pbs_.path_horizon = min(pbs_.max_t, (int)cur_time_ + 1200);

    // The low-level search is capped at start+512, so every committed plan is
    // active within [t, t+512] and constant afterwards.  Sizing the per-node
    // path tables to that window (instead of the full horizon) shrinks every
    // node copy and the conflict scan; the constant tail is restored on commit.
    pbs_.work_len = min(pbs_.max_t, (int)cur_time_ + 512 + 16);

    pbs_.old_paths.assign(pbs_.num_ag, {});
    for (int i = 0; i < pbs_.num_ag; i++) {
        pbs_.old_paths[i].resize(pbs_.path_horizon);
        for (int t = 0; t < pbs_.path_horizon; t++)
            pbs_.old_paths[i][t] = (int)path_table_[i][t];
    }
}

// Flatten every agent's old path ONCE per root batch (see header note).
void Simulation::pbs_build_shared_old_table() {
    const int num_ag = pbs_.num_ag;
    int cap = min(pbs_.path_horizon, (int)cur_time_ + 513);

    int g_active = 1;
    for (int i = 0; i < num_ag; i++) {
        const auto& p = pbs_.old_paths[i];
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
        const auto& p = pbs_.old_paths[i];
        int back = p.empty() ? (int)agents[i].loc : p.back();
        bool moves = false;
        int n = min((int)p.size(), g_active);
        for (int t = 1; t < n; t++) if (p[t] != p[t-1]) { moves = true; break; }
        for (int t = 0; t < g_active; t++)
            shared_old_flat_[(size_t)i * g_active + t] = (t < (int)p.size()) ? p[t] : back;
        shared_old_holds_[i] = back;
        if (moves) shared_old_movers_.push_back(i);
        else if (back >= 0 && back < (int)shared_old_holdcnt_.size())
            shared_old_holdcnt_[back]++;   // parked: occupies this cell at every t
    }

    if (config.single_agent == SA_MLSIPP_SEQUENCE) {
        const int sipp_horizon = min(pbs_.path_horizon, (int)cur_time_ + 300);
        shared_old_sipp_ranges_.assign(num_ag, {});
        for (int i = 0; i < num_ag; i++) {
            const auto& path = pbs_.old_paths[i];
            const int len = min((int)path.size(), sipp_horizon);
            if (len == 0) continue;

            int range_start = 0;
            int range_loc = path[0];
            for (int t = 1; t < len; t++) {
                if (path[t] != range_loc) {
                    shared_old_sipp_ranges_[i].emplace_back(range_loc, range_start, t);
                    range_loc = path[t];
                    range_start = t;
                }
            }
            shared_old_sipp_ranges_[i].emplace_back(
                range_loc, range_start, sipp_horizon);
        }
    } else {
        shared_old_sipp_ranges_.clear();
    }
    shared_old_valid_ = true;
}

// One low-level call, honouring config.single_agent.
vector<int> Simulation::pbs_plan_agent(int agent_id, const vector<vector<int>>& cons_paths,
                                       const vector<vector<int>>& old_paths, bool use_old) {
    switch (config.single_agent) {
    case SA_MLA_SEQUENCE: {
        MLAStarPlanner mla_star(*this);
        return mla_star.solve(MLAStarRequest(
            agent_id, (int)agents[agent_id].loc, (int)cur_time_,
            pbs_.task_groups[agent_id], cons_paths, old_paths, use_old));
    }
    case SA_MLSIPP_SEQUENCE: {
        SIPPPlanner sipp(*this);
        return sipp.solve(SIPPRequest(
            agent_id, (int)agents[agent_id].loc, (int)cur_time_,
            pbs_.goal_seqs[agent_id], cons_paths, old_paths, use_old));
    }
    default:
        cerr << "pbs_plan_agent: single_agent not implemented in this build" << endl;
        return {};
    }
}

// --- root node --------------------------------------------------------------
// Every agent is planned against the OTHERS' committed (old) paths only — no
// priorities yet.  The conflicts that survive become the DFS's work list.
PBSNode* Simulation::pbs_generate_root() {
    PBSNode* root = new PBSNode();
    root->paths.assign(pbs_.num_ag, {});
    for (int i = 0; i < pbs_.num_ag; i++) {
        root->paths[i].resize(pbs_.work_len);
        for (int t = 0; t < pbs_.work_len; t++)
            root->paths[i][t] = (int)path_table_[i][t];
    }

    for (int i = 0; i < pbs_.num_ag; i++) {
        vector<vector<int>> cons_buf;   // no priority constraints at the root
        vector<vector<int>> old_buf;    // MLA* reads the shared table directly
        if (config.single_agent == SA_MLSIPP_SEQUENCE) {
            // SIPP also receives the old paths for its final-goal holding-time
            // calculation; vertex/edge lookups still use the shared tables.
            int trunc = min(pbs_.path_horizon, (int)cur_time_ + 1000);
            old_buf.reserve(pbs_.num_ag - 1);
            for (int j = 0; j < pbs_.num_ag; j++) {
                if (j == i) continue;
                old_buf.emplace_back(pbs_.old_paths[j].begin(),
                                     pbs_.old_paths[j].begin() +
                                     min(trunc, (int)pbs_.old_paths[j].size()));
            }
        }

        shared_old_excl_ = i;
        vector<int> path = pbs_plan_agent(i, cons_buf, old_buf, true);
        shared_old_excl_ = -1;

        if (path.empty()) {
            // Retry ignoring old paths; if even that fails, stay put and let the
            // high level sort out the resulting conflicts.
            path = pbs_plan_agent(i, cons_buf, {}, false);
            if (path.empty()) {
                for (int t = (int)cur_time_; t < pbs_.work_len; t++)
                    root->paths[i][t] = (int)agents[i].loc;
                continue;
            }
        }
        for (int t = (int)cur_time_; t < pbs_.work_len; t++)
            root->paths[i][t] = path[t];
    }

    root->cost = pbs_node_cost(root);
    pbs_detect_conflicts(root->paths, root->conflicts);
    root->num_collisions = (int)root->conflicts.size();
    return root;
}

// Node cost = number of moves (waits are free), matching the reference.
int Simulation::pbs_node_cost(const PBSNode* node) const {
    int total = 0;
    for (int i = 0; i < pbs_.num_ag; i++) {
        int lim = min((int)node->paths[i].size(), pbs_.max_t) - 1;
        for (int t = (int)cur_time_; t < lim; t++)
            if (node->paths[i][t] != node->paths[i][t+1]) total++;
    }
    return total;
}

// Last timestep at which ANY agent still moves.  Beyond it every position is
// frozen, so a conflict in the tail already shows up here — bounding the
// conflict scan at settle+1 is exact and skips the long constant tail.
int Simulation::pbs_settle_time(const vector<vector<int>>& paths) const {
    int last_move = (int)cur_time_;
    for (int i = 0; i < pbs_.num_ag; i++) {
        int li = (int)paths[i].size() - 1;
        while (li > (int)cur_time_ && paths[i][li] == paths[i][li-1]) li--;
        if (li > last_move) last_move = li;
    }
    return last_move;
}

int Simulation::pbs_conflict_horizon(const vector<vector<int>>& paths) const {
    return min(pbs_.max_t, pbs_settle_time(paths) + 1);
}

// First vertex/edge conflict of each agent pair (at most one per pair).
void Simulation::pbs_detect_conflicts(const vector<vector<int>>& paths,
                                      list<Conflict>& out) const {
    int horizon = pbs_conflict_horizon(paths);
    for (int a1 = 0; a1 < pbs_.num_ag; a1++) {
        for (int a2 = a1 + 1; a2 < pbs_.num_ag; a2++) {
            for (int t = (int)cur_time_; t < horizon && t < pbs_.max_t; t++) {
                int loc1 = (t < (int)paths[a1].size()) ? paths[a1][t] : paths[a1].back();
                int loc2 = (t < (int)paths[a2].size()) ? paths[a2][t] : paths[a2].back();
                if (loc1 == loc2) { out.emplace_back(a1, a2, loc1, -1, t); break; }
                if (t > 0) {
                    int pl1 = (t-1 < (int)paths[a1].size()) ? paths[a1][t-1] : paths[a1].back();
                    int pl2 = (t-1 < (int)paths[a2].size()) ? paths[a2][t-1] : paths[a2].back();
                    if (loc1 == pl2 && loc2 == pl1) {
                        out.emplace_back(a1, a2, loc1, loc2, t);
                        break;
                    }
                }
            }
        }
    }
}

// Conflicts involving one specific agent, against a precomputed horizon.
void Simulation::pbs_detect_conflicts_for(int agent_id, const vector<vector<int>>& paths,
                                          int horizon, list<Conflict>& out) const {
    for (int j = 0; j < pbs_.num_ag; j++) {
        if (j == agent_id) continue;
        for (int t = (int)cur_time_; t < horizon && t < pbs_.max_t; t++) {
            int la = (t < (int)paths[agent_id].size()) ? paths[agent_id][t] : paths[agent_id].back();
            int lj = (t < (int)paths[j].size()) ? paths[j][t] : paths[j].back();
            if (la == lj) { out.emplace_back(agent_id, j, la, -1, t); break; }
            if (t > 0) {
                int pa = (t-1 < (int)paths[agent_id].size()) ? paths[agent_id][t-1] : paths[agent_id].back();
                int pj = (t-1 < (int)paths[j].size()) ? paths[j][t-1] : paths[j].back();
                if (la == pj && lj == pa) {
                    out.emplace_back(agent_id, j, la, lj, t);
                    break;
                }
            }
        }
    }
}

// A conflict between two agents whose priority is already fixed is
// inconsistent: the lower-priority one has to be re-planned.
void Simulation::pbs_find_replan_agents(const PriorityGraph& priorities,
                                        const list<Conflict>& conflicts,
                                        unordered_set<int>& replan) const {
    for (auto& conf : conflicts) {
        int ca = get<0>(conf), cb = get<1>(conf);
        if (replan.count(ca) || replan.count(cb)) continue;
        if (priorities.connected(ca, cb))      replan.insert(ca);
        else if (priorities.connected(cb, ca)) replan.insert(cb);
    }
}

// --- child node -------------------------------------------------------------
// "lower yields to higher": add the priority, re-plan `lower` under everyone
// that now outranks it, then cascade until the node is priority-consistent.
// Returns nullptr when the branch is infeasible (cycle / no path / no cascade).
PBSNode* Simulation::pbs_generate_child(PBSNode* parent, int lower, int higher) {
    PBSNode* child = new PBSNode();
    child->parent = parent;
    child->priorities.copy(parent->priorities);
    child->paths = parent->paths;

    if (child->priorities.connected(higher, lower)) {   // would create a cycle
        delete child;
        return nullptr;
    }
    child->priorities.add(lower, higher);
    child->priority = {lower, higher};

    set<int> higher_set = child->priorities.get_higher_priority(lower);
    vector<vector<int>> cons;
    for (int hp : higher_set) cons.push_back(child->paths[hp]);

    // Old paths of ALL other agents stay in the table (reference find_path).
    vector<vector<int>> old_for_agent;
    for (int j = 0; j < pbs_.num_ag; j++) {
        if (j == lower) continue;
        old_for_agent.push_back(pbs_.old_paths[j]);
    }

    vector<int> new_path = pbs_plan_agent(lower, cons, old_for_agent, true);
    if (new_path.empty()) {
        delete child;
        return nullptr;
    }
    for (int t = (int)cur_time_; t < pbs_.work_len; t++)
        child->paths[lower][t] = new_path[t];

    if (!pbs_resolve_cascade(child, lower, parent)) {
        delete child;
        return nullptr;
    }

    child->num_collisions = (int)child->conflicts.size();
    child->cost = pbs_node_cost(child);
    return child;
}

// find_consistent_paths: keep re-planning lower-priority agents until no
// conflict contradicts the priority order.  Failing inside the budget makes the
// whole child invalid (keeping it would let the DFS re-expand it forever).
bool Simulation::pbs_resolve_cascade(PBSNode* node, int replanned_agent, const PBSNode* parent) {
    int horizon = pbs_conflict_horizon(node->paths);

    node->conflicts.clear();
    pbs_detect_conflicts_for(replanned_agent, node->paths, horizon, node->conflicts);
    for (auto& conf : parent->conflicts)
        if (get<0>(conf) != replanned_agent && get<1>(conf) != replanned_agent)
            node->conflicts.push_back(conf);

    unordered_set<int> replan;
    pbs_find_replan_agents(node->priorities, node->conflicts, replan);

    const int cascade_budget = pbs_.num_ag * 5;   // reference: node->paths.size() * 5
    int cascade_count = 0;

    while (!replan.empty()) {
        if (cascade_count > cascade_budget) return false;

        int a = *replan.begin();
        replan.erase(replan.begin());
        cascade_count++;

        set<int> hp = node->priorities.get_higher_priority(a);
        vector<vector<int>> a_cons;
        for (int h : hp) a_cons.push_back(node->paths[h]);

        vector<vector<int>> a_old;
        for (int j = 0; j < pbs_.num_ag; j++) {
            if (j == a) continue;
            a_old.push_back(pbs_.old_paths[j]);
        }

        vector<int> np = pbs_plan_agent(a, a_cons, a_old, true);
        if (np.empty()) return false;
        for (int t = (int)cur_time_; t < pbs_.work_len; t++)
            node->paths[a][t] = np[t];

        node->conflicts.remove_if([a](const Conflict& cf) {
            return get<0>(cf) == a || get<1>(cf) == a;
        });
        list<Conflict> new_confs;
        pbs_detect_conflicts_for(a, node->paths, horizon, new_confs);
        pbs_find_replan_agents(node->priorities, new_confs, replan);
        node->conflicts.splice(node->conflicts.end(), new_confs);
    }
    return true;
}

// Safety net: PBS can run out of budget (or prune both children of a conflict)
// and hand back a plan that still contains a conflict.  Committing that would
// be a real collision, so resolve whatever is left the way PBS would: the agent
// moving into the clash waits one step.  Bounded, so it always terminates.
void Simulation::pbs_final_deconflict(PBSNode* node) {
    auto& P = node->paths;
    int t0 = (int)cur_time_;
    int plen = (int)P[0].size();
    const int MAX_ROUNDS = pbs_.num_ag * pbs_.num_ag * 4 + 16;
    int rounds = 0;
    bool any = true;

    while (any && rounds < MAX_ROUNDS) {
        any = false; rounds++;
        for (int a1 = 0; a1 < pbs_.num_ag && !any; a1++) {
            for (int a2 = a1 + 1; a2 < pbs_.num_ag && !any; a2++) {
                for (int t = t0 + 1; t < plen; t++) {
                    bool vertex = (P[a1][t] == P[a2][t]);
                    bool edge = (P[a1][t] == P[a2][t-1] && P[a2][t] == P[a1][t-1] &&
                                 P[a1][t] != P[a1][t-1]);
                    if (!vertex && !edge) continue;

                    int yielder = a2;
                    if (P[a1][t] != P[a1][t-1] && P[a2][t] == P[a2][t-1]) yielder = a1;
                    else if (P[a1][t] == P[a1][t-1] && P[a2][t] != P[a2][t-1]) yielder = a2;

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
    node->conflicts.clear();
}

// Write a node's (work_len-bounded) paths into the committed state, extending
// each agent's final cell out to the full horizon.
void Simulation::pbs_commit(const PBSNode* node) {
    for (int i = 0; i < pbs_.num_ag; i++) {
        int plen = (int)node->paths[i].size();
        int hold = node->paths[i][min(plen, pbs_.max_t) - 1];
        for (int t = 0; t < pbs_.max_t; t++) {
            int v = (t < plen) ? node->paths[i][t] : hold;
            path_table_[i][t] = v;
            agents[i].path[t] = v;
        }
    }
}

// --- the PBS search ---------------------------------------------------------
bool Simulation::pbs_solve_impl() {
    pbs_build_context();
    pbs_build_shared_old_table();

    vector<PBSNode*> all_nodes;
    PBSNode* root = pbs_generate_root();
    all_nodes.push_back(root);

    if (root->conflicts.empty()) {
        pbs_commit(root);
        for (PBSNode* n : all_nodes) delete n;
        pbs_clear_shared_old_table();
        return true;
    }

    // Depth-first over priority orderings, keeping the best node seen.
    stack<PBSNode*> dfs_stack;
    dfs_stack.push(root);

    PBSNode* best_node = root;
    best_node->conflict = root->conflicts.front();
    for (auto& c : root->conflicts)
        if (get<4>(c) < get<4>(best_node->conflict)) best_node->conflict = c;
    best_node->earliest_collision = get<4>(best_node->conflict);

    int hl_expanded = 0;
    const int max_hl = (pbs_.num_ag > 30) ? 5000 : 50000;

    // Agent pairs whose BOTH children turned out infeasible are dead ends.
    // Resolving such a pair first makes the branch fail (and be pruned)
    // immediately instead of re-discovering it deeper in the tree.
    std::set<std::pair<int,int>> nogood;

    while (!dfs_stack.empty() && hl_expanded < max_hl) {
        PBSNode* curr = dfs_stack.top();
        dfs_stack.pop();

        if (curr->conflicts.empty()) { best_node = curr; break; }

        // choose_conflict: earliest one, unless a known dead end is present.
        Conflict chosen = curr->conflicts.front();
        for (auto& c : curr->conflicts)
            if (get<4>(c) < get<4>(chosen)) chosen = c;
        curr->earliest_collision = get<4>(chosen);
        if (!nogood.empty()) {
            for (auto& c : curr->conflicts) {
                int ca = get<0>(c), cb = get<1>(c);
                if (nogood.count({ca, cb}) || nogood.count({cb, ca})) { chosen = c; break; }
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

        // Two ways to resolve it: a1 yields to a2, or a2 yields to a1.
        PBSNode* child0 = pbs_generate_child(curr, a1, a2);
        PBSNode* child1 = pbs_generate_child(curr, a2, a1);
        if (child0) all_nodes.push_back(child0);
        if (child1) all_nodes.push_back(child1);

        if (child0 && child1) {
            // Explore the cheaper child first (cost, then fewer conflicts).
            if (child0->cost < child1->cost ||
                (child0->cost == child1->cost &&
                 child0->num_collisions < child1->num_collisions)) {
                dfs_stack.push(child1);
                dfs_stack.push(child0);
            } else {
                dfs_stack.push(child0);
                dfs_stack.push(child1);
            }
        } else if (child0) {
            dfs_stack.push(child0);
        } else if (child1) {
            dfs_stack.push(child1);
        } else {
            nogood.emplace(a1, a2);
        }
    }

    if (!best_node->conflicts.empty())
        pbs_final_deconflict(best_node);

    pbs_commit(best_node);

    for (PBSNode* n : all_nodes) delete n;
    pbs_clear_shared_old_table();
    return true;
}

// ============================================================================
//  Section 6 — Single-agent search: STA* / MLA* / MLSIPP
// ============================================================================

bool Simulation::sta_is_constrained(int agent_id, int current_loc, int next_loc,
                                    int next_time, int hidden_agent) const {
    if (next_loc < 0 || next_loc >= (int)mapd_map.grid.size() ||
        !mapd_map.grid[next_loc])
        return true;

    for (int other = 0; other < (int)path_table_.size(); other++) {
        if (other == agent_id || other == hidden_agent) continue;
        if (path_table_[other][next_time] == (unsigned int)next_loc) return true;
        if (path_table_[other][next_time - 1] == (unsigned int)next_loc &&
            path_table_[other][next_time] == (unsigned int)current_loc)
            return true;
    }
    return false;
}

int Simulation::sta_search(Agent& agent, int start_loc, int begin_time,
                           const Endpoint& goal, int hidden_agent) {
    heap_open_t open;
    map<unsigned int, SearchNode*> nodes;

    SearchNode* start = new SearchNode(
        start_loc, 0, goal.h_val[start_loc], nullptr, begin_time);
    open.push(start);
    nodes.emplace((unsigned int)start_loc, start);

    int expanded = 0;
    while (!open.empty()) {
        SearchNode* current = open.top();
        open.pop();
        current->in_openlist = false;

        if (++expanded > 50000) {
            release_search_nodes(nodes);
            // Every non-wPBS configuration using this STA* routine is expected
            // to find the guaranteed path on a well-formed instance. Do not
            // let this artificial safeguard look like an ordinary candidate
            // failure and allow an incomplete run to continue silently.
            if (config.mapf != MAPF_wPBS) {
                throw runtime_error(
                    "STA* exceeded 50000 expansions without finding a path "
                    "for agent " + to_string(agent.id) + " from location " +
                    to_string(start_loc) + " to location " +
                    to_string(goal.loc) + " at time " +
                    to_string(begin_time));
            }
            return -1;
        }

        if (current->loc == goal.loc) {
            bool can_hold = true;
            for (unsigned int t = current->timestep + 1; t < maxtime && can_hold; t++)
                for (int other = 0; other < (int)path_table_.size(); other++)
                    if (other != agent.id && other != hidden_agent &&
                        path_table_[other][t] == (unsigned int)current->loc) {
                        can_hold = false;
                        break;
                    }

            if (can_hold) {
                sta_update_path(agent, *current);
                int arrival_time = current->timestep;
                release_search_nodes(nodes);
                return arrival_time;
            }
        }

        if ((unsigned int)current->timestep >= maxtime - 1) continue;

        // Reference action order: wait, east, west, south, north.
        const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};
        for (int action : actions) {
            int next_loc = current->loc + action;
            int next_time = current->timestep + 1;
            if (sta_is_constrained(agent.id, current->loc, next_loc,
                                   next_time, hidden_agent))
                continue;

            int next_g = current->g_val + 1;
            SearchNode* next = new SearchNode(
                next_loc, next_g, goal.h_val[next_loc], current, next_time);
            unsigned int key = next->loc + next->g_val * mapd_map.row * mapd_map.col;
            if (nodes.find(key) == nodes.end()) {
                nodes.emplace(key, next);
                open.push(next);
            } else {
                delete next;
            }
        }
    }

    release_search_nodes(nodes);
    return -1;
}

void Simulation::sta_update_path(Agent& agent, const SearchNode& goal_node) {
    for (unsigned int t = goal_node.timestep + 1; t < agent.path.size(); t++)
        agent.path[t] = goal_node.loc;

    const SearchNode* node = &goal_node;
    while (node != nullptr) {
        agent.path[node->timestep] = node->loc;
        node = node->parent;
    }
}

pair<int,int> Simulation::plan_token_task(Agent& agent, Task& task,
                                          int hidden_agent) {
    // Shared TP/TPTS low-level switch. Recursive TPTS steals pass the displaced
    // agent as hidden_agent so the replacement path may take over its
    // reservation. The returned pair is (first-goal arrival, final-goal
    // arrival), preserving the timestamps TPTS needs for task stealing.
    switch (config.single_agent) {
    case SA_STA_TASK_EP: {
        STAStarPlanner sta(*this);
        return sta.solveTask(STAStarRequest(agent, task, hidden_agent));
    }
    case SA_MLSIPP_SEQUENCE:
        return plan_task_sipp(agent, task, hidden_agent);
    default:
        cerr << "plan_token_task: single_agent not implemented" << endl;
        return {-1, -1};
    }
}

pair<int,int> Simulation::plan_task_sta_impl(Agent& agent, Task& task,
                                             int hidden_agent) {
    int hidden = hidden_agent >= 0 ? hidden_agent : agent.id;
    vector<unsigned int> saved_path(agent.path.begin(), agent.path.end());

    // TP/TPTS still plan one complete task per token operation, but a task may
    // now contain any number of ordered goals. The first goal keeps the
    // original pickup semantics (including start_wait_time); completion is
    // recorded at the final goal.
    vector<int> goal_locations = task.goals;
    if (goal_locations.empty())
        goal_locations = {task.pickup_loc, task.delivery_loc};

    int first_goal_time = -1;
    int final_goal_time = -1;
    int start_loc = agent.loc;
    int begin_time = (int)cur_time_;

    for (int goal_index = 0;
         goal_index < (int)goal_locations.size(); goal_index++) {
        int endpoint_index;
        if (goal_index == 0)
            endpoint_index = task.pickup;
        else if (goal_index == (int)goal_locations.size() - 1)
            endpoint_index = task.delivery;
        else
            endpoint_index = mapd_map.ep_index(goal_locations[goal_index]);

        if (endpoint_index < 0) {
            agent.path.assign(saved_path.begin(), saved_path.end());
            return {-1, -1};
        }

        int arrival_time = sta_search(
            agent, start_loc, begin_time,
            mapd_map.endpoints[endpoint_index], hidden);
        if (arrival_time < 0) {
            agent.path.assign(saved_path.begin(), saved_path.end());
            return {-1, -1};
        }

        if (goal_index == 0) first_goal_time = arrival_time;
        final_goal_time = arrival_time;
        start_loc = goal_locations[goal_index];
        begin_time = arrival_time;
        if (goal_index == 0) begin_time += task.start_wait_time;
    }

    return {first_goal_time, final_goal_time};
}

pair<int,int> Simulation::plan_task_sipp(Agent& agent, Task& task,
                                         int hidden_agent) {
    const int hidden = hidden_agent >= 0 ? hidden_agent : agent.id;

    vector<vector<int>> constraints;
    constraints.reserve(path_table_.size() - 1);
    for (int other = 0; other < (int)path_table_.size(); other++) {
        if (other == agent.id || other == hidden) continue;
        constraints.emplace_back(path_table_[other].begin(), path_table_[other].end());
    }

    const vector<vector<int>> no_old_paths;
    vector<int> goal_locations = task.goals;
    if (goal_locations.empty())
        goal_locations = {task.pickup_loc, task.delivery_loc};

    vector<pair<int,int>> goals;
    goals.reserve(goal_locations.size());
    for (int goal_index = 0;
         goal_index < (int)goal_locations.size(); goal_index++)
        goals.push_back({
            goal_locations[goal_index],
            goal_index == 0 ? task.release_time : 0});
    SIPPPlanner sipp(*this);
    vector<int> path = sipp.solve(SIPPRequest(
        agent.id, agent.loc, (int)cur_time_, goals,
        constraints, no_old_paths, false));
    if (path.empty()) return {-1, -1};

    int first_goal_time = -1;
    int final_goal_time = -1;
    int goal_index = 0;
    for (int t = (int)cur_time_;
         t < (int)path.size() && goal_index < (int)goals.size(); t++) {
        if (path[t] == goals[goal_index].first && t >= goals[goal_index].second) {
            if (goal_index == 0) first_goal_time = t;
            if (goal_index == (int)goals.size() - 1) final_goal_time = t;
            goal_index++;
        }
    }
    if (first_goal_time < 0 || final_goal_time < 0) return {-1, -1};

    // Validate the returned SIPP path against TP's token table, including edge
    // swaps, and ensure its final goal can be held permanently.
    for (int t = (int)cur_time_ + 1;
         t <= final_goal_time && t < (int)maxtime; t++)
        if (sta_is_constrained(agent.id, path[t - 1], path[t], t, hidden))
            return {-1, -1};

    for (unsigned int t = final_goal_time + 1; t < maxtime; t++)
        for (int other = 0; other < (int)path_table_.size(); other++) {
            if (other == agent.id || other == hidden) continue;
            if (path_table_[other][t] ==
                (unsigned int)goal_locations.back())
                return {-1, -1};
        }

    for (int t = 0; t < (int)path.size() && t < (int)maxtime; t++)
        agent.path[t] = path[t];
    return {first_goal_time, final_goal_time};
}

int Simulation::search_path2_endpoint(Agent& agent, int target_endpoint_loc) {
    const bool hbh_non_task_policy =
        config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
    const bool hbh_open_goal_policy =
        hbh_non_task_policy ||
        (config.endpoint_strategy == NEAREST_AVAILABLE &&
         config.assign_method == AM_CENTRALIZED_GREEDY);
    if (target_endpoint_loc < 0 &&
        config.endpoint_strategy != NEAREST_AVAILABLE &&
        config.endpoint_strategy != WAIT_OR_NEAREST_SAFE &&
        !hbh_non_task_policy) {
        throw logic_error(
            "automatic Path2 endpoint search requires a wait-or-relocate "
            "endpoint strategy");
    }

    queue<SearchNode*> open;
    map<unsigned int, SearchNode*> nodes;
    const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    SearchNode* start = new SearchNode(agent.loc, 0, nullptr, (int)cur_time_);
    nodes.emplace((unsigned int)agent.loc, start);
    open.push(start);

    while (!open.empty()) {
        SearchNode* current = open.front();
        open.pop();
        if ((unsigned int)current->timestep >= maxtime - 1) continue;

        // When selecting a relocation target (target_endpoint_loc < 0), the
        // origin is not a valid answer. The BFS may leave and later revisit the
        // same endpoint, but choose_dummy_endpoint() returns only a location;
        // returning the origin would lose that required detour and be mistaken
        // by the caller for a safe one-step wait.
        const bool selecting_origin =
            target_endpoint_loc < 0 && current->loc == agent.loc;
        int endpoint_id = mapd_map.ep_index(current->loc);
        bool endpoint_type_allowed = endpoint_id >= 0 &&
            (!hbh_non_task_policy ||
             !mapd_map.endpoints[endpoint_id].is_task_endpoint);
        if (endpoint_type_allowed && !selecting_origin &&
            (target_endpoint_loc < 0 || current->loc == target_endpoint_loc)) {
            bool occupied = false;
            // WAIT_OR_NEAREST_SAFE condition 1: the endpoint must remain free of every
            // other agent's committed future path, so it can be held forever.
            for (unsigned int t = current->timestep; t < maxtime && !occupied; t++)
                for (int other = 0; other < (int)agents.size(); other++)
                    if (other != agent.id &&
                        path_table_[other][t] == (unsigned int)current->loc) {
                        occupied = true;
                        break;
                    }
            if (hbh_open_goal_policy) {
                // HBH must not relocate onto any goal of an open task.
                for (Task* task : open_tasks_)
                    for (int goal_location : task->goals)
                        if (!occupied && goal_location == current->loc)
                            occupied = true;
            } else {
                // WAIT_OR_NEAREST_SAFE condition 2: do not park at any
                // post-pickup goal still needed by an open multi-goal task.
                for (Task* task : open_tasks_)
                    for (int goal_index = 1;
                         goal_index < (int)task->goals.size(); goal_index++)
                        if (!occupied &&
                            task->goals[goal_index] == current->loc)
                            occupied = true;
            }

            if (!occupied) {
                sta_update_path(agent, *current);
                agent.finish_time = current->timestep;
                int endpoint_loc = current->loc;
                release_search_nodes(nodes);
                return endpoint_loc;
            }
        }

        for (int action : actions) {
            int next_loc = current->loc + action;
            int next_time = current->timestep + 1;
            if (sta_is_constrained(agent.id, current->loc, next_loc,
                                   next_time, agent.id))
                continue;
            unsigned int key = next_loc +
                (unsigned int)(current->g_val + 1) * mapd_map.row * mapd_map.col;
            if (nodes.find(key) == nodes.end()) {
                SearchNode* next = new SearchNode(
                    next_loc, current->g_val + 1, current, next_time);
                nodes.emplace(key, next);
                open.push(next);
            }
        }
    }

    release_search_nodes(nodes);
    return -1;
}

bool Simulation::plan_path2_to_endpoint(Agent& agent, int endpoint_loc) {
    if (search_path2_endpoint(agent, endpoint_loc) != endpoint_loc)
        return false;

    for (unsigned int t = cur_time_; t < path_table_[agent.id].size(); t++)
        path_table_[agent.id][t] = agent.path[t];
    return true;
}

void Simulation::release_search_nodes(map<unsigned int, SearchNode*>& nodes) {
    for (auto& entry : nodes) delete entry.second;
    nodes.clear();
}

// --------------------------------------------------------------------------
// MLA_SEQUENCE
//
//  Multi-label A*: a state is (location, timestep, goal_id) and reaching
//  goals[goal_id] after its release time advances the label.  The agent must be
//  able to HOLD its final goal forever, which is what makes a committed plan
//  deadlock-free.
// ============================================================================

// --- low-level node ---
namespace {

struct MLANode {
    int loc;
    int g_val;
    int h_val;
    int timestep;
    int goal_id;
    uint64_t tie_breaker;
    MLANode* parent;

    MLANode(int l, int g, int h, int t, int gi, MLANode* p)
        : loc(l), g_val(g), h_val(h), timestep(t), goal_id(gi),
          tie_breaker(RandomTieBreaker::next()), parent(p) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareMLANode {
    bool operator()(const MLANode* a, const MLANode* b) const {
        if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
        if (a->goal_id != b->goal_id) return a->goal_id < b->goal_id;   // prefer progress
        if (a->g_val != b->g_val) return a->g_val < b->g_val;
        return a->tie_breaker > b->tie_breaker;
    }
};

// Free-list allocator for the heap's internal nodes: the search runs millions
// of pushes per solve, so recycling the slots keeps the allocator out of the
// inner loop.  Ordering and behaviour are unchanged — only memory is reused.
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

}  // namespace

// --- task-by-task driver ----------------------------------------------------
// Plan one task group at a time and concatenate: the agent commits to reaching
// this task's goals before the next group is planned.  Only the last group may
// hold its goal (the earlier ones are passed through).
vector<int> Simulation::mla_star_taskwise_impl(
        int agent_id, int start_loc, int start_time,
        const vector<vector<pair<int,int>>>& task_groups,
        const vector<vector<int>>& cons_paths,
        const vector<vector<int>>& old_paths,
        bool use_old_paths) {
    if (task_groups.empty()) return {start_loc};

    const int out_len = (int)maxtime;
    vector<int> full_path(out_len, start_loc);
    for (int t = 0; t < min(start_time, out_len); t++) full_path[t] = start_loc;

    int cur_loc = start_loc;
    int cur_time = start_time;

    for (int g = 0; g < (int)task_groups.size(); g++) {
        const auto& goals = task_groups[g];
        if (goals.empty()) continue;

        bool is_last = (g == (int)task_groups.size() - 1);

        vector<int> segment = seq_mla_star(agent_id, cur_loc, cur_time, goals,
                                           cons_paths, old_paths, use_old_paths,
                                           /*skip_holding=*/!is_last);
        if (segment.empty()) return {};

        int seg_n = (int)segment.size();
        int seg_back = seg_n ? segment[seg_n - 1] : cur_loc;
        for (int t = cur_time; t < out_len; t++)
            full_path[t] = (t < seg_n) ? segment[t] : seg_back;

        // Advance to the timestep at which this group's last goal is reached.
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
        if (!found) return {};
    }

    return full_path;
}

// --- the search -------------------------------------------------------------
vector<int> Simulation::seq_mla_star(int agent_id, int start_loc, int start_time,
                                     const vector<pair<int,int>>& goals,
                                     const vector<vector<int>>& cons_paths,
                                     const vector<vector<int>>& old_paths,
                                     bool use_old_paths,
                                     bool skip_holding) {
    if (goals.empty()) return {start_loc};

    const int map_size = mapd_map.row * mapd_map.col;
    const int max_t = (int)maxtime;
    const int num_goals = (int)goals.size();

    // Read the shared old-path table when this is a root plan for its owner.
    const bool use_shared_old = use_old_paths && shared_old_valid_ &&
                                shared_old_excl_ == agent_id;

    // --- fast path: an idle agent already parked on its only goal ------------
    // At low task demand most agents just hold their endpoint.  If nobody else
    // ever touches that cell the search would return exactly this path, so
    // return it directly instead of setting up the whole A*.
    if (num_goals == 1 && goals[0].first == start_loc && start_time >= goals[0].second &&
        use_shared_old && !skip_holding && cons_paths.empty()) {
        bool excl_parked = (std::find(shared_old_movers_.begin(), shared_old_movers_.end(),
                                      shared_old_excl_) == shared_old_movers_.end());
        int parkers = shared_old_holdcnt_[start_loc];
        if (excl_parked && shared_old_holds_[shared_old_excl_] == start_loc) parkers--;
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
        if (!blocked) return vector<int>(max_t, start_loc);
    }

    // WAIT first, then EAST / NORTH / WEST / SOUTH (reference move order).
    const int action[5] = {0, 1, -mapd_map.col, -1, mapd_map.col};

    // Horizon cap: the forced endpoint hold below is capped at start+384, so a
    // complete path never needs more than ~start+512.  Bounding it here keeps
    // the per-search constraint arrays small.
    int search_horizon = min(max_t, start_time + max(1000, num_goals * 40));
    search_horizon = min(search_horizon, start_time + 512);

    // --- heuristics: BFS distance to each goal (endpoint tables when possible) ---
    vector<const vector<int>*> h_vals_ptr(num_goals, nullptr);
    vector<vector<int>> h_vals_computed;
    for (int g = 0; g < num_goals; g++) {
        int gloc = goals[g].first;
        int ei = mapd_map.ep_index(gloc);
        if (ei >= 0) {
            h_vals_ptr[g] = &mapd_map.endpoints[ei].h_val;
            continue;
        }
        h_vals_computed.emplace_back(map_size, INT_MAX);
        auto& hv = h_vals_computed.back();
        hv[gloc] = 0;
        queue<int> bfs_q;
        bfs_q.push(gloc);
        while (!bfs_q.empty()) {
            int u = bfs_q.front(); bfs_q.pop();
            for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
                int v = u + d;
                if (v >= 0 && v < map_size && mapd_map.grid[v] && hv[v] > hv[u] + 1) {
                    hv[v] = hv[u] + 1;
                    bfs_q.push(v);
                }
            }
        }
        h_vals_ptr[g] = &h_vals_computed.back();
    }

    // h = distance to the current goal + the chain through the remaining ones.
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

    // --- constraint tables ---------------------------------------------------
    // Every committed path is a short ACTIVE prefix followed by a CONSTANT hold,
    // so the per-timestep tables only cover the active prefix and the tail is
    // enforced as a permanent endpoint hold (exactly what the reference's
    // reservation table does, at a fraction of the memory).
    const int n_cons = (int)cons_paths.size();
    const int n_old = use_shared_old ? 0 : (use_old_paths ? (int)old_paths.size() : 0);

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
    int old_active = 0;
    if (use_shared_old) old_active = shared_old_horizon_;
    else for (int i = 0; i < n_old; i++) old_active = max(old_active, active_len(old_paths[i]));

    const int ct_horizon = min(search_horizon, cons_active + 1);
    const int op_horizon = min(search_horizon, old_active + 1);
    const int sh_hor = use_shared_old ? shared_old_horizon_ : 0;

    bool excl_parked = false;
    if (use_shared_old)
        excl_parked = (std::find(shared_old_movers_.begin(), shared_old_movers_.end(),
                                 shared_old_excl_) == shared_old_movers_.end());

    vector<int> cons_locs(n_cons * ct_horizon);
    vector<int> cons_endpoint_holds(n_cons, -1);
    for (int i = 0; i < n_cons; i++) {
        for (int t = 0; t < ct_horizon; t++)
            cons_locs[i * ct_horizon + t] =
                (t < (int)cons_paths[i].size()) ? cons_paths[i][t] : cons_paths[i].back();
        if (ct_horizon > 0) cons_endpoint_holds[i] = (int)cons_paths[i].back();
    }

    vector<int> old_locs;
    vector<int> old_endpoint_holds;
    if (n_old > 0) {
        old_locs.resize(n_old * op_horizon);
        old_endpoint_holds.assign(n_old, -1);
        for (int i = 0; i < n_old; i++) {
            for (int t = 0; t < op_horizon; t++)
                old_locs[i * op_horizon + t] =
                    (t < (int)old_paths[i].size()) ? old_paths[i][t] : old_paths[i].back();
            old_endpoint_holds[i] = (int)old_paths[i].back();
        }
    }

    // Hard constraints: the paths of the agents that outrank this one.
    auto is_constrained_hard = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        if (!mapd_map.grid[next_loc]) return true;
        if (abs_t >= ct_horizon) {
            for (int i = 0; i < n_cons; i++)
                if (cons_endpoint_holds[i] == next_loc) return true;
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

    // The other agents' previously committed paths.
    auto is_constrained_old = [&](int curr_loc, int next_loc, int abs_t) -> bool {
        if (use_shared_old) {
            // Parked agents are an O(1) test; only movers need a table scan.
            int cnt = shared_old_holdcnt_[next_loc];
            if (cnt > 0) {
                bool self_parked_here = excl_parked &&
                    (shared_old_holds_[shared_old_excl_] == next_loc);
                if (!(self_parked_here && cnt == 1)) return true;
            }
            if (abs_t < sh_hor) {
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
                for (int i : shared_old_movers_) {
                    if (i == shared_old_excl_) continue;
                    if (shared_old_holds_[i] == next_loc) return true;
                }
            }
            return false;
        }
        if (n_old == 0) return false;
        if (abs_t >= op_horizon) {
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

    // --- endpoint holding ----------------------------------------------------
    // The agent may only settle on its final goal once nobody else needs that
    // cell.  The forced wait is capped: if the agent settles "too early" and a
    // later arrival conflicts, PBS resolves it by priority like any conflict.
    const int last_goal_loc = goals.back().first;
    const int holding_cap = start_time + 384;
    int earliest_holding = 0;

    if (!skip_holding) {
        for (auto& cp : cons_paths) {
            int scan_limit = min((int)cp.size(), search_horizon);
            for (int t = scan_limit - 1; t >= 0; t--)
                if (cp[t] == last_goal_loc) {
                    earliest_holding = max(earliest_holding, t + 1);
                    break;
                }
        }
        if (use_shared_old) {
            int park_cnt = shared_old_holdcnt_[last_goal_loc];
            if (excl_parked && shared_old_holds_[shared_old_excl_] == last_goal_loc) park_cnt--;
            if (park_cnt > 0)
                earliest_holding = max(earliest_holding, min(search_horizon, search_horizon + 1));
            for (int i : shared_old_movers_) {
                if (i == shared_old_excl_) continue;
                for (int t = sh_hor - 1; t >= 0; t--)
                    if (shared_old_flat_[(size_t)i * sh_hor + t] == last_goal_loc) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
            }
        } else if (use_old_paths) {
            for (auto& op : old_paths) {
                int scan_limit = min({(int)op.size(), search_horizon, search_horizon + 1});
                for (int t = scan_limit - 1; t >= 0; t--)
                    if (op[t] == last_goal_loc) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
            }
        }
        if (earliest_holding > holding_cap) earliest_holding = holding_cap;
    }

    // If a HIGHER-PRIORITY agent is parked on the final goal forever the search
    // is hopeless — it will never be replanned to yield.  (An OLD path does not
    // count: PBS may still move that agent aside.)
    if (start_loc != last_goal_loc) {
        for (int i = 0; i < n_cons; i++)
            if (!cons_paths[i].empty() && cons_paths[i].back() == last_goal_loc)
                return {};
    }

    // --- A* ------------------------------------------------------------------
    typedef boost::heap::fibonacci_heap<MLANode*, boost::heap::compare<CompareMLANode>,
            boost::heap::allocator<MLAPoolAllocator<MLANode*>>> mla_heap_t;
    mla_heap_t open_list;

    struct KeyHash { size_t operator()(uint64_t k) const { return k * 2654435761ULL; } };
    static unordered_map<uint64_t, MLANode*, KeyHash> allNodes;   // retained across calls
    allNodes.clear();
    auto make_key = [](int loc, int gi, int g) -> uint64_t {
        return ((uint64_t)loc << 32) | ((uint64_t)gi << 20) | (uint64_t)g;
    };

    int init_h = compute_h(start_loc, 0);
    if (init_h == INT_MAX) return {};

    // A real path is found in far fewer nodes than this; a search that exhausts
    // the cap has no path within the horizon (typically its goal is occupied
    // until a far-future deadline) and is abandoned cheaply.
    const int mla_max_nodes = min(500000, map_size * 20);

    mla_arena_used_chunk_ = 0;
    mla_arena_used_in_chunk_ = 0;
    auto alloc_node = [&](int l, int g, int h, int t, int gi, MLANode* p) -> MLANode* {
        if (mla_arena_used_chunk_ >= (int)mla_arena_chunks_.size())
            mla_arena_chunks_.push_back(::operator new(sizeof(MLANode) * (size_t)MLA_ARENA_CHUNK));
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

    while (!open_list.empty() && mla_expanded < mla_max_nodes) {
        MLANode* curr = open_list.top();
        open_list.pop();
        mla_expanded++;

        // Label advance: standing on the current goal past its release time
        // completes it — unless it is the final goal and the agent is not yet
        // allowed to settle there.
        int gi = curr->goal_id;
        if (gi < num_goals && curr->loc == goals[gi].first) {
            int rel = goals[gi].second;
            if (curr->timestep >= rel) {
                if (gi == num_goals - 1 && curr->timestep < earliest_holding) {}
                else gi++;
            }
        }

        if (gi >= num_goals) { solution = curr; break; }
        if (curr->timestep >= search_horizon - 1) continue;

        int child_gi = gi;

        for (int i = 0; i < 5; i++) {
            int next_loc = curr->loc + action[i];
            int next_t = curr->timestep + 1;

            if (next_loc < 0 || next_loc >= map_size) continue;
            if (abs(next_loc % mapd_map.col - curr->loc % mapd_map.col) > 1) continue;   // wrapped
            if (is_constrained_hard(curr->loc, next_loc, next_t)) continue;
            if (is_constrained_old(curr->loc, next_loc, next_t)) continue;

            int next_g = curr->g_val + 1;
            int next_h = compute_h(next_loc, child_gi);
            if (next_h == INT_MAX) continue;

            // Deadline-aware heuristic: the agent cannot finish before
            // earliest_holding, so the true remaining cost is at least
            // (earliest_holding - next_t).  Folding that in keeps the heuristic
            // admissible while making A* beeline to the goal and wait there,
            // instead of fanning across the map for the whole holding period.
            if (earliest_holding > 0) {
                int deadline_h = earliest_holding - next_t;
                if (deadline_h > next_h) next_h = deadline_h;
            }

            uint64_t key = make_key(next_loc, child_gi, next_g);
            if (allNodes.find(key) == allNodes.end()) {
                MLANode* next_node = alloc_node(next_loc, next_g, next_h, next_t, child_gi, curr);
                allNodes[key] = next_node;
                open_list.push(next_node);
            }
        }
    }

    // --- reconstruct ---------------------------------------------------------
    vector<int> result;
    if (solution) {
        vector<int> path_locs;
        for (MLANode* node = solution; node != nullptr; node = node->parent)
            path_locs.push_back(node->loc);
        reverse(path_locs.begin(), path_locs.end());

        result.resize(max_t);
        for (int t = 0; t < start_time && t < max_t; t++) result[t] = start_loc;
        for (int i = 0; i < (int)path_locs.size(); i++) {
            int t = start_time + i;
            if (t < max_t) result[t] = path_locs[i];
        }
        int last_loc = path_locs.back();
        for (int t = start_time + (int)path_locs.size(); t < max_t; t++) result[t] = last_loc;
    }
    // Nodes live in the retained arena (trivially destructible); nothing to free.
    return result;
}

// --- Multi-label Safe Interval Path Planning -------------------------------
// Same goal semantics as seq_mla_star(), but states use maximal collision-free
// time intervals instead of one state per timestep.
vector<int> Simulation::sipp_search_impl(
        int agent_id, int start_loc, int start_time,
        const vector<pair<int,int>>& goals,
        const vector<vector<int>>& cons_paths,
        const vector<vector<int>>& old_paths,
        bool use_old_paths,
        bool skip_holding) {
    if (goals.empty()) return vector<int>((int)maxtime, start_loc);

    const int map_size = mapd_map.row * mapd_map.col;
    const int max_t = (int)maxtime;
    const int cols = mapd_map.col;
    const int num_goals = (int)goals.size();
    const int horizon = min(max_t, start_time + max(1000, num_goals * 40));

    auto h_for = [&](int loc, int goal_loc) -> int {
        int endpoint = mapd_map.ep_index(goal_loc);
        return endpoint >= 0 ? mapd_map.endpoints[endpoint].h_val[loc] : INT_MAX;
    };
    auto sum_h = [&](int loc, int goal_id) -> int {
        if (goal_id >= num_goals) return 0;
        int h = h_for(loc, goals[goal_id].first);
        if (h == INT_MAX) return INT_MAX;
        for (int g = goal_id; g < num_goals - 1; g++) {
            int d = h_for(goals[g].first, goals[g + 1].first);
            if (d == INT_MAX) return INT_MAX;
            h += d;
        }
        return h;
    };

    const bool use_shared_old = use_old_paths && shared_old_valid_ &&
                                shared_old_excl_ == agent_id && cons_paths.empty();

    // Build occupied intervals [start,end) from the higher-priority paths and
    // the other agents' previously committed paths.
    if ((int)sipp_ct_.size() != map_size) {
        sipp_ct_.assign(map_size, {});
        sipp_has_ct_.assign(map_size, 0);
        sipp_sit_.assign(map_size, {});
        sipp_sit_done_.assign(map_size, 0);
    }
    auto& occupied = sipp_ct_;
    auto& has_occupied = sipp_has_ct_;
    sipp_ct_touched_.clear();
    sipp_sit_touched_.clear();

    auto touch_occupied = [&](int loc) {
        if (!has_occupied[loc]) {
            has_occupied[loc] = 1;
            sipp_ct_touched_.push_back(loc);
            occupied[loc].clear();
        }
    };
    auto reset_scratch = [&]() {
        for (int loc : sipp_ct_touched_) has_occupied[loc] = 0;
        for (int loc : sipp_sit_touched_) sipp_sit_done_[loc] = 0;
    };

    const int num_constraints = (int)cons_paths.size();
    vector<int> constraint_last_change(num_constraints, 0);
    vector<int> constraint_tail(num_constraints, -1);

    auto add_path = [&](const vector<int>& path, int constraint_index) {
        const int len = min((int)path.size(), horizon);
        if (len == 0) return;

        int last_change = 0;
        for (int t = len - 1; t >= 1; t--)
            if (path[t] != path[t - 1]) {
                last_change = t;
                break;
            }

        int range_start = 0;
        int range_loc = path[0];
        for (int t = 1; t <= last_change; t++) {
            if (path[t] != range_loc) {
                touch_occupied(range_loc);
                occupied[range_loc].push_back({range_start, t});
                range_loc = path[t];
                range_start = t;
            }
        }
        touch_occupied(range_loc);
        occupied[range_loc].push_back({range_start, horizon});

        if (constraint_index >= 0) {
            constraint_last_change[constraint_index] = last_change;
            constraint_tail[constraint_index] = range_loc;
        }
    };

    for (int i = 0; i < num_constraints; i++) add_path(cons_paths[i], i);

    const bool use_shared_old_ranges = use_shared_old &&
        (int)shared_old_sipp_ranges_.size() == (int)agents.size();
    if (use_shared_old_ranges) {
        const int old_horizon = min(start_time + 300, horizon);
        for (int agent = 0; agent < (int)shared_old_sipp_ranges_.size(); agent++) {
            if (agent == shared_old_excl_) continue;
            for (const auto& range : shared_old_sipp_ranges_[agent]) {
                int loc = get<0>(range);
                int begin = get<1>(range);
                int end = min(get<2>(range), old_horizon);
                if (begin >= old_horizon || loc < 0 || loc >= map_size) continue;
                touch_occupied(loc);
                occupied[loc].push_back({begin, end});
            }
        }
    } else if (use_old_paths) {
        const int old_horizon = min(start_time + 300, horizon);
        for (const auto& path : old_paths) {
            const int len = min((int)path.size(), old_horizon);
            if (len == 0) continue;

            int range_start = 0;
            int range_loc = path[0];
            for (int t = 1; t < len; t++) {
                if (path[t] != range_loc) {
                    touch_occupied(range_loc);
                    occupied[range_loc].push_back({range_start, t});
                    range_loc = path[t];
                    range_start = t;
                }
            }
            touch_occupied(range_loc);
            occupied[range_loc].push_back({range_start, old_horizon});
        }
    }

    auto& safe_intervals = sipp_sit_;
    auto& intervals_ready = sipp_sit_done_;
    auto get_safe_intervals = [&](int loc) -> const vector<pair<int,int>>& {
        if (intervals_ready[loc]) return safe_intervals[loc];
        intervals_ready[loc] = 1;
        sipp_sit_touched_.push_back(loc);

        auto& result = safe_intervals[loc];
        result.clear();
        if (!has_occupied[loc]) return result;

        auto& ranges = occupied[loc];
        sort(ranges.begin(), ranges.end());
        int free_start = 0;
        int merged_start = -1;
        int merged_end = -1;
        for (const auto& range : ranges) {
            int begin = range.first;
            int end = range.second;
            if (merged_start < 0) {
                merged_start = begin;
                merged_end = end;
            } else if (begin <= merged_end) {
                merged_end = max(merged_end, end);
            } else {
                if (merged_start > free_start) result.push_back({free_start, merged_start});
                free_start = merged_end;
                merged_start = begin;
                merged_end = end;
            }
        }
        if (merged_start >= 0) {
            if (merged_start > free_start) result.push_back({free_start, merged_start});
            free_start = merged_end;
        }
        if (free_start < horizon) result.push_back({free_start, horizon});
        return result;
    };

    auto edge_blocked = [&](int from, int to, int timestep) -> bool {
        if (timestep <= 0 || timestep >= horizon) return false;
        for (const auto& path : cons_paths) {
            int len = (int)path.size();
            int curr = timestep < len ? path[timestep] : path[len - 1];
            int prev = timestep - 1 < len ? path[timestep - 1] : path[len - 1];
            if (curr == from && prev == to) return true;
        }

        if (use_shared_old) {
            int limit = min(shared_old_horizon_, start_time + 301);
            if (timestep < limit) {
                for (int agent : shared_old_movers_) {
                    if (agent == shared_old_excl_) continue;
                    int curr = shared_old_flat_[(size_t)agent * shared_old_horizon_ + timestep];
                    int prev = shared_old_flat_[(size_t)agent * shared_old_horizon_ + timestep - 1];
                    if (curr == from && prev == to) return true;
                }
            }
        } else if (use_old_paths && timestep <= start_time + 300) {
            for (const auto& path : old_paths) {
                int len = (int)path.size();
                int curr = timestep < len ? path[timestep] : path[len - 1];
                int prev = timestep - 1 < len ? path[timestep - 1] : path[len - 1];
                if (curr == from && prev == to) return true;
            }
        }
        return false;
    };

    // The final endpoint may only be held after all relevant constraints have
    // finished using it.
    const int final_goal = goals.back().first;
    int earliest_holding = 0;
    if (!skip_holding) {
        for (int i = 0; i < num_constraints; i++) {
            const auto& path = cons_paths[i];
            if (path.empty()) continue;
            if (constraint_tail[i] == final_goal) {
                earliest_holding = max(earliest_holding, horizon);
                continue;
            }
            int limit = min(constraint_last_change[i], min((int)path.size(), horizon) - 1);
            for (int t = limit; t >= 0; t--)
                if (path[t] == final_goal) {
                    earliest_holding = max(earliest_holding, t + 1);
                    break;
                }
        }
        if (use_old_paths) {
            for (const auto& path : old_paths) {
                int limit = min(min((int)path.size(), horizon), start_time + 301);
                for (int t = limit - 1; t >= 0; t--)
                    if (path[t] == final_goal) {
                        earliest_holding = max(earliest_holding, t + 1);
                        break;
                    }
            }
        }
    }

    // A higher-priority agent permanently holding the final goal makes this
    // branch infeasible. Old paths do not qualify because PBS may move them.
    if (start_loc != final_goal) {
        for (const auto& path : cons_paths)
            if (!path.empty() && path.back() == final_goal) {
                reset_scratch();
                return {};
            }
    }

    struct SIPPNode {
        int loc;
        int g;
        int h;
        int timestep;
        int goal_id;
        int interval;
        uint64_t tie_breaker;
        SIPPNode* parent;

        int f() const { return g + h; }
    };
    struct CompareSIPPNode {
        bool operator()(const SIPPNode* a, const SIPPNode* b) const {
            if (a->f() != b->f()) return a->f() > b->f();
            if (a->g != b->g) return a->g < b->g;
            return a->tie_breaker > b->tie_breaker;
        }
    };
    struct SIPPKeyHash {
        size_t operator()(uint64_t key) const { return key * 2654435761ULL; }
    };

    boost::heap::fibonacci_heap<SIPPNode*, boost::heap::compare<CompareSIPPNode>> open;
    unordered_map<uint64_t, int, SIPPKeyHash> best_arrival;
    vector<SIPPNode*> nodes;

    auto state_key = [](int loc, int interval, int goal_id) -> uint64_t {
        return ((uint64_t)loc << 32) |
               ((uint64_t)(interval & 0xffff) << 16) |
               (uint64_t)(goal_id & 0xffff);
    };

    int initial_h = sum_h(start_loc, 0);
    if (initial_h == INT_MAX) {
        reset_scratch();
        return {};
    }

    int start_interval = 0;
    const auto& start_intervals = get_safe_intervals(start_loc);
    if (!start_intervals.empty()) {
        for (int i = 0; i < (int)start_intervals.size(); i++)
            if (start_intervals[i].first <= start_time &&
                start_time < start_intervals[i].second) {
                start_interval = i;
                break;
            }
    }

    SIPPNode* root = new SIPPNode{
        start_loc, 0, initial_h, start_time, 0, start_interval,
        RandomTieBreaker::next(), nullptr};
    open.push(root);
    nodes.push_back(root);
    best_arrival[state_key(start_loc, start_interval, 0)] = 0;

    SIPPNode* solution = nullptr;
    int expanded = 0;
    const int max_expansions = min(500000, map_size * 20 * max(1, num_goals));

    while (!open.empty() && expanded < max_expansions) {
        SIPPNode* current = open.top();
        open.pop();

        int goal_id = current->goal_id;
        if (goal_id < num_goals && current->loc == goals[goal_id].first &&
            current->timestep >= goals[goal_id].second) {
            if (!(goal_id == num_goals - 1 && current->timestep < earliest_holding))
                goal_id++;
        }
        if (goal_id >= num_goals) {
            solution = current;
            break;
        }

        uint64_t current_key = state_key(current->loc, current->interval, goal_id);
        auto current_best = best_arrival.find(current_key);
        if (current_best != best_arrival.end() && current_best->second < current->g) continue;
        expanded++;
        if (current->timestep >= horizon - 1) continue;

        const auto& current_intervals = get_safe_intervals(current->loc);
        int current_interval_end = horizon;
        if (!current_intervals.empty() && current->interval < (int)current_intervals.size())
            current_interval_end = current_intervals[current->interval].second;

        const int directions[4] = {1, -cols, -1, cols};
        for (int direction : directions) {
            int next_loc = current->loc + direction;
            if (next_loc < 0 || next_loc >= map_size || !mapd_map.grid[next_loc]) continue;
            if (abs(next_loc % cols - current->loc % cols) > 1) continue;

            int earliest_arrival = current->timestep + 1;
            const auto& next_intervals = get_safe_intervals(next_loc);

            auto add_successor = [&](int interval, int arrival) {
                int next_goal_id = goal_id;
                if (next_goal_id < num_goals && next_loc == goals[next_goal_id].first &&
                    arrival >= goals[next_goal_id].second) {
                    if (!(next_goal_id == num_goals - 1 && arrival < earliest_holding))
                        next_goal_id++;
                }

                int h = sum_h(next_loc, next_goal_id);
                if (h == INT_MAX) return;
                int g = arrival - start_time;
                uint64_t key = state_key(next_loc, interval, next_goal_id);
                auto previous = best_arrival.find(key);
                if (previous != best_arrival.end() && previous->second <= g) return;

                best_arrival[key] = g;
                SIPPNode* node = new SIPPNode{
                    next_loc, g, h, arrival, next_goal_id, interval,
                    RandomTieBreaker::next(), current};
                open.push(node);
                nodes.push_back(node);
            };

            if (next_intervals.empty()) {
                if (!has_occupied[next_loc] && earliest_arrival < horizon &&
                    !edge_blocked(current->loc, next_loc, earliest_arrival))
                    add_successor(0, earliest_arrival);
                continue;
            }

            for (int interval = 0; interval < (int)next_intervals.size(); interval++) {
                if (next_intervals[interval].second <= earliest_arrival) continue;
                int arrival = max(earliest_arrival, next_intervals[interval].first);
                if (arrival >= horizon || arrival > current_interval_end) continue;

                while (arrival < next_intervals[interval].second &&
                       arrival <= current_interval_end && arrival < horizon &&
                       edge_blocked(current->loc, next_loc, arrival))
                    arrival++;

                if (arrival >= next_intervals[interval].second || arrival >= horizon ||
                    arrival > current_interval_end)
                    continue;
                add_successor(interval, arrival);
            }
        }

        // Hard-constraint intervals are non-contiguous, so this successor is
        // normally absent; keep it for completeness if adjacent intervals occur.
        if (!current_intervals.empty() &&
            current->interval + 1 < (int)current_intervals.size()) {
            int next_interval = current->interval + 1;
            if (current_intervals[next_interval].first ==
                current_intervals[current->interval].second) {
                int arrival = current_intervals[next_interval].first;
                if (arrival < horizon) {
                    int next_goal_id = goal_id;
                    if (next_goal_id < num_goals && current->loc == goals[next_goal_id].first &&
                        arrival >= goals[next_goal_id].second) {
                        if (!(next_goal_id == num_goals - 1 && arrival < earliest_holding))
                            next_goal_id++;
                    }
                    int h = sum_h(current->loc, next_goal_id);
                    int g = arrival - start_time;
                    uint64_t key = state_key(current->loc, next_interval, next_goal_id);
                    auto previous = best_arrival.find(key);
                    if (h != INT_MAX &&
                        (previous == best_arrival.end() || previous->second > g)) {
                        best_arrival[key] = g;
                        SIPPNode* node = new SIPPNode{
                            current->loc, g, h, arrival, next_goal_id,
                            next_interval, RandomTieBreaker::next(), current};
                        open.push(node);
                        nodes.push_back(node);
                    }
                }
            }
        }
    }

    vector<int> result;
    if (solution) {
        vector<SIPPNode*> reversed_nodes;
        for (SIPPNode* node = solution; node != nullptr; node = node->parent)
            reversed_nodes.push_back(node);
        reverse(reversed_nodes.begin(), reversed_nodes.end());

        vector<int> locations;
        for (int i = 0; i < (int)reversed_nodes.size(); i++) {
            if (i == 0) {
                locations.push_back(reversed_nodes[i]->loc);
                continue;
            }
            for (int t = reversed_nodes[i - 1]->timestep + 1;
                 t < reversed_nodes[i]->timestep; t++)
                locations.push_back(reversed_nodes[i - 1]->loc);
            locations.push_back(reversed_nodes[i]->loc);
        }

        result.resize(max_t);
        for (int t = 0; t < start_time && t < max_t; t++) result[t] = start_loc;
        for (int i = 0; i < (int)locations.size(); i++) {
            int t = start_time + i;
            if (t < max_t) result[t] = locations[i];
        }
        int final_loc = locations.back();
        for (int t = start_time + (int)locations.size(); t < max_t; t++)
            result[t] = final_loc;
    }

    for (SIPPNode* node : nodes) delete node;
    reset_scratch();
    return result;
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
    uint64_t tie_breaker;

    struct compare_node {
        bool operator()(const WStateTimeAStarNode* n1, const WStateTimeAStarNode* n2) const {
            if (n1->getFVal() != n2->getFVal())
                return n1->getFVal() > n2->getFVal();
            if (n1->g_val != n2->g_val) return n1->g_val < n2->g_val;
            return n1->tie_breaker > n2->tie_breaker;
        }
    };
    struct secondary_compare_node {
        bool operator()(const WStateTimeAStarNode* n1, const WStateTimeAStarNode* n2) const {
            if (n1->conflicts == n2->conflicts) {
                if (n1->goal_id == n2->goal_id) {
                    if (n1->g_val != n2->g_val)
                        return n1->g_val < n2->g_val;
                    return n1->tie_breaker > n2->tie_breaker;
                }
                return n1->goal_id < n2->goal_id;
            }
            return n1->conflicts > n2->conflicts;
        }
    };
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::compare_node> >::handle_type open_handle;
    fibonacci_heap<WStateTimeAStarNode*, compare<WStateTimeAStarNode::secondary_compare_node> >::handle_type focal_handle;

    WStateTimeAStarNode() : g_val(0), h_val(0), parent(nullptr), conflicts(0), depth(0), in_openlist(false), visit_goal_time(0), goal_id(0), vis_goal(false), tie_breaker(RandomTieBreaker::next()) {}
    WStateTimeAStarNode(const WState& state, double g_val, double h_val, WStateTimeAStarNode* parent, int conflicts) :
        state(state), g_val(g_val), h_val(h_val), parent(parent), conflicts(conflicts), in_openlist(false), tie_breaker(RandomTieBreaker::next()) {
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
        uint64_t tie_breaker;
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
            return a->tie_breaker > b->tie_breaker;
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
        SN* nd = new SN{loc, iv, gi, r, h,
                        RandomTieBreaker::next(), nullptr};
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
    uint64_t tie_breaker;
    void clear() { conflicts.clear(); priorities.clear(); }
    WPBSNode() : parent(nullptr), g_val(0), h_val(0), f_val(0), depth(0), makespan(0),
        num_of_collisions(0), earliest_collision(INT_MAX), time_expanded(0),
        time_generated(0), tie_breaker(RandomTieBreaker::next()) {}
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
                const bool first_is_better =
                    n[0]->f_val < n[1]->f_val ||
                    (n[0]->f_val == n[1]->f_val &&
                     (n[0]->num_of_collisions < n[1]->num_of_collisions ||
                      (n[0]->num_of_collisions == n[1]->num_of_collisions &&
                       n[0]->tie_breaker < n[1]->tie_breaker)));
                if (first_is_better)
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
                       bool use_mlsipp) {
    int n = (int)start_locs.size();
    out_paths.assign(n, {});

    WGraph G;
    G.mp = &mp;
    G.rows = mp.row;
    G.cols = mp.col;

    // The configured single-agent method selects the wPBS low-level solver.
    // wPBS-MLA* and wPBS-MLSIPP otherwise share this implementation.
    WStateTimeAStar planner_astar;
    WSippSearch     planner_sipp;
    WSingleAgentSolver& planner = use_mlsipp
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
    wPBSPlanner(*this).solve();
    for (int i = 0; i < (int)agents.size(); i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
            agents[i].current_goal_index = 0;
        }
    }
}

// ============================================================
// Native windowed solve orchestrator (framework-native)
//   Builds the per-solve inputs from the simulator's live state, runs the
//   framework-native windowed PBS (native_wpbs_solve, backed by mapd_map for the
//   grid + heuristics), and writes committed paths back into path_table_ /
//   agents.path. Dispersal uses the shared PAIRWISE_TASK_THEN_HOME endpoint policy:
//   task endpoints first, skip own/already-assigned, then home fallback.
// ============================================================
void Simulation::wpbs_windowed_solve_impl() {
    int num_ag = (int)agents.size();
    int max_t = (int)maxtime;

    // starts = current agent locations
    std::vector<int> start_locs(num_ag);
    for (int i = 0; i < num_ag; i++) start_locs[i] = (int)agents[i].loc;

    // --- Build task-goal part per agent (identical to build_goal_sequences)
    //     with ABSOLUTE release times and the configured task limit. ---
    std::vector<std::vector<std::pair<int,int>>> goal_seqs(num_ag);
    const int task_truncated_size = config.task_sequence_limit;
    for (int i = 0; i < num_ag; i++) {
        int current_task_count = 0;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= task_truncated_size) break;
            Task& task = all_tasks[tid];
            int start_goal = (tid == agents[i].current_task)
                ? agents[i].current_goal_index : 0;
            for (int g = start_goal; g < (int)task.goals.size(); g++)
                goal_seqs[i].push_back({task.goals[g], g == 0 ? task.release_time : 0});
            current_task_count++;
        }
    }

    if (config.dummy_path) {
        vector<int> assigned_dummies(num_ag, -1);
        // Non-free agents first, then free agents (matches reference ordering).
        for (int i = 0; i < num_ag; i++) {
            if (goal_seqs[i].empty()) continue;
            int last_loc = goal_seqs[i].back().first;
            int dummy = choose_dummy_endpoint(
                i, last_loc, assigned_dummies);
            goal_seqs[i].push_back({dummy, 0});
            assigned_dummies[i] = dummy;
        }
        for (int i = 0; i < num_ag; i++) {
            if (!goal_seqs[i].empty()) continue;
            int last_loc = start_locs[i];
            int dummy = choose_dummy_endpoint(
                i, last_loc, assigned_dummies);
            goal_seqs[i].push_back({dummy, 0});
            assigned_dummies[i] = dummy;
        }
    } else {
        for (int i = 0; i < num_ag; i++)
            if (goal_seqs[i].empty())
                goal_seqs[i].push_back({start_locs[i], 0});
    }

    // --- Run the native windowed integrated solve (mapd_map-backed). ---
    std::vector<std::vector<int>> out_paths;
    int window = config.wpbs_replan_window;       // reference planning window
    int time_limit_ms = 30000;                    // generous per-window budget
    native_wpbs_solve(mapd_map, start_locs, goal_seqs,
                      (int)cur_time_, window, time_limit_ms, out_paths,
                      config.single_agent == SA_MLSIPP_SEQUENCE);

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

// ============================================================================
//  Section 7 — System update                             (Algorithm 1, line 13)
//
//  Agents execute their committed paths.  Nothing can change between two
//  events, so instead of stepping one timestep at a time the clock jumps
//  straight to the next event (an agent reaching its next task goal, a task
//  release, or a wPBS window boundary) — the same state as stepping through,
//  without the empty iterations.
// ============================================================================

void Simulation::advance_time() {
    if (config.assign_trigger == AT_ONCE) {
        // Offline TA has already computed every complete agent path and every
        // task completion time. Finalize that schedule atomically instead of
        // replaying it through the online event loop.
        unsigned int final_time = 0;
        for (Task& task : all_tasks) {
            if (task.completion_time < 0) {
                cerr << "advance_time: offline planning left task " << task.id
                     << " incomplete" << endl;
                exit(1);
            }
            final_time = max(
                final_time, (unsigned int)task.completion_time);
            task.status = INT_MAX;
        }

        for (Agent& agent : agents) {
            agent.task_sequence.clear();
            agent.current_task = -1;
            agent.current_goal_index = 0;
            agent.status = AG_FREE;
            agent.finish_time = final_time;
            if (!agent.path.empty())
                agent.loc = agent.path[min(final_time,
                    (unsigned int)agent.path.size() - 1)];
        }

        open_tasks_.clear();
        cur_time_ = max(final_time, (unsigned int)max(t_task, 0)) + 1;
        return;
    }
    if (config.assign_trigger == AT_ON_FREE_WAITS) {
        advance_time_tp();
        return;
    }
    if (config.assign_trigger == AT_EVERY_TIMESTEP ||
        config.assign_trigger == AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE) {
        // CENTRAL must observe every ordered task-goal transition. CENTRAL
        // assigns every step; CENTRAL-fixed merely advances through quiet steps.
        if (cur_time_ + 1 >= maxtime) {
            cerr << "advance_time: CENTRAL exceeded maxtime="
                 << maxtime << endl;
            return;
        }
        cur_time_++;
        for (Agent& agent : agents)
            agent.loc = agent.path[cur_time_];
        process_events();
        return;
    }

    unsigned int next_ts = maxtime;

    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].task_sequence.empty()) continue;
        int task_id = agents[i].task_sequence.front();
        Task& task = all_tasks[task_id];
        if (task.goals.empty()) continue;

        int goal_index = (agents[i].current_task == task_id)
            ? agents[i].current_goal_index : 0;
        if (goal_index < 0 || goal_index >= (int)task.goals.size()) continue;

        int target_loc = task.goals[goal_index];
        int min_time = goal_index == 0 ? task.release_time : 0;

        for (unsigned int t = cur_time_ + 1; t < maxtime; t++)
            if ((int)agents[i].path[t] == target_loc && (int)t >= min_time) {
                if (t < next_ts) next_ts = t;
                break;
            }
    }
    clamp_next_ts_to_task_release(next_ts);
    // A task deferred because of an old dummy endpoint stays in open_tasks_.
    // Re-enter assignment on the next timestep, after the current plan has
    // selected replacement dummy endpoints that avoid all open-task goals.
    if (config.assign_trigger ==
            AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE &&
        new_or_deferred_task_event_)
        next_ts = min(next_ts, cur_time_ + 1);
    if (config.mapf == MAPF_wPBS) {
        unsigned int window_end = cur_time_ + config.wpbs_replan_window;
        if (window_end < next_ts) next_ts = window_end;
    }

    if (next_ts >= maxtime) {
        bool has_work = !all_task_sequences_empty();
        if (has_work || !open_tasks_.empty()) next_ts = cur_time_ + 1;
        else return;                                  // nothing left to do
    }
    if (next_ts <= cur_time_) next_ts = cur_time_ + 1;
    if (next_ts >= maxtime) {
        cerr << "advance_time: exceeded maxtime=" << maxtime << endl;
        for (int i = 0; i < (int)agents.size(); i++)
            if (!agents[i].task_sequence.empty())
                cerr << "  pending agent=" << i
                     << " task=" << agents[i].task_sequence.front()
                     << " status=" << agents[i].status
                     << " goal_index=" << agents[i].current_goal_index
                     << " target="
                     << all_tasks[agents[i].task_sequence.front()].goals[
                            agents[i].current_goal_index]
                     << " finish=" << agents[i].finish_time
                     << " loc=" << agents[i].loc
                     << " path_tail=" << agents[i].path.back() << endl;
        throw runtime_error("advance_time exceeded the planning horizon");
    }

    cur_time_ = next_ts;
    for (auto& ag : agents)
        if (cur_time_ < maxtime) ag.loc = ag.path[cur_time_];

    process_events();
}

// TP assigns at most one task per framework iteration. Before moving time
// forward, repeatedly return control to task_assignment() until every agent at
// the current timestep has either received a task or been given a one-step
// wait. This preserves the original token-passing agent ordering.
void Simulation::advance_time_tp() {
    for (const Agent& agent : agents)
        if (agent.finish_time <= cur_time_)
            return;

    unsigned int next_ts = maxtime;
    for (const Agent& agent : agents)
        if (agent.finish_time > cur_time_ && agent.finish_time < next_ts)
            next_ts = agent.finish_time;

    clamp_next_ts_to_task_release(next_ts);
    if (next_ts >= maxtime) {
        if (!open_tasks_.empty() || (int)cur_time_ <= t_task)
            next_ts = cur_time_ + 1;
        else
            return;
    }
    if (next_ts <= cur_time_) next_ts = cur_time_ + 1;
    if (next_ts >= maxtime) {
        cerr << "advance_time_tp: exceeded maxtime=" << maxtime << endl;
        return;
    }

    cur_time_ = next_ts;
    for (Agent& agent : agents)
        agent.loc = agent.path[cur_time_];
}

// Never jump over a timestep that releases tasks.
void Simulation::clamp_next_ts_to_task_release(unsigned int& next_ts) const {
    for (unsigned int t = cur_time_ + 1; t < maxtime && t <= next_ts; t++)
        if (!task_indices_by_time[t].empty()) {
            if (t < next_ts) next_ts = t;
            break;
        }
}

// --- event detection --------------------------------------------------------
// Bookkeeping only: read the executed paths to advance each agent's task state
// machine, then tell the dispatchers whether to re-assign / re-plan.
void Simulation::process_events() {
    new_available_agent_ = false;

    if (config.assign_method == AM_CENTRAL_HUNGARIAN) {
        // CENTRAL stores only the current task and its next unvisited goal in
        // Agent. Advance that index at pickup and every intermediate goal;
        // complete the task only after its final ordered goal.
        for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++) {
            Agent& agent = agents[agent_id];
            Task* task = central_current_task(agent_id);

            if (agent.status == AG_MOVING_TO_PICKUP &&
                agent.finish_time <= cur_time_) {
                if (task && !task->goals.empty() &&
                    agent.loc == task->goals.front()) {
                    task->status = agent_id;
                    task->ag_arrive_start = (int)cur_time_;
                    agent.status = AG_CARRYING;
                    agent.current_goal_index = 1;
                } else {
                    if (task) task->status = -1;
                    agent.status = AG_FREE;
                    agent.current_task = -1;
                    agent.current_goal_index = 0;
                    agent.last_endpoint = agent.loc;
                    continue;
                }
            }

            task = central_current_task(agent_id);
            if (agent.status != AG_CARRYING || !task ||
                agent.finish_time > cur_time_)
                continue;

            // Consecutive goals may share a location, so consume every goal
            // already reached at this timestep before requesting a new path.
            while (agent.current_goal_index < (int)task->goals.size() &&
                   agent.loc == task->goals[agent.current_goal_index])
                agent.current_goal_index++;

            if (agent.current_goal_index < (int)task->goals.size()) {
                agent.last_endpoint = task->goals[agent.current_goal_index];
                continue;
            }

            if (task->completion_time < 0)
                task->completion_time = (int)cur_time_;
            task->status = INT_MAX;
            open_tasks_.remove(task);
            agent.status = AG_FREE;
            agent.current_task = -1;
            agent.current_goal_index = 0;
            agent.last_endpoint = agent.loc;
            if (config.assign_trigger ==
                    AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE)
                new_available_agent_ = true;
        }
        return;
    }

    // The event-driven clock may jump across several path steps, so inspect the
    // complete segment executed since paths were last planned.
    for (int i = 0; i < (int)agents.size(); i++) {
        unsigned int scan_from = last_path_planning_time_;
        while (!agents[i].task_sequence.empty()) {
            int task_id = agents[i].task_sequence.front();
            Task& task = all_tasks[task_id];
            if (task.goals.empty()) break;

            if (agents[i].current_task != task_id) {
                agents[i].current_task = task_id;
                agents[i].current_goal_index = 0;
                agents[i].status = AG_MOVING_TO_PICKUP;
            }

            bool task_completed = false;
            for (unsigned int t = scan_from; t <= cur_time_ && t < maxtime; t++) {
                while (agents[i].current_goal_index < (int)task.goals.size() &&
                       (int)agents[i].path[t] == task.goals[agents[i].current_goal_index]) {
                    // Only the first goal (the pickup) is release-time constrained.
                    if (agents[i].current_goal_index == 0 && (int)t < task.release_time)
                        break;

                    if (agents[i].current_goal_index == 0) {
                        task.ag_arrive_start = (int)t;
                        agents[i].status = AG_CARRYING;
                    }
                    agents[i].current_goal_index++;

                    if (agents[i].current_goal_index == (int)task.goals.size()) {
                        task.completion_time = (int)t;
                        task.status = INT_MAX;
                        agents[i].task_sequence.pop_front();
                        agents[i].current_task = -1;
                        agents[i].current_goal_index = 0;

                        if (config.mapf == MAPF_TA_HYBRID_TWO_GROUP) {
                            // A TA-Hybrid task agent becomes free after every
                            // completed task, even when its sequence has a next
                            // task. Group 2 must therefore rematch/replan the
                            // free-agent set before that pickup is pursued.
                            agents[i].status = AG_FREE;
                            if (!agents[i].task_sequence.empty())
                                agents[i].current_task =
                                    agents[i].task_sequence.front();
                            new_available_agent_ = true;
                            task.ag = nullptr;
                            task.delivering = false;
                        } else if (!agents[i].task_sequence.empty()) {
                            agents[i].status = AG_MOVING_TO_PICKUP;
                            agents[i].current_task = agents[i].task_sequence.front();
                        } else {
                            agents[i].status = AG_FREE;
                            // This flag represents an agent that actually became
                            // available, not every completed delivery. If another
                            // queued task exists, the agent remains occupied.
                            new_available_agent_ = true;
                        }
                        scan_from = t;
                        task_completed = true;
                        break;
                    }
                }
                if (task_completed) break;
            }

            if (!task_completed) break;
        }
    }

}

// ============================================================================
//  Section 8 — Reporting
// ============================================================================

bool Simulation::fullCollisionCheck(const string& alg_name) const {
    bool ok = true;
    int collision_count = 0;
    int num_ag = (int)path_table_.size();
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
    int makespan = 0, total_wait = 0, tasks_done = 0;
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

void Simulation::saveOutput(const string& filepath, double runtime_ms,
                            const string& algorithm_name) const {
    ofstream out(filepath);
    if (!out.is_open()) {
        cerr << "Error: cannot open output file: " << filepath << endl;
        return;
    }

    int makespan = 0, total_wait = 0, tasks_done = 0;
    for (auto& t : all_tasks) {
        if (t.completion_time > 0) {
            tasks_done++;
            if (t.completion_time > makespan) makespan = t.completion_time;
            total_wait += (t.completion_time - t.release_time);
        }
    }

    out << "# MAPD Framework Output" << endl;
    out << "# Algorithm: " << algorithm_name << endl;
    out << "# Map: " << mapd_map.raw_row << "x" << mapd_map.raw_col
        << " (" << mapd_map.workpoint_num << " task eps, "
        << mapd_map.num_agents << " agents)" << endl;
    out << "# Tasks: " << all_tasks.size() << endl;
    out << "# Makespan: " << makespan << endl;
    out << "# Sum of Task Waiting Time: " << total_wait << endl;
    out << "# Tasks completed: " << tasks_done << "/" << all_tasks.size() << endl;
    out << "# Runtime: " << runtime_ms << " ms" << endl;
    out << endl;

    out << "# Agent paths (agent_id: loc_t0 loc_t1 ... loc_makespan)" << endl;
    for (int i = 0; i < (int)agents.size(); i++) {
        out << i << ":";
        for (int t = 0; t <= makespan && t < (int)maxtime; t++)
            out << " " << agents[i].path[t];
        out << endl;
    }
    out << endl;

    out << "# Tasks (task_id release_time pickup_loc delivery_loc agent completion_time)" << endl;
    for (auto& t : all_tasks)
        out << t.id << " " << t.release_time << " " << t.pickup_loc
            << " " << t.delivery_loc << " " << t.status
            << " " << t.completion_time << endl;

    out.close();
}

// ============================================================================
//  Anytime improvement (config.anytime_improvement)
//  Not implemented in this build: the LNS-PBS preset sets it to false.
// ============================================================================
int Simulation::realpath_lns_imp(int num_rounds, int /*group_size*/) {
    if (num_rounds > 0)
        cerr << "realpath_lns_imp: anytime improvement not implemented in this build" << endl;
    return 0;
}
