// ============================================================================
//  Unified MAPD framework
//
//  An algorithm is a choice of nine components, set by command-line options
//  or a --config file (configs/table1/*.cfg: TP, TPTS, HBH, CENTRAL,
//  CENTRAL-fixed, TA-Prioritized, TA-Hybrid, LNS-PBS, LNS-wPBS;
//  configs/variants/*.cfg: further combinations). Each component has one
//  dispatcher that switches on its value:
//
//   1  when to call task assignment    --assign_trigger      should_assign()
//        ON_FREE_WAITS / EVERY_TIMESTEP / ONCE /
//        ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE
//   2  task assignment                 --assign_method       task_assignment()
//        DECOUPLED_GREEDY (TP) / DECOUPLED_GREEDY_SWAPS (TPTS) /
//        CENTRALIZED_GREEDY (HBH) / CENTRAL_HUNGARIAN / REPEATED_HUNGARIAN /
//        REPEATED_HUNGARIAN_LNS / LKH3_TSP / LKH3_TSP_REASSIGN
//   3  when to plan paths              --replan_on           should_replan()
//        set of AFTER_ASSIGNMENT / EVERY_TIMESTEP / GOAL_REACHED / PERIODIC;
//        NONE = TP, TPTS and HBH plan inside the assignment
//   4  which agents to plan            --agent_selection     select_agents()
//        ALL / CHANGED (the others keep their committed paths)
//   5  high-level planner              --mapf                plan_paths()
//        PP / PP_TASK_SEQUENCE / CBS / PBS / wPBS / TA_HYBRID_TWO_GROUP
//   5.1 window                         --wpbs_replan_window  wPBS = PBS that
//        plans w steps ahead; also the PERIODIC replan interval
//   6  low-level planner               --single_agent        plan_single_agent()
//        STA / MLA / MLSIPP / SIPP2; CBS (cbs.cpp), TA-Hybrid and
//        PP_TASK_SEQUENCE (SEQ_STA) use their own searches
//   7  task endpoint                   --endpoint_strategy   choose_dummy_endpoint()
//        WAIT_OR_NEAREST_SAFE / WAIT_OR_NEAREST_FREE_NONTASK /
//        NEAREST_AVAILABLE / NEAREST_WITH_STRICT_EXCLUSIONS /
//        PAIRWISE_TASK_THEN_HOME / RETURN_TO_HOME;
//        --dummy_path 0/1                                    build_goal_sequences()
//   8  task setup                      --mode                release_tasks()
//        ONLINE / SEMI_ONLINE / OFFLINE
//   9  task type                       --task_type           checked in init();
//        MAPD / MG_MAPD                                      every module reads
//                                                            Task::goals
//
//  validate_config() (src/config_validate.cpp) rejects the combinations that
//  are not implemented before the run starts. advance_time() jumps the clock
//  as far as the planners allow; process_events() advances the task states.
//
//  Tuning options: task_sequence_limit (tasks per PBS/wPBS plan), lns_time,
//  semi_online_lookahead_batches, tie_breaking, the CBS options
//  (ecbs_focal_weight, cbs_conflict_selection, cbs_bypass,
//  cbs_target_reasoning, cbs_rectangle_reasoning, expansion limits) and the
//  runtime limits (whole run, default 1000 s; each planning cycle, 600 s).
//
//  Layout follows the unified pseudocode:
//      Section 0   Initialisation                       (Algorithm 1, lines 1-6)
//      Section 1   Main loop                            (Algorithm 1, lines 7-13)
//      Section 2   Computation dispatchers              (Algorithm 1, line 12)
//      Section 3   Task assignment  TP / TPTS / HBH / CENTRAL Hungarian /
//                                   LKH3 TA / repeated Hungarian / LNS
//      Section 4   Task sequences -> goal sequences, agent selection,
//                  dummy-endpoint selection
//      Section 5   MAPF             CBS / TA-Hybrid / PP /
//                                   PBS (wPBS = PBS with a planning window)
//      Section 6   Single agent     STA* / MLA* / SIPP / MLSIPP
//      Section 7   System update    (Algorithm 1, line 13)
//      Section 8   Reporting
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

// ============================================================================
//  Section 0 — Initialisation                            (Algorithm 1, 1-6)
// ============================================================================

void Simulation::init(const string& map_file, const string& task_file,
                      const MAPDConfig& cfg, const string& tour_file) {
    config = cfg;
    validate_config(config, !tour_file.empty());
    RandomTieBreaker::seed(config.seed);
    TieBreakPolicy::set_reference(config.tie_breaking == TIE_BREAK_REFERENCE);
    tour_file_ = tour_file;

    // --- instance ---
    mapd_map.load(map_file);
    maxtime = mapd_map.maxtime;
    all_tasks = load_tasks(task_file, mapd_map.endpoints);
    if (config.task_type == TASK_MAPD)
        for (const Task& task : all_tasks)
            if (task.goals.size() != 2)
                throw invalid_argument("task_type MAPD needs a pickup and a delivery goal "
                                       "per task; task " + to_string(task.id) + " has " +
                                       to_string(task.goals.size()) + " goals (use MG_MAPD)");

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
    planned_goals_.assign(agents.size(), {});
    has_plan_.assign(agents.size(), false);
    segment_goal_.assign(agents.size(), -1);
    last_goal_time_.assign(agents.size(), 0);
    rested_at_.assign(agents.size(), -1);
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
        RuntimeDeadline::check("simulation main loop");
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
    if (revealed_new_task) new_or_deferred_task_event_ = true;
}

// ============================================================================
//  Section 2 — Computation                               (Algorithm 1, line 12)
//
//  The two halves of the framework: decide WHO does WHAT (task assignment),
//  then decide HOW they get there (path planning).  Each half has its own
//  trigger so an algorithm can re-plan without re-assigning.
// ============================================================================

void Simulation::task_assignment_and_path_planning() {
    RuntimeDeadline::start_pathfinding(
        config.pathfinding_runtime_limit_seconds);
    RuntimeDeadline::check("task assignment and path planning");
    const bool assignment_triggered = should_assign();
    if (assignment_triggered) {
        task_assignment();      // assignment half
        RuntimeDeadline::check("task assignment");
    }

    if (should_replan(assignment_triggered)) {
        path_planning(assignment_triggered); // path-planning half
        RuntimeDeadline::check("path planning");
        last_path_planning_time_ = cur_time_;
    }
    RuntimeDeadline::stop_pathfinding();
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
        return true;
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
        throw logic_error("should_assign: unsupported assign_trigger");
    }
}

// --- AssignTask() : switch on assign_method ---
void Simulation::task_assignment() {
    // TP, TPTS and HBH plan while assigning unless planning is a separate step.
    const bool plans_separately = config.replan_on != 0;
    switch (config.assign_method) {
    case AM_DECOUPLED_GREEDY:
        if (plans_separately) assign_batch_greedy();
        else assign_decoupled_greedy_step();
        break;
    case AM_DECOUPLED_GREEDY_SWAPS:
        if (plans_separately) assign_batch_greedy();
        else assign_tpts_step();
        break;
    case AM_CENTRALIZED_GREEDY:
        if (plans_separately) assign_batch_greedy();
        else assign_hbh_mla();
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
        throw logic_error("task_assignment: unsupported assign_method");
    }
}

// --- ShouldReplan() : the events in config.replan_on ---
bool Simulation::should_replan(bool assignment_triggered) const {
    const int on = config.replan_on;
    if (on & REPLAN_EVERY_TIMESTEP) return true;
    if ((on & REPLAN_AFTER_ASSIGNMENT) && assignment_triggered) return true;
    if ((on & REPLAN_GOAL_REACHED) && agent_needs_next_segment()) return true;
    if ((on & REPLAN_PERIODIC) && !all_task_sequences_empty())
        return (int)cur_time_ - (int)last_path_planning_time_ >= config.wpbs_replan_window;
    return false;
}

// An agent reached a goal and has goals left: a carrying agent has just picked
// up, or reached a later goal of, a task with further ordered goals, or an
// agent reached the end of its CBS segment with tasks left. Segment planners
// (CBS, TA-Hybrid) plan the next leg.
bool Simulation::agent_needs_next_segment() const {
    for (int i = 0; i < (int)agents.size(); i++)
        if (segment_goal_[i] >= 0 && (int)agents[i].loc == segment_goal_[i] &&
            !agents[i].task_sequence.empty())
            return true;
    for (const Agent& agent : agents)
        if (agent.status == AG_CARRYING &&
            agent.current_task >= 0 &&
            agent.current_task < (int)all_tasks.size() &&
            agent.current_goal_index >= 1 &&
            agent.current_goal_index <
                (int)all_tasks[agent.current_task].goals.size() &&
            (all_tasks[agent.current_task].ag_arrive_start == (int)cur_time_ ||
             agent.finish_time <= cur_time_))
            return true;
    return false;
}

// --- Path_Planning() : switch on mapf ---
// A planner that replans only the changed agents (select_agents) records them
// in last_planned_. If their new paths still clash with a path that was kept,
// every agent is replanned.
void Simulation::path_planning(bool assignment_triggered) {
    last_planned_.clear();
    plan_paths(assignment_triggered);
    if (!last_planned_.empty() && kept_paths_conflict()) {
        select_all_ = true;
        plan_paths(assignment_triggered);
        select_all_ = false;
    }
}

// Whether a replanned agent now collides with an agent whose path was kept.
bool Simulation::kept_paths_conflict() const {
    const int num_ag = (int)agents.size();
    int settle = (int)cur_time_;   // after this every agent stands still
    for (int i = 0; i < num_ag; i++)
        for (int t = (int)maxtime - 1; t > settle; t--)
            if (path_table_[i][t] != path_table_[i][t - 1]) { settle = t; break; }
    for (int i = 0; i < num_ag; i++) {
        if (!last_planned_[i]) continue;
        for (int j = 0; j < num_ag; j++) {
            if (last_planned_[j]) continue;
            for (int t = (int)cur_time_ + 1; t <= settle && t < (int)maxtime; t++) {
                if (path_table_[i][t] == path_table_[j][t]) return true;
                if (path_table_[i][t] == path_table_[j][t - 1] &&
                    path_table_[j][t] == path_table_[i][t - 1]) return true;
            }
        }
    }
    return false;
}

void Simulation::plan_paths(bool assignment_triggered) {
    switch (config.mapf) {
    case MAPF_PP_PER_TASK:
        path_planning_pp_per_task();
        break;
    case MAPF_PP_TASK_SEQUENCE:
        path_planning_pp_task_sequence();
        break;
    case MAPF_CBS:
        // CENTRAL assigns each agent one endpoint (a pickup or parking); the
        // other methods assign task sequences, planned one goal at a time.
        if (config.assign_method == AM_CENTRAL_HUNGARIAN)
            path_planning_ecbs(assignment_triggered);
        else
            path_planning_cbs_segments();
        break;
    case MAPF_PBS:
    case MAPF_wPBS:
        path_planning_pbs();
        break;
    case MAPF_TA_HYBRID_TWO_GROUP:
        path_planning_ta_hybrid(assignment_triggered);
        break;
    default:
        throw logic_error("path_planning: unsupported mapf");
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
    if (move_to_rest_endpoint(agent, make_token_endpoint_request(agent.id, agent.loc, false)))
        return true;

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

    const DummyEndpointRequest request = make_token_endpoint_request(agent.id, agent.loc, true);
    // Validate a stationary endpoint too. During a TPTS steal the replacement
    // path may hide the displaced agent's old reservation, so installing an
    // unchecked wait at that agent's forced home can create a real collision.
    // An unreachable rest endpoint falls back to the safe relocation, or to
    // waiting where the agent is when it need not move.
    auto rest_at = [&](int endpoint) {
        if (endpoint < 0 || !plan_path2_to_endpoint(agent, endpoint, false, false))
            return false;
        if (endpoint == agent.loc)
            agent.finish_time = cur_time_ + 1;
        rested_at_[agent.id] = endpoint;
        return true;
    };
    const int endpoint = rest_endpoint(request);
    if (rest_at(endpoint)) return true;
    if (endpoint != agent.loc) {
        const int fallback = request.relocation_required ? endpoint_wait_or_relocate(request)
                                                         : agent.loc;
        if (fallback != endpoint && rest_at(fallback)) return true;
    }

    path_table_ = saved_paths;
    // Restore in place so Agent references held by enclosing recursive calls
    // remain valid.
    for (int i = 0; i < (int)agents.size(); i++)
        agents[i] = saved_agents[i];
    return false;
}

// --- TP / TPTS / HBH when planning is a separate step -----------------------
// The decoupled rules assign only; the configured MAPF method plans afterwards.
// Without paths, an agent's arrival is estimated by its endpoint distance (or
// the task's release, if later), as repeated Hungarian does. TP's rule still applies: a task whose goal is
// another agent's parking endpoint waits until that agent has moved on.
//   TP    each free agent takes the nearest available task;
//   TPTS  as TP, and it may take over a task whose assignee has not reached
//         the pickup when it would get there sooner;
//   HBH   the globally nearest free (agent, task) pairs are matched first.
void Simulation::assign_batch_greedy() {
    unordered_map<int, int> parked;   // endpoint -> agent whose plan ends there
    for (int i = 0; i < (int)agents.size(); i++)
        parked[(int)path_table_[i][maxtime - 1]] = i;
    auto available = [&](const Task* task, int agent_id) {
        for (int goal : task->goals) {
            auto holder = parked.find(goal);
            if (holder != parked.end() && holder->second != agent_id) {
                new_or_deferred_task_event_ = true;   // retry once it moves
                return false;
            }
        }
        return true;
    };
    // Time until the agent could pick the task up: travel, or the release.
    auto distance = [&](int agent_id, const Task* task) {
        return max(mapd_map.endpoints[task->pickup].h_val[agents[agent_id].loc],
                   task->release_time - (int)cur_time_);
    };
    auto assign = [&](int agent_id, Task* task) {
        agents[agent_id].task_sequence.push_back(task->id);
        task->status = agent_id;
        open_tasks_.remove(task);
    };
    auto is_free = [&](int agent_id) {
        return agents[agent_id].task_sequence.empty() &&
               agents[agent_id].status != AG_CARRYING;
    };

    if (config.assign_method == AM_CENTRALIZED_GREEDY) {
        vector<tuple<int, int, Task*>> pairs;   // (distance, agent, task)
        for (int i = 0; i < (int)agents.size(); i++)
            if (is_free(i))
                for (Task* task : open_tasks_)
                    if (task->status == -1 && available(task, i))
                        pairs.emplace_back(distance(i, task), i, task);
        sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) {
            if (get<0>(a) != get<0>(b)) return get<0>(a) < get<0>(b);
            if (get<1>(a) != get<1>(b)) return get<1>(a) < get<1>(b);
            return get<2>(a)->id < get<2>(b)->id;
        });
        for (const auto& pair : pairs)
            if (is_free(get<1>(pair)) && get<2>(pair)->status == -1)
                assign(get<1>(pair), get<2>(pair));
        return;
    }

    const bool swaps = config.assign_method == AM_DECOUPLED_GREEDY_SWAPS;
    deque<int> queue;
    for (int i = 0; i < (int)agents.size(); i++)
        if (is_free(i)) queue.push_back(i);
    while (!queue.empty()) {
        RuntimeDeadline::check_periodic(queue.size(), "greedy task assignment");
        int agent_id = queue.front();
        queue.pop_front();
        Task* best = nullptr;
        int best_distance = INT_MAX;
        for (int i = 0; i < (int)all_tasks.size(); i++) {
            Task* task = &all_tasks[i];
            bool open = task->status == -1 &&
                find(open_tasks_.begin(), open_tasks_.end(), task) != open_tasks_.end();
            // TPTS may take over an assigned task its owner has not picked up.
            int owner = task->status;
            bool stealable = swaps && owner >= 0 && owner != INT_MAX &&
                agents[owner].status != AG_CARRYING &&
                !agents[owner].task_sequence.empty() &&
                agents[owner].task_sequence.front() == task->id &&
                distance(agent_id, task) < distance(owner, task);
            if (!open && !stealable) continue;
            if (!available(task, agent_id)) continue;
            int d = distance(agent_id, task);
            if (d < best_distance) { best = task; best_distance = d; }
        }
        if (!best) continue;
        int owner = best->status;
        if (owner >= 0) {   // a TPTS take-over frees the previous assignee
            agents[owner].task_sequence.pop_front();
            agents[owner].status = AG_FREE;
            agents[owner].current_task = -1;
            agents[owner].current_goal_index = 0;
            queue.push_back(owner);
        }
        assign(agent_id, best);
    }
}

// --- HBH: centralized h-value greedy assignment ----------------------------
// Build all available-agent/open-task pairs, scan them by nondecreasing
// distance to the first task goal, and commit a pair only when the configured
// low-level planner finds a collision-free path through every ordered goal.
void Simulation::assign_hbh_mla() {
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

        vector<int> path = plan_single_agent({agent.id, (int)agent.loc, (int)cur_time_,
                                              task_groups, constraints, no_old_paths, false});
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
        rested_at_[agent.id] = -1;
        open_tasks_.remove(&task);
        assigned_agents.insert(agent.id);
        assigned_tasks.insert(task.id);
    }

    // Paper Algorithm 2: an unassigned available agent waits unless its
    // current location is needed by an open task; in that case it relocates to
    // the closest free non-task endpoint selected by the unified policy.
    for (int agent_id : available_agents) {
        if (assigned_agents.count(agent_id)) continue;
        Agent& agent = agents[agent_id];
        if (move_to_rest_endpoint(agent, make_hbh_endpoint_request(agent_id, agent.loc))) {
            agent.last_endpoint = rested_at_[agent_id];
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
        if (task->status != -1 || task->release_time > (int)cur_time_) continue;
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
        if (TieBreakPolicy::reference()) return lhs.f > rhs.f;
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
        RuntimeDeadline::check_periodic(
            (uint64_t)expanded, "CENTRAL assignment-cost A*");
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
        if (task->status != -1 || task->goals.empty() ||
            task->release_time > (int)cur_time_)
            continue;

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
            int endpoint_location = choose_dummy_endpoint(config.endpoint_strategy,
                make_central_endpoint_request(
                    agent_id, agents[agent_id].loc, reserved_endpoints));
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
// zero; later calls only reassign unfinished tasks. Sequences are visited in
// tour order, as the reference implementation stores them.
void Simulation::assign_ta_hybrid() {
    const int agent_count = (int)agents.size();
    auto distance = [&](int from, int to) {
        int endpoint = mapd_map.ep_index(to);
        return endpoint < 0 ? INT_MAX :
            mapd_map.endpoints[endpoint].h_val[from];
    };
    auto update_makespan_bound = [&]() {
        ta_makespan_bound_ = 0;
        for (int agent_id = 0; agent_id < agent_count; agent_id++)
            ta_makespan_bound_ = max(ta_makespan_bound_,
                ta_sequence_cost(agent_id, (int)cur_time_));
    };

    // Materialize the tour once, when the offline task set is revealed.
    if (!open_tasks_.empty()) {
        TATourAssignment tour = read_ta_tour(
            tour_file_, agent_count, (int)all_tasks.size());
        ta_sequence_rank_.assign(agent_count, 0);
        for (int rank = 0; rank < (int)tour.agent_order.size(); rank++)
            ta_sequence_rank_[tour.agent_order[rank]] = rank;
        vector<bool> in_tour(all_tasks.size(), false);
        for (int owner = 0; owner < agent_count; owner++)
            for (int task_id : tour.tasks_by_agent[owner]) {
                agents[owner].task_sequence.push_back(task_id);
                all_tasks[task_id].seq_id = owner;
                all_tasks[task_id].status = owner;
                in_tour[task_id] = true;
            }
        for (Task* task : open_tasks_)
            if (!in_tour[task->id])
                throw runtime_error(
                    "TA-Hybrid tour does not assign known task " +
                    to_string(task->id));
        open_tasks_.clear();
        update_makespan_bound();
    }

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

    // Reference AssignNewTask(): an agent with an empty sequence can steal
    // the last task, if not yet picked up, of the longest sequence. When it
    // runs every timestep, it runs before that timestep's deliveries, so an
    // agent whose last task was delivered just now steals only one step later.
    const bool delivered_now_waits =
        config.assign_trigger == AT_EVERY_TIMESTEP && cur_time_ > 0;
    const vector<int> order = ta_agents_in_sequence_order();
    bool reassigned = false;
    for (int receiver : order) {
        if (!agents[receiver].task_sequence.empty() ||
            (delivered_now_waits &&
             agents[receiver].finish_time == cur_time_))
            continue;

        int donor = -1;
        int chosen_task = -1;
        int longest_finish = -1;
        for (int owner : order) {
            const deque<int>& sequence = agents[owner].task_sequence;
            if (owner == receiver || sequence.empty() ||
                (sequence.size() == 1 && agents[owner].status == AG_CARRYING))
                continue;
            int task_id = sequence.back();

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
            if (agents[donor].task_sequence.empty()) {
                agents[donor].status = AG_FREE;
                agents[donor].current_task = -1;
                agents[donor].current_goal_index = 0;
            }
            agents[receiver].task_sequence.push_back(chosen_task);
            all_tasks[chosen_task].seq_id = receiver;
            all_tasks[chosen_task].status = receiver;
            reassigned = true;
        }
    }
    if (reassigned) update_makespan_bound();
}

// Reference Cost(): the time at which an agent completes its task sequence
// when it reaches its next goal at time `start`.
int Simulation::ta_sequence_cost(int agent_id, int start) const {
    const Agent& agent = agents[agent_id];
    int time = start;
    int location = -1;
    for (int task_id : agent.task_sequence) {
        const Task& task = all_tasks[task_id];
        int goal = task_id == agent.current_task
            ? max(agent.current_goal_index, 0) : 0;
        for (; goal < (int)task.goals.size(); goal++) {
            if (location >= 0) {
                int endpoint = mapd_map.ep_index(task.goals[goal]);
                int leg = endpoint < 0 ? INT_MAX :
                    mapd_map.endpoints[endpoint].h_val[location];
                if (leg == INT_MAX) return INT_MAX;
                time += leg;
            }
            if (goal == 0) time = max(time, task.release_time);
            location = task.goals[goal];
        }
    }
    return time;
}

vector<int> Simulation::ta_agents_in_sequence_order() const {
    vector<int> order(agents.size());
    for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++)
        order[ta_sequence_rank_[agent_id]] = agent_id;
    return order;
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
        RuntimeDeadline::check("LNS assignment improvement");
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
    // NEAREST_AVAILABLE relaxes selection of the new dummy endpoint, but an
    // endpoint held by the preceding committed plan remains occupied until its
    // agent actually moves. Defer overlapping tasks under either PBS policy.
    // Periodic replanning reaches such a goal once the parked agent leaves;
    // deferring there can wait forever on an idle agent.
    const bool defer_old_dummy_conflicts =
        config.dummy_path &&
        (config.endpoint_strategy == NEAREST_WITH_STRICT_EXCLUSIONS ||
         (config.endpoint_strategy == NEAREST_AVAILABLE &&
          !(config.replan_on & REPLAN_PERIODIC)));
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
        RuntimeDeadline::check("Hungarian task assignment");
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
        RuntimeDeadline::check("LNS repair");
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

// The remaining ordered goals of each agent's first max_tasks tasks. Matches
// the paper's task-truncation parameter: later tasks stay queued for a later
// replan.
vector<vector<pair<int,int>>> Simulation::task_goal_sequences(int max_tasks) const {
    vector<vector<pair<int,int>>> goal_seqs(agents.size());
    for (int i = 0; i < (int)agents.size(); i++) {
        int current_task_count = 0;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= max_tasks) break;
            const Task& task = all_tasks[tid];
            int start_goal = (tid == agents[i].current_task)
                ? agents[i].current_goal_index : 0;
            for (int g = start_goal; g < (int)task.goals.size(); g++)
                goal_seqs[i].push_back({task.goals[g], g == 0 ? task.release_time : 0});
            current_task_count++;
        }
    }
    return goal_seqs;
}

vector<vector<pair<int,int>>> Simulation::build_goal_sequences() {
    return build_goal_sequences(vector<bool>(agents.size(), true));
}

// Goal sequences of the planned agents. An agent that is not planned keeps its
// committed path, so its final endpoint stays reserved.
vector<vector<pair<int,int>>> Simulation::build_goal_sequences(const vector<bool>& planned) {
    int num_ag = (int)agents.size();
    vector<int> assigned_dummies(num_ag, -1);
    vector<vector<pair<int,int>>> goal_seqs =
        task_goal_sequences(config.task_sequence_limit);
    for (int i = 0; i < num_ag; i++)
        if (!planned[i]) assigned_dummies[i] = (int)path_table_[i][maxtime - 1];

    // Dummy endpoints: agents that have work choose first, so idle agents are
    // the ones pushed out to the leftover endpoints. A selected endpoint equal
    // to the last task goal represents a zero-length dummy path.
    vector<int> busy_agents, free_agents;
    for (int i = 0; i < num_ag; i++) {
        if (!planned[i]) continue;
        if (!goal_seqs[i].empty()) busy_agents.push_back(i);
        else                       free_agents.push_back(i);
    }
    for (int i : busy_agents) {
        int dummy = rest_endpoint(make_sequence_endpoint_request(
            i, goal_seqs[i].back().first, assigned_dummies, true));
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }
    for (int i : free_agents) {
        int dummy = rest_endpoint(make_sequence_endpoint_request(
            i, (int)agents[i].loc, assigned_dummies, true));
        assigned_dummies[i] = dummy;
        goal_seqs[i].push_back({dummy, 0});
    }

    return goal_seqs;
}

// --- SelectAgents() : switch on agent_selection ----------------------------
vector<bool> Simulation::select_agents(
    const vector<vector<pair<int,int>>>& task_goals) const {
    const int num_ag = (int)agents.size();
    vector<bool> planned(num_ag, true);
    switch (select_all_ ? SELECT_ALL : config.agent_selection) {
    case SELECT_ALL:
        return planned;
    case SELECT_CHANGED: {
        // Cells another task still needs: an agent parked on one must move.
        unordered_map<int, int> goal_holder;   // location -> agent, -1 = open task
        for (const Task* task : open_tasks_)
            for (int goal : task->goals) goal_holder[goal] = -1;
        for (int i = 0; i < num_ag; i++)
            for (const auto& goal : task_goals[i])
                if (!goal_holder.count(goal.first)) goal_holder[goal.first] = i;
        for (int i = 0; i < num_ag; i++) {
            // Reaching goals only shortens the sequence a path was planned for.
            const auto& before = planned_goals_[i];
            const auto& now = task_goals[i];
            bool unchanged = has_plan_[i] && now.size() <= before.size() &&
                equal(now.begin(), now.end(), before.end() - now.size());
            // A path that ends on one of the agent's own goals moves on from
            // there; any other final cell blocks the task that needs it.
            const int final_cell = (int)path_table_[i][maxtime - 1];
            auto holder = goal_holder.find(final_cell);
            bool own_goal = any_of(now.begin(), now.end(),
                [&](const pair<int,int>& goal) { return goal.first == final_cell; });
            bool blocking = holder != goal_holder.end() && holder->second != i &&
                (holder->second >= 0 || !own_goal);
            // A path that stops short of the next goal (a windowed plan, say)
            // must be extended.
            bool stuck = !now.empty() && !path_reaches(i, now.front());
            planned[i] = !unchanged || blocking || stuck;
        }
        return planned;
    }
    default:
        throw logic_error("select_agents: unsupported agent_selection");
    }
}

// Whether agent_id's committed path visits goal (location, earliest time).
bool Simulation::path_reaches(int agent_id, const pair<int,int>& goal) const {
    for (unsigned int t = max(cur_time_, (unsigned int)max(goal.second, 0)); t < maxtime; t++)
        if ((int)path_table_[agent_id][t] == goal.first) return true;
    return false;
}

void Simulation::remember_plans(const vector<bool>& planned,
                                const vector<vector<pair<int,int>>>& task_goals) {
    for (int i = 0; i < (int)agents.size(); i++)
        if (planned[i]) {
            planned_goals_[i] = task_goals[i];
            has_plan_[i] = true;
        }
    last_planned_ = planned;
}

// Endpoints an agent must not rest at because another agent needs them: goals
// of open tasks and of the other agents' tasks, and the cells where the other
// agents' committed paths end (also as reserved_endpoints, by agent).
void Simulation::exclude_others_endpoints(DummyEndpointRequest& request) const {
    for (Task* task : open_tasks_)
        for (int goal : task->goals)
            request.forbidden_endpoints.insert(goal);
    request.reserved_endpoints.assign(agents.size(), -1);
    for (int other = 0; other < (int)agents.size(); other++) {
        if (other == request.agent_id) continue;
        for (int task_id : agents[other].task_sequence)
            for (int goal : all_tasks[task_id].goals)
                request.forbidden_endpoints.insert(goal);
        request.reserved_endpoints[other] = (int)path_table_[other][maxtime - 1];
        request.forbidden_endpoints.insert(request.reserved_endpoints[other]);
    }
}

DummyEndpointRequest Simulation::make_token_endpoint_request(
    int agent_id, int reference_location,
    bool check_committed_crossings) const {
    DummyEndpointRequest request{agent_id, reference_location,
        (int)agents[agent_id].loc, agents[agent_id].initial_loc, {}};
    exclude_others_endpoints(request);
    request.use_safe_relocation_search = true;
    request.relocation_required = !mapd_map.is_endpoint[reference_location];
    for (Task* task : open_tasks_)
        for (int goal = 1; goal < (int)task->goals.size(); goal++)
            if (task->goals[goal] == reference_location)
                request.relocation_required = true;
    if (check_committed_crossings)
        for (unsigned int time = cur_time_;
             time < maxtime && !request.relocation_required; time++)
            for (int other = 0; other < (int)agents.size(); other++)
                if (other != agent_id &&
                    path_table_[other][time] ==
                        (unsigned int)reference_location) {
                    request.relocation_required = true;
                    break;
                }
    return request;
}

DummyEndpointRequest Simulation::make_hbh_endpoint_request(
    int agent_id, int reference_location) const {
    DummyEndpointRequest request{agent_id, reference_location,
        (int)agents[agent_id].loc, agents[agent_id].initial_loc, {}};
    exclude_others_endpoints(request);
    request.use_safe_relocation_search = true;
    request.parking_endpoints_only =
        config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
    request.exclude_all_open_goals = true;
    request.relocation_required = !mapd_map.is_endpoint[reference_location];
    for (Task* task : open_tasks_)
        for (int goal : task->goals)
            if (goal == reference_location)
                request.relocation_required = true;
    for (unsigned int time = cur_time_ + 1;
         time < maxtime && !request.relocation_required; time++)
        for (int other = 0; other < (int)agents.size(); other++)
            if (other != agent_id &&
                path_table_[other][time] ==
                    (unsigned int)reference_location) {
                request.relocation_required = true;
                break;
            }
    return request;
}

DummyEndpointRequest Simulation::make_central_endpoint_request(
    int agent_id, int reference_location,
    const vector<int>& reserved_endpoints) const {
    return {agent_id, reference_location, (int)agents[agent_id].loc,
        agents[agent_id].initial_loc, reserved_endpoints};
}

// A re-planned sequence never keeps its previous dummy endpoint when
// leave_previous_endpoint is set (the reference's choose_good_endpoint excludes
// every agent's old endpoint, its own included).
DummyEndpointRequest Simulation::make_sequence_endpoint_request(
    int agent_id, int reference_location,
    const vector<int>& reserved_endpoints,
    bool leave_previous_endpoint) const {
    DummyEndpointRequest request{agent_id, reference_location,
        (int)agents[agent_id].loc, agents[agent_id].initial_loc, {}};
    exclude_others_endpoints(request);
    request.reserved_endpoints = reserved_endpoints;
    request.relocation_required = !mapd_map.is_endpoint[reference_location] ||
        request.forbidden_endpoints.count(reference_location);
    for (int other = 0; other < (int)reserved_endpoints.size(); other++)
        if (other != agent_id && reserved_endpoints[other] == reference_location)
            request.relocation_required = true;
    for (int task_id : agents[agent_id].task_sequence)
        for (int goal : all_tasks[task_id].goals)
            if (goal != request.current_location)
                request.forbidden_endpoints.insert(goal);
    if (leave_previous_endpoint)
        request.forbidden_endpoints.insert((int)path_table_[agent_id][maxtime - 1]);
    return request;
}

DummyEndpointRequest Simulation::make_ta_endpoint_request(
    int agent_id, int reference_location,
    const vector<int>& reserved_endpoints, bool fixed_home_only) const {
    DummyEndpointRequest request = make_sequence_endpoint_request(
        agent_id, reference_location, reserved_endpoints, false);
    if (fixed_home_only) {
        request.candidate_endpoints = {request.home_location};
    } else {
        request.candidate_endpoints = {
            request.current_location, request.home_location};
    }
    return request;
}

// Where an agent rests after its last goal (the request's reference
// location): with --dummy_path 1 the strategy's endpoint, with 0 the reference
// location itself unless another agent or task needs that cell.
int Simulation::rest_endpoint(const DummyEndpointRequest& request) {
    // An idle token-passing agent that already moved to its rest endpoint
    // after its last task does not move again unless it must.
    const bool rested = !config.dummy_path ||
        (request.use_safe_relocation_search &&
         rested_at_[request.agent_id] == request.reference_location);
    if (rested && !request.relocation_required) return request.reference_location;
    int endpoint = choose_dummy_endpoint(config.endpoint_strategy, request);
    // Planning in assignment only waits where no committed path passes.
    if (endpoint == request.reference_location && request.relocation_required &&
        request.use_safe_relocation_search)
        endpoint = endpoint_wait_or_relocate(request);
    return endpoint;
}

// Move an idle token-passing agent to its rest endpoint. If that endpoint
// cannot be reached while the agent must vacate its cell, it moves to the
// nearest endpoint the safe relocation search reaches. Whether it moved.
bool Simulation::move_to_rest_endpoint(Agent& agent, const DummyEndpointRequest& request) {
    int endpoint = rest_endpoint(request);
    auto move_to = [&](int target) {
        if (target < 0 || target == agent.loc ||
            !plan_path2_to_endpoint(agent, target, request.parking_endpoints_only,
                                    request.exclude_all_open_goals))
            return false;
        rested_at_[agent.id] = target;
        return true;
    };
    if (move_to(endpoint)) return true;
    if (endpoint == agent.loc) rested_at_[agent.id] = endpoint;
    if (!request.relocation_required) return false;
    int safe = endpoint_wait_or_relocate(request);
    return safe != endpoint && move_to(safe);
}

// --- ChooseEndpoint() : switch on endpoint_strategy -------------------------
// Task endpoints have Endpoint::is_task_endpoint=true, parking/home endpoints
// false. The request (built by the caller's family) supplies the exclusions;
// returning reference_location means no relocation path is needed.
int Simulation::choose_dummy_endpoint(
    EndpointStrategy strategy, const DummyEndpointRequest& request) {
    switch (strategy) {
    case WAIT_OR_NEAREST_SAFE:          // TP/TPTS Path2
    case WAIT_OR_NEAREST_FREE_NONTASK:  // HBH Algorithm 2 (parking endpoints only)
        if (request.use_safe_relocation_search) return endpoint_wait_or_relocate(request);
        // Planners that plan the agents jointly: wait unless the cell is
        // needed, else the nearest available (parking) endpoint.
        if (!request.relocation_required) return request.reference_location;
        {
            DummyEndpointRequest relocation = request;
            relocation.parking_endpoints_only = strategy == WAIT_OR_NEAREST_FREE_NONTASK;
            relocation.forbidden_endpoints.insert(request.reference_location);
            return endpoint_nearest_available(relocation);
        }
    case NEAREST_AVAILABLE:
        // Planning inside assignment relocates only when waiting is unsafe.
        return request.use_safe_relocation_search ? endpoint_wait_or_relocate(request)
                                                  : endpoint_nearest_available(request);
    case PAIRWISE_TASK_THEN_HOME:
        return endpoint_pairwise(request);
    case RETURN_TO_HOME:
        // Offline fixed parking: every agent returns to its own home endpoint.
        return request.home_location;
    case NEAREST_WITH_STRICT_EXCLUSIONS:
        return endpoint_strict(request);
    default:
        throw logic_error("choose_dummy_endpoint: unsupported endpoint strategy");
    }
}

// Wait at the reference location unless it must be vacated; then the nearest
// reachable endpoint that avoids open-task goals and other agents' committed
// paths (parking endpoints only if the request says so).
int Simulation::endpoint_wait_or_relocate(const DummyEndpointRequest& request) {
    if (!request.relocation_required) return request.reference_location;
    // The time-space BFS reconstructs a route from a temporary agent; the real
    // path is planned afterwards.
    Agent probe = agents[request.agent_id];
    probe.loc = request.reference_location;
    return search_path2_endpoint(probe, -1, request.parking_endpoints_only,
                                 request.exclude_all_open_goals);
}

// CENTRAL: the reference location if it is still available, else the nearest
// endpoint not reserved in this batch or forbidden by the caller (the home
// endpoint if there is none).
int Simulation::endpoint_nearest_available(const DummyEndpointRequest& request) const {
    const int last_goal_loc = request.reference_location;
    set<int> forbidden = request.forbidden_endpoints;
    for (int location : request.reserved_endpoints)
        if (location >= 0) forbidden.insert(location);
    if (mapd_map.is_endpoint[last_goal_loc] && !forbidden.count(last_goal_loc))
        return last_goal_loc;

    int best_location = -1;
    int best_distance = INT_MAX;
    for (const Endpoint& endpoint : mapd_map.endpoints) {
        if (!request.candidate_endpoints.empty() &&
            find(request.candidate_endpoints.begin(), request.candidate_endpoints.end(),
                 endpoint.loc) == request.candidate_endpoints.end())
            continue;
        if (forbidden.count(endpoint.loc)) continue;
        if (request.parking_endpoints_only && endpoint.is_task_endpoint) continue;
        int distance = endpoint.h_val[last_goal_loc];
        // The later endpoint wins equal-distance ties, as in the PBS selector.
        if (distance <= best_distance) {
            best_distance = distance;
            best_location = endpoint.loc;
        }
    }
    // Every endpoint taken: the agent's own home, as the strict policy does.
    return best_location >= 0 ? best_location : request.home_location;
}

// wPBS: the nearest task endpoint not chosen by another agent (skipping the
// reference location), then the nearest such parking endpoint; staying is the
// last resort.
int Simulation::endpoint_pairwise(const DummyEndpointRequest& request) const {
    const int last_goal_loc = request.reference_location;
    const vector<int>& reserved = request.reserved_endpoints;
    auto nearest_of_type = [&](bool task_endpoint) {
        int best_loc = -1;
        int best_dist = INT_MAX;
        for (const Endpoint& endpoint : mapd_map.endpoints) {
            if (endpoint.is_task_endpoint != task_endpoint) continue;
            if (endpoint.loc == last_goal_loc) continue;
            bool taken = false;
            for (int i = 0; i < (int)reserved.size() && !taken; i++)
                taken = i != request.agent_id && reserved[i] == endpoint.loc;
            if (taken) continue;
            int distance = endpoint.h_val[last_goal_loc];
            // The later endpoint wins equal-distance ties (the released
            // implementation's std::map assignment).
            if (distance <= best_dist) {
                best_dist = distance;
                best_loc = endpoint.loc;
            }
        }
        return best_loc;
    };
    int endpoint = nearest_of_type(true);
    if (endpoint >= 0) return endpoint;
    endpoint = nearest_of_type(false);
    return endpoint >= 0 ? endpoint : last_goal_loc;
}

// PBS/PP (the reference's choose_good_endpoint): the nearest task endpoint
// distinct from the other new dummy endpoints that avoids unfinished-task goals
// and the old parking endpoints; the home endpoint if there is none.
int Simulation::endpoint_strict(const DummyEndpointRequest& request) const {
    set<int> forbidden = request.forbidden_endpoints;
    for (int i = 0; i < (int)request.reserved_endpoints.size(); i++)
        if (i != request.agent_id && request.reserved_endpoints[i] >= 0)
            forbidden.insert(request.reserved_endpoints[i]);

    int best_loc = -1;
    int best_dist = INT_MAX;
    for (const Endpoint& endpoint : mapd_map.endpoints) {
        if (!endpoint.is_task_endpoint || forbidden.count(endpoint.loc)) continue;
        int d = endpoint.h_val[request.reference_location];
        if (d <= best_dist) { best_dist = d; best_loc = endpoint.loc; }
    }
    return best_loc < 0 ? request.home_location : best_loc;
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
//  Section 5 — MAPF (component 5): CBS, TA-Hybrid, PP, PBS
// ============================================================================

// --- CBS ------------------------------------------------------------------
// CBS plans single goals. A focal weight of 1.0 gives optimal CBS and values
// greater than 1.0 bounded-suboptimal ECBS.

// Paths for agent_ids, each from its current location to its goal, that avoid
// every other agent's committed path; index 0 is the current timestep. Empty
// when CBS finds none.
vector<vector<int>> Simulation::cbs_solve(const vector<int>& agent_ids,
                                          const vector<int>& goals) {
    set<int> batch(agent_ids.begin(), agent_ids.end());
    vector<vector<int>> constraints;
    for (int i = 0; i < (int)agents.size(); i++)
        if (!batch.count(i))
            constraints.emplace_back(path_table_[i].begin(), path_table_[i].end());
    vector<int> starts, endpoint_ids;
    for (int k = 0; k < (int)agent_ids.size(); k++) {
        starts.push_back(agents[agent_ids[k]].loc);
        endpoint_ids.push_back(mapd_map.ep_index(goals[k]));
        if (endpoint_ids.back() < 0)
            throw runtime_error("CBS goal is not an endpoint");
    }
    ECBSResult result = ECBSPlanner().solve(ECBSRequest(
        mapd_map.grid, starts, goals, endpoint_ids, constraints, (int)cur_time_,
        mapd_map.col, config.ecbs_focal_weight,
        config.cbs_high_level_expansion_limit, config.cbs_low_level_expansion_limit,
        mapd_map.endpoints, (int)maxtime, config.cbs_conflict_selection,
        config.cbs_bypass, config.cbs_target_reasoning, config.cbs_rectangle_reasoning));
    if (!result.solution_found || result.paths.size() != agent_ids.size())
        return {};
    for (const vector<int>& path : result.paths)
        if (path.empty()) throw runtime_error("CBS returned an empty path");
    return result.paths;
}

// Commit a path that starts now; the agent then holds its last cell, and
// finish_time is its arrival there.
void Simulation::commit_segment(int agent_id, const vector<int>& path) {
    for (unsigned int t = cur_time_; t < maxtime; t++) {
        unsigned int step = t - cur_time_;
        int location = step < path.size() ? path[step] : path.back();
        path_table_[agent_id][t] = location;
        agents[agent_id].path[t] = location;
    }
    agents[agent_id].finish_time = cur_time_ + path.size() - 1;
}

// --- CENTRAL: CBS to the assigned endpoints ---------------------------------
void Simulation::path_planning_ecbs(bool assignment_triggered) {
    // CENTRAL solves two smaller CBS instances each timestep, as described in
    // the paper: first agents that became occupied now, then all remaining
    // free/non-carrying agents.
    auto solve_batch = [&](const vector<int>& agent_ids, const vector<int>& goals,
                           bool task_goal_batch) {
        if (agent_ids.empty()) return;
        vector<vector<int>> paths = cbs_solve(agent_ids, goals);
        if (paths.empty())
            throw runtime_error("CENTRAL-CBS failed to find the required batch path");
        for (int index = 0; index < (int)agent_ids.size(); index++) {
            int agent_id = agent_ids[index];
            commit_segment(agent_id, paths[index]);
            if (!task_goal_batch) continue;
            Task* task = central_current_task(agent_id);
            if (!task)
                throw runtime_error("CENTRAL-CBS task agent has no current task");
            // Completion and final-goal waiting apply only to the last ordered
            // goal. Intermediate goals trigger another Group-1 CBS segment when
            // the agent reaches them.
            if (agents[agent_id].current_goal_index == (int)task->goals.size() - 1) {
                task->completion_time = (int)agents[agent_id].finish_time;
                agents[agent_id].finish_time += task->goal_wait_time;
            }
        }
    };

    // Group 1: agents that became occupied or reached an intermediate goal at
    // this timestep. Plan one CBS segment to each task's next ordered goal.
    vector<int> task_agents, task_goals;
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
        task_agents.push_back(agent_id);
        task_goals.push_back(task->goals[goal_index]);
    }
    solve_batch(task_agents, task_goals, true);

    // Group 2 is planned after task assignment. CENTRAL does this every
    // timestep; CENTRAL-fixed does it only after a new-task/free-agent event.
    if (!assignment_triggered) return;

    // All free/non-carrying agents are jointly planned to the pickup or
    // parking endpoint stored by assign_central_hungarian().
    vector<int> free_agents, free_goals;
    for (int agent_id = 0; agent_id < (int)agents.size(); agent_id++) {
        if (agents[agent_id].status == AG_CARRYING) continue;
        free_agents.push_back(agent_id);
        free_goals.push_back(agents[agent_id].last_endpoint);
    }
    solve_batch(free_agents, free_goals, false);
}

// --- CBS for task sequences: one goal at a time ----------------------------
// Each planned agent's path leads to the next goal of its goal sequence (a
// task goal, or its endpoint when idle); REPLAN_GOAL_REACHED plans the next
// one when it arrives. CBS paths end at their goal and hold it, so a goal that
// another agent already ends at cannot be shared: that agent waits at the
// nearest free endpoint and is replanned at the next event. If the kept paths
// leave no solution, every agent is replanned.
void Simulation::path_planning_cbs_segments() {
    const int num_ag = (int)agents.size();
    vector<vector<pair<int,int>>> next_goal(num_ag);
    vector<vector<pair<int,int>>> task_goals = task_goal_sequences(1);
    for (int i = 0; i < num_ag; i++)
        if (!task_goals[i].empty()) next_goal[i] = {task_goals[i].front()};
    vector<bool> planned = select_agents(next_goal);
    vector<vector<pair<int,int>>> goal_seqs = build_goal_sequences(planned);

    unordered_set<int> taken;   // cells another agent's path ends at
    for (int i = 0; i < num_ag; i++)
        if (!planned[i]) taken.insert((int)path_table_[i][maxtime - 1]);
    auto nearest_free_endpoint = [&](int location) {
        int best = location, best_distance = INT_MAX;
        for (const Endpoint& endpoint : mapd_map.endpoints)
            if (!taken.count(endpoint.loc) && endpoint.h_val[location] < best_distance) {
                best = endpoint.loc;
                best_distance = endpoint.h_val[location];
            }
        return best;
    };
    vector<int> agent_ids, goals;
    for (int i = 0; i < num_ag; i++) {
        if (!planned[i]) continue;
        pair<int,int> goal = goal_seqs[i].front();
        if (taken.count(goal.first)) goal = {nearest_free_endpoint(agents[i].loc), 0};
        taken.insert(goal.first);
        agent_ids.push_back(i);
        goals.push_back(goal.first);
        next_goal[i] = {goal};
    }
    if (agent_ids.empty()) return;
    vector<vector<int>> paths = cbs_solve(agent_ids, goals);
    if (paths.empty()) {
        if (select_all_) throw runtime_error("CBS found no paths for the agents' next goals");
        select_all_ = true;
        path_planning_cbs_segments();
        select_all_ = false;
        return;
    }
    for (int k = 0; k < (int)agent_ids.size(); k++) {
        commit_segment(agent_ids[k], paths[k]);
        segment_goal_[agent_ids[k]] = goals[k];
    }
    remember_plans(planned, next_goal);
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
            RuntimeDeadline::check_periodic(
                (uint64_t)flow, "TA-Hybrid min-cost flow");
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
        uint64_t expanded = 0;
        while (!queue_.empty()) {
            RuntimeDeadline::check_periodic(
                expanded++, "TA-Hybrid min-cost-flow shortest path");
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
// goals, then reserves their parking paths. As in the reference
// implementation, a group avoids only the task part of the other agents'
// paths, up to their dummy_start_step. A dummy path that then collides is
// re-routed; if that fails, the agent's complete path becomes a constraint
// and the group is planned again. Apart from the sequence ranks and the
// makespan bound kept by assign_ta_hybrid(), all batch state is local.
void Simulation::path_planning_ta_hybrid(bool assignment_triggered) {
    RuntimeDeadline::check("TA-Hybrid planning");
    const int agent_count = (int)agents.size();
    const int map_size = (int)mapd_map.grid.size();
    const int now = (int)cur_time_;
    const int horizon = (int)maxtime;

    // Every other agent's path ends at its parking endpoint, which stays taken.
    auto parking_of = [&](int agent_id, int location) {
        vector<int> reserved(agent_count, -1);
        for (int other = 0; other < agent_count; other++)
            if (other != agent_id) reserved[other] = (int)agents[other].path[horizon - 1];
        return choose_dummy_endpoint(config.endpoint_strategy,
            make_ta_endpoint_request(agent_id, location, reserved, true));
    };

    // The paths that `group` must avoid: the complete path of an idle or held
    // agent, otherwise its task path up to dummy_start_step. The later steps
    // are -1, which constrains nothing.
    auto task_paths = [&](const vector<int>& group, const vector<bool>& held) {
        vector<bool> in_group(agent_count, false);
        for (int agent_id : group) in_group[agent_id] = true;
        vector<vector<int>> paths;
        for (int other = 0; other < agent_count; other++) {
            if (in_group[other]) continue;
            const Agent& agent = agents[other];
            paths.emplace_back(agent.path.begin(), agent.path.end());
            if (!held[other] && !agent.task_sequence.empty())
                fill(paths.back().begin() +
                         max(agent.dummy_start_step, now) + 1,
                     paths.back().end(), -1);
        }
        return paths;
    };

    // Reference GoHome() search: a dummy path from the agent's location at
    // time `start` to its parking endpoint that avoids `constraints`.
    auto plan_dummy_path = [&](int agent_id, int start,
                               const vector<vector<int>>& constraints) {
        vector<unsigned int>& path = agents[agent_id].path;
        int location = path[start];
        int parking = parking_of(agent_id, location);
        vector<int> dummy = plan_ta_dummy_path(
            mapd_map.grid,
            mapd_map.endpoints[mapd_map.ep_index(parking)].h_val,
            constraints, location, parking, mapd_map.col, start, horizon);
        for (int time = start; time < horizon && !dummy.empty(); time++)
            path[time] = dummy[min(time - start, (int)dummy.size() - 1)];
        return !dummy.empty();
    };

    // Reference ReplanDummyPath(): once `group` is planned, re-route the dummy
    // path of every other agent that now collides, against all other
    // complete paths. Returns the agents whose dummy path cannot be re-routed.
    auto repair_dummy_paths = [&](const vector<int>& group) {
        vector<bool> in_group(agent_count, false);
        for (int agent_id : group) in_group[agent_id] = true;
        auto last_move = [&](int agent_id) {
            const vector<unsigned int>& path = agents[agent_id].path;
            int t = horizon - 1;
            while (t > now && path[t] == path[t - 1]) t--;
            return t;
        };
        int settle = now;   // after this every agent stands still
        for (int agent_id = 0; agent_id < agent_count; agent_id++)
            settle = max(settle, last_move(agent_id));
        auto collides = [&](int agent_id) {
            const vector<unsigned int>& path = agents[agent_id].path;
            for (int other = 0; other < agent_count; other++) {
                if (other == agent_id) continue;
                const vector<unsigned int>& other_path = agents[other].path;
                for (int t = now + 1; t <= settle; t++)
                    if (path[t] == other_path[t] ||
                        (path[t] == other_path[t - 1] &&
                         path[t - 1] == other_path[t]))
                        return true;
            }
            return false;
        };
        vector<int> failed;
        for (int agent_id = 0; agent_id < agent_count; agent_id++) {
            Agent& agent = agents[agent_id];
            if (in_group[agent_id] || agent.task_sequence.empty() ||
                !collides(agent_id))
                continue;
            vector<vector<int>> others;
            for (int other = 0; other < agent_count; other++)
                if (other != agent_id)
                    others.emplace_back(agents[other].path.begin(),
                                        agents[other].path.end());
            if (!plan_dummy_path(agent_id, max(agent.dummy_start_step, now),
                                 others))
                failed.push_back(agent_id);
            else
                settle = max(settle, last_move(agent_id));
        }
        return failed;
    };
    auto commit_paths = [&]() {
        for (int agent_id = 0; agent_id < agent_count; agent_id++)
            for (int time = now; time < horizon; time++)
                path_table_[agent_id][time] = agents[agent_id].path[time];
    };

    // Reference pickup check: a free agent at the pickup of its next task,
    // at or after the release time, becomes a task agent now.
    auto pick_up = [&]() {
        bool picked_up = false;
        for (Agent& agent : agents) {
            if (agent.status == AG_CARRYING || agent.task_sequence.empty())
                continue;
            Task& task = all_tasks[agent.task_sequence.front()];
            if (agent.loc != task.goals.front() || now < task.release_time)
                continue;
            task.ag_arrive_start = now;
            agent.status = AG_CARRYING;
            agent.current_task = task.id;
            agent.current_goal_index = 1;
            agent.finish_time = now;
            picked_up = true;
        }
        return picked_up;
    };

    // ---------------- Group 1: task agents -> next ordered goal ---------
    // CHANGED plans the carriers whose segment ended (they picked up or
    // reached a goal); ALL replans every carrier.
    pick_up();
    vector<int> group1;
    for (int agent_id : ta_agents_in_sequence_order()) {
        const Agent& agent = agents[agent_id];
        if (agent.status == AG_CARRYING && agent.current_task >= 0 &&
            agent.current_task < (int)all_tasks.size() &&
            agent.current_goal_index <
                (int)all_tasks[agent.current_task].goals.size() &&
            (config.agent_selection == SELECT_ALL || agent.finish_time <= cur_time_))
            group1.push_back(agent_id);
    }

    vector<bool> held(agent_count, false);
    while (!group1.empty()) {
        vector<vector<int>> fixed_paths = task_paths(group1, held);

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
            int parking = choose_dummy_endpoint(config.endpoint_strategy,
                make_ta_endpoint_request(
                agent_id, goal, reserved, true));
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
            if (arrival < 0 &&
                parking != agents[agent_id].initial_loc) {
                parking = agents[agent_id].initial_loc;
                parking_endpoint = mapd_map.ep_index(parking);
                arrival = astar_with_dummy(
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
            }
            if (arrival < 0) return -1;
            path.assign(probe.path.begin(), probe.path.end());
            return arrival;
        };

        auto detect_conflict = [&](HybridCBSNode& node) {
            node.conflicts = 0;
            node.first_agent = -1;
            int earliest = INT_MAX;
            uint64_t scanned = 0;
            for (int first = 0; first < (int)group1.size(); first++)
                for (int second = first + 1;
                     second < (int)group1.size(); second++)
                    for (int time = now; time < horizon; time++) {
                        RuntimeDeadline::check_periodic(
                            scanned++, "TA-Hybrid conflict detection");
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
                ta_sequence_cost(group1[index], root->arrivals[index]));
        }
        detect_conflict(*root);
        open.push(root);
        allocated.push_back(root);

        HybridCBSNode* solution = nullptr;
        int expanded = 0;
        while (!open.empty()) {
            RuntimeDeadline::check_periodic(
                (uint64_t)expanded, "TA-Hybrid Group 1 CBS");
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
                        ta_sequence_cost(group1[index], child->arrivals[index]));
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

        vector<int> failed = repair_dummy_paths(group1);
        if (failed.empty()) break;
        for (int agent_id : failed) held[agent_id] = true;
    }

    // Group 2 is recomputed only when assignment ran, which the TA_HYBRID
    // preset does every timestep. Group 1 above remains independently event
    // driven for intermediate multi-goal transitions.
    if (!assignment_triggered) {
        commit_paths();
        return;
    }

    vector<int> remaining;
    for (int agent_id : ta_agents_in_sequence_order())
        if (agents[agent_id].status != AG_CARRYING &&
            !agents[agent_id].task_sequence.empty())
            remaining.push_back(agent_id);

    held.assign(agent_count, false);
    while (!remaining.empty()) {
        RuntimeDeadline::check("TA-Hybrid Group 2 batching");
        // Take the free agents in sequence order while their next pickups
        // are pairwise distinct; the flow routes the agents in id order.
        const vector<int> candidates = remaining;
        vector<int> subgroup;
        vector<Task*> tasks;
        set<int> used_pickups;
        vector<int> deferred;
        for (int owner : remaining) {
            Task* task = &all_tasks[agents[owner].task_sequence.front()];
            if (used_pickups.insert(task->goals.front()).second) {
                subgroup.push_back(owner);
                tasks.push_back(task);
            } else {
                deferred.push_back(owner);
            }
        }
        remaining.swap(deferred);
        sort(subgroup.begin(), subgroup.end());

        vector<int> arrival(agent_count, now);
        vector<int> target(agent_count, -1);   // task each agent is routed to
        vector<vector<int>> fixed_paths = task_paths(subgroup, held);
        vector<vector<int>> flow_paths;
        while (flow_paths.empty()) {
            RuntimeDeadline::check("TA-Hybrid Group 2 flow construction");
            // Each pickup must be reached early enough for its sequence to
            // finish within the makespan bound, which grows until the flow
            // routes every agent.
            vector<int> deadlines(tasks.size(), now);
            int maximum_deadline = now;
            for (int index = 0; index < (int)tasks.size(); index++) {
                int owner = tasks[index]->seq_id;
                int& deadline = deadlines[index];
                while (deadline + 1 < horizon - 1 &&
                       ta_sequence_cost(owner, deadline + 1) <=
                           ta_makespan_bound_)
                    deadline++;
                maximum_deadline = max(maximum_deadline, deadline);
            }

            // Every flow path costs its arrival time, so the min-cost flow
            // only needs the layers its optimal paths can use. The flow is
            // anonymous: the path leaving agent i may end at any pickup, so
            // agent i's lower bound is its earliest arrival at any pickup.
            // Start from the arrivals that the hardest pickup and agent force
            // and double the window until either it reaches the deadline
            // horizon or the solution is provably optimal for the full graph:
            // a path longer than the window would cost more than the window
            // optimum minus the other agents' lower bounds.
            const int full_horizon = maximum_deadline - now + 1;
            const int task_count = (int)tasks.size();
            vector<vector<int>> arrival_bound(
                task_count, vector<int>(task_count));
            for (int task_index = 0; task_index < task_count; task_index++) {
                int endpoint =
                    mapd_map.ep_index(tasks[task_index]->goals.front());
                int earliest = max(now, tasks[task_index]->release_time) - now;
                for (int agent_index = 0; agent_index < task_count;
                     agent_index++) {
                    int distance = endpoint >= 0
                        ? mapd_map.endpoints[endpoint].h_val[
                              agents[subgroup[agent_index]].loc]
                        : 0;
                    if (distance == INT_MAX) distance = full_horizon;
                    arrival_bound[agent_index][task_index] =
                        max(distance, earliest);
                }
            }
            vector<int> lower_bounds(task_count, INT_MAX);
            int lower_bound_sum = 0, initial_window = 1;
            for (int agent_index = 0; agent_index < task_count;
                 agent_index++) {
                for (int bound : arrival_bound[agent_index])
                    lower_bounds[agent_index] =
                        min(lower_bounds[agent_index], bound);
                lower_bound_sum += lower_bounds[agent_index];
                initial_window =
                    max(initial_window, lower_bounds[agent_index]);
            }
            for (int task_index = 0; task_index < task_count; task_index++) {
                int closest = INT_MAX;
                for (int agent_index = 0; agent_index < task_count;
                     agent_index++)
                    closest = min(closest,
                                  arrival_bound[agent_index][task_index]);
                initial_window = max(initial_window, closest);
            }
            int relative_horizon = min(full_horizon, 2 * initial_window);
            while (true) {
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

                // Wait first, then the reference move order.
                for (int time = 0; time < relative_horizon; time++) {
                    RuntimeDeadline::check("TA-Hybrid flow graph construction");
                    for (int location = 0; location < map_size; location++) {
                        if (!mapd_map.grid[location]) continue;
                        flow.add_edges(in_node(location, time),
                                       out_node(location, time), 1, 0, -1);
                        for (int action : {0, -1, 1, -mapd_map.col,
                                           mapd_map.col}) {
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
                    // An agent may already stand at a released pickup.
                    int first = max(now, tasks[index]->release_time) - now;
                    int last = min(deadlines[index] - now,
                                   relative_horizon - 1);
                    for (int time = first; time <= last; time++)
                        flow.add_edges(
                            out_node(tasks[index]->goals.front(), time),
                            task_base + index, 1, 0, -1);
                    flow.add_edges(task_base + index, sink, 1, 0, -1);
                }

                for (const vector<int>& path : fixed_paths) {
                    for (int time = 1; time < relative_horizon; time++) {
                        int location = path[now + time];
                        if (location >= 0)
                            flow.remove_edge(in_node(location, time),
                                             out_node(location, time));
                    }
                    for (int time = 0; time < relative_horizon; time++) {
                        int current = path[now + time];
                        int next = path[now + time + 1];
                        if (current >= 0 && next >= 0)
                            flow.remove_edge(out_node(next, time),
                                             in_node(current, time + 1));
                    }
                }

                int routed = flow.solve();
                if (routed == (int)subgroup.size()) {
                    vector<vector<int>> paths = flow.extract_paths();
                    int cost = 0;
                    for (const vector<int>& path : paths)
                        cost += (int)path.size() - 1;
                    bool proven = relative_horizon >= full_horizon;
                    if (!proven) {
                        proven = true;
                        for (int index = 0; index < task_count; index++)
                            if (relative_horizon < cost -
                                    (lower_bound_sum - lower_bounds[index]))
                                proven = false;
                    }
                    if (proven) {
                        flow_paths = paths;
                        break;
                    }
                } else if (relative_horizon >= full_horizon) {
                    break;
                }
                relative_horizon = min(full_horizon, 2 * relative_horizon);
            }
            if (flow_paths.empty() && ++ta_makespan_bound_ >= horizon)
                throw runtime_error(
                    "TA-Hybrid Group 2 min-cost flow exceeded the horizon");
        }

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
            int pickup = agents[agent_id].path[arrival[agent_id]];
            for (int time = arrival[agent_id]; time < horizon; time++)
                agents[agent_id].path[time] = pickup;
        }

        // The vertex-capacitated flow network prevents vertex conflicts. As
        // in the reference implementation, resolve any opposite-edge swaps by
        // exchanging the two path suffixes; this preserves both endpoints and
        // total flow cost while eliminating the edge collision.
        bool swapped = true;
        while (swapped) {
            RuntimeDeadline::check("TA-Hybrid edge-swap repair");
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

        for (int agent_id : subgroup) {
            int pickup = agents[agent_id].path[arrival[agent_id]];
            for (int index = 0; index < (int)tasks.size(); index++)
                if (tasks[index]->goals.front() == pickup)
                    target[agent_id] = index;
            if (target[agent_id] < 0)
                throw runtime_error(
                    "TA-Hybrid flow returned an unknown pickup endpoint");
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
                Task& task = *tasks[target[agent_id]];
                int pickup = task.goals.front();
                int pickup_endpoint = mapd_map.ep_index(pickup);
                vector<int> reserved(agent_count, -1);
                int parking = choose_dummy_endpoint(config.endpoint_strategy,
                    make_ta_endpoint_request(
                    agent_id, pickup, reserved, true));
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
                int result = plan_dummy_path(
                    agent_id, arrival[agent_id], constraints)
                    ? arrival[agent_id] : -1;
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
                    if (result < 0 &&
                        parking != agents[agent_id].initial_loc) {
                        parking = agents[agent_id].initial_loc;
                        parking_endpoint = mapd_map.ep_index(parking);
                        result = astar_with_dummy(
                            agents[agent_id], agents[agent_id].loc, now,
                            pickup, parking,
                            mapd_map.endpoints[pickup_endpoint].h_val,
                            mapd_map.endpoints[parking_endpoint].h_val,
                            constraints, task.release_time, false);
                    }
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
                Task& task = *tasks[target[agent_id]];
                int pickup = task.goals.front();
                int pickup_endpoint = mapd_map.ep_index(pickup);
                vector<int> reserved(agent_count, -1);
                int parking = choose_dummy_endpoint(config.endpoint_strategy,
                    make_ta_endpoint_request(
                    agent_id, pickup, reserved, true));
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
                if (result < 0 &&
                    parking != agents[agent_id].initial_loc) {
                    parking = agents[agent_id].initial_loc;
                    parking_endpoint = mapd_map.ep_index(parking);
                    result = astar_with_dummy(
                        probe, probe.loc, now, pickup, parking,
                        mapd_map.endpoints[pickup_endpoint].h_val,
                        mapd_map.endpoints[parking_endpoint].h_val,
                        constraints, task.release_time, false);
                }
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

        for (int agent_id : subgroup)
            agents[agent_id].dummy_start_step = arrival[agent_id];
        vector<int> failed = repair_dummy_paths(subgroup);
        if (!failed.empty()) {
            // Plan this batch again, now avoiding these complete paths.
            for (int agent_id : failed) held[agent_id] = true;
            remaining = candidates;
            continue;
        }
        held.assign(agent_count, false);

        // A flow may send an agent to another sequence's pickup. Move that
        // complete sequence, with its tour position, to the assigned agent,
        // matching the anonymous reassignment step in the paper and released
        // implementation.
        vector<deque<int>> old_sequences(agent_count);
        vector<int> old_ranks = ta_sequence_rank_;
        vector<int> source_owner(agent_count, -1);
        for (int agent_id : subgroup) {
            old_sequences[agent_id] = agents[agent_id].task_sequence;
            source_owner[agent_id] = tasks[target[agent_id]]->seq_id;
        }
        for (int agent_id : subgroup) {
            agents[agent_id].task_sequence =
                old_sequences[source_owner[agent_id]];
            ta_sequence_rank_[agent_id] = old_ranks[source_owner[agent_id]];
            for (int task_id : agents[agent_id].task_sequence) {
                all_tasks[task_id].seq_id = agent_id;
                all_tasks[task_id].status = agent_id;
            }
            Task& task = all_tasks[agents[agent_id].task_sequence.front()];
            task.ag = &agents[agent_id];
            agents[agent_id].status = AG_MOVING_TO_PICKUP;
            agents[agent_id].current_task = task.id;
            agents[agent_id].current_goal_index = 0;
            agents[agent_id].finish_time = arrival[agent_id];
        }
    }
    commit_paths();
    // Agents routed to the pickup they already stand on start their task now.
    if (pick_up()) path_planning_ta_hybrid(false);
}

// --- TA-Prioritized --------------------------------------------------------
// At every priority level, tentatively plan every remaining agent and select
// the one with the largest actual task-sequence completion time. This follows
// the paper's improved prioritized-planning rule; the released implementation
// uses the LKH tour order only as the deterministic tie breaker.
void Simulation::path_planning_pp_task_sequence() {
    RuntimeDeadline::check("TA-Prioritized planning");
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
            RuntimeDeadline::check("TA-Prioritized task sequence");
            const Task& task = all_tasks[task_id];
            int first_goal = task_id == candidate.agent.current_task
                ? candidate.agent.current_goal_index : 0;
            if (first_goal < (int)task.goals.size())
                final_goal = task.goals.back();
        }

        int parking = config.dummy_path
            ? choose_dummy_endpoint(config.endpoint_strategy,
                make_ta_endpoint_request(
                agent_id, final_goal, selected_endpoints, false))
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
            RuntimeDeadline::check("TA-Prioritized task path planning");
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
        RuntimeDeadline::check("TA-Prioritized priority selection");
        int best_agent = -1;
        CandidatePlan best_plan;

        for (int candidate_id = 0; candidate_id < agent_count;
             candidate_id++) {
            RuntimeDeadline::check_periodic(
                (uint64_t)candidate_id,
                "TA-Prioritized candidate planning");
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
            config.endpoint_strategy,
            make_ta_endpoint_request(best_agent,
            agents[best_agent].task_sequence.empty()
                ? (int)agents[best_agent].loc
                : all_tasks[agents[best_agent].task_sequence.back()].goals.back(),
            selected_endpoints, false));

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
        if (TieBreakPolicy::reference()) return lhs->g <= rhs->g;
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
    uint64_t expanded = 0;
    while (!open.empty()) {
        RuntimeDeadline::check_periodic(
            expanded++, "TA prioritized/dummy A*");
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
    const vector<vector<int>> no_old_paths;

    vector<vector<int>> old_paths(num_agents, vector<int>(max_time));
    for (int i = 0; i < num_agents; i++)
        for (int t = 0; t < max_time; t++)
            old_paths[i][t] = (int)path_table_[i][t];

    // PP plans the next task of each selected agent; the others keep their
    // committed paths and endpoints.
    const vector<vector<pair<int,int>>> task_goals = task_goal_sequences(1);
    const vector<bool> selected = select_agents(task_goals);

    vector<pair<int,int>> plan_order;
    for (int i = 0; i < num_agents; i++)
        if (selected[i])
            plan_order.push_back({agents[i].task_sequence.empty() ? 1 : 0, i});
    sort(plan_order.begin(), plan_order.end());

    vector<vector<int>> new_paths(num_agents);
    vector<bool> planned(num_agents, false);
    vector<int> assigned_dummies(num_agents, -1);
    for (int i = 0; i < num_agents; i++)
        if (!selected[i]) assigned_dummies[i] = old_paths[i][max_time - 1];

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
        vector<pair<int,int>> goals = task_goals[agent_id];

        int last_goal = goals.empty() ? (int)agent.loc : goals.back().first;
        int dummy = rest_endpoint(make_sequence_endpoint_request(
            agent_id, last_goal, assigned_dummies, true));
        assigned_dummies[agent_id] = dummy;
        goals.push_back({dummy, 0});

        const vector<vector<pair<int,int>>> task_groups =
            split_into_task_groups(agent_id, goals);
        vector<int> path = plan_single_agent({agent_id, (int)agent.loc, (int)cur_time_,
                                              task_groups, constraints, no_old_paths, false});

        // Preserve the archived PP behavior: if SIPP cannot find a path within
        // its search bound, retry with MLA* before falling back to waiting.
        if (path.empty() && config.single_agent == SA_MLSIPP_SEQUENCE)
            path = mla_star_taskwise_impl(agent_id, (int)agent.loc, (int)cur_time_,
                                          task_groups, constraints, no_old_paths, false);

        new_paths[agent_id].resize(max_time);
        if (!path.empty()) {
            for (int t = 0; t < max_time; t++) new_paths[agent_id][t] = path[t];
        } else {
            for (int t = 0; t < max_time; t++) new_paths[agent_id][t] = (int)agent.loc;
        }

        planned[agent_id] = true;
        for (int t = (int)cur_time_; t < max_time; t++) {   // keep the executed history
            path_table_[agent_id][t] = new_paths[agent_id][t];
            agents[agent_id].path[t] = new_paths[agent_id][t];
        }
        agents[agent_id].finish_time = path_end_time(agent_id);
    }
    remember_plans(selected, task_goals);

    for (int i = 0; i < num_agents; i++) {
        if (!agents[i].task_sequence.empty() && agents[i].status == AG_FREE) {
            agents[i].status = AG_MOVING_TO_PICKUP;
            agents[i].current_task = agents[i].task_sequence.front();
            agents[i].current_goal_index = 0;
        }
    }
}

// --- PBS: depth-first search over partial priority orders. Each node fixes
// "agent a yields to agent b"; the low level then replans the yielding agent
// against the paths of everyone who outranks it.
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
    pbs_.task_goals = task_goal_sequences(config.task_sequence_limit);
    pbs_.planned = select_agents(pbs_.task_goals);
    pbs_.goal_seqs = build_goal_sequences(pbs_.planned);
    pbs_.fixed_paths.clear();
    for (int i = 0; i < pbs_.num_ag; i++)
        if (!pbs_.planned[i])
            pbs_.fixed_paths.emplace_back(path_table_[i].begin(), path_table_[i].end());

    pbs_.task_groups.assign(pbs_.num_ag, {});
    for (int i = 0; i < pbs_.num_ag; i++)
        pbs_.task_groups[i] = split_into_task_groups(i, pbs_.goal_seqs[i]);

    pbs_.path_horizon = min(pbs_.max_t, (int)cur_time_ + 1200);
    pbs_.window = config.mapf == MAPF_wPBS ? config.wpbs_replan_window : INT_MAX;

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

// One low-level call for agent_id under the given constraints.  A windowed
// (wPBS) plan ignores the old paths and stops at the end of its window.
vector<int> Simulation::pbs_plan_agent(int agent_id, const vector<vector<int>>& cons_paths,
                                       const vector<vector<int>>& old_paths, bool use_old) {
    const bool windowed = pbs_.window != INT_MAX;
    return plan_single_agent({agent_id, (int)agents[agent_id].loc, (int)cur_time_,
                              pbs_.task_groups[agent_id], cons_paths, old_paths,
                              use_old && !windowed,
                              windowed ? (int)cur_time_ + pbs_.window : INT_MAX});
}

// The other agents' old paths, which stay in the table (reference find_path);
// a windowed plan has none.
vector<vector<int>> Simulation::pbs_others_old_paths(int agent_id) const {
    vector<vector<int>> old;
    if (pbs_.window != INT_MAX) return old;
    for (int j = 0; j < pbs_.num_ag; j++)
        if (j != agent_id) old.push_back(pbs_.old_paths[j]);
    return old;
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
        if (!pbs_.planned[i]) continue;   // keeps its committed path
        // No priorities yet; only agents that are not planned constrain the root.
        vector<vector<int>> cons_buf = pbs_.fixed_paths;
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

    for (int i = 0; i < pbs_.num_ag; i++)
        root->cost += pbs_path_cost(i, root->paths[i]);
    pbs_detect_conflicts(root->paths, root->conflicts);
    root->num_collisions = (int)root->conflicts.size();
    return root;
}

// Path cost = arrival time at the final goal, waits included (the reference's
// unit-weight path cost).  A windowed path that stops short of it costs the
// window plus the remaining distance through its unvisited goals (the f-value
// the reference's low level returns), so progress inside the window counts.
int Simulation::pbs_path_cost(int agent_id, const vector<int>& path) const {
    const auto& goals = pbs_.goal_seqs[agent_id];
    if (pbs_.window != INT_MAX && !goals.empty()) {
        // Goals reached in order inside the window; the final one must be held.
        const int end = min((int)cur_time_ + pbs_.window, (int)path.size() - 1);
        size_t g = 0;
        for (int t = (int)cur_time_; t <= end && g + 1 < goals.size(); t++)
            while (g + 1 < goals.size() && path[t] == goals[g].first && t >= goals[g].second) g++;
        if (g + 1 < goals.size() || path[end] != goals.back().first) {
            int h = mapd_map.endpoints[mapd_map.ep_index(goals[g].first)].h_val[path[end]];
            for (; g + 1 < goals.size(); g++)
                h += mapd_map.endpoints[mapd_map.ep_index(goals[g + 1].first)].h_val[goals[g].first];
            return pbs_.window + h;
        }
    }
    int last = min((int)path.size(), pbs_.max_t) - 1;
    while (last > (int)cur_time_ && path[last] == path[last-1]) last--;
    return last - (int)cur_time_;
}

// Install a re-planned path and update the node cost (sum of path costs).
void Simulation::pbs_set_path(PBSNode* node, int agent_id, const vector<int>& path) {
    node->cost -= pbs_path_cost(agent_id, node->paths[agent_id]);
    for (int t = (int)cur_time_; t < pbs_.work_len; t++)
        node->paths[agent_id][t] = path[t];
    node->cost += pbs_path_cost(agent_id, node->paths[agent_id]);
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

// Conflicts count up to the settle time, or only inside the wPBS window.
int Simulation::pbs_conflict_horizon(const vector<vector<int>>& paths) const {
    int horizon = min(pbs_.max_t, pbs_settle_time(paths) + 1);
    if (pbs_.window != INT_MAX) horizon = min(horizon, (int)cur_time_ + pbs_.window + 1);
    return horizon;
}

// First vertex/edge conflict of each agent pair (at most one per pair).
void Simulation::pbs_detect_conflicts(const vector<vector<int>>& paths,
                                      list<Conflict>& out) const {
    int horizon = pbs_conflict_horizon(paths);
    for (int a1 = 0; a1 < pbs_.num_ag; a1++) {
        if (!pbs_.planned[a1]) continue;   // fixed paths are not branched on
        for (int a2 = a1 + 1; a2 < pbs_.num_ag; a2++) {
            if (!pbs_.planned[a2]) continue;
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
        if (j == agent_id || !pbs_.planned[j]) continue;
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
                                        set<int>& replan) const {
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
    child->cost = parent->cost;

    if (child->priorities.connected(higher, lower)) {   // would create a cycle
        delete child;
        return nullptr;
    }
    child->priorities.add(lower, higher);
    child->priority = {lower, higher};

    set<int> higher_set = child->priorities.get_higher_priority(lower);
    vector<vector<int>> cons = pbs_.fixed_paths;
    for (int hp : higher_set) cons.push_back(child->paths[hp]);

    vector<int> new_path = pbs_plan_agent(lower, cons, pbs_others_old_paths(lower), true);
    if (new_path.empty()) {
        delete child;
        return nullptr;
    }
    pbs_set_path(child, lower, new_path);

    if (!pbs_resolve_cascade(child, lower, parent)) {
        delete child;
        return nullptr;
    }

    child->num_collisions = (int)child->conflicts.size();
    return child;
}

// find_consistent_paths: keep re-planning lower-priority agents until no
// conflict contradicts the priority order.  Failing inside the budget makes the
// whole child invalid (keeping it would let the DFS re-expand it forever).
bool Simulation::pbs_resolve_cascade(PBSNode* node, int replanned_agent, const PBSNode* parent) {
    int horizon = pbs_conflict_horizon(node->paths);

    // Parent's conflicts first, the re-planned agent's new ones appended (the
    // order breaks ties when choosing the earliest conflict).
    node->conflicts.clear();
    for (auto& conf : parent->conflicts)
        if (get<0>(conf) != replanned_agent && get<1>(conf) != replanned_agent)
            node->conflicts.push_back(conf);
    pbs_detect_conflicts_for(replanned_agent, node->paths, horizon, node->conflicts);

    set<int> replan;
    pbs_find_replan_agents(node->priorities, node->conflicts, replan);

    // Budget as in the reference: five low-level calls per distinct agent
    // re-planned in this node, counting the one that opened it.
    set<int> replanned = {replanned_agent};
    int calls = 1;

    while (!replan.empty()) {
        RuntimeDeadline::check_periodic((uint64_t)calls, "PBS cascade replanning");
        if (calls > 5 * (int)replanned.size()) return false;

        int a = *replan.begin();
        replan.erase(replan.begin());
        replanned.insert(a);
        calls++;

        set<int> hp = node->priorities.get_higher_priority(a);
        vector<vector<int>> a_cons = pbs_.fixed_paths;
        for (int h : hp) a_cons.push_back(node->paths[h]);

        vector<int> np = pbs_plan_agent(a, a_cons, pbs_others_old_paths(a), true);
        if (np.empty()) return false;
        pbs_set_path(node, a, np);

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
        agents[i].finish_time = path_end_time(i);
    }
}

// When agent_id's committed path stops moving.
unsigned int Simulation::path_end_time(int agent_id) const {
    unsigned int t = maxtime - 1;
    while (t > cur_time_ && path_table_[agent_id][t] == path_table_[agent_id][t - 1]) t--;
    return t;
}

// --- the PBS search ---------------------------------------------------------
bool Simulation::pbs_solve_impl() {
    pbs_build_context();
    if (pbs_.window == INT_MAX) pbs_build_shared_old_table();

    vector<PBSNode*> all_nodes;
    PBSNode* root = pbs_generate_root();
    all_nodes.push_back(root);

    if (root->conflicts.empty()) {
        pbs_commit(root);
        remember_plans(pbs_.planned, pbs_.task_goals);
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
        RuntimeDeadline::check_periodic(
            (uint64_t)hl_expanded, "PBS high-level search");
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

        // Two ways to resolve it: a1 yields to a2, or a2 yields to a1.  A
        // conflict-free child at the root's cost (a lower bound) ends the search.
        auto solves = [&](const PBSNode* n) {
            return n && n->conflicts.empty() && n->cost == root->cost;
        };
        PBSNode* child0 = pbs_generate_child(curr, a1, a2);
        if (child0) all_nodes.push_back(child0);
        if (solves(child0)) { best_node = child0; break; }
        PBSNode* child1 = pbs_generate_child(curr, a2, a1);
        if (child1) all_nodes.push_back(child1);
        if (solves(child1)) { best_node = child1; break; }

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
    remember_plans(pbs_.planned, pbs_.task_goals);

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

void Simulation::sta_update_path(Agent& agent, const SearchNode& goal_node) {
    for (unsigned int t = goal_node.timestep + 1; t < agent.path.size(); t++)
        agent.path[t] = goal_node.loc;

    const SearchNode* node = &goal_node;
    while (node != nullptr) {
        agent.path[node->timestep] = node->loc;
        node = node->parent;
    }
}

// TP/TPTS plan one task with the configured single-agent search against every
// other agent's committed path; a TPTS steal hides the displaced agent. The path
// is committed only if it is collision-free and its final goal can be held
// forever, the token-passing invariant. Returns (first-goal arrival, final-goal
// arrival), or (-1, -1).
pair<int,int> Simulation::plan_token_task(Agent& agent, Task& task,
                                          int hidden_agent) {
    const int hidden = hidden_agent >= 0 ? hidden_agent : agent.id;
    rested_at_[agent.id] = -1;

    vector<vector<int>> constraints;
    constraints.reserve(path_table_.size() - 1);
    for (int other = 0; other < (int)path_table_.size(); other++) {
        if (other == agent.id || other == hidden) continue;
        constraints.emplace_back(path_table_[other].begin(), path_table_[other].end());
    }

    vector<pair<int,int>> goals;
    for (int goal_index = 0; goal_index < (int)task.goals.size(); goal_index++)
        goals.push_back({task.goals[goal_index],
                         goal_index == 0 ? task.release_time : 0});
    const vector<vector<pair<int,int>>> task_group(1, goals);
    const vector<vector<int>> no_old_paths;
    vector<int> path = plan_single_agent({agent.id, (int)agent.loc, (int)cur_time_,
                                          task_group, constraints, no_old_paths, false});
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

    for (int t = (int)cur_time_ + 1;
         t <= final_goal_time && t < (int)maxtime; t++)
        if (sta_is_constrained(agent.id, path[t - 1], path[t], t, hidden))
            return {-1, -1};

    for (unsigned int t = final_goal_time + 1; t < maxtime; t++)
        for (int other = 0; other < (int)path_table_.size(); other++) {
            if (other == agent.id || other == hidden) continue;
            if (path_table_[other][t] == (unsigned int)goals.back().first)
                return {-1, -1};
        }

    for (int t = (int)cur_time_; t < (int)path.size() && t < (int)maxtime; t++)
        agent.path[t] = path[t];   // keep the executed history
    return {first_goal_time, final_goal_time};
}

vector<int> Simulation::plan_sipp_segments(
    int agent_id, int start_loc, int start_time,
    const vector<pair<int,int>>& goals,
    const vector<vector<int>>& constraint_paths,
    const vector<vector<int>>& old_paths, bool use_old_paths) {
    if (goals.empty()) return {};

    vector<int> combined(maxtime, start_loc);
    int segment_start = start_time;
    int segment_location = start_loc;

    for (int goal_index = 0; goal_index < (int)goals.size(); goal_index++) {
        vector<pair<int,int>> segment_goal = {goals[goal_index]};
        bool intermediate = goal_index + 1 < (int)goals.size();
        vector<int> segment = sipp_search_impl(
            agent_id, segment_location, segment_start, segment_goal,
            constraint_paths, old_paths, use_old_paths, intermediate);
        if (segment.empty()) return {};

        int arrival = -1;
        for (int time = segment_start;
             time < (int)segment.size() && time < (int)maxtime; time++)
            if (segment[time] == goals[goal_index].first &&
                time >= goals[goal_index].second) {
                arrival = time;
                break;
            }
        if (arrival < 0) return {};

        int copy_through = intermediate ? arrival : (int)maxtime - 1;
        for (int time = segment_start; time <= copy_through; time++)
            combined[time] = segment[time];
        segment_start = arrival;
        segment_location = goals[goal_index].first;
    }
    return combined;
}

int Simulation::search_path2_endpoint(
    Agent& agent, int target_endpoint_loc,
    bool parking_endpoints_only, bool exclude_all_open_goals) {

    queue<SearchNode*> open;
    map<unsigned int, SearchNode*> nodes;
    const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    SearchNode* start = new SearchNode(agent.loc, 0, nullptr, (int)cur_time_);
    nodes.emplace((unsigned int)agent.loc, start);
    open.push(start);

    uint64_t expanded = 0;
    while (!open.empty()) {
        RuntimeDeadline::check_periodic(
            expanded++, "endpoint Path2 search");
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
            (!parking_endpoints_only ||
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
            if (exclude_all_open_goals) {
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

bool Simulation::plan_path2_to_endpoint(
    Agent& agent, int endpoint_loc,
    bool parking_endpoints_only, bool exclude_all_open_goals) {
    if (search_path2_endpoint(
            agent, endpoint_loc, parking_endpoints_only,
            exclude_all_open_goals) != endpoint_loc)
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
        if (TieBreakPolicy::reference()) return a->g_val <= b->g_val;
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

// --- Single_Agent_Search() : switch on single_agent -------------------------
// The goal groups fix how the search is segmented: STA* searches goal by goal,
// MLA* task by task, MLSIPP the whole sequence at once, and SIPP2 goal by goal
// over safe intervals.
vector<int> Simulation::plan_single_agent(const SingleAgentRequest& request) {
    vector<pair<int,int>> all_goals;
    for (const auto& group : request.goal_groups)
        all_goals.insert(all_goals.end(), group.begin(), group.end());

    switch (config.single_agent) {
    case SA_STA_TASK_EP: {
        vector<vector<pair<int,int>>> one_per_group;
        for (const auto& goal : all_goals) one_per_group.push_back({goal});
        return mla_star_taskwise_impl(request.agent_id, request.start_loc,
                                      request.start_time, one_per_group,
                                      request.constraint_paths, request.old_paths,
                                      request.use_old_paths, request.stop_time);
    }
    case SA_MLA_SEQUENCE:
        return mla_star_taskwise_impl(request.agent_id, request.start_loc,
                                      request.start_time, request.goal_groups,
                                      request.constraint_paths, request.old_paths,
                                      request.use_old_paths, request.stop_time);
    case SA_MLSIPP_SEQUENCE:
        return sipp_search_impl(request.agent_id, request.start_loc, request.start_time,
                                all_goals, request.constraint_paths, request.old_paths,
                                request.use_old_paths, false, request.stop_time);
    case SA_SIPP_SEGMENTS:
        return plan_sipp_segments(request.agent_id, request.start_loc, request.start_time,
                                  all_goals, request.constraint_paths, request.old_paths,
                                  request.use_old_paths);
    default:
        throw logic_error("plan_single_agent: unsupported single_agent");
    }
}

// --- task-by-task driver ----------------------------------------------------
// Plan one task group at a time and concatenate: the agent commits to reaching
// this task's goals before the next group is planned.  Only the last group may
// hold its goal (the earlier ones are passed through).
vector<int> Simulation::mla_star_taskwise_impl(
        int agent_id, int start_loc, int start_time,
        const vector<vector<pair<int,int>>>& task_groups,
        const vector<vector<int>>& cons_paths,
        const vector<vector<int>>& old_paths,
        bool use_old_paths, int stop_time) {
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
                                           /*skip_holding=*/!is_last, stop_time);
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
        // A windowed search that stopped short of this group's goals is done.
        if (!found) return stop_time == INT_MAX ? vector<int>() : full_path;
    }

    return full_path;
}

// --- the search -------------------------------------------------------------
vector<int> Simulation::seq_mla_star(int agent_id, int start_loc, int start_time,
                                     const vector<pair<int,int>>& goals,
                                     const vector<vector<int>>& cons_paths,
                                     const vector<vector<int>>& old_paths,
                                     bool use_old_paths,
                                     bool skip_holding, int stop_time) {
    if (goals.empty()) return {start_loc};
    const bool windowed = stop_time != INT_MAX;

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
        uint64_t bfs_expanded = 0;
        while (!bfs_q.empty()) {
            RuntimeDeadline::check_periodic(
                bfs_expanded++, "MLA* heuristic BFS");
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
    // count: PBS may still move that agent aside.)  A windowed search still
    // plans its window.
    if (start_loc != last_goal_loc && !windowed) {
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
        RuntimeDeadline::check_periodic(
            (uint64_t)mla_expanded, "MLA* low-level search");
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

        if (gi >= num_goals || curr->timestep >= stop_time) { solution = curr; break; }
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
            // A windowed search ranks its partial paths by the plain heuristic.
            if (earliest_holding > 0 && !windowed) {
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
        bool skip_holding, int stop_time) {
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
    // A windowed search still plans its window.
    if (start_loc != final_goal && stop_time == INT_MAX) {
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
            if (TieBreakPolicy::reference()) return a->g <= b->g;
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
        RuntimeDeadline::check_periodic(
            (uint64_t)expanded, "MLSIPP low-level search");
        SIPPNode* current = open.top();
        open.pop();

        int goal_id = current->goal_id;
        if (goal_id < num_goals && current->loc == goals[goal_id].first &&
            current->timestep >= goals[goal_id].second) {
            if (!(goal_id == num_goals - 1 && current->timestep < earliest_holding))
                goal_id++;
        }
        if (goal_id >= num_goals || current->timestep >= stop_time) {
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

// ============================================================================
//  Section 7 — System update                             (Algorithm 1, line 13)
//
//  Agents execute their committed paths.  Nothing can change between two
//  events, so instead of stepping one timestep at a time the clock jumps
//  straight to the next event (an agent reaching its next task goal, a task
//  release, or a wPBS window boundary) — the same state as stepping through,
//  without the empty iterations.
// ============================================================================

//  How far the clock may jump depends on how paths are planned:
//    * PP_TASK_SEQUENCE plans every complete path at once, so the schedule is
//      final and is finished in one step;
//    * planning inside assignment (replan_on NONE) returns to the assignment
//      whenever an agent is free (advance_time_tp);
//    * CENTRAL's per-goal agent state, and assigning or replanning at every
//      timestep, advance one step at a time;
//    * otherwise the clock jumps to the next goal arrival, task release or
//      periodic replan.
void Simulation::advance_time() {
    if (config.mapf == MAPF_PP_TASK_SEQUENCE) {
        // Offline TA has already computed every complete agent path and every
        // task completion time. Finalize that schedule atomically instead of
        // replaying it through the online event loop.
        unsigned int final_time = 0;
        for (Task& task : all_tasks) {
            if (task.completion_time < 0)
                throw runtime_error("offline planning left task " +
                                    to_string(task.id) + " incomplete");
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
    if (config.replan_on == 0) {
        advance_time_tp();
        return;
    }
    if (config.assign_method == AM_CENTRAL_HUNGARIAN ||
        config.assign_trigger == AT_EVERY_TIMESTEP ||
        (config.replan_on & REPLAN_EVERY_TIMESTEP)) {
        if (cur_time_ + 1 >= maxtime)
            throw runtime_error("advance_time exceeded the planning horizon");
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
    if (config.replan_on & REPLAN_PERIODIC) {
        // The next periodic replan is due one window after the last plan, not
        // after the current event; jumping past it would leave agents parked at
        // the end of their windowed paths.
        unsigned int window_end =
            (unsigned int)last_path_planning_time_ + config.wpbs_replan_window;
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
    if (next_ts >= maxtime)
        throw runtime_error("advance_time exceeded the planning horizon");

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
            new_available_agent_ = true;
        }
        return;
    }

    // The event-driven clock may jump across several path steps, so inspect the
    // complete segment executed since paths were last planned.
    for (int i = 0; i < (int)agents.size(); i++) {
        // Goals are reached in order, so never look before the last one.
        unsigned int scan_from = max(last_path_planning_time_, last_goal_time_[i]);
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
                    last_goal_time_[i] = t;

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

// Every task is picked up no earlier than its release and completed no
// earlier than its pickup.
bool Simulation::taskOrderCheck(const string& alg_name) const {
    int violations = 0;
    for (const Task& task : all_tasks) {
        if (task.ag_arrive_start < 0) continue;
        if (task.ag_arrive_start < task.release_time ||
            (task.completion_time >= 0 && task.completion_time < task.ag_arrive_start)) {
            if (violations < 10)
                cout << "[" << alg_name << "] TASK ORDER VIOLATION: task " << task.id
                     << " released " << task.release_time << " picked up "
                     << task.ag_arrive_start << " completed " << task.completion_time << endl;
            violations++;
        }
    }
    if (violations == 0) cout << "[" << alg_name << "] TASK ORDER CHECK PASSED" << endl;
    else cout << "[" << alg_name << "] TASK ORDER CHECK FAILED - " << violations << " tasks" << endl;
    return violations == 0;
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

