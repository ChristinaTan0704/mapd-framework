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
//        WHEN_ASSIGNED (the empty set) = TP, TPTS and HBH plan each agent
//        inside the assignment
//   4  which agents to plan            --agent_selection     select_agents()
//        ALL / CHANGED (the others keep their committed paths)
//   5  high-level planner              --mapf                plan_paths()
//        PP / PP_TASK_SEQUENCE / CBS / PBS / wPBS / TA_HYBRID_TWO_GROUP
//   5.1 window                         --wpbs_replan_window  wPBS = PBS that
//        plans w steps ahead; also the PERIODIC replan interval
//   6  low-level planner               --single_agent        plan_single_agent()
//        STA / MLA / MLSIPP / SIPP2 / SEQ_STA (also the low level of CBS
//        and of TA-Hybrid's carriers)
//   7  task endpoint                   --endpoint_strategy   choose_dummy_endpoint()
//        WAIT_OR_NEAREST_SAFE / WAIT_OR_NEAREST_FREE_NONTASK /
//        NEAREST_AVAILABLE / NEAREST_WITH_STRICT_EXCLUSIONS /
//        PAIRWISE_TASK_THEN_HOME / RETURN_TO_HOME;
//        --dummy_path 0/1                                    rest_endpoint()
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
//  Tuning options: task_sequence_limit (tasks per PBS/wPBS plan, and kept per
//  agent by repeated Hungarian/LNS), lns_time,
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
#include "cbs.h"
#include "ecbs_reference.h"
#include "config_names.h"

#include <deque>
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
    last_release_checked_ = -1;
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
    ta_sequence_rank_.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++) ta_sequence_rank_[i] = i;
    last_goal_time_.assign(agents.size(), 0);
    rested_at_.assign(agents.size(), -1);
    token_time_ = 0;
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

    // A task known in advance is still released at its release time, which
    // is an event too (an assignment may wait for it).
    for (int t = last_release_checked_ + 1; t <= (int)cur_time_ && t < (int)maxtime; t++)
        if (!task_indices_by_time[t].empty()) revealed_new_task = true;
    last_release_checked_ = (int)cur_time_;
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
    case AT_ON_AGENT_FREE_OR_AT_PICKUP:
        return cur_time_ == 0 || new_available_agent_ || pickup_event_ ||
               agent_on_open_pickup() || nobody_on_open_tasks();
    default:
        throw logic_error("should_assign: unsupported assign_trigger");
    }
}

// An agent that carries no task stands at the pickup of a released task that
// nobody has picked up.
bool Simulation::agent_on_open_pickup() const {
    vector<char> pickup(mapd_map.row * mapd_map.col, 0);
    for (const Task* task : open_tasks_)
        if (!task->goals.empty() && task->release_time <= (int)cur_time_)
            pickup[task->goals.front()] = 1;
    for (const Agent& agent : agents)
        if (agent.status != AG_CARRYING && pickup[agent.loc]) return true;
    return false;
}

// A released task is open while no agent carries or is matched to a task: the
// reference CENTRAL would wait for a lucky pickup cell for good (it never does
// on kiva small, where every task is released at time 0 or an agent is busy).
bool Simulation::nobody_on_open_tasks() const {
    for (const Agent& agent : agents)
        if (agent.status == AG_CARRYING || !agent.task_sequence.empty()) return false;
    for (const Task* task : open_tasks_)
        if (task->release_time <= (int)cur_time_) return true;
    return false;
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
    case AM_CENTRAL_HUNGARIAN_FIXED:
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
    if (replan_pending_) return true;
    const int on = config.replan_on;
    if (on & REPLAN_EVERY_TIMESTEP) return true;
    if ((on & REPLAN_AFTER_ASSIGNMENT) && assignment_triggered) return true;
    if ((on & REPLAN_GOAL_REACHED) && agent_needs_next_segment()) return true;
    if ((on & REPLAN_PERIODIC) && !all_task_sequences_empty())
        return (int)cur_time_ - (int)last_path_planning_time_ >= config.wpbs_replan_window;
    return false;
}

// A carrying agent has just picked up, or reached a later goal of, a task with
// further ordered goals. A CENTRAL agent, planned to its pickup only, and
// TA-Hybrid's groups plan the next leg then.
bool Simulation::agent_needs_next_segment() const {
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
// in last_planned_. If it fails, or their new paths clash with a path that was
// kept, every agent is replanned; PBS, wPBS and CBS then fall back to
// prioritized planning, which is retried with the agents it could not plan
// first (retry_pp_failed_first).
// A planner that still fails leaves the agents it could not plan on their
// committed paths, which stay collision-free (after a clash, every agent goes
// back to its path from before this replan), and plans again at the next
// timestep.
void Simulation::path_planning(bool assignment_triggered) {
    const vector<vector<unsigned int>> committed = path_table_;
    last_planned_.clear();
    // Kept paths can block the changed agents for good (they then wait or
    // oscillate): when CHANGED has made no progress for a while, plan every
    // agent.
    select_all_ = config.agent_selection != SELECT_ALL && selection_stalled();
    bool solved = plan_paths(assignment_triggered);
    const bool clash = !last_planned_.empty() && kept_paths_conflict();
    if (!solved || clash) {
        select_all_ = true;
        if (config.agent_selection != SELECT_ALL)
            solved = plan_paths(assignment_triggered);
        if (!solved && (config.mapf == MAPF_PBS || config.mapf == MAPF_wPBS ||
                        config.mapf == MAPF_CBS))
            solved = path_planning_pp();
        if (!solved && config.mapf != MAPF_TA_HYBRID_TWO_GROUP)
            solved = retry_pp_failed_first();
    }
    select_all_ = false;
    replan_pending_ = !solved;
    if (solved) return;
    cerr << "Warning: " << enum_name(config.mapf) << " could not plan every agent at t="
         << cur_time_ << "; they keep their paths until the next timestep" << endl;
    has_plan_.assign(agents.size(), false);   // the next replan plans every agent
    if (!clash) return;
    path_table_ = committed;
    for (int i = 0; i < (int)agents.size(); i++) {
        agents[i].path = committed[i];
        agents[i].finish_time = path_end_time(i);
    }
}

// Whether no agent has reached a task goal for 2 (rows + cols) steps although
// some agent has a released goal to reach.
bool Simulation::selection_stalled() const {
    unsigned int progress = 0;
    for (unsigned int t : last_goal_time_) progress = max(progress, t);
    if (cur_time_ < progress + 2 * (unsigned int)(mapd_map.row + mapd_map.col)) return false;
    for (const Agent& agent : agents) {
        if (agent.task_sequence.empty()) continue;
        const Task& task = all_tasks[agent.task_sequence.front()];
        const int goal = agent.current_task == task.id ? agent.current_goal_index : 0;
        if (goal > 0 || task.release_time <= (int)cur_time_) return true;
    }
    return false;
}

// Prioritized planning could not plan every agent: plan the agents it missed
// first, each against the agents planned before it and the kept ones (see
// path_planning_pp). The retry is kept only if it plans every agent.
bool Simulation::retry_pp_failed_first() {
    pp_first_.clear();
    for (int i = 0; i < (int)last_planned_.size(); i++)
        if (!last_planned_[i]) pp_first_.push_back(i);
    if (pp_first_.empty()) return false;
    const vector<vector<unsigned int>> table = path_table_;
    const auto goals = planned_goals_;
    const vector<bool> has_plan = has_plan_, last_planned = last_planned_;
    pp_retry_ = true;
    const bool solved = path_planning_pp();
    pp_retry_ = false;
    if (solved) return true;
    path_table_ = table;
    planned_goals_ = goals;
    has_plan_ = has_plan;
    last_planned_ = last_planned;
    for (int i = 0; i < (int)agents.size(); i++) {
        agents[i].path = table[i];
        agents[i].finish_time = path_end_time(i);
    }
    return false;
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

// Whether every planned agent received a conflict-free path.
bool Simulation::plan_paths(bool assignment_triggered) {
    switch (config.mapf) {
    case MAPF_PP_PER_TASK:
    case MAPF_PP_TASK_SEQUENCE:
        return path_planning_pp();
    case MAPF_CBS:
        return path_planning_cbs();
    case MAPF_PBS:
    case MAPF_wPBS:
        return path_planning_pbs();
    case MAPF_TA_HYBRID_TWO_GROUP:
        path_planning_ta_hybrid(assignment_triggered);
        return true;
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
    Agent* agent = &next_token_holder();
    if (agent->finish_time <= cur_time_) {
        token_time_ = agent->finish_time;
        assign_decoupled_greedy(*agent);
    }
}

// The reference token passing's waiting line: an agent still free at the
// token's time (the lowest id from 1 up, agent 0 last), else the earliest free
// agent (the lowest id among them).
Agent& Simulation::next_token_holder() {
    Agent* agent = &agents[0];
    for (int i = 1; i < (int)agents.size(); i++) {
        if (agents[i].finish_time == token_time_) return agents[i];
        if (agents[i].finish_time < agent->finish_time) agent = &agents[i];
    }
    return *agent;
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
    Agent* agent = &next_token_holder();

    // A TPTS task remains in the token while its assigned agent is travelling
    // to its first goal, allowing another agent with an earlier arrival to
    // steal it. Once that first goal is reached, execution of the ordered
    // multi-goal task has begun and the task can no longer be swapped.
    tpts_purge_picked_up_tasks();
    if (agent->finish_time <= cur_time_) {
        token_time_ = agent->finish_time;
        assign_tpts(*agent);
    }
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
        pair<int,int> arrival = plan_token_task(agent, *task, hidden_agent,
                                                unassigned ? INT_MAX : task->ag_arrive_start);
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
        if (move_to_rest_endpoint(agent, make_hbh_endpoint_request(agent_id, agent.loc)))
            continue;

        for (unsigned int timestep = cur_time_ + 1;
             timestep < maxtime; timestep++) {
            agent.path[timestep] = agent.loc;
            path_table_[agent.id][timestep] = agent.loc;
        }
        agent.finish_time = cur_time_ + 1;
    }
}

// --- CENTRAL: Hungarian matching of agents to pickups or parking -----------
// CENTRAL assigns endpoints. At every assignment each agent that is not
// carrying a task is matched to the first goal (pickup) of a released task or
// to a pairwise-distinct parking endpoint. A matched task joins the agent's
// task sequence but stays unowned (status -1) and in open_tasks_ until the
// agent picks it up, so the agent plans to the pickup only, and the next
// assignment undoes every match that has not been picked up. A parking
// endpoint is the final goal of an agent without tasks (Agent::parking_endpoint).

// The remaining goals of every carried task.
vector<bool> Simulation::central_carried_goals() const {
    vector<bool> held(mapd_map.row * mapd_map.col, false);
    for (const Agent& agent : agents) {
        if (agent.status != AG_CARRYING) continue;
        const Task& task = all_tasks[agent.current_task];
        for (int g = max(1, agent.current_goal_index); g < (int)task.goals.size(); g++)
            held[task.goals[g]] = true;
    }
    return held;
}

// Phase 1: an agent resting on the first goal of a released open task picks
// it up at once, unless a later goal of that task is held by a carried task
// or ends any agent's path. The new task's goals are added to `held`.
// Returns the agents that picked up a task they still carry.
vector<int> Simulation::central_phase1_instant_pickup(vector<bool>& held) {
    vector<int> agent_at_location(mapd_map.row * mapd_map.col, -1);
    vector<int> endpoint_hold_count(mapd_map.row * mapd_map.col, 0);
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].status != AG_CARRYING)
            agent_at_location[agents[i].loc] = i;
        endpoint_hold_count[path_table_[i][maxtime - 1]]++;
    }

    vector<Task*> picked_up;
    vector<int> new_carriers;
    for (Task* task : open_tasks_) {
        if (task->status != -1 || task->release_time > (int)cur_time_) continue;
        if (task->goals.empty()) continue;
        int pickup_location = task->goals.front();
        int agent_id = agent_at_location[pickup_location];
        if (agent_id < 0) continue;

        // The reference counts every path end, the agent's own included.
        bool remaining_goal_blocked = false;
        for (int goal_index = 1;
             goal_index < (int)task->goals.size(); goal_index++) {
            int goal_location = task->goals[goal_index];
            if (endpoint_hold_count[goal_location] > 0 || held[goal_location]) {
                remaining_goal_blocked = true;
                break;
            }
        }
        if (remaining_goal_blocked)
            continue;

        agent_at_location[pickup_location] = -1;
        Agent& agent = agents[agent_id];
        task->status = agent_id;
        task->ag_arrive_start = (int)cur_time_;
        last_goal_time_[agent_id] = cur_time_;
        picked_up.push_back(task);
        if (task->goals.size() == 1) {
            task->completion_time = (int)cur_time_;
            task->status = INT_MAX;
            continue;
        }

        agent.task_sequence = {task->id};
        agent.current_task = task->id;
        agent.current_goal_index = 1;
        agent.status = AG_CARRYING;
        new_carriers.push_back(agent_id);
        for (int goal_index = 1;
             goal_index < (int)task->goals.size(); goal_index++)
            held[task->goals[goal_index]] = true;
    }
    for (Task* task : picked_up)
        open_tasks_.remove(task);
    return new_carriers;
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

    // Only the cost is returned, which ties do not change; deeper nodes first
    // reach a goal that frees up late without expanding every earlier layer.
    struct CostNode { int loc, g, f, time; };
    auto compare = [](const CostNode& lhs, const CostNode& rhs) {
        if (lhs.f != rhs.f) return lhs.f > rhs.f;
        return lhs.g < rhs.g;
    };
    priority_queue<CostNode, vector<CostNode>, decltype(compare)> open(compare);
    visited.clear();

    const int max_time = (int)maxtime - 1;
    if (goal_last_occupation >= max_time) return map_size;   // never holdable
    // The agent can stay at the goal only after its last occupation, so the
    // heuristic is the larger of the distance and the time until then
    // (consistent, so the cost is still exact).
    auto heuristic = [&](int loc, int time) {
        return max(goal.h_val[loc], goal_last_occupation + 1 - time);
    };
    int initial_h = goal.h_val[start_loc];
    if (initial_h == INT_MAX) return map_size;
    open.push({start_loc, 0, heuristic(start_loc, start_time), start_time});

    const int actions[5] = {0, -mapd_map.col, 1, mapd_map.col, -1};
    uint64_t expanded = 0;

    while (!open.empty()) {
        RuntimeDeadline::check_periodic(expanded++, "CENTRAL assignment-cost A*");
        CostNode current = open.top();
        open.pop();
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

            if (goal.h_val[next_loc] == INT_MAX) continue;
            open.push({next_loc, current.g + 1,
                       current.g + 1 + heuristic(next_loc, next_time), next_time});
        }
    }
    return map_size;
}

bool Simulation::central_assignment() const {
    return config.assign_method == AM_CENTRAL_HUNGARIAN ||
           config.assign_method == AM_CENTRAL_HUNGARIAN_FIXED;
}

void Simulation::assign_central_hungarian() {
    // Undo the matches that were not picked up (CENTRAL-fixed keeps them).
    const bool keep_matches = config.assign_method == AM_CENTRAL_HUNGARIAN_FIXED;
    auto unmatch = [](Agent& agent) {
        agent.task_sequence.clear();
        agent.status = AG_FREE;
        agent.current_task = -1;
        agent.current_goal_index = 0;
    };
    for (Agent& agent : agents) {
        agent.parking_endpoint = -1;
        if (agent.status == AG_CARRYING) continue;
        if (!keep_matches || agent.task_sequence.empty()) unmatch(agent);
    }
    vector<bool> held = central_carried_goals();
    const vector<int> new_carriers = central_phase1_instant_pickup(held);

    // A kept match whose task phase 1 gave to another agent is undone; the
    // other kept matches hold their goals like the tasks matched below.
    vector<char> matched(agents.size(), 0);
    for (Agent& agent : agents) {
        if (agent.status == AG_CARRYING || agent.task_sequence.empty()) continue;
        if (all_tasks[agent.task_sequence.front()].status != -1) {
            unmatch(agent);
            continue;
        }
        matched[agent.id] = 1;
        for (int goal : all_tasks[agent.task_sequence.front()].goals) held[goal] = true;
    }

    // Phase 2 matches the agents that are still not carrying (or matched to) a task.
    vector<int> agent_ids;
    for (int i = 0; i < (int)agents.size(); i++)
        if (agents[i].status != AG_CARRYING && !matched[i]) agent_ids.push_back(i);
    if (agent_ids.empty()) return;

    // Known tasks released within a trip across the map are matched too
    // (SEMI_ONLINE, OFFLINE): the agent heads there and waits at the pickup
    // for the release, if it arrives early.
    vector<Task*> candidate_tasks;
    vector<int> candidate_endpoint_ids;
    vector<char> kept_task(all_tasks.size(), 0);
    for (int i = 0; i < (int)agents.size(); i++)
        if (matched[i]) kept_task[agents[i].task_sequence.front()] = 1;
    for (Task* task : open_tasks_) {
        if (task->status != -1 || task->goals.empty() || kept_task[task->id] ||
            task->release_time > (int)cur_time_ + mapd_map.row + mapd_map.col)
            continue;

        // As in the reference AssignTasks, only the pickup is tested against
        // the goals of executing tasks and of earlier candidates in T' (the
        // delivery test is commented out there); all goals are then held.
        if (held[task->goals.front()]) continue;

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

    // Two-stage planning (Ma's thesis, Sec 6.5.2): the agents that have just
    // picked up a task are planned first, with the configured planner, against
    // every other agent's committed path; the costs below are measured against
    // the carriers' paths, theirs included. The other agents are planned after
    // this assignment. If this plan fails, they are planned with the others.
    if (!new_carriers.empty()) {
        stage_filter_.assign(agents.size(), 0);
        for (int agent_id : new_carriers) stage_filter_[agent_id] = 1;
        plan_paths(true);
        stage_filter_.clear();
    }
    vector<vector<int>> fixed_paths;
    for (int i = 0; i < (int)agents.size(); i++)
        if (agents[i].status == AG_CARRYING)
            fixed_paths.emplace_back(path_table_[i].begin(), path_table_[i].end());

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
    // 64-bit: with hundreds of agents (2 * cells - path) * scale exceeds int.
    const long scale = (long)agent_count * destination_count;
    dlib::matrix<long> costs(matrix_size, matrix_size);
    for (int row = 0; row < matrix_size; row++) {
        for (int column = 0; column < matrix_size; column++) {
            if (row >= agent_count || column >= destination_count) {
                costs(row, column) = 0;
                continue;
            }
            int agent_id = agent_ids[row];
            int endpoint_id = candidate_endpoint_ids[column];
            // A home is parking for its own agent only: another agent parked
            // there would block its owner's way home.
            if (column >= task_destination_count && config.endpoint_strategy == RETURN_TO_HOME &&
                agent_ids[column - task_destination_count] != agent_id) {
                costs(row, column) = -4L * map_size * scale;
                continue;
            }
            int path_cost = central_path_cost(
                agent_id, agents[agent_id].loc,
                mapd_map.endpoints[endpoint_id].loc, (int)cur_time_, fixed_paths,
                &vertex_reservations, (int)reservation_count,
                &last_occupation, cost_visited);
            if (column < task_destination_count)
                path_cost = max(path_cost,
                                candidate_tasks[column]->release_time - (int)cur_time_);
            if (path_cost >= map_size) path_cost = 2 * map_size;
            if (column < task_destination_count)
                costs(row, column) = (2L * map_size - path_cost) * scale;
            else
                costs(row, column) = (long)map_size * scale - path_cost;
        }
    }

    vector<long> assignment = dlib::max_cost_assignment(costs);

    for (int i = 0; i < (int)agent_ids.size(); i++) {
        Agent& agent = agents[agent_ids[i]];
        int destination = assignment[i];
        if (destination >= (int)candidate_endpoint_ids.size())
            agent.parking_endpoint = agent.loc;   // no destination left: stay
        else if (destination >= task_destination_count)
            agent.parking_endpoint = mapd_map.endpoints[candidate_endpoint_ids[destination]].loc;
        else {
            Task* task = candidate_tasks[destination];
            agent.task_sequence = {task->id};
            agent.current_task = task->id;
            agent.current_goal_index = 0;
            agent.status = AG_MOVING_TO_PICKUP;
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
    for (int rank = 0; rank < (int)assignment.agent_order.size(); rank++)
        ta_sequence_rank_[assignment.agent_order[rank]] = rank;

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
    // The old endpoints stay occupied until their agents move on to fresh
    // ones when the previous plan holds them for good: dummy paths under a
    // planner that is neither windowed (wPBS) nor refreshed periodically, and
    // a strategy that picks a new endpoint after each task. A WAIT_* agent
    // leaves only once a task needs its cell, and a periodic replan reaches
    // such a goal once the parked agent leaves: deferring there would wait
    // forever on an idle agent.
    const bool waits = config.endpoint_strategy == WAIT_OR_NEAREST_SAFE ||
                       config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
    const bool defer_old_dummy_conflicts =
        config.dummy_path && !waits && config.mapf != MAPF_wPBS &&
        !(config.replan_on & REPLAN_PERIODIC);
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

        // Square cost matrix; padding cells are unusable (INT_MIN reward). The
        // matrix is 64-bit: on the reference's int matrix (MGMAPD) dlib's
        // label arithmetic overflows on INT_MIN, which changes assignments and
        // can leave every agent on a padding cell, so that this loop never ends.
        dlib::matrix<long> cost_mat(row, row);
        for (int i = 0; i < row; i++)
            for (int j = 0; j < row; j++)
                cost_mat(i, j) = (i >= num_ag || j >= remain) ? INT_MIN
                    // maximised, so the arrival time is negated
                    : -hungarian_arrival_estimate(agents[i], *remaining_tasks[j]);
        const vector<long> assignment = dlib::max_cost_assignment(cost_mat);
        vector<Task*> assigned;
        for (int i = 0; i < num_ag; i++) {
            if (assignment[i] < remain) {
                Task* task = remaining_tasks[assignment[i]];
                agents[i].task_sequence.push_back(task->id);
                task->status = i;
                assigned.push_back(task);
            }
        }
        if (assigned.empty()) break;

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
    // As many tasks as the planner plans: one for PP, the paper's
    // configurable C-task prefix for PBS, wPBS and CBS, all for
    // PP_TASK_SEQUENCE.
    const int keep_count = planned_task_count();

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

namespace {
// The later of `time` and the latest release among `goals`: a search cannot
// finish before it, so planning horizons are measured from it.
int latest_release(int time, const vector<pair<int,int>>& goals) {
    for (const auto& goal : goals) time = max(time, goal.second);
    return time;
}
} // namespace

// ============================================================================
//  Section 4 — Task sequences -> goal sequences
//
//  The planner never sees tasks, only an ordered list of (location, earliest
//  time) goals per agent. When dummy_path is enabled, the sequence ends with a
//  selected endpoint. If that endpoint equals the delivery location, the dummy
//  path has zero length; otherwise it is the post-delivery route to the endpoint.
// ============================================================================

// The goals the assignment gave each agent: the remaining ordered goals of
// its first max_tasks tasks (the paper's task-truncation parameter: later
// tasks stay queued for a later replan), or the endpoint it parks at.
vector<vector<pair<int,int>>> Simulation::task_goal_sequences(int max_tasks) const {
    vector<vector<pair<int,int>>> goal_seqs(agents.size());
    for (int i = 0; i < (int)agents.size(); i++) {
        int current_task_count = 0;
        for (int tid : agents[i].task_sequence) {
            if (current_task_count >= max_tasks) break;
            const Task& task = all_tasks[tid];
            if (task.status != i) {   // a CENTRAL match: plan to the pickup only
                goal_seqs[i].push_back({task.goals.front(), task.release_time});
                break;
            }
            int start_goal = (tid == agents[i].current_task)
                ? agents[i].current_goal_index : 0;
            for (int g = start_goal; g < (int)task.goals.size(); g++)
                goal_seqs[i].push_back({task.goals[g], g == 0 ? task.release_time : 0});
            current_task_count++;
        }
        if (agents[i].parking_endpoint >= 0)
            goal_seqs[i].push_back({agents[i].parking_endpoint, 0});
    }
    return goal_seqs;
}

// Whether the assignment fixed the agent's final goal, so it gets no dummy
// endpoint: CENTRAL parks it, or matched it to a pickup it does not own yet.
// CENTRAL rematches every agent at every assignment, so without dummy paths a
// carrier ends at its delivery and is matched again from there.
bool Simulation::final_goal_assigned(int agent_id) const {
    const Agent& agent = agents[agent_id];
    if (agent.parking_endpoint >= 0) return true;
    if (agent.task_sequence.empty()) return false;
    return all_tasks[agent.task_sequence.front()].status != agent_id ||
           (central_assignment() && !config.dummy_path);
}

vector<vector<pair<int,int>>> Simulation::build_goal_sequences() {
    return build_goal_sequences(vector<bool>(agents.size(), true));
}

// Goal sequences of the planned agents. An agent that is not planned keeps its
// committed path, so its final endpoint stays reserved, as does a final goal
// the assignment fixed. Of the others, agents that have work choose their
// endpoints first, so idle agents are the ones pushed out to the leftover
// endpoints.
vector<vector<pair<int,int>>> Simulation::build_goal_sequences(const vector<bool>& planned) {
    int num_ag = (int)agents.size();
    vector<int> assigned_dummies(num_ag, -1);
    vector<vector<pair<int,int>>> goal_seqs = task_goal_sequences(planned_task_count());
    for (int i = 0; i < num_ag; i++)
        if (!planned[i]) assigned_dummies[i] = (int)path_table_[i][maxtime - 1];
        else if (final_goal_assigned(i)) assigned_dummies[i] = goal_seqs[i].back().first;

    for (bool busy : {true, false})
        for (int i = 0; i < num_ag; i++)
            if (planned[i] && goal_seqs[i].empty() != busy)
                append_endpoint(i, goal_seqs[i], assigned_dummies);
    return goal_seqs;
}

// How many tasks of its sequence the configured planner plans for an agent.
int Simulation::planned_task_count() const {
    switch (config.mapf) {
    case MAPF_PP_PER_TASK: return 1;
    case MAPF_PP_TASK_SEQUENCE: return INT_MAX;
    default: return max(1, config.task_sequence_limit);
    }
}

// Ends an agent's goals (its task goals) where it stays afterwards. With a
// dummy path that is the endpoint chosen by the endpoint strategy, which may
// be the last goal itself (a zero-length dummy path; the WAIT_* strategies
// stay there unless another task or agent needs the cell). Without one the
// agent stops at its last goal, or stays where it is when idle, unless that
// cell is needed; then it too moves on to the strategy's endpoint.
// `reserved` holds the other agents' endpoints and receives this one.
void Simulation::append_endpoint(int agent_id, vector<pair<int,int>>& goals,
                                 vector<int>& reserved) {
    if (final_goal_assigned(agent_id)) {   // CENTRAL fixed where it ends
        reserved[agent_id] = goals.back().first;
        return;
    }
    const int stop = goals.empty() ? (int)agents[agent_id].loc : goals.back().first;
    int endpoint = rest_endpoint(
        make_sequence_endpoint_request(agent_id, stop, reserved, true));
    if (endpoint < 0) endpoint = stop;   // no endpoint is available
    reserved[agent_id] = endpoint;
    if (config.dummy_path || goals.empty() || endpoint != stop)
        goals.push_back({endpoint, 0});
}

// --- SelectAgents() : switch on agent_selection ----------------------------
vector<bool> Simulation::select_agents(
    const vector<vector<pair<int,int>>>& task_goals) const {
    vector<bool> planned = select_agents_unstaged(task_goals);
    if (!stage_filter_.empty())
        for (int i = 0; i < (int)planned.size(); i++)
            planned[i] = planned[i] && stage_filter_[i];
    return planned;
}

vector<bool> Simulation::select_agents_unstaged(
    const vector<vector<pair<int,int>>>& task_goals) const {
    const int num_ag = (int)agents.size();
    vector<bool> planned(num_ag, true);
    switch (select_all_ ? SELECT_ALL : config.agent_selection) {
    case SELECT_ALL:
        return planned;
    case SELECT_CHANGED:
    case SELECT_CHANGED_OR_NOT_CARRYING: {
        // Cells another task still needs: an agent parked on one must move.
        // location -> the agent whose goal it is, -1 = an open task's goal,
        // -2 = the goal of several agents
        unordered_map<int, int> goal_holder;
        for (const Task* task : open_tasks_)
            for (int goal : task->goals) goal_holder[goal] = -1;
        for (int i = 0; i < num_ag; i++)
            for (const auto& goal : task_goals[i]) {
                auto holder = goal_holder.emplace(goal.first, i).first;
                if (holder->second >= 0 && holder->second != i) holder->second = -2;
            }
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
                (holder->second != -1 || !own_goal);
            // A path that stops short of the next goal (a windowed plan, say)
            // must be extended.
            bool stuck = !now.empty() && !path_reaches(i, now.front());
            planned[i] = !unchanged || blocking || stuck ||
                (config.agent_selection == SELECT_CHANGED_OR_NOT_CARRYING &&
                 agents[i].status != AG_CARRYING);
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
    request.avoid_forbidden = true;   // the rest endpoint is held for good
    request.parking_endpoints_only =
        config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
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
    request.avoid_forbidden = true;   // the rest endpoint is held for good
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

// CENTRAL's destinations (pickups and parking endpoints) are pairwise
// distinct: `taken` holds those chosen so far, which the endpoint must avoid.
// An agent that is off an endpoint, or on a taken one, must move.
DummyEndpointRequest Simulation::make_central_endpoint_request(
    int agent_id, int reference_location, const vector<int>& taken) const {
    DummyEndpointRequest request{agent_id, reference_location,
        (int)agents[agent_id].loc, agents[agent_id].initial_loc, {}};
    request.forbidden_endpoints.insert(taken.begin(), taken.end());
    request.avoid_forbidden = true;
    request.parking_endpoints_only = request.exclude_all_open_goals =
        config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
    request.relocation_required = !mapd_map.is_endpoint[reference_location] ||
        request.forbidden_endpoints.count(reference_location);
    return request;
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
    request.parking_endpoints_only = request.exclude_all_open_goals =
        config.endpoint_strategy == WAIT_OR_NEAREST_FREE_NONTASK;
    request.avoid_forbidden = config.mapf != MAPF_wPBS;
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

// Where an agent rests after its last goal (the request's reference
// location): with --dummy_path 1 the strategy's endpoint. With 0 no path
// after the last goal is planned in advance: an idle agent stays where it is
// unless another agent or task needs that cell.
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
        return endpoint_nearest_available(request);
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
// reachable endpoint that avoids open-task goals, other agents' committed
// paths and their reserved endpoints (parking endpoints only if the request
// says so).
int Simulation::endpoint_wait_or_relocate(const DummyEndpointRequest& request) {
    if (!request.relocation_required) return request.reference_location;
    // The time-space BFS reconstructs a route from a temporary agent; the real
    // path is planned afterwards.
    Agent probe = agents[request.agent_id];
    probe.loc = request.reference_location;
    return search_path2_endpoint(probe, -1, request.parking_endpoints_only,
                                 request.exclude_all_open_goals,
                                 request.reserved_endpoints);
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
        // The first endpoint wins equal-distance ties, as in the reference
        // CENTRAL's parking choice.
        if (distance < best_distance) {
            best_distance = distance;
            best_location = endpoint.loc;
        }
    }
    if (best_location >= 0) return best_location;
    // Every endpoint taken: the agent's own home, as the strict policy does,
    // unless another agent ends there; then the nearest endpoint of any kind
    // that no other agent ends at.
    auto reserved_by_other = [&](int location) {
        for (int other = 0; other < (int)request.reserved_endpoints.size(); other++)
            if (other != request.agent_id && request.reserved_endpoints[other] == location)
                return true;
        return false;
    };
    if (!reserved_by_other(request.home_location)) return request.home_location;
    for (const Endpoint& endpoint : mapd_map.endpoints) {
        if (reserved_by_other(endpoint.loc)) continue;
        int distance = endpoint.h_val[last_goal_loc] +
                       (forbidden.count(endpoint.loc) ? (int)maxtime : 0);
        if (distance <= best_distance) {
            best_distance = distance;
            best_location = endpoint.loc;
        }
    }
    return best_location >= 0 ? best_location : request.home_location;
}

// wPBS: the nearest task endpoint not chosen by another agent (skipping the
// reference location), then the nearest such parking endpoint; staying is the
// last resort. A plan that holds the endpoint for good also skips the
// endpoints others need (the request's forbidden endpoints).
int Simulation::endpoint_pairwise(const DummyEndpointRequest& request) const {
    const int last_goal_loc = request.reference_location;
    const vector<int>& reserved = request.reserved_endpoints;
    auto nearest_of_type = [&](bool task_endpoint) {
        int best_loc = -1;
        int best_dist = INT_MAX;
        for (const Endpoint& endpoint : mapd_map.endpoints) {
            if (endpoint.is_task_endpoint != task_endpoint) continue;
            if (endpoint.loc == last_goal_loc) continue;
            if (request.avoid_forbidden && request.forbidden_endpoints.count(endpoint.loc))
                continue;
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

    for (int tid : agents[agent_id].task_sequence) {
        if (idx == (int)goal_seq.size()) break;
        const Task& task = all_tasks[tid];
        int start_goal = (tid == agents[agent_id].current_task)
            ? agents[agent_id].current_goal_index : 0;

        vector<pair<int,int>> group;
        for (int g = start_goal; g < (int)task.goals.size() && idx < (int)goal_seq.size(); g++)
            group.push_back(goal_seq[idx++]);
        if (!group.empty()) groups.push_back(group);
    }

    if (idx < (int)goal_seq.size()) groups.push_back({goal_seq[idx]});   // the dummy
    return groups;
}

// ============================================================================
//  Section 5 — MAPF (component 5): CBS, TA-Hybrid, PP, PBS
// ============================================================================

// --- CBS ------------------------------------------------------------------
// Conflict-based search over the selected agents' goal sequences, the same
// input PBS plans: each agent's first task_sequence_limit tasks and its
// endpoint. Each high-level node plans one agent with the configured
// single-agent search under the node's constraints, avoiding the agents that
// keep their paths (cbs.cpp holds the high level). A focal weight above 1.0
// makes the high level bounded-suboptimal. A failed search reports it;
// path_planning() then replans every agent, and falls back to prioritized
// planning.
bool Simulation::path_planning_cbs() {
    pbs_build_context();
    vector<int> agent_ids;
    for (int i = 0; i < pbs_.num_ag; i++)
        if (pbs_.planned[i]) agent_ids.push_back(i);
    if (reference_ecbs_applies(agent_ids)) return path_planning_reference_ecbs(agent_ids);

    const int now = (int)cur_time_;
    auto absolute = [now](const vector<tuple<int,int,int>>& relative,
                          const vector<vector<int>>& others = {}) {
        vector<tuple<int,int,int>> constraints = relative;
        for (auto& constraint : constraints) get<2>(constraint) += now;
        return CBSConstraintTable(constraints, others, now);
    };
    const vector<vector<int>> no_old_paths;
    CBSLowLevel low_level;
    low_level.find_path = [&](int k, const vector<tuple<int,int,int>>& relative,
                              const vector<vector<int>>& paths, vector<int>& path) {
        const int agent_id = agent_ids[k];
        vector<vector<int>> others = paths;
        others[k].clear();
        const CBSConstraintTable constraints = absolute(relative, others);
        vector<int> full = plan_single_agent({agent_id, (int)agents[agent_id].loc, now,
            pbs_.task_groups[agent_id], pbs_.fixed_paths, no_old_paths, false, INT_MAX,
            &constraints});
        if (full.empty()) return false;
        int end = (int)full.size() - 1;   // the path ends where it stops moving
        while (end > now && full[end] == full[end - 1]) end--;
        path.assign(full.begin() + now, full.begin() + end + 1);
        return true;
    };
    low_level.mdd_singletons = [&](int k, const vector<tuple<int,int,int>>& relative,
                                   int cost) {
        return cbs_mdd_singletons(agent_ids[k], absolute(relative), cost);
    };

    CBSSearch search((int)agent_ids.size(), low_level, mapd_map.col,
                     config.ecbs_focal_weight, config.cbs_high_level_expansion_limit,
                     config.cbs_conflict_selection, config.cbs_bypass,
                     config.cbs_target_reasoning, config.cbs_rectangle_reasoning);
    if (!search.run()) return false;
    for (int k = 0; k < (int)agent_ids.size(); k++)
        commit_path(agent_ids[k], search.paths[k]);
    remember_plans(pbs_.planned, pbs_.task_goals);
    start_assigned_tasks();
    return true;
}

// CBS configured as plain ECBS (earliest conflict, none of the ICBS
// improvements, reference ties) on agents that each go to one released
// endpoint and stay there is the ECBS of Ma's CENTRAL code; it then runs that
// code's port, which returns the same paths.
bool Simulation::reference_ecbs_applies(const vector<int>& agent_ids) const {
    if (config.cbs_conflict_selection != CBS_CONFLICT_EARLIEST || config.cbs_bypass ||
        config.cbs_target_reasoning || config.cbs_rectangle_reasoning ||
        !TieBreakPolicy::reference())
        return false;
    for (int agent_id : agent_ids) {
        const auto& groups = pbs_.task_groups[agent_id];
        int goals = 0;
        for (const auto& group : groups) goals += (int)group.size();
        if (goals != 1) return false;
        for (const auto& group : groups)
            for (const auto& goal : group)
                if (goal.second > (int)cur_time_ || mapd_map.ep_index(goal.first) < 0)
                    return false;
    }
    return true;
}

// Agents that carry a task come first, in the order of their tasks (the order
// the reference picks them up), then the others by index.
bool Simulation::path_planning_reference_ecbs(vector<int> agent_ids) {
    auto rank = [&](int i) {
        return agents[i].status == AG_CARRYING ? agents[i].current_task : -1; };
    stable_sort(agent_ids.begin(), agent_ids.end(), [&](int a, int b) {
        const int ra = rank(a), rb = rank(b);
        if ((ra >= 0) != (rb >= 0)) return ra >= 0;
        return ra >= 0 ? ra < rb : a < b;
    });
    vector<ecbs_reference::AgentGoal> goals;
    for (int agent_id : agent_ids) {
        for (const auto& group : pbs_.task_groups[agent_id])
            if (!group.empty()) {
                const int g = group.front().first;
                goals.push_back({(int)agents[agent_id].loc, g,
                                 &mapd_map.endpoints[mapd_map.ep_index(g)].h_val});
                break;
            }
    }
    vector<bool> free_cells(mapd_map.grid.begin(), mapd_map.grid.end());
    ecbs_reference::ECBSSearch search(free_cells, goals, pbs_.fixed_paths, (int)cur_time_,
        mapd_map.col, (int)maxtime, config.ecbs_focal_weight,
        (uint64_t)config.cbs_high_level_expansion_limit);
    if (!search.run()) return false;
    for (int k = 0; k < (int)agent_ids.size(); k++)
        commit_path(agent_ids[k], search.paths[k]);
    remember_plans(pbs_.planned, pbs_.task_goals);
    start_assigned_tasks();
    return true;
}

// ICBS MDD summary of agent_id's goal sequence (see CBSLowLevel): the
// (location, goals reached) states of every path that reaches the goals in
// order within `cost` steps and then holds the final goal. Empty when a goal
// is not an endpoint or no such path exists.
vector<bool> Simulation::cbs_mdd_singletons(int agent_id, const CBSConstraintTable& constraints,
                                            int cost) const {
    vector<pair<int,int>> goals;
    for (const auto& group : pbs_.task_groups[agent_id])
        goals.insert(goals.end(), group.begin(), group.end());
    const int n = (int)goals.size(), now = (int)cur_time_;
    const int map_size = mapd_map.row * mapd_map.col, labels = n + 1;
    if (n == 0 || now + cost >= (int)maxtime) return {};
    vector<const vector<int>*> h(n);
    for (int g = 0; g < n; g++) {
        int endpoint = mapd_map.ep_index(goals[g].first);
        if (endpoint < 0) return {};
        h[g] = &mapd_map.endpoints[endpoint].h_val;
    }
    auto remaining = [&](int loc, int label) {   // distance through the goals left
        long d = 0;
        for (int g = label; g < n; g++)
            d += (*h[g])[g == label ? loc : goals[g - 1].first];
        return d;
    };
    const int final_goal = goals.back().first;
    if (constraints.holdable_from(final_goal) > now + cost) return {};
    for (const vector<int>& path : pbs_.fixed_paths)
        for (int t = now + cost; t < (int)maxtime; t++)
            if (path[t] == final_goal) return {};
    auto blocked = [&](int from, int to, int t) {   // t: relative arrival time
        if (!mapd_map.grid[to] || constraints.blocks(from, to, now + t)) return true;
        for (const vector<int>& path : pbs_.fixed_paths)
            if (path[now + t] == to ||
                (path[now + t] == from && path[now + t - 1] == to)) return true;
        return false;
    };
    // One label per timestep, as the searches advance it.
    auto advance = [&](int loc, int label, int t) {
        return label < n && loc == goals[label].first && now + t >= goals[label].second
            ? label + 1 : label;
    };
    const int moves[5] = {0, -mapd_map.col, 1, mapd_map.col, -1};
    auto successors = [&](int loc, int t, auto visit) {
        for (int move : moves) {
            int next = loc + move;
            if (next < 0 || next >= map_size ||
                abs(next % mapd_map.col - loc % mapd_map.col) > 1 || blocked(loc, next, t + 1))
                continue;
            visit(next);
        }
    };

    vector<vector<int>> layers(cost + 1);   // states loc * labels + label
    vector<int> seen(map_size * labels, -1);
    int start = agents[agent_id].loc;
    layers[0].push_back(start * labels + advance(start, 0, 0));
    uint64_t work = 0;
    for (int t = 0; t < cost; t++)
        for (int state : layers[t]) {
            RuntimeDeadline::check_periodic(work++, "CBS MDD construction");
            int loc = state / labels, label = state % labels;
            successors(loc, t, [&](int next) {
                int next_label = advance(next, label, t + 1);
                int next_state = next * labels + next_label;
                if (seen[next_state] == t + 1 || remaining(next, next_label) > cost - t - 1)
                    return;
                seen[next_state] = t + 1;
                layers[t + 1].push_back(next_state);
            });
        }

    vector<bool> singletons(cost + 1, false), kept(map_size * labels, false);
    int goal_state = final_goal * labels + n;
    if (seen[goal_state] != cost && !(cost == 0 && layers[0][0] == goal_state)) return {};
    kept[goal_state] = true;
    singletons[cost] = true;
    for (int t = cost - 1; t >= 0; t--) {
        vector<int> layer;
        for (int state : layers[t]) {
            int loc = state / labels, label = state % labels;
            bool keep = false;
            successors(loc, t, [&](int next) {
                keep = keep || kept[next * labels + advance(next, label, t + 1)];
            });
            if (keep) layer.push_back(state);
        }
        if (layer.empty()) return {};
        for (int state : layers[t + 1]) kept[state] = false;
        for (int state : layer) kept[state] = true;
        singletons[t] = all_of(layer.begin(), layer.end(), [&](int state) {
            return state / labels == layer.front() / labels; });
    }
    return singletons;
}

// Commit a path that starts now; the agent then holds its last cell, and
// finish_time is its arrival there.
void Simulation::commit_path(int agent_id, const vector<int>& path) {
    for (unsigned int t = cur_time_; t < maxtime; t++) {
        unsigned int step = t - cur_time_;
        int location = step < path.size() ? path[step] : path.back();
        path_table_[agent_id][t] = location;
        agents[agent_id].path[t] = location;
    }
    agents[agent_id].finish_time = cur_time_ + path.size() - 1;
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

    // Where an agent rests after reaching `location` (rest_endpoint: the
    // strategy's endpoint, or with --dummy_path 0 `location` itself unless it
    // is needed); one that `must_move` leaves `location`. Every other agent's
    // path ends at its parking endpoint, which stays taken.
    auto parking_of = [&](int agent_id, int location, bool must_move) {
        vector<int> reserved(agent_count, -1);
        for (int other = 0; other < agent_count; other++)
            if (other != agent_id) reserved[other] = (int)agents[other].path[horizon - 1];
        DummyEndpointRequest request =
            make_sequence_endpoint_request(agent_id, location, reserved, false);
        if (must_move) {
            request.relocation_required = true;
            request.forbidden_endpoints.insert(location);
        }
        return rest_endpoint(request);
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
    // time `start` to its parking endpoint that avoids `constraints`. A
    // re-routed one (`must_move`) leaves that location, where it collided.
    auto plan_dummy_path = [&](int agent_id, int start,
                               const vector<vector<int>>& constraints, bool must_move) {
        vector<unsigned int>& path = agents[agent_id].path;
        int location = path[start];
        int parking = parking_of(agent_id, location, must_move);
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
                                 others, true))
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
            last_goal_time_[agent.id] = now;
            picked_up = true;
            if (task.goals.size() == 1) {   // a one-goal task is done here
                task.completion_time = now;
                task.status = INT_MAX;
                task.ag = nullptr;
                task.delivering = false;
                agent.task_sequence.pop_front();
                agent.status = AG_FREE;
                agent.current_task = agent.task_sequence.empty() ? -1 : agent.task_sequence.front();
                agent.current_goal_index = 0;
                new_available_agent_ = true;
            }
        }
        return picked_up;
    };

    // The configured single-agent search (component 6) for the agent's path
    // through `goal` (not before `release`) on to `parking`, avoiding
    // `constraints` and the CBS constraints `cbs` (a vertex constraint at its
    // time, an edge constraint on the move arriving at it); the path is written
    // to probe.path. Returns the goal arrival, or -1. SEQ_STA is TA-Hybrid's
    // own search (goal-optimal first if `goal_optimal_first`, as in the
    // reference); the other searches run through plan_single_agent.
    auto search_through = [&](Agent& probe, int goal, int release, int parking,
                              const vector<vector<int>>& constraints,
                              const vector<tuple<int,int,int>>& cbs,
                              bool goal_optimal_first) {
        if (config.single_agent == SA_SEQ_STA) {
            const vector<int>& goal_h = mapd_map.endpoints[mapd_map.ep_index(goal)].h_val;
            const vector<int>& parking_h =
                mapd_map.endpoints[mapd_map.ep_index(parking)].h_val;
            int arrival = -1;
            if (goal_optimal_first)
                arrival = astar_with_dummy(probe, probe.loc, now, goal, parking, goal_h,
                                           parking_h, constraints, release, true, cbs);
            if (arrival < 0)
                arrival = astar_with_dummy(probe, probe.loc, now, goal, parking, goal_h,
                                           parking_h, constraints, release, false, cbs);
            return arrival;
        }
        vector<tuple<int,int,int>> departures = cbs;   // the table times an edge by its departure
        for (auto& constraint : departures)
            if (get<1>(constraint) >= 0) get<2>(constraint)--;
        const CBSConstraintTable table(departures);
        const vector<vector<pair<int,int>>> goal_groups = {{{goal, release}}, {{parking, 0}}};
        const vector<vector<int>> no_old_paths;
        const vector<int> path = plan_single_agent({probe.id, (int)probe.loc, now, goal_groups,
            constraints, no_old_paths, false, INT_MAX, cbs.empty() ? nullptr : &table});
        for (int t = max(now, release); t < (int)path.size() && t < horizon; t++)
            if (path[t] == goal) {
                for (int time = now; time < horizon; time++)
                    probe.path[time] = path[min(time, (int)path.size() - 1)];
                return t;
            }
        return -1;
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

        // Each carrier's parking endpoint, chosen in group order: not where
        // another agent parks (its path's end) or a carrier before it chose.
        vector<int> group_parking(group1.size());
        {
            vector<int> reserved(agent_count, -1);
            for (int other = 0; other < agent_count; other++)
                reserved[other] = (int)agents[other].path[horizon - 1];
            for (int index = 0; index < (int)group1.size(); index++) {
                const int agent_id = group1[index];
                const Agent& agent = agents[agent_id];
                const int goal = all_tasks[agent.current_task].goals[agent.current_goal_index];
                reserved[agent_id] = -1;
                group_parking[index] = rest_endpoint(
                    make_sequence_endpoint_request(agent_id, goal, reserved, false));
                reserved[agent_id] = group_parking[index];
            }
        }

        auto low_level = [&](int group_index,
                             const vector<tuple<int,int,int>>& constraints,
                             vector<int>& path) {
            int agent_id = group1[group_index];
            Agent probe = agents[agent_id];
            const Task& task = all_tasks[probe.current_task];
            int goal = task.goals[probe.current_goal_index];
            int goal_endpoint = mapd_map.ep_index(goal);
            int parking = group_parking[group_index];
            int parking_endpoint = mapd_map.ep_index(parking);
            if (goal_endpoint < 0 || parking_endpoint < 0)
                throw runtime_error(
                    "TA-Hybrid Group 1 received a non-endpoint goal");

            int arrival = search_through(probe, goal, 0, parking, fixed_paths,
                                         constraints, true);
            if (arrival < 0 && parking != agents[agent_id].initial_loc)
                arrival = search_through(probe, goal, 0, agents[agent_id].initial_loc,
                                         fixed_paths, constraints, true);
            if (arrival < 0) return -1;
            path.assign(probe.path.begin(), probe.path.end());
            return arrival;
        };

        auto detect_conflict = [&](HybridCBSNode& node) {
            node.conflicts = 0;
            node.first_agent = -1;
            int earliest = INT_MAX;
            uint64_t scanned = 0;
            // After `settle` every path stands still, so a pair conflicts at
            // each later step exactly when it shares a cell at `settle`.
            int settle = now;
            for (const vector<int>& path : node.paths)
                for (int t = horizon - 1; t > settle; t--)
                    if (path[t] != path[t - 1]) { settle = t; break; }
            for (int first = 0; first < (int)group1.size(); first++)
                for (int second = first + 1;
                     second < (int)group1.size(); second++) {
                    if (settle + 1 < horizon &&
                        node.paths[first][settle] == node.paths[second][settle])
                        node.conflicts += horizon - 1 - settle;
                    for (int time = now; time <= settle && time < horizon; time++) {
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
                }
        };

        // When the CBS fails, the carriers are planned one at a time, each
        // avoiding the new paths of those before it and the current paths of
        // the rest; one that cannot be planned keeps its current path.
        auto plan_in_order = [&]() {
            HybridCBSNode* result = new HybridCBSNode();
            result->paths.resize(group1.size());
            result->arrivals.assign(group1.size(), -1);
            result->constraints.resize(group1.size());
            for (int index = 0; index < (int)group1.size(); index++) {
                const size_t base = fixed_paths.size();
                for (int other = 0; other < (int)group1.size(); other++) {
                    if (other == index) continue;
                    if (result->arrivals[other] >= 0) fixed_paths.push_back(result->paths[other]);
                    else fixed_paths.emplace_back(agents[group1[other]].path.begin(),
                                                  agents[group1[other]].path.end());
                }
                result->arrivals[index] = low_level(index, {}, result->paths[index]);
                fixed_paths.resize(base);
            }
            return result;
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
            if (root->arrivals[index] < 0) break;
            root->cost = max(root->cost,
                ta_sequence_cost(group1[index], root->arrivals[index]));
        }
        allocated.push_back(root);
        if (find(root->arrivals.begin(), root->arrivals.end(), -1) == root->arrivals.end()) {
            detect_conflict(*root);
            open.push(root);
        }

        HybridCBSNode* solution = nullptr;
        int expanded = 0;
        // Every node holds the whole group's paths: bound the search to about
        // 2 GB of them (the default runs on kiva expand at most ~50 nodes).
        const long node_ints = (long)group1.size() * horizon;
        const int expansion_limit = (int)max(1000L, min(100000L, 250000000L / node_ints));
        while (!open.empty()) {
            RuntimeDeadline::check_periodic(
                (uint64_t)expanded, "TA-Hybrid Group 1 CBS");
            HybridCBSNode* current = open.top();
            open.pop();
            if (current->first_agent < 0) {
                solution = current;
                break;
            }
            if (++expanded > expansion_limit)
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
            solution = plan_in_order();
            allocated.push_back(solution);
        }
        for (int index = 0; index < (int)group1.size(); index++) {
            int agent_id = group1[index];
            if (solution->arrivals[index] < 0) continue;   // keeps its current path
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

    // An idle agent (no tasks left) whose path ends on a cell another agent
    // or task needs moves off it to its rest endpoint, as the other planners'
    // agent selection moves blocking agents; no group plans it otherwise.
    for (int agent_id = 0; agent_id < agent_count; agent_id++) {
        const Agent& agent = agents[agent_id];
        if (!agent.task_sequence.empty() || agent.status == AG_CARRYING) continue;
        vector<int> reserved(agent_count, -1);
        vector<vector<int>> others;
        for (int other = 0; other < agent_count; other++)
            if (other != agent_id) {
                reserved[other] = (int)agents[other].path[horizon - 1];
                others.emplace_back(agents[other].path.begin(), agents[other].path.end());
            }
        if (make_sequence_endpoint_request(agent_id, (int)agent.path[horizon - 1], reserved, false)
                .relocation_required)
            plan_dummy_path(agent_id, now, others, true);
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
                int parking = parking_of(agent_id, pickup, false);
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
                    agent_id, arrival[agent_id], constraints, false)
                    ? arrival[agent_id] : -1;
                if (result < 0) {
                    // Reference GoHome retries the flow with a later pickup
                    // hold time. The equivalent local fallback is to replan
                    // this agent's complete current->pickup->home path against
                    // the already fixed paths, allowing STA* to delay pickup
                    // arrival as much as necessary.
                    result = search_through(agents[agent_id], pickup, task.release_time,
                                            parking, constraints, {}, false);
                    if (result < 0 && parking != agents[agent_id].initial_loc)
                        result = search_through(agents[agent_id], pickup, task.release_time,
                                                agents[agent_id].initial_loc, constraints,
                                                {}, false);
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
                int parking = parking_of(agent_id, pickup, false);

                vector<vector<int>> constraints = fixed_paths;
                for (const vector<int>& path : planned_paths)
                    constraints.push_back(path);
                for (int other : subgroup)
                    if (other != agent_id && !planned[other])
                        constraints.emplace_back(
                            maxtime, agents[other].initial_loc);

                Agent probe = agents[agent_id];
                int result = search_through(probe, pickup, task.release_time, parking,
                                            constraints, {}, false);
                if (result < 0 && parking != agents[agent_id].initial_loc)
                    result = search_through(probe, pickup, task.release_time,
                                            agents[agent_id].initial_loc, constraints,
                                            {}, false);
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

// The last time a constraint path or vertex constraint occupies `cell` (-1 if
// none): an agent can stay there for good from then on.
int last_occupied(const vector<vector<int>>& constraint_paths,
                  const vector<tuple<int,int,int>>& cbs_constraints, int cell) {
    int last = -1;
    for (const vector<int>& constraint : constraint_paths)
        for (int t = (int)constraint.size() - 1; t > last; t--)
            if (constraint[t] == cell) { last = t; break; }
    for (const auto& constraint : cbs_constraints)
        if (get<1>(constraint) < 0 && get<0>(constraint) == cell)
            last = max(last, get<2>(constraint));
    return last;
}
}

// Space-time A* whose accepted path reaches goal_loc and then the selected
// endpoint. The second leg is the dummy path and may have zero length.
// hold_from is last_occupied() of the endpoint, computed here if not given.
int Simulation::astar_with_dummy(
        Agent& agent, int start_loc, int start_time,
        int goal_loc, int endpoint_loc,
        const vector<int>& goal_h, const vector<int>& endpoint_h,
        const vector<vector<int>>& constraint_paths,
        int release_time, bool goal_optimal,
        const vector<tuple<int,int,int>>& cbs_constraints, int stop_time,
        int hold_from, const CBSConstraintTable* table) {
    const int map_size = mapd_map.row * mapd_map.col;
    const int max_time = (int)maxtime;
    const int relative_release = max(0, release_time - start_time);
    const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    priority_queue<TADummyNode*, vector<TADummyNode*>, CompareTADummyNode> open;
    deque<TADummyNode> nodes;               // stable addresses
    unordered_set<unsigned long long> seen;

    // Before the goal, h also covers the wait for its release.
    int initial_h = goal_h[start_loc];
    if (initial_h == INT_MAX) initial_h = 0;
    initial_h = max(initial_h, relative_release);
    nodes.emplace_back(start_loc, 0, initial_h, start_time, nullptr, false, 0);
    seen.insert((unsigned int)start_loc);
    open.push(&nodes.back());

    // The endpoint can be held from the last time a constraint occupies it.
    if (hold_from == INT_MIN)
        hold_from = last_occupied(constraint_paths, cbs_constraints, endpoint_loc);
    // Occupied at the horizon (another agent stays there): it never can be.
    if (hold_from >= max_time - 1) return -1;
    // As MLA*, search at most 512 steps past the latest of the start, the
    // goal's release and the endpoint's hold time.
    const int horizon = min(max_time, max({start_time, release_time, hold_from}) + 512);

    // Write the path to `last` (held afterwards) into agent.path; returns the
    // goal arrival, or stop_time when a windowed search ends before the goal.
    auto finish = [&](TADummyNode* last) {
        vector<int> path;
        for (TADummyNode* node = last; node; node = node->parent)
            path.push_back(node->location);
        reverse(path.begin(), path.end());

        for (int offset = 0; offset < (int)path.size(); offset++) {
            int t = start_time + offset;
            if (t < max_time) agent.path[t] = path[offset];
        }
        for (int t = start_time + path.size(); t < max_time; t++)
            agent.path[t] = last->location;

        return last->visited_goal ? start_time + last->goal_length - 1 : stop_time;
    };

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

        if (current->visited_goal && current->location == endpoint_loc &&
            current->timestep >= hold_from)
            return finish(current);
        // A windowed search ends at stop_time; the agent then holds its cell.
        if (current->timestep >= stop_time) return finish(current);

        if (current->timestep >= horizon - 1) continue;

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
            if (table && table->blocks(current->location, next_location, next_time))
                constrained = true;
            for (const auto& constraint : cbs_constraints) {
                if (constrained) break;
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
            if (!current->visited_goal) next_h = max(next_h, relative_release - next_g);

            unsigned long long key =
                (unsigned long long)next_location +
                (unsigned long long)next_g * map_size * 2ULL +
                (current->visited_goal ? (unsigned long long)map_size : 0ULL);
            if (seen.count(key) || next_g >= max_time - start_time)
                continue;

            nodes.emplace_back(next_location, next_g, next_h, next_time, current,
                               current->visited_goal, current->goal_length);
            seen.insert(key);
            open.push(&nodes.back());
        }
    }
    return -1;
}

// --- Prioritized planning (PP, PP_TASK_SEQUENCE) ---------------------------
// The selected agents are planned one at a time, each against the paths of the
// agents planned before it and the committed paths of the others. PP plans
// each agent's next task, active agents first. PP_TASK_SEQUENCE plans each
// agent's whole task sequence and, at every priority level, tentatively plans
// every remaining agent and takes the one that finishes its tasks last
// (TA-Prioritized's rule; ties go to the sequence that comes first in the LKH3
// tour, else to the lower agent id). An agent that cannot be planned keeps its
// committed path, which every agent planned before it has avoided, so the
// joint plan stays collision-free; the planner then reports the failure.
// A retry pass (pp_retry_) plans pp_first_ first, and each agent only against
// the agents planned before it and those not selected: the agents still to be
// planned move out of its way (two agents standing on each other's next goal
// cannot both avoid the other's committed path).
bool Simulation::path_planning_pp() {
    const bool whole_sequence = config.mapf == MAPF_PP_TASK_SEQUENCE;
    const int num_agents = (int)agents.size();
    const int max_time = (int)maxtime;
    const vector<vector<int>> no_old_paths;

    const vector<vector<pair<int,int>>> task_goals = task_goal_sequences(planned_task_count());
    const vector<bool> selected = select_agents(task_goals);

    // Every agent's current path (committed, or planned in this call) and endpoint.
    vector<vector<int>> paths(num_agents);
    vector<int> endpoints(num_agents, -1);
    vector<int> remaining;
    for (int i = 0; i < num_agents; i++) {
        paths[i].assign(path_table_[i].begin(), path_table_[i].end());
        if (!selected[i]) endpoints[i] = paths[i][max_time - 1];
        else if (final_goal_assigned(i)) endpoints[i] = task_goals[i].back().first;
        if (selected[i]) remaining.push_back(i);
    }
    stable_sort(remaining.begin(), remaining.end(), [&](int a, int b) {
        return agents[a].task_sequence.empty() < agents[b].task_sequence.empty();
    });
    vector<bool> planned(num_agents, false);
    deque<int> first;   // planned before the others, in this order
    if (pp_retry_)
        for (int agent_id : pp_first_)
            if (selected[agent_id]) first.push_back(agent_id);

    struct Plan {
        int agent;
        vector<int> path;   // empty when the search failed
        int endpoint;
        int finish;         // when the path completes the agent's task goals
    };
    auto plan = [&](int agent_id) {
        vector<pair<int,int>> goals = task_goals[agent_id];
        vector<int> reserved = endpoints;
        append_endpoint(agent_id, goals, reserved);
        const vector<vector<pair<int,int>>> task_groups =
            split_into_task_groups(agent_id, goals);
        const int loc = (int)agents[agent_id].loc;

        // The constraints are the other agents' paths: set the agent's own
        // aside (their order does not matter to the searches). A retry pass
        // also sets aside the agents still to be planned.
        vector<int> aside;
        for (int other = 0; other < num_agents; other++)
            if (other == agent_id || (pp_retry_ && selected[other] && !planned[other]))
                aside.push_back(other);
        vector<vector<int>> set_aside(aside.size());
        for (int k = (int)aside.size() - 1; k >= 0; k--) {
            swap(paths[aside[k]], paths.back());
            set_aside[k] = std::move(paths.back());
            paths.pop_back();
        }
        Plan result{agent_id, plan_single_agent({agent_id, loc, (int)cur_time_, task_groups,
                                                 paths, no_old_paths, false}),
                    reserved[agent_id], (int)cur_time_};
        for (int k = 0; k < (int)aside.size(); k++) {
            paths.push_back(std::move(set_aside[k]));
            swap(paths[aside[k]], paths.back());
        }

        if (!result.path.empty())
            for (const auto& goal : task_goals[agent_id])
                while (result.finish < max_time &&
                       !(result.path[result.finish] == goal.first &&
                         result.finish >= goal.second))
                    result.finish++;
        return result;
    };

    while (!remaining.empty()) {
        RuntimeDeadline::check("prioritized planning");
        const bool forced = !first.empty();
        if (forced) {
            remaining.erase(find(remaining.begin(), remaining.end(), first.front()));
            remaining.insert(remaining.begin(), first.front());
            first.pop_front();
        }
        Plan next = plan(remaining.front());
        if (whole_sequence && !forced)
            for (size_t k = 1; k < remaining.size(); k++) {
                Plan candidate = plan(remaining[k]);
                if (candidate.path.empty()) continue;
                if (next.path.empty() || candidate.finish > next.finish ||
                    (candidate.finish == next.finish &&
                     ta_sequence_rank_[candidate.agent] < ta_sequence_rank_[next.agent]))
                    next = std::move(candidate);
            }
        if (next.path.empty()) {
            // This agent (PP), or every remaining agent, none of which could be
            // planned (PP_TASK_SEQUENCE), keeps its committed path.
            const size_t failed = whole_sequence ? remaining.size() : 1;
            for (size_t k = 0; k < failed; k++)
                endpoints[remaining[k]] = paths[remaining[k]][max_time - 1];
            remaining.erase(remaining.begin(), remaining.begin() + failed);
            continue;
        }

        const int agent_id = next.agent;
        paths[agent_id] = std::move(next.path);
        endpoints[agent_id] = next.endpoint;
        planned[agent_id] = true;
        remaining.erase(find(remaining.begin(), remaining.end(), agent_id));
        for (int t = (int)cur_time_; t < max_time; t++) {   // keep the executed history
            path_table_[agent_id][t] = paths[agent_id][t];
            agents[agent_id].path[t] = paths[agent_id][t];
        }
        agents[agent_id].finish_time = path_end_time(agent_id);
    }
    remember_plans(planned, task_goals);
    start_assigned_tasks();
    return planned == selected;
}

// An agent that just received work starts moving toward its pickup.
void Simulation::start_assigned_tasks() {
    for (Agent& agent : agents)
        if (!agent.task_sequence.empty() && agent.status == AG_FREE) {
            agent.status = AG_MOVING_TO_PICKUP;
            agent.current_task = agent.task_sequence.front();
            agent.current_goal_index = 0;
        }
}

// --- PBS: depth-first search over partial priority orders. Each node fixes
// "agent a yields to agent b"; the low level then replans the yielding agent
// against the paths of everyone who outranks it.
bool Simulation::path_planning_pbs() {
    const bool solved = PBSPlanner(*this).solve();
    start_assigned_tasks();
    return solved;
}

// --- per-solve context ------------------------------------------------------
void Simulation::pbs_build_context() {
    pbs_.num_ag = (int)agents.size();
    pbs_.max_t = (int)maxtime;
    pbs_.task_goals = task_goal_sequences(planned_task_count());
    pbs_.planned = select_agents(pbs_.task_goals);
    pbs_.goal_seqs = build_goal_sequences(pbs_.planned);
    pbs_.fixed_paths.clear();
    for (int i = 0; i < pbs_.num_ag; i++)
        if (!pbs_.planned[i])
            pbs_.fixed_paths.emplace_back(path_table_[i].begin(), path_table_[i].end());

    pbs_.task_groups.assign(pbs_.num_ag, {});
    for (int i = 0; i < pbs_.num_ag; i++)
        pbs_.task_groups[i] = split_into_task_groups(i, pbs_.goal_seqs[i]);

    // Goals released in the future move every horizon below with them.
    pbs_.last_release = (int)cur_time_;
    for (const auto& goals : pbs_.goal_seqs)
        pbs_.last_release = latest_release(pbs_.last_release, goals);
    pbs_.path_horizon = min(pbs_.max_t, pbs_.last_release + 1200);
    pbs_.window = config.mapf == MAPF_wPBS ? config.wpbs_replan_window : INT_MAX;

    // The low-level search is capped 512 steps past the latest release, so
    // every committed plan is active until then and constant afterwards.
    // Sizing the per-node path tables to that window (instead of the full
    // horizon) shrinks every node copy and the conflict scan; the constant tail
    // is restored on commit. The tables also cover every committed move.
    int last_move = (int)cur_time_;
    for (int i = 0; i < pbs_.num_ag; i++)
        last_move = max(last_move, (int)path_end_time(i));
    pbs_.work_len = min(pbs_.max_t, max(pbs_.last_release + 512 + 16, last_move + 1));

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
    int cap = min(pbs_.path_horizon, pbs_.last_release + 513);

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
        const int sipp_horizon = min(pbs_.path_horizon, pbs_.last_release + 300);
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
    vector<int> path = plan_single_agent({agent_id, (int)agents[agent_id].loc, (int)cur_time_,
                                          pbs_.task_groups[agent_id], cons_paths, old_paths,
                                          use_old && !windowed,
                                          windowed ? (int)cur_time_ + pbs_.window : INT_MAX});
    // A path still moving beyond the node tables cannot be checked: not found.
    for (int t = pbs_.work_len; t < (int)path.size(); t++)
        if (path[t] != path[t - 1]) return {};
    return path;
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
        if (config.single_agent != SA_MLA_SEQUENCE &&
            config.single_agent != SA_MLSIPP_SEQUENCE) {
            // The other searches do not read the shared table: they get the
            // old paths themselves.
            old_buf = pbs_others_old_paths(i);
        } else if (config.single_agent == SA_MLSIPP_SEQUENCE) {
            // SIPP also receives the old paths for its final-goal holding-time
            // calculation; vertex/edge lookups still use the shared tables.
            int trunc = min(pbs_.path_horizon, pbs_.last_release + 1000);
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

// Write a node's (work_len-bounded) paths into the committed state, extending
// each agent's final cell out to the full horizon.
void Simulation::pbs_commit(const PBSNode* node) {
    for (int i = 0; i < pbs_.num_ag; i++) {
        if (!pbs_.planned[i]) continue;   // keeps its committed path
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

    // A plan that still has conflicts is not committed: every agent keeps its
    // committed path, and path_planning() handles the failure.
    const bool solved = best_node->conflicts.empty();
    if (solved) {
        pbs_commit(best_node);
        remember_plans(pbs_.planned, pbs_.task_goals);
    }

    for (PBSNode* n : all_nodes) delete n;
    pbs_clear_shared_old_table();
    return solved;
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
// other agent's committed path; a TPTS steal hides the displaced agent. As in
// the reference token passing (one A* to the pickup, then one to the
// delivery), the task's goals are planned one at a time, each reached when
// the agent could stay there for good. The path is committed only if it is
// collision-free and its final goal can be held forever, the token-passing
// invariant. Returns (first-goal arrival, final-goal arrival), or (-1, -1).
pair<int,int> Simulation::plan_token_task(Agent& agent, Task& task,
                                          int hidden_agent, int first_goal_before) {
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
    const vector<vector<int>> no_old_paths;
    vector<int> path;
    int first_goal_time = -1;
    int final_goal_time = -1;
    int location = (int)agent.loc;
    int time = (int)cur_time_;
    for (const auto& goal : goals) {
        const vector<vector<pair<int,int>>> one_goal(1, {goal});
        SingleAgentRequest request{agent.id, location, time, one_goal,
                                   constraints, no_old_paths, false};
        request.wait_early = true;
        if (&goal == &goals.front()) request.arrive_before = first_goal_before;
        vector<int> segment = plan_single_agent(request);
        if (segment.empty() || segment.back() != goal.first) return {-1, -1};
        // The goal is reached where the path settles on it for good.
        int arrival = (int)segment.size() - 1;
        while (arrival > time && segment[arrival - 1] == goal.first) arrival--;
        arrival = max(arrival, goal.second);
        if (path.empty()) path = std::move(segment);
        else for (int t = time; t < (int)path.size(); t++)
            path[t] = segment[min(t, (int)segment.size() - 1)];
        if (first_goal_time < 0) {
            // A steal that does not reach the first goal earlier fails
            // whatever the later goals give, so they are not planned.
            if (arrival >= first_goal_before) return {-1, -1};
            first_goal_time = arrival;
        }
        final_goal_time = arrival;
        location = goal.first;
        // The reference starts the next search after the pickup's wait.
        time = arrival + (&goal == &goals.front() ? task.start_wait_time : 0);
    }
    if (first_goal_time < 0 || final_goal_time < 0) return {-1, -1};

    // The plan avoids every other committed path (sta_is_constrained, here
    // one path at a time, which reads the table in order), and nobody else
    // enters the final goal after the agent stays there.
    const int last = min(final_goal_time, (int)maxtime - 1);
    for (int t = (int)cur_time_ + 1; t <= last; t++)
        if (path[t] < 0 || path[t] >= (int)mapd_map.grid.size() || !mapd_map.grid[path[t]])
            return {-1, -1};
    const unsigned int final_goal = (unsigned int)goals.back().first;
    for (int other = 0; other < (int)path_table_.size(); other++) {
        if (other == agent.id || other == hidden) continue;
        const vector<unsigned int>& row = path_table_[other];
        for (int t = (int)cur_time_ + 1; t <= last; t++)
            if (row[t] == (unsigned int)path[t] ||
                (row[t - 1] == (unsigned int)path[t] && row[t] == (unsigned int)path[t - 1]))
                return {-1, -1};
        for (unsigned int t = final_goal_time + 1; t < maxtime; t++)
            if (row[t] == final_goal) return {-1, -1};
    }

    for (int t = (int)cur_time_; t < (int)path.size() && t < (int)maxtime; t++)
        agent.path[t] = path[t];   // keep the executed history
    return {first_goal_time, final_goal_time};
}

vector<int> Simulation::plan_sipp_segments(
    int agent_id, int start_loc, int start_time,
    const vector<pair<int,int>>& goals,
    const vector<vector<int>>& constraint_paths,
    const vector<vector<int>>& old_paths, bool use_old_paths, int stop_time,
    const CBSConstraintTable* constraints) {
    if (goals.empty()) return {};

    vector<int> combined(maxtime, start_loc);
    int segment_start = start_time;
    int segment_location = start_loc;

    for (int goal_index = 0; goal_index < (int)goals.size(); goal_index++) {
        vector<pair<int,int>> segment_goal = {goals[goal_index]};
        bool intermediate = goal_index + 1 < (int)goals.size();
        vector<int> segment = sipp_search_impl(
            agent_id, segment_location, segment_start, segment_goal,
            constraint_paths, old_paths, use_old_paths, intermediate, stop_time,
            constraints);
        if (segment.empty()) return {};

        int arrival = -1;
        for (int time = segment_start;
             time < (int)segment.size() && time < (int)maxtime; time++)
            if (segment[time] == goals[goal_index].first &&
                time >= goals[goal_index].second) {
                arrival = time;
                break;
            }
        // A windowed search may end before the goal; the agent then holds.
        if (arrival < 0 && stop_time == INT_MAX) return {};

        int copy_through = intermediate && arrival >= 0 ? arrival : (int)maxtime - 1;
        for (int time = segment_start; time <= copy_through; time++)
            combined[time] = segment[time];
        if (arrival < 0) break;
        segment_start = arrival;
        segment_location = goals[goal_index].first;
    }
    return combined;
}

int Simulation::search_path2_endpoint(
    Agent& agent, int target_endpoint_loc,
    bool parking_endpoints_only, bool exclude_all_open_goals,
    const vector<int>& reserved_endpoints) {

    queue<SearchNode*> open;
    map<unsigned int, SearchNode*> nodes;
    const int actions[5] = {0, 1, -1, mapd_map.col, -mapd_map.col};

    // A search for a given endpoint fails at once if another agent stays
    // there for good, and otherwise looks at most 512 steps past the time it
    // can be held (as the other searches do).
    unsigned int horizon = maxtime;
    if (target_endpoint_loc >= 0) {
        int hold_from = (int)cur_time_;
        for (int other = 0; other < (int)agents.size(); other++) {
            if (other == agent.id) continue;
            for (int t = (int)maxtime - 1; t > hold_from; t--)
                if (path_table_[other][t] == (unsigned int)target_endpoint_loc) {
                    hold_from = t;
                    break;
                }
        }
        if (hold_from >= (int)maxtime - 1) return -1;
        horizon = min(maxtime, (unsigned int)hold_from + 512);
    }

    SearchNode* start = new SearchNode(agent.loc, 0, nullptr, (int)cur_time_);
    nodes.emplace((unsigned int)agent.loc, start);
    open.push(start);

    uint64_t expanded = 0;
    while (!open.empty()) {
        RuntimeDeadline::check_periodic(
            expanded++, "endpoint Path2 search");
        SearchNode* current = open.front();
        open.pop();
        if ((unsigned int)current->timestep >= horizon - 1) continue;

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
            // The endpoint must not be reserved by another agent ...
            bool occupied = false;
            for (int other = 0; other < (int)reserved_endpoints.size(); other++)
                if (other != agent.id && reserved_endpoints[other] == current->loc)
                    occupied = true;
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
    int conflicts;   // CBS: soft conflicts with the other agents' paths
    uint64_t tie_breaker;
    MLANode* parent;

    MLANode(int l, int g, int h, int t, int gi, int c, MLANode* p)
        : loc(l), g_val(g), h_val(h), timestep(t), goal_id(gi), conflicts(c),
          tie_breaker(RandomTieBreaker::next()), parent(p) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareMLANode {
    bool operator()(const MLANode* a, const MLANode* b) const {
        if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
        if (a->conflicts != b->conflicts) return a->conflicts > b->conflicts;
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
    case SA_STA_TASK_EP:
        return plan_sta_star(request, all_goals);
    case SA_MLA_SEQUENCE:
        return mla_star_taskwise_impl(request.agent_id, request.start_loc,
                                      request.start_time, request.goal_groups,
                                      request.constraint_paths, request.old_paths,
                                      request.use_old_paths, request.stop_time,
                                      request.constraints);
    case SA_MLSIPP_SEQUENCE:
        return sipp_search_impl(request.agent_id, request.start_loc, request.start_time,
                                all_goals, request.constraint_paths, request.old_paths,
                                request.use_old_paths, false, request.stop_time,
                                request.constraints, request.wait_early);
    case SA_SIPP_SEGMENTS:
        return plan_sipp_segments(request.agent_id, request.start_loc, request.start_time,
                                  all_goals, request.constraint_paths, request.old_paths,
                                  request.use_old_paths, request.stop_time,
                                  request.constraints);
    case SA_SEQ_STA:
        return plan_sequential_sta(request, all_goals);
    default:
        throw logic_error("plan_single_agent: unsupported single_agent");
    }
}

// --- STA* (TP's and TPTS's low level) ---------------------------------------
// The reference token-passing space-time A* (COBRA's Agent::AStar), one goal
// at a time: each search reaches its goal (not before the goal's release),
// the last one at a time from which the agent can stay there for good, and
// the next search starts there. Nodes are (location, g), the first one
// generated is kept, and successors come in the reference order wait, east,
// west, south, north. The other agents' paths (and old paths when requested)
// and CBS constraints are obeyed; a windowed search ends at stop_time.
vector<int> Simulation::plan_sta_star(const SingleAgentRequest& request,
                                      const vector<pair<int,int>>& goals) {
    const int max_time = (int)maxtime;
    const int map_size = mapd_map.row * mapd_map.col;
    const int cols = mapd_map.col;
    vector<const vector<int>*> others;
    for (const vector<int>& path : request.constraint_paths) others.push_back(&path);
    if (request.use_old_paths)
        for (const vector<int>& path : request.old_paths) others.push_back(&path);
    auto at = [](const vector<int>& path, int t) {
        return path[min(t, (int)path.size() - 1)];
    };
    // The other paths' cells at each timestep, (cell, path) sorted by cell,
    // built when the search first reaches that timestep: a move is checked
    // with two lookups instead of a pass over every other path.
    const int base = request.start_time;
    vector<vector<pair<int,int>>> occupants;
    vector<char> occupants_ready;
    auto occupants_at = [&](int t) -> const vector<pair<int,int>>& {
        const int i = t - base;
        if (i >= (int)occupants.size()) {
            occupants.resize(i + 64);
            occupants_ready.resize(i + 64, 0);
        }
        if (!occupants_ready[i]) {
            occupants_ready[i] = 1;
            vector<pair<int,int>>& cells = occupants[i];
            cells.reserve(others.size());
            for (int p = 0; p < (int)others.size(); p++) cells.push_back({at(*others[p], t), p});
            sort(cells.begin(), cells.end());
        }
        return occupants[i];
    };
    auto blocked = [&](int from, int to, int t) {
        const vector<pair<int,int>>& cells = occupants_at(t);
        auto first = [&](int cell) {
            return lower_bound(cells.begin(), cells.end(), make_pair(cell, INT_MIN)); };
        auto it = first(to);
        if (it != cells.end() && it->first == to) return true;                // vertex
        for (it = first(from); it != cells.end() && it->first == from; ++it)   // swap
            if (at(*others[it->second], t - 1) == to) return true;
        return request.constraints && request.constraints->blocks(from, to, t);
    };

    vector<int> full(max_time, request.start_loc);
    int location = request.start_loc;
    int time = request.start_time;
    for (int k = 0; k < (int)goals.size() && time < request.stop_time; k++) {
        const int goal = goals[k].first;
        const int release = goals[k].second;
        const vector<int>& goal_h = distances_to(goal);
        // The last goal can be held once nothing else occupies it later.
        int hold_from = INT_MIN;
        if (k + 1 == (int)goals.size()) {
            for (const vector<int>* path : others)
                for (int t = (int)path->size() - 1; t > hold_from && t >= 0; t--)
                    if ((*path)[t] == goal) { hold_from = t; break; }
            if (request.constraints) {
                const int holdable = request.constraints->holdable_from(goal);
                if (holdable == INT_MAX) return {};
                hold_from = max(hold_from, holdable - 1);
            }
        }
        // As the reference AStar, the heuristic does not know when the goal can
        // be held from: a larger one would search far less on crowded maps
        // but changes which of the equally fast paths is taken, and TP's
        // results with it (kiva small: up to 18% worse than the original).
        auto heuristic = [&](int loc, int t) {
            return max(goal_h[loc], release - t);
        };
        if (goal_h[location] == INT_MAX || hold_from >= max_time - 1) return {};
        // As MLA*, search at most 512 steps past the latest of the start, the
        // goal's release and the time the goal can be held.
        const int horizon = min(max_time, max({time, release, hold_from}) + 512);

        heap_open_t open;
        deque<SearchNode> nodes;
        unordered_set<uint64_t> seen;
        nodes.emplace_back(location, 0, heuristic(location, time), nullptr, time);
        open.push(&nodes.back());
        seen.insert((uint64_t)location);
        const SearchNode* reached = nullptr;
        uint64_t expanded = 0;
        while (!open.empty()) {
            RuntimeDeadline::check_periodic(expanded++, "STA* low-level search");
            SearchNode* current = open.top();
            open.pop();
            current->in_openlist = false;
            // f never decreases (consistent heuristic), and a goal node's f is
            // its arrival time.
            if (k == 0 && current->timestep + current->h_val >= request.arrive_before) break;
            if ((current->loc == goal && current->timestep >= release &&
                 current->timestep >= hold_from) ||
                current->timestep >= request.stop_time) {
                reached = current;
                break;
            }
            if (current->timestep >= horizon - 1) continue;
            const int actions[5] = {0, 1, -1, cols, -cols};
            for (int action : actions) {
                const int next = current->loc + action;
                const int next_time = current->timestep + 1;
                if (next < 0 || next >= map_size || !mapd_map.grid[next] ||
                    abs(next % cols - current->loc % cols) > 1 ||
                    goal_h[next] == INT_MAX || blocked(current->loc, next, next_time))
                    continue;
                const int g = current->g_val + 1;
                if (!seen.insert((uint64_t)next + (uint64_t)g * map_size).second) continue;
                nodes.emplace_back(next, g, heuristic(next, next_time), current, next_time);
                open.push(&nodes.back());
            }
        }
        if (!reached) return {};
        for (const SearchNode* node = reached; node; node = node->parent)
            full[node->timestep] = node->loc;
        for (int t = reached->timestep + 1; t < max_time; t++) full[t] = reached->loc;
        location = reached->loc;
        time = reached->timestep;
    }
    return full;
}

// --- SEQ_STA (TA-Prioritized's low level) -----------------------------------
// One space-time A* leg per goal; each leg also reaches the final goal (the
// endpoint) and must be able to hold it there, so the agent is never cut off,
// and the next leg starts where this one reached its goal. A final goal that
// is released later is a leg of its own; without any leg the agent takes a
// dummy path from its start to the endpoint. Old paths are avoided like the
// constraint paths, and CBS constraints are obeyed (the endpoint is held only
// after the last one there).
vector<int> Simulation::plan_sequential_sta(const SingleAgentRequest& request,
                                            const vector<pair<int,int>>& goals) {
    vector<vector<int>> with_old;
    if (request.use_old_paths && !request.old_paths.empty()) {
        with_old = request.constraint_paths;
        with_old.insert(with_old.end(), request.old_paths.begin(), request.old_paths.end());
    }
    const vector<vector<int>>& constraints =
        with_old.empty() ? request.constraint_paths : with_old;
    Agent probe = agents[request.agent_id];   // receives the path
    const int endpoint = goals.back().first;
    const vector<int>& endpoint_h = distances_to(endpoint);
    int hold_from = last_occupied(constraints, {}, endpoint);   // shared by the legs
    if (request.constraints) {
        const int holdable = request.constraints->holdable_from(endpoint);
        if (holdable == INT_MAX) return {};
        hold_from = max(hold_from, holdable - 1);
    }
    int location = request.start_loc;
    int time = request.start_time;
    auto leg = [&](int goal, const vector<int>& goal_h, int release, bool goal_optimal) {
        return astar_with_dummy(probe, location, time, goal, endpoint, goal_h, endpoint_h,
                                constraints, release, goal_optimal, {}, request.stop_time,
                                hold_from, request.constraints);
    };

    const int legs = (int)goals.size() - 1 + (goals.back().second > time);
    if (legs == 0 && leg(location, endpoint_h, 0, false) < 0) return {};
    for (int k = 0; k < legs && time < request.stop_time; k++) {
        const vector<int>& goal_h = distances_to(goals[k].first);
        int arrival = leg(goals[k].first, goal_h, goals[k].second, true);
        if (arrival < 0) arrival = leg(goals[k].first, goal_h, goals[k].second, false);
        if (arrival < 0) return {};
        location = goals[k].first;
        time = arrival;
    }
    return vector<int>(probe.path.begin(), probe.path.end());
}

// BFS distances to `location`: the endpoint's table, or one computed once.
const vector<int>& Simulation::distances_to(int location) {
    int endpoint = mapd_map.ep_index(location);
    if (endpoint >= 0) return mapd_map.endpoints[endpoint].h_val;
    auto cached = distance_tables_.find(location);
    if (cached != distance_tables_.end()) return cached->second;

    const int map_size = mapd_map.row * mapd_map.col;
    vector<int>& distance = distance_tables_[location];
    distance.assign(map_size, INT_MAX);
    distance[location] = 0;
    queue<int> open;
    open.push(location);
    uint64_t expanded = 0;
    while (!open.empty()) {
        RuntimeDeadline::check_periodic(expanded++, "distance BFS");
        int u = open.front(); open.pop();
        for (int d : {1, -1, mapd_map.col, -mapd_map.col}) {
            int v = u + d;
            if (v >= 0 && v < map_size && mapd_map.grid[v] && distance[v] > distance[u] + 1) {
                distance[v] = distance[u] + 1;
                open.push(v);
            }
        }
    }
    return distance;
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
        bool use_old_paths, int stop_time,
        const CBSConstraintTable* constraints) {
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
                                           /*skip_holding=*/!is_last, stop_time,
                                           constraints);
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
                                     bool skip_holding, int stop_time,
                                     const CBSConstraintTable* constraints) {
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

    // Horizon cap: the forced endpoint hold below is capped 384 steps past the
    // latest release, so a complete path never needs more than ~512 steps past
    // it.  Bounding it here keeps the per-search constraint arrays small.
    const int last_release = latest_release(start_time, goals);
    const int search_horizon = min(max_t, last_release + 512);

    vector<const vector<int>*> h_vals_ptr(num_goals);
    for (int g = 0; g < num_goals; g++) h_vals_ptr[g] = &distances_to(goals[g].first);

    // h = the earliest completion of the remaining goals without other agents
    // (travel through them, waiting for each release), minus the current time.
    auto compute_h = [&](int loc, int time, int goal_id) -> int {
        if (goal_id >= num_goals) return 0;
        int h = (*h_vals_ptr[goal_id])[loc];
        if (h == INT_MAX) return INT_MAX;
        int finish = max(time + h, goals[goal_id].second);
        for (int g = goal_id; g < num_goals - 1; g++) {
            int d = (*h_vals_ptr[g + 1])[goals[g].first];
            if (d == INT_MAX) return INT_MAX;
            finish = max(finish + d, goals[g + 1].second);
        }
        return finish - time;
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
        if (constraints && constraints->blocks(curr_loc, next_loc, abs_t)) return true;
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
    const int holding_cap = last_release + 384;
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
        // CBS constraints are hard: the goal is held only after the last one.
        if (constraints) {
            int holdable = constraints->holdable_from(last_goal_loc);
            if (holdable >= search_horizon) return {};
            earliest_holding = max(earliest_holding, holdable);
        }
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

    int init_h = compute_h(start_loc, start_time, 0);
    if (init_h == INT_MAX) return {};

    // A real path is found in far fewer nodes than this; a search that exhausts
    // the cap has no path within the horizon (typically its goal is occupied
    // until a far-future deadline) and is abandoned cheaply. Waiting for a
    // release far ahead (SEMI_ONLINE, OFFLINE) takes a layer per step.
    const int release_wait = max(0, last_release - start_time);
    const int mla_max_nodes = (int)min(500000L, (long)map_size * (20 + release_wait));

    mla_arena_used_chunk_ = 0;
    mla_arena_used_in_chunk_ = 0;
    auto alloc_node = [&](int l, int g, int h, int t, int gi, int c, MLANode* p) -> MLANode* {
        if (mla_arena_used_chunk_ >= (int)mla_arena_chunks_.size())
            mla_arena_chunks_.push_back(::operator new(sizeof(MLANode) * (size_t)MLA_ARENA_CHUNK));
        MLANode* chunk = static_cast<MLANode*>(mla_arena_chunks_[mla_arena_used_chunk_]);
        MLANode* node = &chunk[mla_arena_used_in_chunk_];
        if (++mla_arena_used_in_chunk_ >= MLA_ARENA_CHUNK) {
            mla_arena_used_chunk_++;
            mla_arena_used_in_chunk_ = 0;
        }
        return new (node) MLANode(l, g, h, t, gi, c, p);
    };

    MLANode* root = alloc_node(start_loc, 0, init_h, start_time, 0, 0, nullptr);
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
            int next_h = compute_h(next_loc, next_t, child_gi);
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
                int conflicts = curr->conflicts +
                    (constraints ? constraints->conflicts(next_loc, next_t) : 0);
                MLANode* next_node = alloc_node(next_loc, next_g, next_h, next_t, child_gi,
                                                conflicts, curr);
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
// MLSIPP. SIPP enters every cell as early as possible, so it waits right
// before a busy cell, often in a corridor. Token passing plans one agent at a
// time against the paths planned before it, and an agent waiting in a
// corridor there blocks the agents planned after it; with wait_early, of the
// equally fast paths MLSIPP takes one that reaches the final goal as early as
// possible and waits there, stepping aside when another agent passes (as the
// reference STA* does), and otherwise waits as early as possible.
vector<int> Simulation::sipp_search_impl(
        int agent_id, int start_loc, int start_time,
        const vector<pair<int,int>>& goals,
        const vector<vector<int>>& cons_paths,
        const vector<vector<int>>& old_paths,
        bool use_old_paths,
        bool skip_holding, int stop_time,
        const CBSConstraintTable* constraints, bool wait_early) {
    vector<int> path = sipp_search_core(agent_id, start_loc, start_time, goals, cons_paths,
                                        old_paths, use_old_paths, skip_holding, stop_time,
                                        constraints, wait_early);
    if (!wait_early || path.empty() || goals.empty() || skip_holding || stop_time != INT_MAX)
        return path;
    // The time from which a padded path stays where it ends.
    auto settled = [&](const vector<int>& p) {
        int t = (int)p.size() - 1;
        while (t > start_time && p[t - 1] == p.back()) t--;
        return t;
    };
    const int arrival = settled(path);
    vector<int> reach = sipp_search_core(agent_id, start_loc, start_time, goals, cons_paths,
                                         old_paths, use_old_paths, true, INT_MAX, constraints,
                                         true);
    if (reach.empty()) return path;
    const int reached = settled(reach);
    if (reached >= arrival) return path;
    vector<int> hold = sipp_search_core(agent_id, goals.back().first, reached,
                                        {goals.back()}, cons_paths, old_paths, use_old_paths,
                                        false, INT_MAX, constraints, true);
    if (hold.empty() || settled(hold) != arrival) return path;
    for (int t = reached; t < (int)hold.size() && t < (int)reach.size(); t++) reach[t] = hold[t];
    return reach;
}

vector<int> Simulation::sipp_search_core(
        int agent_id, int start_loc, int start_time,
        const vector<pair<int,int>>& goals,
        const vector<vector<int>>& cons_paths,
        const vector<vector<int>>& old_paths,
        bool use_old_paths,
        bool skip_holding, int stop_time,
        const CBSConstraintTable* constraints, bool wait_early) {
    if (goals.empty()) return vector<int>((int)maxtime, start_loc);

    const int map_size = mapd_map.row * mapd_map.col;
    const int max_t = (int)maxtime;
    const int cols = mapd_map.col;
    const int num_goals = (int)goals.size();
    const int horizon = min(max_t, latest_release(start_time, goals) + max(1000, num_goals * 40));

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

        // A location of -1 (TA-Hybrid's task paths) constrains nothing.
        auto occupy = [&](int loc, int begin, int end) {
            if (loc < 0) return;
            touch_occupied(loc);
            occupied[loc].push_back({begin, end});
        };
        int range_start = 0;
        int range_loc = path[0];
        for (int t = 1; t <= last_change; t++) {
            if (path[t] != range_loc) {
                occupy(range_loc, range_start, t);
                range_loc = path[t];
                range_start = t;
            }
        }
        occupy(range_loc, range_start, horizon);

        if (constraint_index >= 0) {
            constraint_last_change[constraint_index] = last_change;
            constraint_tail[constraint_index] = range_loc;
        }
    };

    for (int i = 0; i < num_constraints; i++) add_path(cons_paths[i], i);
    if (constraints)
        constraints->for_each_forbidden_interval(horizon, [&](int loc, int begin, int end) {
            touch_occupied(loc);
            occupied[loc].push_back({begin, end});
        });

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
        if (constraints && constraints->blocks(from, to, timestep)) return true;
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
        if (constraints) {
            int holdable = constraints->holdable_from(final_goal);
            if (holdable >= horizon) {
                reset_scratch();
                return {};
            }
            earliest_holding = max(earliest_holding, holdable);
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

        // Successor (loc, interval) reached at `arrival`; it completes the
        // current goal there once released (the final goal once holdable).
        auto add_successor = [&](int loc, int interval, int arrival) {
            int next_goal_id = goal_id;
            if (next_goal_id < num_goals && loc == goals[next_goal_id].first &&
                arrival >= goals[next_goal_id].second) {
                if (!(next_goal_id == num_goals - 1 && arrival < earliest_holding))
                    next_goal_id++;
            }

            int h = sum_h(loc, next_goal_id);
            if (h == INT_MAX) return;
            int g = arrival - start_time;
            uint64_t key = state_key(loc, interval, next_goal_id);
            auto previous = best_arrival.find(key);
            if (previous != best_arrival.end() && previous->second <= g) return;

            best_arrival[key] = g;
            SIPPNode* node = new SIPPNode{
                loc, g, h, arrival, next_goal_id, interval,
                RandomTieBreaker::next(), current};
            open.push(node);
            nodes.push_back(node);
        };

        const int directions[4] = {1, -cols, -1, cols};
        for (int direction : directions) {
            int next_loc = current->loc + direction;
            if (next_loc < 0 || next_loc >= map_size || !mapd_map.grid[next_loc]) continue;
            if (abs(next_loc % cols - current->loc % cols) > 1) continue;

            int earliest_arrival = current->timestep + 1;
            const auto& next_intervals = get_safe_intervals(next_loc);

            if (next_intervals.empty()) {
                if (!has_occupied[next_loc] && earliest_arrival < horizon &&
                    !edge_blocked(current->loc, next_loc, earliest_arrival))
                    add_successor(next_loc, 0, earliest_arrival);
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
                add_successor(next_loc, interval, arrival);
            }
        }

        // Hard-constraint intervals are non-contiguous, so this successor is
        // normally absent; keep it for completeness if adjacent intervals occur.
        if (!current_intervals.empty() &&
            current->interval + 1 < (int)current_intervals.size()) {
            int next_interval = current->interval + 1;
            int arrival = current_intervals[next_interval].first;
            if (arrival == current_intervals[current->interval].second && arrival < horizon)
                add_successor(current->loc, next_interval, arrival);
        }

        // Standing on a goal that is not released yet: wait there for the
        // release if the cell stays safe that long.
        if (goal_id < num_goals && current->loc == goals[goal_id].first &&
            current->timestep < goals[goal_id].second &&
            goals[goal_id].second < current_interval_end)
            add_successor(current->loc, current->interval, goals[goal_id].second);
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

        // wait_early: of the paths along the same route that arrive at the
        // same time, the one that waits as early as possible: from the last
        // move back, each cell is entered as late as the previous cell stays
        // free and the move is allowed.
        auto free_during = [&](int loc, int from, int to) {   // [from, to]
            const auto& intervals = get_safe_intervals(loc);
            if (intervals.empty()) return !has_occupied[loc];
            for (const auto& interval : intervals)
                if (interval.first <= from && to < interval.second) return true;
            return false;
        };
        vector<int> cells, arrive;   // the route and its arrival offsets
        for (int i = 0; i < (int)locations.size(); i++)
            if (i == 0 || locations[i] != locations[i - 1]) {
                cells.push_back(locations[i]);
                arrive.push_back(i);
            }
        for (int j = wait_early ? (int)cells.size() - 2 : 0; j >= 1; j--) {
            for (int latest = arrive[j + 1] - 1; latest > arrive[j]; latest--) {
                const int t = start_time + latest;
                if (free_during(cells[j - 1], start_time + arrive[j - 1], t - 1) &&
                    !edge_blocked(cells[j - 1], cells[j], t)) {
                    arrive[j] = latest;
                    break;
                }
            }
        }
        for (int j = 0; j < (int)cells.size(); j++) {
            const int end = j + 1 < (int)cells.size() ? arrive[j + 1] : (int)locations.size();
            for (int i = arrive[j]; i < end; i++) locations[i] = cells[j];
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
//    * planning inside assignment (replan_on WHEN_ASSIGNED) returns to the assignment
//      whenever an agent is free (advance_time_tp);
//    * assigning or replanning at every timestep advances one step at a time;
//    * otherwise the clock jumps to the next goal arrival, task release or
//      periodic replan.
void Simulation::advance_time() {
    if (config.replan_on == 0) {
        advance_time_tp();
        return;
    }
    // The pickup trigger looks at where the agents are at every timestep.
    if (config.assign_trigger == AT_EVERY_TIMESTEP ||
        config.assign_trigger == AT_ON_AGENT_FREE_OR_AT_PICKUP ||
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
    // A planner that failed plans again at the next timestep.
    if ((config.assign_trigger ==
             AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE &&
         new_or_deferred_task_event_) || replan_pending_)
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
    pickup_event_ = false;

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

            // A plan that the assignment ends at the task's last goal (CENTRAL
            // without dummy paths) may pass that cell before the agent can
            // stay there: the goal is reached when the agent arrives to stay,
            // as the reference CENTRAL records it.
            const bool ends_at_last_goal = agents[i].task_sequence.size() == 1 &&
                final_goal_assigned(i) &&
                (int)path_table_[i][maxtime - 1] == task.goals.back();
            unsigned int final_arrival = 0;
            if (ends_at_last_goal) {
                final_arrival = maxtime - 1;
                while (final_arrival > 0 &&
                       path_table_[i][final_arrival] == path_table_[i][final_arrival - 1])
                    final_arrival--;
            }
            bool task_completed = false;
            for (unsigned int t = scan_from; t <= cur_time_ && t < maxtime; t++) {
                while (agents[i].current_goal_index < (int)task.goals.size() &&
                       (int)agents[i].path[t] == task.goals[agents[i].current_goal_index]) {
                    // Only the first goal (the pickup) is release-time constrained.
                    if (agents[i].current_goal_index == 0 && (int)t < task.release_time)
                        break;
                    if (ends_at_last_goal && t < final_arrival &&
                        agents[i].current_goal_index + 1 == (int)task.goals.size())
                        break;

                    if (agents[i].current_goal_index == 0) {
                        // A CENTRAL match is not the agent's yet: the next
                        // assignment's phase 1 picks it up if its delivery is
                        // free (the pickup is deferred until then).
                        if (task.status != i) {
                            new_or_deferred_task_event_ = true;
                            break;
                        }
                        task.ag_arrive_start = (int)t;
                        agents[i].status = AG_CARRYING;
                        open_tasks_.remove(&task);
                        pickup_event_ = true;
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

