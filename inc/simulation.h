#pragma once
// ============================================================================
//  Unified MAPD framework: the simulation and one dispatcher per component.
//  See the header of src/simulation.cpp for the component -> dispatcher map.
// ============================================================================
#include "types.h"
#include "config.h"
#include "map_loader.h"
#include "path_planners.h"
#include <functional>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <random>
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <set>
#include <stack>
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>

// Space-time A* node used by TP's two sequential low-level searches.
struct SearchNode {
    int loc;
    int g_val;
    int h_val;
    int timestep;
    std::uint64_t tie_breaker;
    SearchNode* parent;
    bool in_openlist;

    SearchNode(int location, int g, int h, SearchNode* previous, int time)
        : loc(location), g_val(g), h_val(h), timestep(time),
          tie_breaker(RandomTieBreaker::next()),
          parent(previous), in_openlist(true) {}
    SearchNode(int location, int g, SearchNode* previous, int time)
        : loc(location), g_val(g), h_val(0), timestep(time),
          tie_breaker(RandomTieBreaker::next()),
          parent(previous), in_openlist(true) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareNode {
    bool operator()(const SearchNode* lhs, const SearchNode* rhs) const {
        if (lhs->getFVal() != rhs->getFVal())
            return lhs->getFVal() > rhs->getFVal();
        if (TieBreakPolicy::reference()) return lhs->g_val <= rhs->g_val;
        if (lhs->g_val != rhs->g_val)
            return lhs->g_val < rhs->g_val;
        return lhs->tie_breaker > rhs->tie_breaker;
    }
};

typedef boost::heap::fibonacci_heap<SearchNode*,
        boost::heap::compare<CompareNode>> heap_open_t;

// ============================================================================
//  Priority graph — PBS's partial order over agents (Section 7)
//  add(from, to): `from` has LOWER priority than `to` (from yields to to).
// ============================================================================
class PriorityGraph {
public:
    void clear() { adj_.clear(); }
    void copy(const PriorityGraph& other) { adj_ = other.adj_; }
    void add(int from, int to) { adj_[from].insert(to); }

    // Is there a directed path from `from` to `to`?
    bool connected(int from, int to) const {
        if (from == to) return false;
        std::list<int> open;
        std::set<int> closed;
        open.push_back(from);
        closed.insert(from);
        while (!open.empty()) {
            int curr = open.back(); open.pop_back();
            auto it = adj_.find(curr);
            if (it == adj_.end()) continue;
            for (int next : it->second) {
                if (next == to) return true;
                if (closed.find(next) == closed.end()) {
                    open.push_back(next);
                    closed.insert(next);
                }
            }
        }
        return false;
    }

    // All agents reachable from `root` = the agents that outrank it.
    std::set<int> get_higher_priority(int root) const {
        std::list<int> open;
        std::set<int> closed;
        open.push_back(root);
        while (!open.empty()) {
            int curr = open.back(); open.pop_back();
            auto it = adj_.find(curr);
            if (it == adj_.end()) continue;
            for (int next : it->second) {
                if (closed.find(next) == closed.end()) {
                    open.push_back(next);
                    closed.insert(next);
                }
            }
        }
        return closed;
    }
private:
    std::map<int, std::set<int>> adj_;
};

// ============================================================================
//  PBS high-level node (Section 7)
// ============================================================================
typedef std::tuple<int,int,int,int,int> Conflict;   // (a1, a2, loc1, loc2, timestep)

// A PBS node's paths, one row per agent ((*paths[agent])[t] = location). A
// child shares its parent's rows and copies only the rows it re-plans.
typedef std::vector<std::shared_ptr<const std::vector<int>>> PBSPaths;

struct PBSNode {
    PriorityGraph priorities;
    PBSPaths paths;
    std::list<Conflict> conflicts;
    Conflict conflict;                     // the conflict chosen for branching
    std::pair<int,int> priority;           // (lower, higher) added by this node
    int num_collisions;
    int earliest_collision;
    int cost;

    PBSNode() : num_collisions(0), earliest_collision(INT_MAX), cost(0) {}
};

struct DummyEndpointRequest {
    int agent_id;
    int reference_location;
    int current_location;
    int home_location;
    vector<int> reserved_endpoints;
    set<int> forbidden_endpoints;
    vector<int> candidate_endpoints; // empty means every map endpoint
    bool relocation_required = false;
    bool use_safe_relocation_search = false;
    bool parking_endpoints_only = false;
    bool exclude_all_open_goals = false;
    // PAIRWISE also skips the forbidden endpoints: the plan holds the endpoint
    // for good (every planner but windowed wPBS), or CENTRAL needs pairwise
    // distinct destinations.
    bool avoid_forbidden = false;
};

// ============================================================================
//  Simulation
// ============================================================================
class Simulation {
public:
    Simulation() {}
    ~Simulation() { for (void* p : mla_arena_chunks_) ::operator delete(p); }

    // --- Driver-facing API ---
    void init(const string& map_file, const string& task_file, const MAPDConfig& config,
              const string& tour_file = "");
    void run();
    bool fullCollisionCheck(const string& alg_name) const;
    bool taskOrderCheck(const string& alg_name) const;
    void showTask() const;
    void saveOutput(const string& filepath, double runtime_ms,
                    const string& algorithm_name) const;

private:
    friend class PBSPlanner;

    // ======================================================================
    //  Instance + system state
    // ======================================================================
    MAPDConfig config;
    MAPDMap mapd_map;
    vector<Agent> agents;
    vector<Task> all_tasks;
    vector<vector<int>> task_indices_by_time;  // task ids indexed by release time

    vector<vector<unsigned int>> path_table_;  // path_table_[agent][t] = cell (committed)
    // Complete assignment pool for the selected method. For Hungarian/LNS it
    // includes released, unfinished, unassigned tasks, including suffixes
    // removed by the planned-prefix limit.
    list<Task*> open_tasks_;
    unsigned int cur_time_ = 0;                // t
    unsigned int maxtime = 0;                  // planning horizon bound
    int t_task = 0;                            // last task release time
    int last_released_time_ = -1;              // latest task-release time revealed to assignment
    int last_release_checked_ = -1;            // latest time checked for actual releases

    // ======================================================================
    //  Section 1 — Algorithm 1: the unified main loop
    // ======================================================================
    bool end() const;                           // End()
    void release_tasks();                       // T <- T u {tau_j | r_j = t}
    void task_assignment_and_path_planning();   // Computation
    void advance_time();                        // agents move, t <- t+1
    void advance_time_tp();                     // TP processes free agents before advancing

    bool any_agent_busy() const {
        for (auto& a : agents)
            if (a.status != AG_FREE && a.finish_time > cur_time_) return true;
        return false;
    }
    bool all_task_sequences_empty() const;
    void assign_batch_greedy();
    bool agent_needs_next_segment() const;
    void clamp_next_ts_to_task_release(unsigned int& next_ts) const;

    // ======================================================================
    //  Section 2 — Computation: dispatchers on the config axes
    // ======================================================================
    bool should_assign();            // test and consume assign_trigger events
    bool agent_on_open_pickup() const;
    bool nobody_on_open_tasks() const;
    void task_assignment();          // assign_method
    bool should_replan(bool assignment_triggered) const;
    void path_planning(bool assignment_triggered); // mapf
    bool replan_pending_ = false;    // a planner failed: plan again next timestep
    bool planned_in_assignment_ = false;   // REPEATED_HUNGARIAN_REAL_PATH_LNS planned: skip the planning step

    // Assignment-trigger state is separate from open_tasks_: that pool may
    // contain truncated suffixes that should not cause reassignment by
    // themselves. The task-event flag represents a new task reveal or a
    // strict-PBS task deferred for retry in the next iteration.
    void process_events();
    bool new_or_deferred_task_event_ = false;
    bool new_available_agent_ = false; // newly available for event-triggered reassignment
    bool pickup_event_ = false;        // an agent picked up a task this timestep
    unsigned int last_path_planning_time_ = 0;

    // LKH3 tour input used by the offline TA algorithms.
    string tour_file_;
    // TA-Hybrid: tour position of the sequence each agent executes (it moves
    // with the sequence when Group 2 swaps sequences), and the makespan bound
    // that limits each pickup time in the Group-2 flow.
    vector<int> ta_sequence_rank_;
    int ta_makespan_bound_ = 0;

    // ======================================================================
    //  Section 3 — Task assignment: TP / HUNGARIAN / REPEATED_HUNGARIAN_LNS
    // ======================================================================
    void assign_decoupled_greedy_step();
    bool assign_decoupled_greedy(Agent& agent);
    void assign_tpts_step();
    bool assign_tpts(Agent& agent, int depth = 0);
    void tpts_purge_picked_up_tasks();
    void assign_hbh_mla();
    void assign_ta_tsp();
    void assign_ta_hybrid();
    int ta_sequence_cost(int agent_id, int start) const;
    vector<int> ta_agents_in_sequence_order() const;
    vector<bool> central_carried_goals() const;
    vector<int> central_phase1_instant_pickup(vector<bool>& held);
    void assign_central_hungarian();
    int central_path_cost(int agent_id, int start_loc, int goal_loc,
                          int start_time, const vector<vector<int>>& constraints,
                          const vector<char>* vertex_reservations,
                          int reservation_size, const vector<int>* last_occupation,
                          unordered_map<long long, int>& visited);
    // assign_central_hungarian()'s (time, cell) table of the fixed paths, kept
    // between calls and all zero outside them.
    vector<char> central_reservations_;
    void assign_repeated_hungarian_lns();      // Hungarian, then the 1 s LNS spin
    void assign_repeated_hungarian();          //   Phase 1
    void truncate_online_task_sequences();     // keep only the executable C-task prefix
    int  hungarian_arrival_estimate(const Agent& ag, const Task& task) const;
    bool lns_has_reassignable_task() const;
    int  estimate_sequence_cost(int agent_id) const;
    void lns_destroy(vector<int>& removed_tasks);
    void lns_repair(vector<int>& removed_tasks);
    void lns_estimate_task_times(unordered_map<int, pair<int,int>>& task_times) const;
    // REPEATED_HUNGARIAN_REAL_PATH_LNS: repeated Hungarian, then LNS (random
    // destroy, the regret repair above) whose candidates are planned by the
    // configured prioritized planner and judged on the real paths. It leaves
    // the best candidate truncated and planned (planned_in_assignment_).
    void assign_real_path_lns();
    long long real_path_cost(const vector<deque<int>>& full_sequences,
                             const vector<Agent>& planned) const;
    void real_path_lns_destroy(vector<int>& removed_tasks);
    void real_path_lns_repair(vector<int>& removed_tasks);
    std::mt19937 real_path_lns_rng_;       // its random moves (seeded once, from --seed)
    // The prioritized planner's order (agent ids, highest first) while the
    // method plans its candidates; empty = the planner's own order.
    vector<int> pp_order_;
    vector<int> real_path_lns_order_;      // the order the method last chose (its next start)


    // ======================================================================
    //  Section 4 — Task sequences -> goal sequences (planner input)
    // ======================================================================
    vector<vector<pair<int,int>>> task_goal_sequences(int max_tasks) const;
    bool final_goal_assigned(int agent_id) const;
    vector<vector<pair<int,int>>> build_goal_sequences();
    vector<vector<pair<int,int>>> build_goal_sequences(const vector<bool>& planned);

    // Component 4: which agents a replan plans (switch on agent_selection),
    // and the task goals each committed path was planned for.
    vector<bool> select_agents(const vector<vector<pair<int,int>>>& task_goals) const;
    vector<bool> select_agents_unstaged(const vector<vector<pair<int,int>>>& task_goals) const;
    vector<char> stage_filter_;   // planned stage: only these agents (empty = all)
    bool path_reaches(int agent_id, const pair<int,int>& goal) const;
    void remember_plans(const vector<bool>& planned,
                        const vector<vector<pair<int,int>>>& task_goals);
    vector<vector<pair<int,int>>> planned_goals_;
    vector<bool> has_plan_;
    vector<bool> last_planned_;    // agents planned by the current replan
    bool pp_retry_ = false;        // path_planning_pp: retry pass (pp_first_ first)
    vector<int> pp_first_;
    bool select_all_ = false;      // fall back to replanning every agent
    // Set by a planner whose replan of every agent would repeat its failed plan.
    bool repeat_on_replan_all_ = false;
    bool kept_paths_conflict() const;
    bool plan_paths(bool assignment_triggered);

    DummyEndpointRequest make_token_endpoint_request(
        int agent_id, int reference_location, bool check_committed_crossings) const;
    DummyEndpointRequest make_hbh_endpoint_request(
        int agent_id, int reference_location) const;
    DummyEndpointRequest make_central_endpoint_request(
        int agent_id, int reference_location,
        const vector<int>& taken) const;
    DummyEndpointRequest make_sequence_endpoint_request(
        int agent_id, int reference_location,
        const vector<int>& reserved_endpoints, bool leave_previous_endpoint) const;
    int choose_dummy_endpoint(EndpointStrategy strategy,
                              const DummyEndpointRequest& request);
    int rest_endpoint(const DummyEndpointRequest& request);
    bool move_to_rest_endpoint(Agent& agent, const DummyEndpointRequest& request);
    vector<int> rested_at_;   // where an idle token-passing agent rests, else -1
    unsigned int token_time_ = 0;   // when the token was last passed (TP, TPTS)
    Agent& next_token_holder();
    void exclude_others_endpoints(DummyEndpointRequest& request) const;
    int endpoint_wait_or_relocate(const DummyEndpointRequest& request);
    int endpoint_nearest_available(const DummyEndpointRequest& request) const;
    int endpoint_pairwise(const DummyEndpointRequest& request) const;
    int endpoint_strict(const DummyEndpointRequest& request) const;
    int nearest_endpoint(int location,
                         const std::function<bool(const Endpoint&)>& allowed) const;
    vector<vector<pair<int,int>>> split_into_task_groups(
        int agent_id, const vector<pair<int,int>>& goal_seq) const;
    int planned_task_count() const;
    void append_endpoint(int agent_id, vector<pair<int,int>>& goals, vector<int>& reserved);

    // ======================================================================
    //  Section 5 — MAPF: PP / PBS / wPBS
    // ======================================================================
    // Public algorithm boundaries live in path_planners.h. Simulation calls
    // PBSPlanner (wPBS is PBS with a planning window); every planner reaches
    // the low level through plan_single_agent() with a SingleAgentRequest.
    bool path_planning_pp();
    void path_planning_ta_hybrid(bool assignment_triggered);
    bool path_planning_pbs();
    bool path_planning_cbs();
    bool retry_pp_failed_first();
    bool selection_stalled() const;
    vector<bool> cbs_mdd_singletons(int agent_id, const CBSConstraintTable& constraints,
                                    int cost) const;
    void commit_path(int agent_id, const vector<int>& path);
    void start_assigned_tasks();
    vector<unsigned int> last_goal_time_;   // when each agent last reached a task goal
    unsigned int path_end_time(int agent_id) const;
    bool pbs_solve_impl();

    // Per-solve context, shared by the pbs_* helpers (set up by pbs_solve).
    struct PBSContext {
        int num_ag = 0;
        int max_t = 0;
        int last_release = 0;    // the later of now and the latest goal release
        int work_len = 0;        // length of the per-node path tables
        int path_horizon = 0;    // how much of the committed path is copied as "old"
        vector<vector<pair<int,int>>> goal_seqs;
        vector<vector<vector<pair<int,int>>>> task_groups;
        vector<vector<int>> old_paths;   // committed paths from the previous solve
        vector<vector<pair<int,int>>> task_goals;
        vector<bool> planned;            // false: keeps its committed path
        vector<vector<int>> fixed_paths; // paths of the agents that are not planned
        // wPBS planning window (RHCR): only [t, t + window] is planned and made
        // conflict-free, without the old paths; INT_MAX for PBS.
        int window = INT_MAX;
    };
    PBSContext pbs_;

    void pbs_build_context();
    bool reference_ecbs_applies(const vector<int>& agent_ids) const;
    bool central_assignment() const;   // CENTRAL_HUNGARIAN or its fixed variant
    bool path_planning_reference_ecbs(vector<int> agent_ids);
    vector<int> pbs_plan_agent(int agent_id, const vector<vector<int>>& cons_paths,
                               bool use_old);
    PBSNode* pbs_generate_root();
    void pbs_detect_conflicts(const PBSPaths& paths, list<Conflict>& out) const;
    void pbs_detect_conflicts_for(int agent_id, const PBSPaths& paths,
                                  int horizon, list<Conflict>& out) const;
    void pbs_find_replan_agents(const PriorityGraph& priorities,
                                const list<Conflict>& conflicts,
                                set<int>& replan) const;
    PBSNode* pbs_generate_child(PBSNode* parent, int lower, int higher);
    bool pbs_resolve_cascade(PBSNode* node, int replanned_agent, const PBSNode* parent);
    int  pbs_path_cost(int agent_id, const vector<int>& path) const;
    void pbs_set_path(PBSNode* node, int agent_id, const vector<int>& path);
    int  pbs_last_move(const vector<int>& path) const;
    int  pbs_settle_time(const PBSPaths& paths) const;
    int  pbs_conflict_horizon(int settle) const;
    void pbs_commit(const PBSNode* node);

    // --- Shared old-path constraint table -------------------------------
    // Every root search re-plans one agent against the SAME set of other agents'
    // committed paths, so the flattened table is built once per root batch and
    // each search just skips itself.  shared_old_holdcnt_ counts the agents that
    // are permanently parked on a cell, so the common case is an O(1) test.
    bool shared_old_valid_ = false;
    int  shared_old_excl_ = -1;          // agent being planned (skipped in the table)
    int  shared_old_horizon_ = 0;        // active columns in the flat table
    std::vector<int> shared_old_flat_;   // [num_ag * horizon] location of agent i at t
    std::vector<int> shared_old_holds_;  // [num_ag] permanently-held final location
    std::vector<int> shared_old_holdcnt_;// [map_size] # agents parked on each cell
    std::vector<int> shared_old_movers_; // agents whose old path actually moves
    // MLSIPP: each old path's stays (agent, begin, end) = on one cell during
    // [begin, end), by cell: cell loc's are [first[loc], first[loc + 1])
    std::vector<std::tuple<int,int,int>> shared_old_sipp_ranges_;
    std::vector<int> shared_old_sipp_first_;

    void pbs_build_shared_old_table();
    void pbs_clear_shared_old_table() { shared_old_valid_ = false; }

    // ======================================================================
    //  Section 6 — Single-agent search: MLA* / MLSIPP
    // ======================================================================
    // The configured single-agent search (config.single_agent); every planner
    // reaches the low level through here.
    vector<int> plan_single_agent(const SingleAgentRequest& request);
    vector<int> plan_sta_star(const SingleAgentRequest& request,
                              const vector<pair<int,int>>& goals);
    vector<int> plan_sequential_sta(const SingleAgentRequest& request,
                                    const vector<pair<int,int>>& goals);
    const vector<int>& distances_to(int location);
    unordered_map<int, vector<int>> distance_tables_;   // non-endpoint cells
    vector<int> mla_star_taskwise_impl(int agent_id, int start_loc, int start_time,
                                      const vector<vector<pair<int,int>>>& task_groups,
                                      const vector<vector<int>>& cons_paths,
                                      const vector<vector<int>>& old_paths,
                                      bool use_old_paths, int stop_time = INT_MAX,
                                      const CBSConstraintTable* constraints = nullptr);
    vector<int> seq_mla_star(int agent_id, int start_loc, int start_time,
                             const vector<pair<int,int>>& goals,
                             const vector<vector<int>>& cons_paths,
                             const vector<vector<int>>& old_paths,
                             bool use_old_paths,
                             bool skip_holding = false, int stop_time = INT_MAX,
                             const CBSConstraintTable* constraints = nullptr,
                             int extra_horizon = 0);

    vector<int> sipp_search_impl(int agent_id, int start_loc, int start_time,
                                const vector<pair<int,int>>& goals,
                                const vector<vector<int>>& cons_paths,
                                const vector<vector<int>>& old_paths,
                                bool use_old_paths,
                                bool skip_holding = false, int stop_time = INT_MAX,
                                const CBSConstraintTable* constraints = nullptr,
                                bool wait_early = false);
    vector<int> sipp_search_core(int agent_id, int start_loc, int start_time,
                                const vector<pair<int,int>>& goals,
                                const vector<vector<int>>& cons_paths,
                                const vector<vector<int>>& old_paths,
                                bool use_old_paths, bool skip_holding, int stop_time,
                                const CBSConstraintTable* constraints, bool wait_early,
                                int extra_horizon = 0);

    // TP/TPTS ordered-goal low-level planning and endpoint-vacating fallback.
    vector<int> plan_sipp_segments(
        int agent_id, int start_loc, int start_time,
        const vector<pair<int,int>>& goals,
        const vector<vector<int>>& constraint_paths,
        const vector<vector<int>>& old_paths, bool use_old_paths,
        int stop_time = INT_MAX, const CBSConstraintTable* constraints = nullptr);
    pair<int,int> plan_token_task(Agent& agent, Task& task,
                                   int hidden_agent = -1, int first_goal_before = INT_MAX);
    // Planning inside assignment (TP, TPTS, HBH) keeps, per committed path
    // (row of path_table_), the time from which it stays where it ends, and
    // plan_token_task's full-length int copies of the rows. Both are
    // refreshed only after the row changed: every write to path_table_ calls
    // token_row_changed (-1 = every row).
    mutable vector<int> path_settle_;
    mutable vector<char> path_settle_stale_;
    vector<vector<int>> token_rows_;
    vector<char> token_row_stale_;
    vector<unsigned int> token_row_version_;   // counts token_row_changed calls per row
    void token_row_changed(int agent_id);
    // Every committed path as an int row (index = agent), moved out of
    // token_rows_ (stale rows copied first) and moved back by give_token_rows;
    // a row whose path changed meanwhile is copied again before its next use.
    vector<unsigned int> take_token_rows(vector<vector<int>>& rows);
    void give_token_rows(vector<vector<int>>& rows, const vector<unsigned int>& versions);
    int path_settle(int agent_id) const;
    // Whether an agent other than agent_id/hidden is at loc at some time >= from.
    bool others_at(int agent_id, int hidden, int loc, int from) const;
    // The last time > after at which an agent other than agent_id is at loc, else after.
    int others_last_at(int agent_id, int loc, int after) const;
    bool sta_is_constrained(int agent_id, int current_loc, int next_loc,
                            int next_time, int hidden_agent) const;
    void sta_update_path(Agent& agent, const SearchNode& goal_node);
    // TP/TPTS WAIT_OR_NEAREST_SAFE is split into endpoint selection and path construction.
    int search_path2_endpoint(Agent& agent, int target_endpoint_loc,
                              bool parking_endpoints_only,
                              bool exclude_all_open_goals,
                              const vector<int>& reserved_endpoints = {});
    bool plan_path2_to_endpoint(Agent& agent, int endpoint_loc,
                                bool parking_endpoints_only,
                                bool exclude_all_open_goals);
    void release_search_nodes(map<unsigned int, SearchNode*>& nodes);
    int astar_with_dummy(Agent& agent, int start_loc, int start_time,
                         int goal_loc, int endpoint_loc,
                         const vector<int>& h_goal,
                         const vector<int>& h_endpoint,
                         const vector<vector<int>>& constraint_paths,
                         int release_time, bool goal_optimal = true,
                         const vector<tuple<int,int,int>>& cbs_constraints = {},
                         int stop_time = INT_MAX, int hold_from = INT_MIN,
                         const CBSConstraintTable* table = nullptr);

    // Reused safe-interval buffers; only cells touched by the previous search
    // are reset, avoiding an O(map size) allocation for every PBS low-level call.
    std::vector<std::vector<std::pair<int,int>>> sipp_ct_;
    std::vector<char> sipp_has_ct_;
    std::vector<int> sipp_ct_touched_;
    std::vector<std::vector<std::pair<int,int>>> sipp_sit_;
    std::vector<char> sipp_sit_done_;
    std::vector<int> sipp_sit_touched_;

    // MLA* node arena: nodes are bump-allocated from retained chunks so the
    // millions of tiny searches do not hit the allocator per node.  Chunks never
    // move, so parent pointers / heap entries stay valid exactly as with `new`.
    static const int MLA_ARENA_CHUNK = 4096;
    std::vector<void*> mla_arena_chunks_;
    int mla_arena_used_chunk_ = 0;
    int mla_arena_used_in_chunk_ = 0;
};
