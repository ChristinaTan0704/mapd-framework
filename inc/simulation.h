#pragma once
#include "types.h"
#include "config.h"
#include "map_loader.h"
#include <map>
#include <unordered_map>
#include <queue>
#include <functional>
#include <vector>
#include <list>
#include <string>
#include <set>
#include <unordered_set>
#include <stack>
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>

// ============ CostFlow (Min-Cost Max-Flow for TA-Hybrid) ============
#define COSTFLOW_INF 1111111111

struct CostFlowEdge {
    int from, to, capcity, cost, loc, next, origin_capcity;
};

class CostFlow {
public:
    CostFlow(int node_cnt, int source, int sink);
    void AddEdges(int from, int to, int capcity, int cost, int loc);
    void RemoveEdges(int from, int to);
    int MinCostFlow();
    std::vector<std::vector<int>> GetPath();
    int cost;
private:
    std::queue<int> Q;
    std::vector<CostFlowEdge> edges;
    std::vector<int> head;
    std::vector<int> pre;
    std::vector<bool> in_queue;
    std::vector<int> dis;
    int node_cnt, source, sink;
    void AddEdge(int from, int to, int capcity, int cost, int loc);
    bool SPFA();
};

// ============ Node for A* search ============
struct SearchNode {
    int loc;
    int g_val;
    int h_val;
    int timestep;
    SearchNode* parent;
    bool in_openlist;

    SearchNode(int l, int g, int h, SearchNode* p, int t)
        : loc(l), g_val(g), h_val(h), parent(p), timestep(t), in_openlist(true) {}
    SearchNode(int l, int g, SearchNode* p, int t)
        : loc(l), g_val(g), h_val(0), parent(p), timestep(t), in_openlist(true) {}
    int getFVal() const { return g_val + h_val; }
};

struct CompareNode {
    bool operator()(const SearchNode* n1, const SearchNode* n2) const {
        if (n1->getFVal() != n2->getFVal()) return n1->getFVal() > n2->getFVal();
        return n1->g_val <= n2->g_val;
    }
};

typedef boost::heap::fibonacci_heap<SearchNode*, boost::heap::compare<CompareNode>> heap_open_t;

// ============ PriorityGraph for PBS ============
class PriorityGraph {
public:
    void clear() { adj_.clear(); }
    void copy(const PriorityGraph& other) { adj_ = other.adj_; }
    // add: from has LOWER priority than to (from yields to to)
    void add(int from, int to) { adj_[from].insert(to); }
    // connected: is there a directed path from 'from' to 'to'?
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
    // get all nodes reachable from root (higher-priority agents)
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

// ============ PBS Node for priority-based search ============
struct PBSNode {
    PBSNode* parent;
    PriorityGraph priorities;
    std::vector<std::vector<int>> paths;      // paths[agent] = location path
    std::list<std::tuple<int,int,int,int,int>> conflicts; // (a1, a2, loc1, loc2, timestep)
    std::tuple<int,int,int,int,int> conflict; // chosen conflict
    std::pair<int,int> priority;              // (lower, higher)
    int num_collisions;
    int earliest_collision;
    int cost;

    PBSNode() : parent(nullptr), num_collisions(0), earliest_collision(INT_MAX), cost(0) {}
};

// ============ Simulation ============
class Simulation {
public:
    Simulation() {}
    ~Simulation() { for (void* p : mla_arena_chunks_) ::operator delete(p); }
    void init(const string& map_file, const string& task_file, const MAPDConfig& config,
              const string& tour_file = "");
    void run();
    bool fullCollisionCheck(const string& alg_name) const;
    void showTask() const;
    void saveOutput(const string& filepath, double runtime_ms) const;

private:
    MAPDConfig config;
    MAPDMap mapd_map;
    vector<Agent> agents;
    vector<Task> all_tasks;
    vector<vector<int>> task_indices_by_time;
    // Shared simulation state (formerly bundled in a `Token` struct):
    vector<vector<unsigned int>> path_table_;  // path_table_[agent][t] = cell (global path log)
    vector<bool> passable_map_;                // passable cells (mirror of mapd_map.grid)
    vector<bool> endpoint_mask_;               // endpoint cells (mirror of mapd_map.is_endpoint)
    list<Task*> open_tasks_;                    // pool of currently-unassigned tasks
    unsigned int cur_time_ = 0;                 // current simulation timestep
    unsigned int maxtime;
    int t_task;

    // --- CENTRAL state management ---
    vector<Task*> agent_pending_task;

    // Reusable vertex reservation buffer for assign_hungarian() cost matrix
    // (avoids reallocating/zeroing a map_size*maxtime table every assignment round).
    std::vector<char> aco_vres_;
    // Reusable closed-set for astar_cost_only (avoids per-call hash-map allocation;
    // millions of cost searches run per large/low-freq instance).
    std::unordered_map<long long, int> aco_visited_;

    // --- Shared state between assign_hungarian() and path_planning() ---
    vector<int> phase2_free_ids_;
    vector<Task*> phase2_tasks_;
    vector<int> phase2_goal_locs_;
    vector<int> phase2_goal_eps_;

    // --- Unified Main Loop (Section 2 of pseudocode) ---
    // True if any agent is still executing a committed path past the current timestep.
    bool any_agent_busy() const {
        for (auto& a : agents)
            if (a.status != AG_FREE && a.finish_time > cur_time_) return true;
        return false;
    }
    bool end() const;
    bool end_offline_ta() const;   // offline LKH3-TSP / TA-Hybrid single-iteration termination
    bool end_online() const;       // online PBS/LNS task_sequence emptiness check
    void release_tasks();
    void update_system();
    void task_assignment_and_path_planning();

    // --- Per-algorithm initialization (dispatched from init_algorithm_state) ---
    void init_algorithm_state();   // pure dispatch on config (flags default via in-class initializers)
    void init_tp_state();          // TP / TPTS (AT_ON_FREE_WAITS)
    void init_central_state();     // CENTRAL / CENTRAL_FIXED
    void init_pbs_state();         // Hungarian PBS / wPBS online
    void init_lns_state();         // LNS PBS / wPBS online

    // --- Per-algorithm update_system dispatch (from update_system) ---
    void update_system_online();   // PBS/LNS online (AT_ON_UNASSIGNED_OR_FREE)
    bool tp_pre_step();            // TP/TPTS pre-step; returns true to return early
    void update_system_stepwise(); // shared step-advance (CENTRAL/TA/TP-fallthrough)
    // Shared finish-time transition loops (delivery completion CARRYING->FREE and
    // pickup arrival MOVING_TO_PICKUP->CARRYING/FREE) run identically by
    // update_system_stepwise and tp_pre_step; the only per-caller difference is the
    // event-flag bookkeeping, exposed via the out-params (thrown away by tp_pre_step).
    void detect_finish_time_transitions(bool& has_event, bool& reassign_event);
    void clamp_next_ts_to_task_release(unsigned int& next_ts) const; // pull next_ts back to earliest task release in window
    void task_goals(const Task& task, int& first_goal, int& last_goal) const; // pickup goal + delivery goal for a task

    // --- Per-algorithm task_assignment dispatch (from task_assignment) ---
    void central_phase1_instant_pickup();  // CENTRAL/CENTRAL_FIXED Phase-1a/1b
    void central_deliver_single_astar(int aid, Task* task);  // CENTRAL single-agent A* delivery fallback
    void central_clear_phase2_scratch();   // CENTRAL (AM_HUNGARIAN) phase-2 scratch reset
    void tp_handle_no_assignment();        // TP/TPTS "no task found" bump/vacate

    // --- Dispatchers within task_assignment_and_path_planning ---
    bool should_assign() const;        // Section 5 — switches on assign_trigger
    void task_assignment();            // Section 6 — switches on assign_method
    bool should_replan() const;        // true for CENTRAL, false for TP/TPTS
    void path_planning();              // Section 11 — switches on mapf

    // --- Task Assignment: Decoupled Greedy (Section 9.1) ---
    bool assign_decoupled_greedy(Agent& ag);

    // --- Task Assignment: Decoupled Greedy with Swaps (Section 9.2) ---
    bool assign_tpts(Agent& ag, int depth = 0);
    void tpts_purge_picked_up_tasks();     // TPTS: drop already-picked-up tasks from token before TPTR

    // --- Task Assignment: Centralized Greedy (Section 9.3) ---
    void assign_centralized_greedy();

    // --- Task Assignment: Hungarian (Section 9.4) ---
    void assign_hungarian();

    // --- Path Planning: CBS (Section 11.1) ---
    void path_planning_cbs();

    // --- Path Planning: CBS Group 1 + PBS Group 2 (CENTRAL with PBS override) ---
    void path_planning_cbs_with_pp();

    // --- Path Planning: Prioritized Planning (Section 11.2) ---
    void path_planning_pp();

    // --- Path Planning: HBH-MLA* Decoupled PP with MLA* (Section 12.1) ---
    void plan_hbh_mla();

    // --- Single-Agent Search: Space-Time A* (Section 13.1) ---
    int astar(Agent& ag, int start_loc, int begin_time, const Endpoint& goal, int ag_hide);
    int astar_cost_only(int agent_id, int start_loc, int goal_loc,
                        int start_time, const vector<vector<int>>& cons_paths,
                        const std::vector<char>* vres = nullptr, int vres_len = 0,
                        const std::vector<int>* last_occ = nullptr);
    bool isConstrained(int agent_id, int curr_id, int next_id, int next_timestep, int ag_hide);
    void updatePath(Agent& ag, const SearchNode& goal_node);

    // --- Token-based MLA* for pickup+delivery in one search ---
    // ag_hide: additional agent ID to ignore in constraint checks (-1 = none)
    // Returns (arrive_start, arrive_goal) or (-1,-1) on failure.
    // Writes planned path to ag.path.
    pair<int,int> token_mla_star(Agent& ag, Task& task, int ag_hide = -1);

    // --- Plan one task: switches between 2xA* and token MLA* based on config ---
    // ag_hide: additional agent ID to ignore in constraint checks (-1 = none)
    // Returns (arrive_start, arrive_goal) or (-1,-1) on failure.
    // Writes path to ag.path (caller must commit to token.path).
    pair<int,int> plan_task_token(Agent& ag, Task& task, int ag_hide = -1);

    // --- Dummy Path / Move to Endpoint (Section 12.0) ---
    bool move2EP(Agent& ag);

    // --- Helpers ---
    void releaseNodes(map<unsigned int, SearchNode*>& table);
    int findEndpointIndex(int loc) const;

    // --- Loop state ---
    int last_released_time_;  // tracks up to which timestep tasks have been released
    bool ta_planning_done_ = false;   // for offline algorithms: set after first iteration
    bool tp_timestep_advanced_ = false;  // true if update_system just advanced the timestep (TP/TPTS)

    // --- Loop state for CENTRAL ---
    bool central_has_event_ = false;       // any event (pickup arrival, delivery done, new tasks)
    bool central_reassign_event_ = false;  // only delivery-done or new-tasks (for CENTRAL_FIXED)

    // --- TA-Prioritized (Section 10.1 + 12.0 + 14.5) ---
    string tour_file_;
    vector<vector<int>> all_pairs_dist_;   // BFS all-pairs shortest paths

    void assign_ta_tsp();                  // parse tour file, build per-agent task sequences
    void plan_ta_prioritized();            // prioritized path planning for TA methods

    // Two-phase A*: start → goal → parking
    // Returns the timestep when goal_loc is reached, or -1 on failure.
    // Writes the full path (including dummy to parking) into the agent's path array.
    int astar_with_dummy(Agent& ag, int start_loc, int start_time,
                         int goal_loc, int park_loc,
                         const vector<int>& h_goal, const vector<int>& h_park,
                         const vector<vector<int>>& cons_paths,
                         int release_time,
                         bool goal_optimal = true,
                         const vector<tuple<int,int,int>>& cbs_cons = {});

    // BFS all-pairs distances (only free cells)
    void compute_all_pairs_bfs();

    // --- TA-Hybrid (Section 20+) ---
    vector<queue<Task*>> hybrid_seqs_;       // per-sequence task queues
    vector<int> hybrid_prefer_agent_;        // prefer_agent[seq_id] = agent_id
    vector<int> hybrid_tsp_agent_;           // initial agent assignment from tour
    int hybrid_global_makespan_;             // estimated global makespan for cost flow horizon
    unsigned int hybrid_timestep_;           // per-timestep simulation counter

    void plan_ta_hybrid();                   // main per-timestep TA-Hybrid loop

    // TA-Hybrid helpers
    int hybrid_cost(int t, queue<Task*> seq);  // estimate completion time for a sequence
    void hybrid_calc_flow(vector<Agent*>& flow_agents, vector<Task*>& flow_tasks,
                          const vector<vector<int>>& cons_paths,
                          vector<int> len, int& flow,
                          vector<vector<int>>& paths);
    int hybrid_go_home(vector<Agent*>& ags);   // plan dummy paths to parking
    bool hybrid_replan_dummy(Agent* ag);        // replan dummy path if collision
    void hybrid_assign_new_task(int seq_id);    // steal a task from another sequence

    // Group 1: delivery planning via prioritized A* with dummy
    bool hybrid_group1_plan(vector<Agent*>& delivery_agents,
                            vector<Agent*>& constraint_agents);

    // --- HUNGARIAN_PBS / HUNGARIAN_wPBS ---

    // State for online PBS loop
    bool pbs_has_event_ = false;          // any event (replan or assign)
    bool pbs_assign_event_ = false;       // real event needing re-assignment (not periodic)
    int pbs_last_replan_time_ = 0;    // for wPBS periodic replanning
    int lns_release_period_ = 1;      // task-release period (for gating the 1s LNS spin like the reference)
    bool lns_agent_finished_ = false;     // an agent reached its last goal this event (reference new_agent_finish)

    // Repeated Hungarian task assignment (builds task_sequences for all agents)
    void assign_repeated_hungarian();

    // Repeated Hungarian + LNS improvement (LNS-PBS, LNS-wPBS)
    void assign_repeated_hungarian_lns();
    int estimate_sequence_cost(int agent_id) const;
    void lns_destroy(vector<int>& removed_tasks);
    void lns_repair(vector<int>& removed_tasks);

    // Build goal sequences from task_sequences + dummy endpoints
    // Returns goal_sequences[agent] = vector<pair<loc, release_time>>
    vector<vector<pair<int,int>>> build_goal_sequences();

    // Choose a flexible dummy endpoint for an agent
    int choose_dummy_endpoint(int agent_id, int last_goal_loc,
                              const vector<int>& assigned_dummies,
                              bool strict);

    // --- Shared old-path constraint table (built ONCE per PBS root batch) ---
    // At a low task frequency the PBS root re-plans every agent against the same set of other
    // agents' committed ("old") paths.  Rebuilding the flattened constraint table inside every
    // per-agent search was the dominant cost (O(num_ag * horizon) per search * num_ag searches).
    // These buffers hold the table for ALL agents once; seq_mla_star reads them and skips the
    // self agent.  shared_old_active_ = max active prefix length over all agents (the table is
    // sized to this); shared_old_holds_[i] = agent i's permanently-held final location.
    bool   shared_old_valid_ = false;   // set while the shared table is usable (root batch only)
    int    shared_old_excl_  = -1;      // agent id to exclude (the one being planned)
    int    shared_old_horizon_ = 0;     // active horizon (columns in the flat table)
    std::vector<int> shared_old_flat_;  // [num_ag * shared_old_horizon_]: loc of agent i at t
    std::vector<int> shared_old_holds_; // [num_ag]: agent i's permanent endpoint-hold location
    std::vector<int> shared_old_holdcnt_; // [map_size]: # agents permanently parked on each cell
    std::vector<int> shared_old_movers_;  // agent ids whose old path actually moves (active>1)
    // SIPP analog of the shared flat table: each agent's old path precompressed into CT ranges
    // (loc,start,end), built once per PBS root batch and reused by sipp_search across the many
    // per-agent searches (excluding self).  Compressed to exactly the window sipp_search uses.
    std::vector<std::vector<std::tuple<int,int,int>>> shared_old_sipp_ranges_;
    int shared_old_sipp_h_ = 0;

    // --- Persistent per-call scratch buffers for sipp_search ---
    // sipp_search was allocating O(map_size) vectors (ct_ranges, has_ct) plus two unordered_maps
    // on EVERY call (tens of thousands of calls per solve at low task frequency); the O(map_size)
    // allocation/zero-fill dominated SIPP setup time.  These reusable members are sized once (to
    // map_size) and cleared in O(touched cells) via the touched lists, eliminating the per-call
    // O(map_size) work.  Each holds CT ranges (constraint occupancy) and the derived safe-interval
    // table per cell; the per-cell sub-vectors keep their capacity across calls (clear() only).
    std::vector<std::vector<std::pair<int,int>>> sipp_ct_;   // per-cell constraint ranges (s,e)
    std::vector<char> sipp_has_ct_;                          // per-cell: any CT range?
    std::vector<int>  sipp_ct_touched_;                      // cells touched this call (to reset)
    std::vector<std::vector<std::pair<int,int>>> sipp_sit_;  // per-cell safe-interval table (lazy)
    std::vector<char> sipp_sit_done_;                        // per-cell: SIT computed this call?
    std::vector<int>  sipp_sit_touched_;                     // cells with computed SIT (to reset)

    // --- MLA* low-level node arena ---
    // seq_mla_star allocated one heap MLANode per generated state and deleted them all at the end
    // of every call.  In windowed wPBS at mid task frequency there are hundreds of thousands of
    // tiny searches (each a ~10-step window) generating millions of nodes total, so the per-node
    // malloc/free churn dominated the low-level loop time.  This arena hands out pointer-stable
    // node storage from reusable fixed-size chunks: nodes are bump-allocated within a chunk, and
    // chunks are RETAINED across calls (only the in-use count is reset), so steady-state has zero
    // allocator traffic.  Pointer stability (chunks never move) keeps parent pointers / heap /
    // dedup-map entries valid exactly as raw new did.  Behaviour-identical to per-node new.
    static const int MLA_ARENA_CHUNK = 4096;
    std::vector<void*> mla_arena_chunks_;  // owned raw chunks (freed in destructor)
    int mla_arena_used_chunk_ = 0;         // index of current chunk
    int mla_arena_used_in_chunk_ = 0;      // bump offset within current chunk (in nodes)

    // SeqMLA*: plans through ALL goals in one search
    // constraint_window: if >0, only enforce constraints up to this absolute timestep
    vector<int> seq_mla_star(int agent_id, int start_loc, int start_time,
                             const vector<pair<int,int>>& goals,
                             const vector<vector<int>>& cons_paths,
                             const vector<vector<int>>& old_paths,
                             bool use_old_paths,
                             bool skip_holding = false,
                             int constraint_window = -1);

    // SIPP search: uses safe intervals for faster multi-goal planning
    // Same interface as seq_mla_star — drop-in replacement
    vector<int> sipp_search(int agent_id, int start_loc, int start_time,
                            const vector<pair<int,int>>& goals,
                            const vector<vector<int>>& cons_paths,
                            const vector<vector<int>>& old_paths,
                            bool use_old_paths,
                            bool skip_holding = false,
                            int constraint_window = -1);

    // Task-by-task MLA*: plans each task group separately
    vector<int> mla_star_taskwise(int agent_id, int start_loc, int start_time,
                                   const vector<vector<pair<int,int>>>& task_groups,
                                   const vector<vector<int>>& cons_paths,
                                   const vector<vector<int>>& old_paths,
                                   bool use_old_paths,
                                   int constraint_window = -1);

    // Split flat goal sequence into per-task groups for mla_star_taskwise
    vector<vector<pair<int,int>>> split_into_task_groups(
        int agent_id,
        const vector<pair<int,int>>& goal_seq) const;

    // PP+MLA* path planning for HUNGARIAN/LNS (alternative to PBS/wPBS)
    void path_planning_pp_mla();
    vector<deque<int>> pp_mla_prev_seqs_;

    // PBS path planning (MLA* low-level)
    void path_planning_pbs();

    // PBS path planning with SIPP low-level (faster, opt-in via --sipp)
    bool pbs_core_sipp();

    // wPBS path planning
    void path_planning_wpbs();

    // Framework-native windowed PBS solve (mapd_map-backed StateTimeAStar +
    // ReservationTable + PBS high level); replaces the removed ref_solve module.
    void wpbs_windowed_solve();

    // Post-simulation safety net: relocate any two agents that end parked on the same
    // cell to distinct free endpoints (bounded, guaranteed-terminating). Called at the
    // end of run().

    // Shared PBS core
    bool pbs_core(bool windowed);

    // PBS: find earliest conflict between two agent paths
    bool pbs_find_conflict(const vector<int>& p1, const vector<int>& p2,
                           int a1, int a2,
                           tuple<int,int,int,int,int>& conflict);

    // Shared online event-detection post-step. Always run for online
    // methods (all AT_ON_UNASSIGNED_OR_FREE: Hungarian AND LNS, PBS AND
    // wPBS) after the timestep advances. It does bookkeeping, not planning:
    // detects pickup/delivery arrivals and newly released tasks and sets
    // pbs_has_event_/pbs_assign_event_/lns_agent_finished_ accordingly.
    void process_online_events();

    // --- REALPATH_LNS_IMP: Generic Anytime Improvement (Chen et al. 2021, Sec IV-D) ---
    // Independent post-processing: destroy tasks, re-assign with real collision-free
    // path planning, accept if cost improves. Can be called after any algorithm.
    // Returns number of improving iterations found.
public:
    int realpath_lns_imp(int num_rounds, int group_size = 5);

private:
    // Compute real-path cost: sum of (completion_time - release_time) for all assigned tasks
    int compute_realpath_cost() const;

    // RMCA-style destroy: RANDOM, WORST, MULTIPLE
    void rmca_destroy(vector<int>& removed, int group_size);

    // RMCA-style repair: regret-based re-insertion
    void rmca_repair(vector<int>& removed, vector<vector<int>>& agent_task_lists);

    // Re-plan a single agent's path using token-based A* for its current task
    // Returns true if planning succeeded
    bool replan_agent_path(int agent_id);
};
