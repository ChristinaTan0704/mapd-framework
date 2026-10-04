#pragma once
#include "types.h"
#include "map_loader.h"
#include "config.h"
#include <map>
#include <memory>
#include <list>
#include <tuple>
#include <vector>
#include <functional>
#include <boost/heap/fibonacci_heap.hpp>

// ============ Low-Level Node ============

struct LLNode {
    int loc, timestep;
    double g_val, h_val;
    std::uint64_t tie_breaker;
    LLNode* parent;
    bool in_openlist;

    double getFVal() const { return g_val + h_val; }

    struct CompareOpen {
        bool operator()(const LLNode* a, const LLNode* b) const {
            if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
            if (TieBreakPolicy::reference()) return a->g_val <= b->g_val;
            if (a->g_val != b->g_val) return a->g_val < b->g_val;
            return a->tie_breaker > b->tie_breaker;
        }
    };
    struct CompareFocal {
        bool operator()(const LLNode* a, const LLNode* b) const {
            if (TieBreakPolicy::reference()) {
                if (a->getFVal() == b->getFVal()) return a->g_val <= b->g_val;
                return a->getFVal() >= b->getFVal();
            }
            if (a->getFVal() != b->getFVal()) return a->getFVal() > b->getFVal();
            if (a->g_val != b->g_val) return a->g_val < b->g_val;
            return a->tie_breaker > b->tie_breaker;
        }
    };

    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<CompareOpen>>::handle_type open_handle_t;
    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<CompareFocal>>::handle_type focal_handle_t;

    open_handle_t open_handle;
    focal_handle_t focal_handle;

    LLNode(int l, double g, double h, LLNode* p, int t, bool io);
};

// ============ Low-Level: single-agent space-time A* (TA-Hybrid) ============

class SingleAgentECBS {
public:
    vector<int> path;
    double path_cost;
    double min_f_val;
    int num_expanded;
    int num_generated;

    // Moves are expanded as wait, N, E, S, W; wait_last = true expands the
    // wait last, as the TA reference code does.
    SingleAgentECBS(const vector<vector<int>>& cons_paths,
                    const vector<int>& heuristic,
                    const vector<bool>& grid,
                    int ag_id, int start_loc, int goal_loc,
                    int col, int curr_time, int max_time,
                    int expansion_limit, bool wait_last = false);

    // Space-time A* to a goal the agent can hold forever, avoiding cons_paths.
    bool findPath();

private:
    const vector<vector<int>>& cons_paths_;
    const vector<int>& heuristic_;
    const vector<bool>& grid_;
    int ag_id_, start_loc_, goal_loc_, curr_time_, max_time_;
    int expansion_limit_;
    int map_size_;
    int actions_[5];

    bool isConstrained(int curr, int next, int next_t);
    bool canHoldGoal(int timestep) const;
    void updatePath(LLNode* goal);
    void releaseNodes(map<unsigned int, LLNode*>& table);
};

// ============ High-Level CBS Node ============

struct HLNode {
    int agent_id;
    // Constraints added for agent_id: (loc, -1, t) vertex, (from, to, t) edge,
    // (loc, -2, t) range. Barrier nodes carry several vertex constraints;
    // bypass nodes carry none.
    vector<tuple<int, int, int>> constraints;
    vector<int> path;
    double g_val;      // sum of the path costs
    double h_val;      // colliding agent pairs
    double path_cost;
    std::uint64_t tie_breaker;
    HLNode* parent;
    int time_generated;
    int time_expanded;
    // Lazily built MDD singletons for agent_id under this node's constraints.
    std::shared_ptr<const vector<bool>> mdd_singletons;


    struct CompareOpen {
        bool operator()(const HLNode* a, const HLNode* b) const {
            if (TieBreakPolicy::reference()) return a->g_val >= b->g_val;
            if (a->g_val != b->g_val) return a->g_val > b->g_val;
            return a->tie_breaker > b->tie_breaker;
        }
    };
    struct CompareFocal {
        bool operator()(const HLNode* a, const HLNode* b) const {
            if (TieBreakPolicy::reference()) {
                if (a->h_val == b->h_val) return a->g_val >= b->g_val;
                return a->h_val >= b->h_val;
            }
            if (a->h_val != b->h_val) return a->h_val > b->h_val;
            if (a->g_val != b->g_val) return a->g_val > b->g_val;
            return a->tie_breaker > b->tie_breaker;
        }
    };

    typedef boost::heap::fibonacci_heap<HLNode*, boost::heap::compare<CompareOpen>>::handle_type open_handle_t;
    typedef boost::heap::fibonacci_heap<HLNode*, boost::heap::compare<CompareFocal>>::handle_type focal_handle_t;

    open_handle_t open_handle;
    focal_handle_t focal_handle;

    HLNode();
};

// ============ High-Level CBS/ECBS Search ============

// The single-agent search of a CBS instance. Constraints are (loc, -1, t)
// vertex, (from, to, t) edge and (loc, -2, t) range constraints (see HLNode),
// with t relative to the current time.
struct CBSLowLevel {
    // The agent's path from the current time (index 0) to its final goal,
    // which it then holds, preferring fewer conflicts with the other agents'
    // paths (an empty path is ignored); false when there is none.
    std::function<bool(int agent, const vector<tuple<int,int,int>>& constraints,
                       const vector<vector<int>>& paths, vector<int>& path)> find_path;
    // ICBS: entry t is true when every path of exactly `cost` steps under the
    // constraints occupies a single location at t; empty when unknown.
    std::function<vector<bool>(int agent, const vector<tuple<int,int,int>>& constraints,
                               int cost)> mdd_singletons;
};

class CBSSearch {
public:
    vector<vector<int>> paths;
    bool solution_found;
    double solution_cost;

    CBSSearch(int num_agents, const CBSLowLevel& low_level, int col, double focal_w,
              int high_level_expansion_limit, CBSConflictSelection conflict_selection,
              bool bypass, bool target_reasoning, bool rectangle_reasoning);

    bool run();
    ~CBSSearch();

private:
    int num_agents_;
    const CBSLowLevel& low_level_;
    double focal_w_;
    int high_level_expansion_limit_;
    CBSConflictSelection conflict_selection_;
    bool bypass_;
    bool target_reasoning_;
    bool rectangle_reasoning_;
    int col_;

    vector<vector<int>> paths_found_initially_;
    vector<double> paths_costs_found_initially_;
    vector<double> paths_costs_;
    vector<std::shared_ptr<const vector<bool>>> root_mdd_singletons_;

    typedef boost::heap::fibonacci_heap<HLNode*, boost::heap::compare<HLNode::CompareOpen>> hl_open_t;
    typedef boost::heap::fibonacci_heap<HLNode*, boost::heap::compare<HLNode::CompareFocal>> hl_focal_t;

    hl_open_t hl_open_;
    hl_focal_t hl_focal_;
    double min_sum_f_vals_;
    double focal_list_threshold_;

    HLNode* dummy_start_;
    vector<HLNode*> all_nodes_;
    int HL_num_expanded_;
    int HL_num_generated_;

    tuple<int,int,int,int,int> earliest_conflict_;

    int getAgentLocation(int agent_id, size_t timestep);
    bool switchedLocations(int a1, int a2, size_t timestep);
    void updatePaths(HLNode* curr, HLNode* root);
    bool updateCBSNode(HLNode* leaf, HLNode* root);
    vector<tuple<int,int,int>> collectConstraints(HLNode* node, int agent_id);
    const vector<bool>& getMDDSingletons(HLNode* node, int agent_id);
    bool isSingletonSide(HLNode* node, int agent_id, int edge_to, int timestep);
    void selectCardinalConflict(HLNode* node,
        const vector<tuple<int,int,int,int,int>>& collisions);
    bool generateChild(HLNode* child, HLNode* parent);
    bool addRectangleBarriers(int a1, int a2, int loc, int timestep,
                              HLNode* n1, HLNode* n2);
    int computeNumOfCollidingPairs();
    vector<tuple<int,int,int,int,int>>* extractCollisions();
    void updateFocalList(double old_lb, double new_lb);
};
