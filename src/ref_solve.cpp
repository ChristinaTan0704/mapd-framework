// ============================================================================
// ref_solve.cpp — VERBATIM port of the reference (MGMAPD/LNS-wPBS) integrated
// windowed solve, wired together as ONE unit:
//     StateTimeAStar (low level, dual fibonacci-heap focal, +window cutoff)
//   + ReservationTable (CT from higher-priority reachable paths, window hold)
//   + PBS high-level (DFS, generate_root_node sequential prioritized,
//     choose_conflict, generate_child cascade via find_replan_agents, nogood)
//
// Everything lives in namespace refsolve to avoid clashing with the reimpl's own
// PBSNode/PriorityGraph/State types.  The BasicGraph here is a MINIMAL concrete
// grid graph (4-neighbour, uniform weight, no rotation) built from the reimpl's
// padded grid, with heuristics computed by BFS on demand.  All algorithmic code
// (ReservationTable, StateTimeAStar, PBS) is copied faithfully from the reference;
// the ONLY behavioural edit is the low-level goal cutoff literal 10 -> rt.window
// (the reference used plan_window=10 so start+10 == start+window; parametrising it
// keeps the executed window and the low-level horizon consistent).
// ============================================================================
#include "ref_solve.h"

#include <utility>
#include <tuple>
#include <list>
#include <vector>
#include <queue>
#include <string>
#include <climits>
#include <cfloat>
#include <ctime>
#include <algorithm>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace refsolve {

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
using std::string;
using std::max;
using std::min;

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict;
typedef tuple<int, int, bool> Interval; // [t_min, t_max), have conflicts or not
#define INTERVAL_MAX 10000

// ------------------------------------------------------------------ State
struct State
{
    int location;
    int timestep;
    int orientation;

    State wait() const {return State(location, timestep + 1, orientation); }

    struct Hasher
    {
        std::size_t operator()(const State& n) const
        {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t time_hash = std::hash<int>()(n.timestep);
            size_t ori_hash = std::hash<int>()(n.orientation);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        }
    };

    void operator = (const State& other)
    { timestep = other.timestep; location = other.location; orientation = other.orientation; }
    bool operator == (const State& other) const
    { return timestep == other.timestep && location == other.location && orientation == other.orientation; }
    bool operator != (const State& other) const
    { return timestep != other.timestep || location != other.location || orientation != other.orientation; }

    State(): location(-1), timestep(-1), orientation(-1) {}
    State(int location, int timestep = -1, int orientation = -1):
            location(location), timestep(timestep), orientation(orientation) {}
    State(const State& other) {location = other.location; timestep = other.timestep; orientation = other.orientation; }
};

typedef std::vector<State> Path;

// ------------------------------------------------------------------ BasicGraph
// Minimal concrete grid graph: 4-neighbour, uniform weight 1, no rotation.
class BasicGraph
{
public:
    int rows = 0, cols = 0;
    vector<std::string> types;                       // "Travel" / "Obstacle"
    vector<char> passable;                           // 1 = free
    unordered_map<int, vector<double>> heuristics;   // goal_loc -> per-cell BFS dist

    int size() const { return rows * cols; }

    // get_neighbors: wait + up-to-4 moves (orientation ignored / -1)
    list<State> get_neighbors(const State& s) const
    {
        list<State> nb;
        if (s.location < 0) return nb;
        nb.push_back(State(s.location, s.timestep + 1, -1)); // wait
        static const int d4[4] = {1, -1, 0, 0};
        int move[4] = {1, -1, cols, -cols};
        for (int i = 0; i < 4; i++)
        {
            int nl = s.location + move[i];
            if (nl < 0 || nl >= size()) continue;
            // guard against wrap-around on the +-1 moves
            if ((i == 0 || i == 1) && (nl / cols != s.location / cols)) continue;
            (void)d4;
            if (passable[nl])
                nb.push_back(State(nl, s.timestep + 1, -1));
        }
        return nb;
    }

    double get_weight(int, int) const { return 1.0; } // uniform

    // BFS distances from root over free cells (uniform weight); cache in heuristics.
    const vector<double>& ensure_heuristic(int root)
    {
        auto it = heuristics.find(root);
        if (it != heuristics.end()) return it->second;
        vector<double> res(size(), (double)INT_MAX);
        if (root >= 0 && root < size() && passable[root])
        {
            std::queue<int> Q;
            res[root] = 0; Q.push(root);
            int move[4] = {1, -1, cols, -cols};
            while (!Q.empty())
            {
                int c = Q.front(); Q.pop();
                for (int i = 0; i < 4; i++)
                {
                    int nl = c + move[i];
                    if (nl < 0 || nl >= size()) continue;
                    if ((i == 0 || i == 1) && (nl / cols != c / cols)) continue;
                    if (passable[nl] && res[nl] >= (double)INT_MAX)
                    { res[nl] = res[c] + 1; Q.push(nl); }
                }
            }
        }
        auto ins = heuristics.emplace(root, std::move(res));
        return ins.first->second;
    }
};

// ------------------------------------------------------------------ PriorityGraph
class PriorityGraph
{
public:
    double runtime = 0;
    typedef boost::unordered_map<int, boost::unordered_set<int> > PGraph_t;
    PGraph_t G;

    void clear() { G.clear(); }
    bool empty() const {return G.empty(); }
    void copy(const PriorityGraph& other) { this->G = other.G; }
    void add(int from, int to) { G[from].insert(to); }
    void remove(int from, int to) { if (G.find(from) != G.end()) G[from].erase(to); }

    bool connected(int from, int to) const
    {
        std::list<int> open_list;
        boost::unordered_set<int> closed_list;
        open_list.push_back(from); closed_list.insert(from);
        while (!open_list.empty())
        {
            int curr = open_list.back(); open_list.pop_back();
            auto neighbors = G.find(curr);
            if (neighbors == G.end()) continue;
            for (auto next : neighbors->second)
            {
                if (next == to) return true;
                if (closed_list.find(next) == closed_list.end())
                { open_list.push_back(next); closed_list.insert(next); }
            }
        }
        return false;
    }

    boost::unordered_set<int> get_reachable_nodes(int root)
    {
        clock_t t = std::clock();
        std::list<int> open_list;
        boost::unordered_set<int> closed_list;
        open_list.push_back(root);
        while (!open_list.empty())
        {
            int curr = open_list.back(); open_list.pop_back();
            auto neighbors = G.find(curr);
            if (neighbors == G.end()) continue;
            for (auto next : neighbors->second)
                if (closed_list.find(next) == closed_list.end())
                { open_list.push_back(next); closed_list.insert(next); }
        }
        runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
        return closed_list;
    }
};

// ------------------------------------------------------------------ ReservationTable
class ReservationTable
{
public:
    size_t map_size = 0;
    int num_of_agents = 0;
    int k_robust = 0;
    int window = 0;
    bool use_cat = false;
    bool hold_endpoints = false;
    bool prioritize_start = false;
    double runtime = 0;

    void clear() {sit.clear(); ct.clear(); cat.clear(); }
    void copy(const ReservationTable& other) {sit = other.sit; ct = other.ct; cat = other.cat; }

    void build(const vector<Path*>& paths,
               const list< tuple<int, int, int> >& initial_constraints,
               const boost::unordered_set<int>& high_priority_agents, int current_agent, int start_location);
    void insertPath2CT(const Path& path);

    bool isConstrained(int curr_id, int next_id, int next_timestep) const;
    bool isConflicting(int curr_id, int next_id, int next_timestep) const;
    int getHoldingTimeFromCT(int location) const;

    ReservationTable(const BasicGraph& G): G(G) {}
private:
    const BasicGraph& G;
    unordered_map<size_t, list<pair<int, int> > > ct;
    vector<vector<bool> > cat;
    unordered_map<size_t, list<Interval > > sit;

    void insertConstraints4starts(const vector<Path*>& paths, int current_agent, int start_location);
    void insertPath2CAT(const Path& path);
    void addInitialConstraints(const list< tuple<int, int, int> >& initial_constraints, int current_agent);
    inline int getEdgeIndex(int from, int to) const {return (from + 1) * map_size + to; }
};

int ReservationTable::getHoldingTimeFromCT(int location) const
{
    const auto& it = ct.find(location);
    if (it == ct.end()) return 0;
    int t = 0;
    for (auto time_range : it->second)
        if (time_range.second > t) t = time_range.second;
    return t;
}

void ReservationTable::insertPath2CT(const Path& path)
{
    if (path.empty()) return;
    auto prev = path.begin();
    auto curr = path.begin();
    ++curr;
    while (curr != path.end() && curr->timestep - k_robust <= window)
    {
        if (prev->location != curr->location)
        {
            if (G.types[prev->location] != "Magic")
                ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
            if (k_robust == 0)
                ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
            prev = curr;
        }
        ++curr;
    }
    if (curr != path.end())
    {
        if (G.types[prev->location] != "Magic")
            ct[prev->location].emplace_back(prev->timestep - k_robust, curr->timestep + k_robust);
        if (k_robust == 0)
            ct[getEdgeIndex(curr->location, prev->location)].emplace_back(curr->timestep, curr->timestep + 1);
    }
    else
    {
        if (G.types[prev->location] != "Magic")
            ct[prev->location].emplace_back(prev->timestep - k_robust, path.back().timestep + 1 + k_robust);
        if (k_robust == 0)
            ct[getEdgeIndex(path.back().location, prev->location)].emplace_back(path.back().timestep, path.back().timestep + 1);
    }
    ct[path.back().location].emplace_back(path.back().timestep, INTERVAL_MAX);
}

void ReservationTable::addInitialConstraints(const list< tuple<int, int, int> >& initial_constraints, int current_agent)
{
    for (auto con : initial_constraints)
    {
        if (std::get<0>(con) != current_agent && 0 <= std::get<1>(con) && std::get<1>(con) < (int)G.types.size() &&
            G.types[std::get<1>(con)] != "Magic")
            ct[std::get<1>(con)].emplace_back(0, min(window, std::get<2>(con)));
    }
}

void ReservationTable::insertPath2CAT(const Path& path)
{
    if (path.empty()) return;
    int max_timestep = min((int)path.size() - 1, k_robust + window);
    int timestep = 0;
    while (timestep <= max_timestep)
    {
        int location = path[timestep].location;
        if (G.types[location] != "Magic")
            for (int t = max(0, timestep - k_robust); t <= min((int)cat.size() - 1, timestep + k_robust); t++)
                cat[t][location] = true;
        timestep++;
    }
    if (G.types[path.back().location] != "Magic")
        while (timestep < (int)cat.size())
        { cat[timestep][path.back().location] = true; timestep++; }
}

void ReservationTable::build(const vector<Path*>& paths,
        const list< tuple<int, int, int> >& initial_constraints,
        const boost::unordered_set<int>& high_priority_agents, int current_agent, int start_location)
{
    clock_t t = std::clock();
    vector<bool> soft(num_of_agents, true);
    for (auto i : high_priority_agents)
    {
        if (paths[i] == nullptr) continue;
        insertPath2CT(*paths[i]);
        soft[i] = false;
    }
    if (prioritize_start)
        insertConstraints4starts(paths, current_agent, start_location);
    addInitialConstraints(initial_constraints, current_agent);
    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
    if (!use_cat) return;

    soft[current_agent] = false;
    for (int i = 0; i < num_of_agents; i++)
    {
        if(!soft[i] || paths[i] == nullptr) continue;
        insertPath2CAT(*paths[i]);
    }
    runtime = (std::clock() - t) * 1.0  / CLOCKS_PER_SEC;
}

void ReservationTable::insertConstraints4starts(const vector<Path*>& paths, int current_agent, int)
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] == nullptr) continue;
        else if (i != current_agent)
        {
            int start = paths[i]->front().location;
            if (start < 0 || G.types[start] == "Magic") continue;
            for (auto state : (*paths[i]))
                if (state.location != start)
                { ct[start].emplace_back(0, state.timestep + k_robust); break; }
        }
    }
}

bool ReservationTable::isConstrained(int curr_id, int next_id, int next_timestep) const
{
    auto it = ct.find(next_id);
    if (it != ct.end())
        for (auto time_range : it->second)
            if (next_timestep >= time_range.first && next_timestep < time_range.second)
                return true;
    if (curr_id != next_id)
    {
        it = ct.find(getEdgeIndex(curr_id, next_id));
        if (it != ct.end())
            for (auto time_range : it->second)
                if (next_timestep >= time_range.first && next_timestep < time_range.second)
                    return true;
    }
    return false;
}

bool ReservationTable::isConflicting(int curr_id, int next_id, int next_timestep) const
{
    if (next_timestep >= (int)cat.size()) return false;
    if (cat[next_timestep][next_id]) return true;
    else if (curr_id != next_id && cat[next_timestep][curr_id] && cat[next_timestep - 1][next_id]) return true;
    else return false;
}

// ------------------------------------------------------------------ SingleAgentSolver
class SingleAgentSolver
{
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

    double compute_h_value(BasicGraph& G, int curr, int goal_id,
        const vector<pair<int, int> >& goal_location) const
    {
        double h = G.ensure_heuristic(goal_location[goal_id].first)[curr];
        goal_id++;
        while (goal_id < (int) goal_location.size())
        {
            h += G.ensure_heuristic(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
            goal_id++;
        }
        return h;
    }

    virtual Path run(BasicGraph& G, const State& start, const vector<pair<int, int> >& goal_location, ReservationTable& RT) = 0;
    virtual ~SingleAgentSolver() {}
};

// ------------------------------------------------------------------ StateTimeAStar
class StateTimeAStarNode
{
public:
    State state;
    double g_val;
    double h_val;
    StateTimeAStarNode* parent;
    int conflicts;
    int depth;
    bool in_openlist;
    int visit_goal_time;
    int goal_id;
    int goal_length;
    bool vis_goal;

    struct compare_node
    {
        bool operator()(const StateTimeAStarNode* n1, const StateTimeAStarNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val;
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };
    struct secondary_compare_node
    {
        bool operator()(const StateTimeAStarNode* n1, const StateTimeAStarNode* n2) const
        {
            if (n1->conflicts == n2->conflicts)
            {
                if (n1->goal_id == n2->goal_id)
                    return n1->g_val <= n2->g_val;
                return n1->goal_id <= n2->goal_id;
            }
            return n1->conflicts >= n2->conflicts;
        }
    };

    fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::compare_node> >::handle_type open_handle;
    fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::secondary_compare_node> >::handle_type focal_handle;

    StateTimeAStarNode(): g_val(0), h_val(0), parent(nullptr), conflicts(0), depth(0), in_openlist(false), goal_id(0), visit_goal_time(0), vis_goal(false) {}
    StateTimeAStarNode(const State& state, double g_val, double h_val, StateTimeAStarNode* parent, int conflicts):
        state(state), g_val(g_val), h_val(h_val), parent(parent), conflicts(conflicts), in_openlist(false)
    {
        if(parent != nullptr)
        {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
            if (parent->vis_goal) visit_goal_time = parent->visit_goal_time;
            else visit_goal_time = 0;
            vis_goal = parent->vis_goal;
        }
        else { depth = 0; goal_id = 0; visit_goal_time = 0; vis_goal = false; }
    }

    inline double getFVal() const { return g_val + h_val; }

    struct EqNode
    {
        bool operator() (const StateTimeAStarNode* n1, const StateTimeAStarNode* n2) const
        {
            return (n1 == n2) ||
                   (n1 && n2 && n1->state == n2->state && n1->goal_id == n2->goal_id);
        }
    };
    struct Hasher
    {
        std::size_t operator()(const StateTimeAStarNode* n) const { return State::Hasher()(n->state); }
    };
};

class StateTimeAStar: public SingleAgentSolver
{
public:
    Path run(BasicGraph& G, const State& start, const vector<pair<int, int> >& goal_location, ReservationTable& RT);
private:
    fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::compare_node> > open_list;
    fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::secondary_compare_node> > focal_list;
    unordered_set< StateTimeAStarNode*, StateTimeAStarNode::Hasher, StateTimeAStarNode::EqNode> allNodes_table;
    inline void releaseClosedListNodes()
    {
        for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++) delete (*it);
        allNodes_table.clear();
    }
    Path updatePath(const StateTimeAStarNode* goal, const State& start)
    {
        Path path(goal->state.timestep + 1 - start.timestep);
        path_cost = goal->getFVal();
        num_of_conf = goal->conflicts;
        temp_h_val = goal->h_val;
        const StateTimeAStarNode* curr = goal;
        for(int t = goal->state.timestep - start.timestep; t >= 0; t--)
        {
            path[t] = curr->state;
            path[t].timestep = path[t].timestep - start.timestep;
            curr = curr->parent;
        }
        return path;
    }
};

Path StateTimeAStar::run(BasicGraph& G, const State& start,
    const vector<pair<int, int> >& goal_location, ReservationTable& rt)
{
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();
    double h_val = compute_h_value(G, start.location, 0, goal_location);
    if (h_val > INT_MAX) return Path();
    if (rt.isConstrained(start.location, start.location, 0)) return Path();

    StateTimeAStarNode* root = new StateTimeAStarNode(start, 0, h_val, nullptr, 0);
    num_generated++;
    root->open_handle = open_list.push(root);
    root->focal_handle = focal_list.push(root);
    root->in_openlist = true;
    allNodes_table.insert(root);
    min_f_val = root->getFVal();
    double lower_bound = min_f_val;
    int earliest_holding_time = 0;
    earliest_holding_time = rt.getHoldingTimeFromCT(goal_location.back().first);

    const int cutoff = rt.window;   // reference literal was 10 (== plan_window)

    while (!focal_list.empty())
    {
        StateTimeAStarNode* curr = focal_list.top();
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        if (curr->state.location == goal_location[curr->goal_id].first &&
            curr->state.timestep >= goal_location[curr->goal_id].second &&
            !(curr->goal_id == (int)goal_location.size() - 1
              && earliest_holding_time > curr->state.timestep - start.timestep))
            curr->goal_id++;

        if (curr->goal_id == (int)goal_location.size() || curr->state.timestep >= start.timestep + cutoff)
        {
            Path path = updatePath(curr, start);
            releaseClosedListNodes();
            open_list.clear(); focal_list.clear();
            runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
            return path;
        }
        for (auto next_state: G.get_neighbors(curr->state))
        {
            if (!rt.isConstrained(curr->state.location, next_state.location, next_state.timestep - start.timestep))
            {
                double next_g_val = curr->g_val + G.get_weight(curr->state.location, next_state.location);
                double next_h_val = compute_h_value(G, next_state.location, curr->goal_id, goal_location);
                if (next_h_val >= INT_MAX) continue;
                int next_conflicts = curr->conflicts;
                if (rt.isConflicting(curr->state.location, next_state.location, next_state.timestep - start.timestep))
                    next_conflicts++;
                auto next = new StateTimeAStarNode(next_state, next_g_val, next_h_val, curr, next_conflicts);
                auto it = allNodes_table.find(next);
                if (it == allNodes_table.end())
                {
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    num_generated++;
                    if (next->getFVal() <= lower_bound)
                        next->focal_handle = focal_list.push(next);
                    allNodes_table.insert(next);
                }
                else
                {
                    StateTimeAStarNode* existing_next = *it;
                    if (existing_next->in_openlist)
                    {
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts))
                        {
                            bool add_to_focal = false, update_in_focal = false, update_open = false;
                            if ((next_g_val + next_h_val) <= lower_bound)
                            {
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
                    }
                    else
                    {
                        if (existing_next->getFVal() > next_g_val + next_h_val ||
                            (existing_next->getFVal() == next_g_val + next_h_val && existing_next->conflicts > next_conflicts))
                        {
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
                    delete(next);
                }
            }
        }
        if (open_list.size() == 0) break;
        StateTimeAStarNode* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val)
        {
            double new_min_f_val = open_head->getFVal();
            double new_lower_bound = std::max(lower_bound, new_min_f_val);
            for (StateTimeAStarNode* n : open_list)
                if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
                    n->focal_handle = focal_list.push(n);
            min_f_val = new_min_f_val;
            lower_bound = new_lower_bound;
        }
    }
    releaseClosedListNodes();
    open_list.clear(); focal_list.clear();
    return Path();
}

// ------------------------------------------------------------------ PBSNode
class PBSNode
{
public:
    std::list<Conflict> conflicts;
    Conflict conflict;
    PBSNode* parent;
    list< pair<int, Path> > paths;
    std::pair<int, int> priority;
    PriorityGraph priorities;
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

    PBSNode(): parent(nullptr), g_val(0), h_val(0), f_val(0), depth(0), makespan(0),
        num_of_collisions(0), earliest_collision(INT_MAX), time_expanded(0), time_generated(0) {}
};

// ------------------------------------------------------------------ PBS
class PBS
{
public:
    bool lazyPriority = false;
    bool prioritize_start = false;

    PBSNode* dummy_start = nullptr;
    PBSNode* best_node = nullptr;
    uint64_t HL_num_expanded = 0, HL_num_generated = 0, LL_num_expanded = 0, LL_num_generated = 0;
    double min_f_val = -1;

    // config (from MAPFSolver base in the reference)
    int k_robust = 0;
    int window = 0;
    bool hold_endpoints = false;
    double runtime = 0;
    int screen = 0;
    bool solution_found = false;
    double solution_cost = -2;
    double avg_path_length = -1;
    double min_sum_of_costs = 0;
    vector<Path> solution;
    vector<int> vis_goal_time;
    ReservationTable initial_rt;
    vector<Path> initial_paths;
    std::unordered_map<int, double> travel_times;

    list< tuple<int, int, int> > initial_constraints;

    bool run(const vector<State>& starts,
             const vector< vector<pair<int, int> > >& goal_locations,
             int time_limit);

    PBS(BasicGraph& G, SingleAgentSolver& path_planner)
        : initial_rt(G), G(G), path_planner(path_planner), rt(G) {}
    ~PBS() { release_closed_list(); }

    void update_paths(PBSNode* curr);
    void setRT(bool use_cat, bool ps) { rt.use_cat = use_cat; rt.prioritize_start = ps; }
    void clear();

private:
    BasicGraph& G;
    SingleAgentSolver& path_planner;
    vector<State> starts;
    vector< vector<pair<int, int> > > goal_locations;

    std::vector< Path* > paths;
    list<PBSNode*> allNodes_table;
    list<PBSNode*> dfs;
    std::clock_t start;
    int num_of_agents = 0;
    double min_sum_of_costs_ = 0;
    int max_makespan = 0;
    int time_limit = 0;
    unordered_set<pair<int, int>> nogood;
    ReservationTable rt;

    bool generate_root_node();
    void push_node(PBSNode* node);
    PBSNode* pop_node();
    bool find_path(PBSNode* node, int ag);
    bool find_consistent_paths(PBSNode* node, int a);
    void resolve_conflict(const Conflict& conflict, PBSNode* n1, PBSNode* n2);
    bool generate_child(PBSNode* child, PBSNode* curr);
    void remove_conflicts(list<Conflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<Conflict>& old_conflicts, list<Conflict> & new_conflicts, int new_agent);
    void find_conflicts(list<Conflict> & conflicts, int a1, int a2);
    void find_conflicts(list<Conflict> & new_conflicts, int new_agent);
    void find_conflicts(list<Conflict> & new_conflicts);
    void choose_conflict(PBSNode &parent);
    void copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent);
    double get_path_cost(const Path& path) const;
    void get_solution();
    inline void release_closed_list();
    void update_best_node(PBSNode* node);
    bool wait_at_start(const Path& path, int start_location, int timestep) const;
    void find_replan_agents(PBSNode* node, const list<Conflict>& conflicts, unordered_set<int>& replan);
    bool validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G) const;
};

void PBS::clear()
{
    runtime = 0;
    HL_num_expanded = HL_num_generated = LL_num_expanded = LL_num_generated = 0;
    solution_found = false; solution_cost = -2; min_f_val = -1; avg_path_length = -1;
    paths.clear(); nogood.clear(); dfs.clear();
    release_closed_list();
    starts.clear(); goal_locations.clear(); best_node = nullptr;
}

void PBS::update_paths(PBSNode* curr)
{
    vector<bool> updated(num_of_agents, false);
    while (curr != nullptr)
    {
        for (auto p = curr->paths.begin(); p != curr->paths.end(); ++p)
            if (!updated[std::get<0>(*p)])
            { paths[std::get<0>(*p)] = &(std::get<1>(*p)); updated[std::get<0>(*p)] = true; }
        curr = curr->parent;
    }
}

void PBS::copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent)
{
    for (auto conflict : conflicts)
        if (excluded_agent != std::get<0>(conflict) && excluded_agent != std::get<1>(conflict))
            copy.push_back(conflict);
}

void PBS::find_conflicts(list<Conflict>& conflicts, int a1, int a2)
{
    if (paths[a1] == nullptr || paths[a2] == nullptr) return;
    int size1 = min(window + 1, (int)paths[a1]->size());
    int size2 = min(window + 1, (int)paths[a2]->size());
    int max_size = max(size1, size2);
    for (int timestep = 0; timestep < max_size; timestep++)
    {
        int loc1 = 0, loc2 = 0;
        if (timestep <= size1 - 1) loc1 = paths[a1]->at(timestep).location;
        else loc1 = paths[a1]->at(size1 - 1).location;
        if (timestep <= size2 - 1) loc2 = paths[a2]->at(timestep).location;
        else loc2 = paths[a2]->at(size2 - 1).location;
        if (loc1 == loc2)
        { conflicts.emplace_back(a1, a2, loc1, -1, timestep); return; }
        if (timestep < size1 - 1 && timestep < size2 - 1)
            if (loc1 != loc2 && loc1 == paths[a2]->at(timestep + 1).location
                && loc2 == paths[a1]->at(timestep + 1).location)
            { conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); return; }
    }
}

void PBS::find_conflicts(list<Conflict>& conflicts)
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
            find_conflicts(conflicts, a1, a2);
}

void PBS::find_conflicts(list<Conflict>& new_conflicts, int new_agent)
{
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
        if(new_agent == a2) continue;
        find_conflicts(new_conflicts, new_agent, a2);
    }
}

void PBS::find_conflicts(const list<Conflict>& old_conflicts, list<Conflict>& new_conflicts, int new_agent)
{
    copy_conflicts(old_conflicts, new_conflicts, new_agent);
    find_conflicts(new_conflicts, new_agent);
}

void PBS::remove_conflicts(list<Conflict>& conflicts, int excluded_agent)
{
    for (auto it = conflicts.begin(); it != conflicts.end();)
    {
        if(std::get<0>(*it) == excluded_agent || std::get<1>(*it) == excluded_agent)
            it = conflicts.erase(it);
        else ++it;
    }
}

void PBS::choose_conflict(PBSNode &node)
{
    if (node.conflicts.empty()) return;
    node.conflict = node.conflicts.front();
    for (auto conflict : node.conflicts)
        if (std::get<4>(conflict) < std::get<4>(node.conflict))
            node.conflict = conflict;
    node.earliest_collision = std::get<4>(node.conflict);
    if (!nogood.empty())
        for (auto conflict : node.conflicts)
        {
            int a1 = std::get<0>(conflict);
            int a2 = std::get<1>(conflict);
            for (auto p : nogood)
                if ((a1 == p.first && a2 == p.second) || (a1 == p.second && a2 == p.first))
                { node.conflict = conflict; return; }
        }
}

double PBS::get_path_cost(const Path& path) const
{
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
        cost += G.get_weight(path[i].location, path[i + 1].location) * 1;
    return cost;
}

bool PBS::find_path(PBSNode* node, int agent)
{
    Path path;
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

bool PBS::wait_at_start(const Path& path, int start_location, int timestep) const
{
    for (auto& state : path)
    {
        if (state.timestep > timestep) return true;
        else if (state.location != start_location) return false;
    }
    return false;
}

void PBS::find_replan_agents(PBSNode* node, const list<Conflict>& conflicts, unordered_set<int>& replan)
{
    for (const auto& conflict : conflicts)
    {
        int a1, a2, v1, v2, t;
        std::tie(a1, a2, v1, v2, t) = conflict;
        if (replan.find(a1) != replan.end() || replan.find(a2) != replan.end()) continue;
        else if (prioritize_start && wait_at_start(*paths[a1], v1, t)) { replan.insert(a2); continue; }
        else if (prioritize_start && wait_at_start(*paths[a2], v2, t)) { replan.insert(a1); continue; }
        if (node->priorities.connected(a1, a2)) { replan.insert(a1); continue; }
        if (node->priorities.connected(a2, a1)) { replan.insert(a2); continue; }
    }
}

bool PBS::find_consistent_paths(PBSNode* node, int agent)
{
    int count = 0;
    unordered_set<int> replan;
    if (agent >= 0 && agent < num_of_agents) replan.insert(agent);
    find_replan_agents(node, node->conflicts, replan);
    while (!replan.empty())
    {
        if (count > (int) node->paths.size() * 5) return false;
        int a = *replan.begin();
        replan.erase(a);
        count++;
        if (!find_path(node, a)) return false;
        remove_conflicts(node->conflicts, a);
        list<Conflict> new_conflicts;
        find_conflicts(new_conflicts, a);
        find_replan_agents(node, new_conflicts, replan);
        node->conflicts.splice(node->conflicts.end(), new_conflicts);
    }
    return true;
}

bool PBS::validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &Gp) const
{
    for (auto conflict : conflicts)
    {
        int a1 = std::get<0>(conflict);
        int a2 = std::get<1>(conflict);
        if (Gp.connected(a1, a2)) return false;
        else if (Gp.connected(a2, a1)) return false;
    }
    return true;
}

bool PBS::generate_child(PBSNode* node, PBSNode* parent)
{
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

bool PBS::generate_root_node()
{
    dummy_start = new PBSNode();
    paths.resize(num_of_agents, nullptr);
    dummy_start->path_cost_list.resize(num_of_agents);
    dummy_start->vis_goal_time.resize(num_of_agents);

    if (!initial_paths.empty())
        for (int i = 0; i < num_of_agents; i++)
            if (!initial_paths[i].empty())
            {
                dummy_start->paths.emplace_back(make_pair(i, initial_paths[i]));
                paths[i] = &dummy_start->paths.back().second;
                dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
                dummy_start->g_val += get_path_cost(*paths[i]);
            }

    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] != nullptr) continue;
        Path path;
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
        if(!find_consistent_paths(dummy_start, -1)) return false;
    dummy_start->f_val = dummy_start->g_val + dummy_start->h_val;
    dummy_start->num_of_collisions = dummy_start->conflicts.size();
    min_f_val = dummy_start->f_val;
    best_node = dummy_start;
    push_node(dummy_start);
    return true;
}

void PBS::push_node(PBSNode* node) { dfs.push_back(node); allNodes_table.push_back(node); }
PBSNode* PBS::pop_node() { PBSNode* node = dfs.back(); dfs.pop_back(); return node; }

void PBS::update_best_node(PBSNode* node)
{
    if (node->earliest_collision > best_node->earliest_collision) best_node = node;
    else if (node->earliest_collision == best_node->earliest_collision && node->f_val < best_node->f_val)
        best_node = node;
}

bool PBS::run(const vector<State>& starts_,
              const vector< vector<pair<int, int> > >& goal_locations_,
              int time_limit_)
{
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

    while (!dfs.empty() && !solution_found)
    {
        runtime = (std::clock() - start) * 1.0 / CLOCKS_PER_SEC;
        if (runtime > time_limit)
        { solution_cost = -1; solution_found = false; break; }
        PBSNode* curr = pop_node();
        update_paths(curr);
        if (curr->conflicts.empty())
        { solution_found = true; solution_cost = curr->f_val; best_node = curr; break; }
        choose_conflict(*curr);
        update_best_node(curr);
        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;
        PBSNode* n[2];
        for (int i = 0; i < 2; i++) n[i] = new PBSNode();
        resolve_conflict(curr->conflict, n[0], n[1]);
        vector<Path*> copy(paths);
        for (int i = 0; i < 2; i++)
        {
            bool sol = generate_child(n[i], curr);
            if (sol) { HL_num_generated++; n[i]->time_generated = HL_num_generated; }
            if (sol)
            {
                if (n[i]->f_val == min_f_val && n[i]->num_of_collisions == 0)
                {
                    solution_found = true; solution_cost = n[i]->f_val; best_node = n[i];
                    allNodes_table.push_back(n[i]); break;
                }
            }
            else { delete (n[i]); n[i] = nullptr; }
            paths = copy;
        }

        if (!solution_found)
        {
            if (n[0] != nullptr && n[1] != nullptr)
            {
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

void PBS::resolve_conflict(const Conflict& conflict, PBSNode* n1, PBSNode* n2)
{
    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = conflict;
    n1->priority = std::make_pair(a1, a2);
    n2->priority = std::make_pair(a2, a1);
    (void)v1; (void)v2; (void)t;
}

inline void PBS::release_closed_list()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++) delete *it;
    allNodes_table.clear();
}

void PBS::get_solution()
{
    update_paths(best_node);
    solution.resize(num_of_agents);
    vis_goal_time.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        solution[k] = *paths[k];
        vis_goal_time[k] = best_node->vis_goal_time[k];
    }
    avg_path_length = 0;
    for (int k = 0; k < num_of_agents; k++)
    {
        if (goal_locations[k].size() == 0) continue;
        avg_path_length += paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}

} // namespace refsolve

// ============================================================================
// Adapter — build the reference graph + solve, convert paths back to plain ints.
// ============================================================================
bool ref_wpbs_solve(int rows, int cols,
                    const std::vector<char>& passable,
                    const std::vector<int>& start_locs,
                    const std::vector<std::vector<std::pair<int,int>>>& goal_seqs,
                    int cur_time, int window, int time_limit_ms,
                    std::vector<std::vector<int>>& out_paths)
{
    using namespace refsolve;
    int n = (int)start_locs.size();
    out_paths.assign(n, {});

    BasicGraph G;
    G.rows = rows; G.cols = cols;
    G.passable = passable;
    G.types.assign((size_t)rows * cols, "Travel");
    for (int i = 0; i < rows * cols; i++)
        if (!passable[i]) G.types[i] = "Obstacle";

    StateTimeAStar planner;
    PBS pbs(G, planner);
    pbs.lazyPriority = false;
    pbs.prioritize_start = false;
    pbs.hold_endpoints = false;
    pbs.k_robust = 0;
    pbs.window = window;
    pbs.screen = 0;
    pbs.setRT(false /*use_cat*/, false /*prioritize_start*/);
    pbs.initial_rt.k_robust = 0;
    pbs.initial_rt.window = window;
    pbs.initial_rt.hold_endpoints = false;
    pbs.initial_rt.use_cat = false;

    std::vector<State> starts;
    starts.reserve(n);
    for (int i = 0; i < n; i++)
        starts.emplace_back(start_locs[i], cur_time, -1);  // absolute start time

    // goal_locations for the reference (loc, release_relative)
    std::vector<std::vector<std::pair<int,int>>> goals = goal_seqs;

    int tl_sec = time_limit_ms / 1000;
    if (tl_sec < 1) tl_sec = 1;              // run()'s time_limit is in SECONDS
    bool ok = pbs.run(starts, goals, tl_sec);
    (void)ok;

    for (int i = 0; i < n; i++)
    {
        if (i < (int)pbs.solution.size() && !pbs.solution[i].empty())
        {
            const Path& p = pbs.solution[i];
            out_paths[i].resize(p.size());
            for (int t = 0; t < (int)p.size(); t++)
                out_paths[i][t] = p[t].location;
        }
        else
        {
            out_paths[i] = { start_locs[i] };
        }
    }
    return true;
}
