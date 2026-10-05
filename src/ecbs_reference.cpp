// Port of the ECBS of Ma's CENTRAL reference code ("Centralized - ECBS"). The
// structure, heaps, comparators, successor order and focal-list updates follow
// ecbs_search.cpp, single_agent_ecbs.cpp, node.h and ecbs_node.h, so equal
// inputs give the reference's paths. Departures: nodes live in pools instead of
// leaking, an initial path that cannot be found fails the search (the
// reference goes on with an empty path), indices past a constraint path's end
// read its last cell, and the high level stops after `expansion_limit` nodes.
#include "ecbs_reference.h"

#include <boost/heap/fibonacci_heap.hpp>
#include <climits>
#include <map>
#include <memory>

using std::get;
using std::list;
using std::make_pair;
using std::make_tuple;
using std::map;
using std::pair;
using std::tuple;
using std::unique_ptr;
using std::vector;

namespace ecbs_reference {
namespace {

using boost::heap::compare;
using boost::heap::fibonacci_heap;

// --- node.h -----------------------------------------------------------------
struct Node {
    int loc;
    double g_val;
    double h_val = 0;
    Node* parent;
    int timestep = 0;
    int num_internal_conf = 0;  // used in ECBS
    bool in_openlist = false;

    // OPEN: min f-val, then larger g-val.
    struct compare_node {
        bool operator()(const Node* n1, const Node* n2) const {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val;
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };
    // FOCAL: fewest internal conflicts, then min f-val, then larger g-val.
    struct secondary_compare_node {
        bool operator()(const Node* n1, const Node* n2) const {
            if (n1->num_internal_conf == n2->num_internal_conf) {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    return n1->g_val <= n2->g_val;
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            return n1->num_internal_conf >= n2->num_internal_conf;
        }
    };
    typedef fibonacci_heap<Node*, compare<compare_node>>::handle_type open_handle_t;
    typedef fibonacci_heap<Node*, compare<secondary_compare_node>>::handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;

    Node(int loc, double g_val, double h_val, Node* parent, int timestep,
         int num_internal_conf = 0, bool in_openlist = false)
        : loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
          num_internal_conf(num_internal_conf), in_openlist(in_openlist) {}
    double getFVal() const { return g_val + h_val; }
};

typedef vector<list<pair<int, int>>> ConstraintsByTime;  // cons[t]: <loc1, loc2>, loc2 = -1 vertex

// --- single_agent_ecbs.cpp --------------------------------------------------
class SingleAgentECBS {
public:
    typedef fibonacci_heap<Node*, compare<Node::compare_node>> heap_open_t;
    typedef fibonacci_heap<Node*, compare<Node::secondary_compare_node>> heap_focal_t;

    vector<int> path;
    double path_cost = 0;
    double lower_bound = 0;
    double min_f_val = 0;

    SingleAgentECBS(const vector<vector<int>>& cons_paths, const vector<int>& my_heuristic,
                    const vector<bool>& my_map, int start_location, int goal_location,
                    int col, int curr_time, int max_time)
        : cons_paths(cons_paths), my_heuristic(my_heuristic), my_map(my_map),
          start_location(start_location), goal_location(goal_location),
          curr_time(curr_time), max_time(max_time), map_size((int)my_map.size()) {
        actions_offset[0] = 0; actions_offset[1] = -col; actions_offset[2] = 1;
        actions_offset[3] = col; actions_offset[4] = -1;   // WAIT, NORTH, EAST, SOUTH, WEST
    }

    bool findPath(double f_weight, const ConstraintsByTime* constraints,
                  const vector<char>& res_table, int max_plan_len);

private:
    const vector<vector<int>>& cons_paths;
    const vector<int>& my_heuristic;
    const vector<bool>& my_map;
    int start_location, goal_location, curr_time, max_time, map_size;
    int actions_offset[5];
    heap_open_t open_list;
    heap_focal_t focal_list;
    vector<unique_ptr<Node>> pool;

    Node* make_node(int loc, double g, double h, Node* parent, int timestep, int conf) {
        pool.emplace_back(new Node(loc, g, h, parent, timestep, conf, false));
        return pool.back().get();
    }
    int cons_at(const vector<int>& p, int t) const {
        return p[t < (int)p.size() ? t : (int)p.size() - 1];
    }

    void updatePath(const Node* goal) {
        path.clear();
        const Node* curr = goal;
        while (curr->timestep != 0) {
            path.push_back(curr->loc);
            curr = curr->parent;
        }
        path.push_back(start_location);
        std::reverse(path.begin(), path.end());
        path_cost = goal->g_val;
    }

    // The latest timestep with a constraint involving the goal location.
    static int extractLastGoalTimestep(int goal_location, const ConstraintsByTime* cons) {
        if (cons != nullptr)
            for (int t = (int)cons->size() - 1; t > 0; t--)
                for (const auto& c : cons->at(t))
                    if (c.first == goal_location || c.second == goal_location) return t;
        return -1;
    }

    bool isConstrained(int curr_loc, int next_loc, int next_timestep,
                       const ConstraintsByTime* cons) const {
        if (!my_map[next_loc]) return true;
        // the committed paths of the agents not planned here
        for (const vector<int>& p : cons_paths) {
            if (cons_at(p, curr_time + next_timestep) == next_loc) return true;
            if (cons_at(p, curr_time + next_timestep) == curr_loc &&
                cons_at(p, curr_time + next_timestep - 1) == next_loc) return true;
        }
        if (cons == nullptr) return false;
        if (next_timestep < (int)cons->size())
            for (const auto& c : cons->at(next_timestep))
                if (c.second == -1 && c.first == next_loc) return true;
        if (next_timestep > 0 && next_timestep - 1 < (int)cons->size())
            for (const auto& c : cons->at(next_timestep - 1))
                if (c.first == curr_loc && c.second == next_loc) return true;
        return false;
    }

    int numOfConflictsForStep(int curr_id, int next_id, int next_timestep,
                              const vector<char>& res_table, int max_plan_len) const {
        int retVal = 0;
        if (next_timestep >= max_plan_len) {
            if (res_table[next_id + (max_plan_len - 1) * map_size]) retVal++;
        } else {
            if (res_table[next_id + next_timestep * map_size]) retVal++;
            if (res_table[curr_id + next_timestep * map_size] &&
                res_table[next_id + (next_timestep - 1) * map_size]) retVal++;
        }
        return retVal;
    }

    void updateFocalList(double old_lower_bound, double new_lower_bound) {
        for (Node* n : open_list)
            if (n->getFVal() > old_lower_bound && n->getFVal() <= new_lower_bound)
                n->focal_handle = focal_list.push(n);
    }
};

bool SingleAgentECBS::findPath(double f_weight, const ConstraintsByTime* constraints,
                               const vector<char>& res_table, int max_plan_len) {
    open_list.clear();
    focal_list.clear();
    pool.clear();
    map<unsigned int, Node*> allNodes_table;   // key: loc + g_val * map_size

    Node* start = make_node(start_location, 0, my_heuristic[start_location], nullptr, 0, 0);
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(make_pair((unsigned int)start_location, start));
    min_f_val = start->getFVal();
    lower_bound = f_weight * min_f_val;

    int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints);

    while (!focal_list.empty()) {
        Node* curr = focal_list.top();
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;

        if (curr->loc == goal_location && curr->timestep > lastGoalConsTime) {
            // the agent must be able to stay at its goal for good
            bool hold = true;
            for (size_t ag = 0; ag < cons_paths.size() && hold; ag++)
                for (size_t t = curr->timestep + curr_time + 1;
                     t < cons_paths[ag].size() && hold; t++)
                    if (cons_paths[ag][t] == curr->loc) hold = false;
            if (hold) {
                updatePath(curr);
                return true;
            }
        }

        for (int direction = 0; direction < 5; direction++) {
            int next_id = curr->loc + actions_offset[direction];
            int next_timestep = curr->timestep + 1;
            if (next_id < 0 || next_id >= map_size) continue;
            if (isConstrained(curr->loc, next_id, next_timestep, constraints)) continue;
            double next_g_val = curr->g_val + 1;
            double next_h_val = my_heuristic[next_id];
            int next_internal_conflicts = 0;
            if (max_plan_len > 0)
                next_internal_conflicts = curr->num_internal_conf +
                    numOfConflictsForStep(curr->loc, next_id, next_timestep, res_table,
                                          max_plan_len);
            if (!(next_g_val < max_time - curr_time)) continue;
            auto it = allNodes_table.find(
                (unsigned int)(next_id + next_g_val * my_map.size()));
            if (it == allNodes_table.end()) {
                Node* next = make_node(next_id, next_g_val, next_h_val, curr, next_timestep,
                                       next_internal_conflicts);
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                if (next->getFVal() <= lower_bound)
                    next->focal_handle = focal_list.push(next);
                allNodes_table.insert(make_pair(
                    (unsigned int)(next->loc + next->g_val * my_map.size()), next));
                continue;
            }
            Node* existing_next = it->second;
            const double next_f = next_g_val + next_h_val;
            const bool better = existing_next->getFVal() > next_f ||
                (existing_next->getFVal() == next_f &&
                 existing_next->num_internal_conf > next_internal_conflicts);
            if (!better) continue;
            if (existing_next->in_openlist) {
                bool add_to_focal = false, update_in_focal = false, update_open = false;
                if (next_f <= lower_bound) {
                    if (existing_next->getFVal() > lower_bound) add_to_focal = true;
                    else update_in_focal = true;
                }
                if (existing_next->getFVal() > next_f) update_open = true;
                existing_next->g_val = next_g_val;
                existing_next->h_val = next_h_val;
                existing_next->parent = curr;
                existing_next->num_internal_conf = next_internal_conflicts;
                if (update_open) open_list.increase(existing_next->open_handle);
                if (add_to_focal) existing_next->focal_handle = focal_list.push(existing_next);
                if (update_in_focal) focal_list.update(existing_next->focal_handle);
            } else {   // reopen a closed node
                existing_next->g_val = next_g_val;
                existing_next->h_val = next_h_val;
                existing_next->parent = curr;
                existing_next->num_internal_conf = next_internal_conflicts;
                existing_next->open_handle = open_list.push(existing_next);
                existing_next->in_openlist = true;
                if (existing_next->getFVal() <= lower_bound)
                    existing_next->focal_handle = focal_list.push(existing_next);
            }
        }

        if (open_list.empty()) return false;
        Node* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val) {
            double new_min_f_val = open_head->getFVal();
            double new_lower_bound = f_weight * new_min_f_val;
            updateFocalList(lower_bound, new_lower_bound);
            min_f_val = new_min_f_val;
            lower_bound = new_lower_bound;
        }
    }
    path.clear();
    return false;
}

// --- ecbs_node.h --------------------------------------------------------------
struct ECBSNode {
    int agent_id = -1;
    tuple<int, int, int> constraint = make_tuple(-1, -1, -1);  // <loc1, loc2, t>, loc2 = -1 vertex
    ECBSNode* parent = nullptr;
    vector<int> path;
    double g_val = 0;           // total cost
    double h_val = 0;           // number of colliding agents
    int time_expanded = -1;
    int time_generated = -1;
    double sum_min_f_vals = -1;
    double ll_min_f_val = 0;
    double path_cost = 0;

    struct compare_node {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const {
            return n1->sum_min_f_vals >= n2->sum_min_f_vals;
        }
    };
    struct secondary_compare_node {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const {
            if (n1->h_val == n2->h_val) return n1->g_val >= n2->g_val;
            return n1->h_val >= n2->h_val;
        }
    };
    typedef fibonacci_heap<ECBSNode*, compare<compare_node>>::handle_type open_handle_t;
    typedef fibonacci_heap<ECBSNode*, compare<secondary_compare_node>>::handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;
};

}  // namespace

// --- ecbs_search.cpp ----------------------------------------------------------
struct ECBSSearch::Impl {
    typedef fibonacci_heap<ECBSNode*, compare<ECBSNode::compare_node>> heap_open_t;
    typedef fibonacci_heap<ECBSNode*, compare<ECBSNode::secondary_compare_node>> heap_focal_t;

    int num_of_agents, map_size;
    double focal_w, focal_list_threshold = 0, min_sum_f_vals = 0;
    vector<vector<int>> paths, paths_found_initially;
    vector<double> ll_min_f_vals, ll_min_f_vals_found_initially;
    vector<double> paths_costs, paths_costs_found_initially;
    vector<unique_ptr<SingleAgentECBS>> search_engines;
    vector<unique_ptr<ECBSNode>> nodes;
    heap_open_t open_list;
    heap_focal_t focal_list;
    ECBSNode* dummy_start = nullptr;
    tuple<int, int, int, int, int> earliest_conflict;
    std::uint64_t HL_num_expanded = 0;

    int getAgentLocation(int agent_id, size_t timestep) const {
        if (timestep >= paths[agent_id].size())
            return paths[agent_id][paths[agent_id].size() - 1];
        return paths[agent_id][timestep];
    }
    bool switchedLocations(int a1, int a2, size_t timestep) const {
        if (timestep >= paths[a1].size() && timestep >= paths[a2].size()) return false;
        return getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep + 1) &&
               getAgentLocation(a1, timestep + 1) == getAgentLocation(a2, timestep);
    }
    size_t getPathsMaxLength() const {
        size_t retVal = 0;
        for (int ag = 0; ag < num_of_agents; ag++)
            if (!paths[ag].empty() && paths[ag].size() > retVal) retVal = paths[ag].size();
        return retVal;
    }
    void updateReservationTable(vector<char>& res_table, size_t max_path_len, int exclude_agent) const {
        for (int ag = 0; ag < num_of_agents; ag++)
            if (ag != exclude_agent && !paths[ag].empty())
                for (size_t t = 0; t < max_path_len; t++)
                    res_table[getAgentLocation(ag, t) + t * map_size] = 1;
    }
    // Whether any two paths collide; the earliest collision (first pair in
    // index order at the earliest timestep, vertex before edge) is kept.
    bool extractCollisions() {
        bool found = false;
        earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
        for (int a1 = 0; a1 < num_of_agents; a1++)
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
                size_t max_len = std::max(paths[a1].size(), paths[a2].size());
                for (size_t t = 0; t < max_len; t++) {
                    if (getAgentLocation(a1, t) == getAgentLocation(a2, t)) {
                        found = true;
                        if ((int)t < get<4>(earliest_conflict))
                            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, t), -1, (int)t);
                        break;   // later collisions of this pair are not earlier
                    }
                    if (switchedLocations(a1, a2, t)) {
                        found = true;
                        if ((int)t < get<4>(earliest_conflict))
                            earliest_conflict = make_tuple(a1, a2, getAgentLocation(a1, t),
                                                           getAgentLocation(a2, t), (int)t);
                        break;
                    }
                }
            }
        return found;
    }
    // As the reference: for each a1, 1 if it collides with some later agent.
    int computeNumOfCollidingAgents() const {
        int retVal = 0;
        for (int a1 = 0; a1 < num_of_agents; a1++)
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
                size_t max_len = std::max(paths[a1].size(), paths[a2].size());
                bool collide = false;
                for (size_t t = 0; t < max_len && !collide; t++)
                    collide = getAgentLocation(a1, t) == getAgentLocation(a2, t) ||
                              switchedLocations(a1, a2, t);
                if (collide) { retVal++; break; }
            }
        return retVal;
    }
    void updatePaths(ECBSNode* curr) {
        paths = paths_found_initially;
        ll_min_f_vals = ll_min_f_vals_found_initially;
        paths_costs = paths_costs_found_initially;
        vector<bool> updated(num_of_agents, false);
        while (curr != dummy_start) {
            if (!updated[curr->agent_id]) {
                paths[curr->agent_id] = curr->path;
                ll_min_f_vals[curr->agent_id] = curr->ll_min_f_val;
                paths_costs[curr->agent_id] = curr->path_cost;
                updated[curr->agent_id] = true;
            }
            curr = curr->parent;
        }
    }
    bool updateECBSNode(ECBSNode* leaf_node) {
        list<tuple<int, int, int>> constraints;
        int agent_id = leaf_node->agent_id;
        for (ECBSNode* curr = leaf_node; curr != dummy_start; curr = curr->parent)
            if (curr->agent_id == agent_id) constraints.push_front(curr->constraint);
        int max_timestep = -1;
        for (const auto& c : constraints) max_timestep = std::max(max_timestep, get<2>(c));
        ConstraintsByTime cons_vec(max_timestep + 1);
        for (const auto& c : constraints)
            cons_vec[get<2>(c)].push_back(make_pair(get<0>(c), get<1>(c)));
        size_t max_plan_len = getPathsMaxLength();
        vector<char> res_table(map_size * max_plan_len, 0);
        updateReservationTable(res_table, max_plan_len, agent_id);
        SingleAgentECBS& engine = *search_engines[agent_id];
        if (!engine.findPath(focal_w, &cons_vec, res_table, (int)max_plan_len)) return false;
        leaf_node->path = engine.path;
        leaf_node->ll_min_f_val = engine.min_f_val;
        leaf_node->path_cost = engine.path_cost;
        return true;
    }
    void updateFocalList(double old_lower_bound, double new_lower_bound) {
        for (ECBSNode* n : open_list)
            if (n->sum_min_f_vals > old_lower_bound && n->sum_min_f_vals <= new_lower_bound)
                n->focal_handle = focal_list.push(n);
    }
    ECBSNode* new_node() {
        nodes.emplace_back(new ECBSNode());
        return nodes.back().get();
    }
    // A child resolving the conflict for its agent, pushed if a path exists.
    void generate(ECBSNode* curr, ECBSNode* child) {
        if (!updateECBSNode(child)) return;
        child->g_val = curr->g_val - paths_costs[child->agent_id] + child->path_cost;
        vector<int> temp_old_path = paths[child->agent_id];
        paths[child->agent_id] = child->path;
        child->h_val = computeNumOfCollidingAgents();
        paths[child->agent_id] = temp_old_path;
        child->sum_min_f_vals = curr->sum_min_f_vals - ll_min_f_vals[child->agent_id] +
                                child->ll_min_f_val;
        child->open_handle = open_list.push(child);
        if (child->sum_min_f_vals <= focal_list_threshold)
            child->focal_handle = focal_list.push(child);
    }
};

ECBSSearch::ECBSSearch(const vector<bool>& map, const vector<AgentGoal>& agents,
                       const vector<vector<int>>& cons_paths, int curr_time, int col,
                       int max_time, double focal_w, std::uint64_t expansion_limit)
    : map_(map), agents_(agents), cons_paths_(cons_paths), curr_time_(curr_time), col_(col),
      max_time_(max_time), focal_w_(focal_w), expansion_limit_(expansion_limit) {}

bool ECBSSearch::run() {
    Impl s;
    s.num_of_agents = (int)agents_.size();
    s.map_size = (int)map_.size();
    s.focal_w = focal_w_;
    s.ll_min_f_vals.assign(s.num_of_agents, 0);
    s.paths_costs.assign(s.num_of_agents, 0);
    s.ll_min_f_vals_found_initially.assign(s.num_of_agents, 0);
    s.paths_costs_found_initially.assign(s.num_of_agents, 0);
    for (const AgentGoal& agent : agents_)
        s.search_engines.emplace_back(new SingleAgentECBS(cons_paths_, *agent.h, map_,
            agent.start, agent.goal, col_, curr_time_, max_time_));

    // Initial paths, each avoiding (softly) the agents planned before it.
    s.paths_found_initially.assign(s.num_of_agents, {});
    for (int i = 0; i < s.num_of_agents; i++) {
        s.paths = s.paths_found_initially;
        size_t max_plan_len = s.getPathsMaxLength();
        vector<char> res_table(s.map_size * max_plan_len, 0);
        s.updateReservationTable(res_table, max_plan_len, i);
        if (!s.search_engines[i]->findPath(focal_w_, nullptr, res_table, (int)max_plan_len))
            return false;
        s.paths_found_initially[i] = s.search_engines[i]->path;
        s.ll_min_f_vals_found_initially[i] = s.search_engines[i]->min_f_val;
        s.paths_costs_found_initially[i] = s.search_engines[i]->path_cost;
    }
    s.paths = s.paths_found_initially;
    s.ll_min_f_vals = s.ll_min_f_vals_found_initially;
    s.paths_costs = s.paths_costs_found_initially;

    s.dummy_start = s.new_node();
    s.dummy_start->agent_id = -1;
    s.dummy_start->g_val = 0;
    for (int i = 0; i < s.num_of_agents; i++) s.dummy_start->g_val += s.paths_costs[i];
    s.dummy_start->ll_min_f_val = 0;
    s.dummy_start->sum_min_f_vals = 0;
    for (int i = 0; i < s.num_of_agents; i++) s.dummy_start->sum_min_f_vals += s.ll_min_f_vals[i];
    s.dummy_start->open_handle = s.open_list.push(s.dummy_start);
    s.dummy_start->focal_handle = s.focal_list.push(s.dummy_start);
    s.min_sum_f_vals = s.dummy_start->sum_min_f_vals;
    s.focal_list_threshold = focal_w_ * s.dummy_start->sum_min_f_vals;

    while (!s.focal_list.empty()) {
        if (s.HL_num_expanded >= expansion_limit_) return false;
        ECBSNode* curr = s.focal_list.top();
        s.focal_list.pop();
        s.open_list.erase(curr->open_handle);
        s.HL_num_expanded++;
        curr->time_expanded = (int)s.HL_num_expanded;
        s.updatePaths(curr);
        if (!s.extractCollisions()) {
            paths = s.paths;
            return true;
        }
        int agent1_id, agent2_id, location1, location2, timestep;
        std::tie(agent1_id, agent2_id, location1, location2, timestep) = s.earliest_conflict;
        ECBSNode* n1 = s.new_node();
        ECBSNode* n2 = s.new_node();
        n1->agent_id = agent1_id;
        n2->agent_id = agent2_id;
        if (location2 == -1) {   // vertex constraint
            n1->constraint = make_tuple(location1, -1, timestep);
            n2->constraint = make_tuple(location1, -1, timestep);
        } else {                 // edge constraint
            n1->constraint = make_tuple(location1, location2, timestep);
            n2->constraint = make_tuple(location2, location1, timestep);
        }
        n1->parent = curr;
        n2->parent = curr;
        s.generate(curr, n1);
        s.generate(curr, n2);
        if (s.open_list.empty()) return false;
        ECBSNode* open_head = s.open_list.top();
        if (open_head->sum_min_f_vals > s.min_sum_f_vals) {
            s.min_sum_f_vals = open_head->sum_min_f_vals;
            double new_focal_list_threshold = s.min_sum_f_vals * focal_w_;
            s.updateFocalList(s.focal_list_threshold, new_focal_list_threshold);
            s.focal_list_threshold = new_focal_list_threshold;
        }
    }
    return false;
}

}  // namespace ecbs_reference
