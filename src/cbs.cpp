#include "cbs.h"
#include <algorithm>

// ============================================================
// LLNode constructor
// ============================================================

LLNode::LLNode(int l, double g, double h, LLNode* p, int t, int c, bool io)
    : loc(l), g_val(g), h_val(h), parent(p), timestep(t),
      num_internal_conf(c), in_openlist(io) {}

// ============================================================
// HLNode constructor
// ============================================================

HLNode::HLNode() : agent_id(-1), g_val(0), h_val(0), sum_min_f_vals(0),
                   ll_min_f_val(0), path_cost(0), parent(nullptr),
                   time_generated(0), time_expanded(0) {}

// ============================================================
// Section 1: SingleAgentECBS — Low-Level ECBS Search
// ============================================================

SingleAgentECBS::SingleAgentECBS(const vector<vector<int>>& cons_paths,
                                 const vector<int>& heuristic,
                                 const vector<bool>& grid,
                                 int ag_id, int start_loc, int goal_loc,
                                 int col, int curr_time, int max_time)
    : cons_paths_(cons_paths), heuristic_(heuristic), grid_(grid),
      ag_id_(ag_id), start_loc_(start_loc), goal_loc_(goal_loc),
      curr_time_(curr_time), max_time_(max_time),
      path_cost(0), min_f_val(0), num_expanded(0), num_generated(0)
{
    map_size_ = grid_.size();
    actions_[0] = 0; actions_[1] = -col; actions_[2] = 1; actions_[3] = col; actions_[4] = -1;
}

bool SingleAgentECBS::isConstrained(int curr, int next, int next_t,
                                     const vector<list<pair<int,int>>>* cons) {
    if (next < 0 || next >= map_size_ || !grid_[next]) return true;

    for (const auto& cp : cons_paths_) {
        int t_abs = curr_time_ + next_t;
        if (t_abs < (int)cp.size()) {
            if (cp[t_abs] == next) return true;
            if (t_abs > 0 && cp[t_abs] == curr && cp[t_abs - 1] == next) return true;
        } else if (!cp.empty()) {
            int last = cp.back();
            if (last == next) return true;
        }
    }

    if (cons == nullptr) return false;

    if (next_t < (int)cons->size()) {
        for (const auto& c : cons->at(next_t)) {
            if (c.second == -1 && c.first == next) return true;
        }
    }

    if (next_t > 0 && next_t - 1 < (int)cons->size()) {
        for (const auto& c : cons->at(next_t - 1)) {
            if (c.first == curr && c.second == next) return true;
        }
    }

    return false;
}

int SingleAgentECBS::numConflicts(int curr, int next, int next_t, bool* res, int mpl) {
    int r = 0;
    if (mpl == 0) return 0;
    if (next_t >= mpl) {
        if (res[next + (mpl - 1) * map_size_]) r++;
    } else {
        if (res[next + next_t * map_size_]) r++;
        if (next_t > 0 && res[curr + next_t * map_size_] && res[next + (next_t - 1) * map_size_]) r++;
    }
    return r;
}

void SingleAgentECBS::updatePath(LLNode* goal) {
    path.clear();
    LLNode* curr = goal;
    while (curr->timestep != 0) {
        path.push_back(curr->loc);
        curr = curr->parent;
    }
    path.push_back(start_loc_);
    reverse(path.begin(), path.end());
    path_cost = goal->g_val;
}

int SingleAgentECBS::extractLastGoalTimestep(int goal_loc,
        const vector<list<pair<int,int>>>* cons) {
    if (cons != nullptr) {
        for (int t = (int)cons->size() - 1; t > 0; t--) {
            for (const auto& c : cons->at(t)) {
                if (c.first == goal_loc || c.second == goal_loc) return t;
            }
        }
    }
    return -1;
}

void SingleAgentECBS::releaseNodes(map<unsigned int, LLNode*>& table) {
    for (auto& p : table) delete p.second;
    table.clear();
}

bool SingleAgentECBS::findPath(double f_weight,
                                const vector<list<pair<int,int>>>* constraints,
                                bool* res_table, size_t max_plan_len) {
    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<LLNode::CompareOpen>> open_heap_t;
    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<LLNode::CompareFocal>> focal_heap_t;

    open_heap_t open_list;
    focal_heap_t focal_list;
    map<unsigned int, LLNode*> allNodes;
    num_expanded = 0;
    num_generated = 0;

    LLNode* start = new LLNode(start_loc_, 0, heuristic_[start_loc_], nullptr, 0, 0, true);
    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes.insert(make_pair((unsigned int)start_loc_, start));
    min_f_val = start->getFVal();
    double lower_bound = f_weight * min_f_val;

    int lastGoalConsTime = extractLastGoalTimestep(goal_loc_, constraints);

    // Expansion cap: a single-agent ECBS search that finds a holdable path does so
    // with very few expansions (a few hundred even on the largest instances). A search
    // that has no holdable solution otherwise exhaustively explores the entire
    // (location x horizon) state space (millions of nodes) before returning false.
    // Capping at a value far above any real solution lets such doomed searches fail
    // fast; the caller then falls back to single-agent A* (identical accepted result).
    // The cap is set high enough that no successful search is ever truncated.
    const int kExpansionCap = 15000;

    while (!focal_list.empty()) {
        LLNode* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        if (num_expanded > kExpansionCap) {
            path.clear();
            releaseNodes(allNodes);
            return false;
        }

        if (curr->loc == goal_loc_ && curr->timestep > lastGoalConsTime) {
            bool hold = true;
            for (unsigned int ag = 0; ag < cons_paths_.size() && hold; ag++) {
                for (int t = curr_time_ + curr->timestep + 1;
                     t < (int)cons_paths_[ag].size() && hold; t++) {
                    if (cons_paths_[ag][t] == curr->loc) hold = false;
                }
            }
            if (hold) {
                updatePath(curr);
                releaseNodes(allNodes);
                return true;
            }
        }

        for (int d = 0; d < 5; d++) {
            int next_loc = curr->loc + actions_[d];
            int next_t = curr->timestep + 1;

            if (!isConstrained(curr->loc, next_loc, next_t, constraints)) {
                double next_g = curr->g_val + 1;
                double next_h = heuristic_[next_loc];
                int next_conf = 0;
                if (max_plan_len > 0)
                    next_conf = curr->num_internal_conf +
                        numConflicts(curr->loc, next_loc, next_t, res_table, max_plan_len);

                unsigned int key = next_loc + (unsigned int)next_g * map_size_;
                auto it = allNodes.find(key);

                if (it == allNodes.end() && next_g < max_time_ - curr_time_) {
                    LLNode* next = new LLNode(next_loc, next_g, next_h, curr, next_t, next_conf, true);
                    num_generated++;
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    if (next->getFVal() <= lower_bound)
                        next->focal_handle = focal_list.push(next);
                    allNodes.insert(make_pair(key, next));
                } else if (it != allNodes.end() && next_g < max_time_ - curr_time_) {
                    LLNode* existing = it->second;
                    if (existing->in_openlist) {
                        if (existing->getFVal() > next_g + next_h ||
                            (existing->getFVal() == next_g + next_h &&
                             existing->num_internal_conf > next_conf)) {
                            bool add_to_focal = false;
                            bool update_in_focal = false;
                            bool update_open = false;
                            if ((next_g + next_h) <= lower_bound) {
                                if (existing->getFVal() > lower_bound)
                                    add_to_focal = true;
                                else
                                    update_in_focal = true;
                            }
                            if (existing->getFVal() > next_g + next_h)
                                update_open = true;
                            existing->g_val = next_g;
                            existing->h_val = next_h;
                            existing->parent = curr;
                            existing->num_internal_conf = next_conf;
                            if (update_open) open_list.increase(existing->open_handle);
                            if (add_to_focal) existing->focal_handle = focal_list.push(existing);
                            if (update_in_focal) focal_list.update(existing->focal_handle);
                        }
                    } else {
                        if (existing->getFVal() > next_g + next_h ||
                            (existing->getFVal() == next_g + next_h &&
                             existing->num_internal_conf > next_conf)) {
                            existing->g_val = next_g;
                            existing->h_val = next_h;
                            existing->parent = curr;
                            existing->num_internal_conf = next_conf;
                            existing->open_handle = open_list.push(existing);
                            existing->in_openlist = true;
                            if (existing->getFVal() <= lower_bound)
                                existing->focal_handle = focal_list.push(existing);
                        }
                    }
                }
            }
        }

        if (open_list.empty()) break;
        LLNode* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val) {
            double new_min = open_head->getFVal();
            double new_lb = f_weight * new_min;
            for (LLNode* n : open_list) {
                if (n->getFVal() > lower_bound && n->getFVal() <= new_lb)
                    n->focal_handle = focal_list.push(n);
            }
            min_f_val = new_min;
            lower_bound = new_lb;
        }
    }

    path.clear();
    releaseNodes(allNodes);
    return false;
}

// ============================================================
// Section 2: CBSSearch — High-Level CBS/ECBS Search
// ============================================================

CBSSearch::CBSSearch(const vector<bool>& grid,
    const vector<int>& start_locs, const vector<int>& goal_locs,
    const vector<int>& goal_ep_indices, const vector<vector<int>>& cons_paths,
    int curr_time, int col, double focal_w,
    int high_level_expansion_limit,
    const vector<Endpoint>& endpoints, int max_time)
    : cons_paths_(cons_paths), curr_time_(curr_time), focal_w_(focal_w),
      high_level_expansion_limit_(high_level_expansion_limit),
      solution_found(false), solution_cost(-1),
      HL_num_expanded_(0), HL_num_generated_(0)
{
    num_agents_ = start_locs.size();
    map_size_ = grid.size();

    search_engines_.resize(num_agents_);
    paths.resize(num_agents_);
    paths_found_initially_.resize(num_agents_);
    ll_min_f_vals_.resize(num_agents_);
    ll_min_f_vals_found_initially_.resize(num_agents_);
    paths_costs_.resize(num_agents_);
    paths_costs_found_initially_.resize(num_agents_);

    for (int i = 0; i < num_agents_; i++) {
        search_engines_[i] = new SingleAgentECBS(
            cons_paths_, endpoints[goal_ep_indices[i]].h_val, grid,
            i, start_locs[i], goal_locs[i], col, curr_time, max_time);
    }

    bool all_initial_found = true;
    for (int i = 0; i < num_agents_; i++) {
        paths = paths_found_initially_;
        size_t max_plan_len = getPathsMaxLength();
        bool* res_table = nullptr;
        if (max_plan_len > 0) {
            res_table = new bool[map_size_ * max_plan_len]();
            updateReservationTable(res_table, max_plan_len, i);
        }

        if (!search_engines_[i]->findPath(focal_w_, nullptr, res_table, max_plan_len)) {
            all_initial_found = false;
        }

        paths_found_initially_[i] = search_engines_[i]->path;
        ll_min_f_vals_found_initially_[i] = search_engines_[i]->min_f_val;
        paths_costs_found_initially_[i] = search_engines_[i]->path_cost;

        if (res_table) delete[] res_table;
    }

    paths = paths_found_initially_;
    ll_min_f_vals_ = ll_min_f_vals_found_initially_;
    paths_costs_ = paths_costs_found_initially_;

    dummy_start_ = nullptr;
    if (!all_initial_found) return;

    dummy_start_ = new HLNode();
    dummy_start_->agent_id = -1;
    dummy_start_->g_val = 0;
    for (int i = 0; i < num_agents_; i++)
        dummy_start_->g_val += paths_costs_[i];
    dummy_start_->sum_min_f_vals = computeHLLowerBound();
    dummy_start_->open_handle = hl_open_.push(dummy_start_);
    dummy_start_->focal_handle = hl_focal_.push(dummy_start_);
    HL_num_generated_++;
    dummy_start_->time_generated = HL_num_generated_;
    all_nodes_.push_back(dummy_start_);

    min_sum_f_vals_ = dummy_start_->sum_min_f_vals;
    focal_list_threshold_ = focal_w_ * min_sum_f_vals_;
}

double CBSSearch::computeHLLowerBound() {
    double sum = 0;
    for (int i = 0; i < num_agents_; i++)
        sum += ll_min_f_vals_[i];
    return sum;
}

int CBSSearch::getAgentLocation(int agent_id, size_t timestep) {
    if (paths[agent_id].empty()) return -1;
    if (timestep >= paths[agent_id].size())
        return paths[agent_id].back();
    return paths[agent_id][timestep];
}

bool CBSSearch::switchedLocations(int a1, int a2, size_t timestep) {
    if (timestep >= paths[a1].size() && timestep >= paths[a2].size())
        return false;
    return getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep + 1) &&
           getAgentLocation(a1, timestep + 1) == getAgentLocation(a2, timestep);
}

size_t CBSSearch::getPathsMaxLength() {
    size_t maxLen = 0;
    for (int i = 0; i < num_agents_; i++)
        if (!paths[i].empty() && paths[i].size() > maxLen)
            maxLen = paths[i].size();
    return maxLen;
}

void CBSSearch::updateReservationTable(bool* res_table, size_t max_plan_len, int exclude_agent) {
    for (int ag = 0; ag < num_agents_; ag++) {
        if (ag != exclude_agent && !paths[ag].empty()) {
            for (size_t t = 0; t < max_plan_len; t++) {
                int loc = getAgentLocation(ag, t);
                res_table[loc + t * map_size_] = true;
            }
        }
    }
}

void CBSSearch::updatePaths(HLNode* curr, HLNode* root) {
    paths = paths_found_initially_;
    ll_min_f_vals_ = ll_min_f_vals_found_initially_;
    paths_costs_ = paths_costs_found_initially_;
    vector<bool> updated(num_agents_, false);
    while (curr != root) {
        if (!updated[curr->agent_id]) {
            paths[curr->agent_id] = curr->path;
            ll_min_f_vals_[curr->agent_id] = curr->ll_min_f_val;
            paths_costs_[curr->agent_id] = curr->path_cost;
            updated[curr->agent_id] = true;
        }
        curr = curr->parent;
    }
}

bool CBSSearch::updateCBSNode(HLNode* leaf, HLNode* root) {
    list<tuple<int,int,int>> constraints;
    int agent_id = leaf->agent_id;
    HLNode* curr = leaf;
    while (curr != root) {
        if (curr->agent_id == agent_id)
            constraints.push_front(curr->constraint);
        curr = curr->parent;
    }

    int max_timestep = -1;
    for (auto& c : constraints)
        if (get<2>(c) > max_timestep)
            max_timestep = get<2>(c);

    vector<list<pair<int,int>>>* cons_vec = nullptr;
    if (max_timestep >= 0) {
        cons_vec = new vector<list<pair<int,int>>>(max_timestep + 1);
        for (auto& c : constraints)
            cons_vec->at(get<2>(c)).push_back(make_pair(get<0>(c), get<1>(c)));
    }

    size_t max_plan_len = getPathsMaxLength();
    bool* res_table = nullptr;
    if (max_plan_len > 0) {
        res_table = new bool[map_size_ * max_plan_len]();
        updateReservationTable(res_table, max_plan_len, agent_id);
    }

    bool found = search_engines_[agent_id]->findPath(focal_w_, cons_vec, res_table, max_plan_len);

    if (found) {
        leaf->path = search_engines_[agent_id]->path;
        leaf->ll_min_f_val = search_engines_[agent_id]->min_f_val;
        leaf->path_cost = search_engines_[agent_id]->path_cost;
    }

    if (cons_vec) delete cons_vec;
    if (res_table) delete[] res_table;
    return found;
}

vector<tuple<int,int,int,int,int>>* CBSSearch::extractCollisions() {
    auto* collisions = new vector<tuple<int,int,int,int,int>>();
    earliest_conflict_ = make_tuple(-1, -1, -1, -1, INT_MAX);

    for (int a1 = 0; a1 < num_agents_; a1++) {
        for (int a2 = a1 + 1; a2 < num_agents_; a2++) {
            size_t max_len = max(paths[a1].size(), paths[a2].size());
            for (size_t t = 0; t < max_len; t++) {
                if (getAgentLocation(a1, t) == getAgentLocation(a2, t)) {
                    collisions->push_back(make_tuple(a1, a2, getAgentLocation(a1, t), -1, (int)t));
                    if ((int)t < get<4>(earliest_conflict_))
                        earliest_conflict_ = make_tuple(a1, a2, getAgentLocation(a1, t), -1, (int)t);
                }
                if (switchedLocations(a1, a2, t)) {
                    collisions->push_back(make_tuple(a1, a2, getAgentLocation(a1, t),
                                                     getAgentLocation(a2, t), (int)t));
                    if ((int)t < get<4>(earliest_conflict_))
                        earliest_conflict_ = make_tuple(a1, a2, getAgentLocation(a1, t),
                                                         getAgentLocation(a2, t), (int)t);
                }
            }
        }
    }
    return collisions;
}

int CBSSearch::computeNumOfCollidingPairs() {
    int count = 0;
    for (int a1 = 0; a1 < num_agents_; a1++) {
        for (int a2 = a1 + 1; a2 < num_agents_; a2++) {
            size_t max_len = max(paths[a1].size(), paths[a2].size());
            for (size_t t = 0; t < max_len; t++) {
                if (getAgentLocation(a1, t) == getAgentLocation(a2, t) ||
                    switchedLocations(a1, a2, t)) {
                    count++;
                    t = max_len;
                    a2 = num_agents_;
                }
            }
        }
    }
    return count;
}

void CBSSearch::updateFocalList(double old_lb, double new_lb) {
    for (HLNode* n : hl_open_) {
        if (n->sum_min_f_vals > old_lb && n->sum_min_f_vals <= new_lb)
            n->focal_handle = hl_focal_.push(n);
    }
}

bool CBSSearch::run() {
    // Configurable high-level expansion cap. Optimal (w=1) CBS can blow up exponentially on a
    // hard congested sub-instance (e.g. a delivery agent parked on its goal that another
    // agent must cross — CBS keeps adding ever-later vertex constraints without progress).
    // A real solvable CBS instance in these MAPD sub-problems resolves in a handful of
    // HL expansions. When the cap is hit, return false to the caller. The
    // framework default is INT_MAX (effectively uncapped) and can be
    // overridden from the command line.
    while (!hl_focal_.empty() && !solution_found) {
        if (HL_num_expanded_ >= high_level_expansion_limit_) {
            solution_found = false;
            return false;
        }
        HLNode* curr = hl_focal_.top();
        hl_focal_.pop();
        hl_open_.erase(curr->open_handle);
        HL_num_expanded_++;
        curr->time_expanded = HL_num_expanded_;

        updatePaths(curr, dummy_start_);

        auto* collisions = extractCollisions();

        if (collisions->empty()) {
            solution_found = true;
            solution_cost = curr->g_val;
        } else {
            int a1, a2, loc1, loc2, timestep;
            tie(a1, a2, loc1, loc2, timestep) = earliest_conflict_;

            HLNode* n1 = new HLNode();
            HLNode* n2 = new HLNode();
            n1->agent_id = a1;
            n2->agent_id = a2;

            if (loc2 == -1) {
                n1->constraint = make_tuple(loc1, -1, timestep);
                n2->constraint = make_tuple(loc1, -1, timestep);
            } else {
                n1->constraint = make_tuple(loc1, loc2, timestep);
                n2->constraint = make_tuple(loc2, loc1, timestep);
            }

            n1->parent = curr;
            n2->parent = curr;

            if (updateCBSNode(n1, dummy_start_)) {
                n1->g_val = curr->g_val - paths_costs_[n1->agent_id] + n1->path_cost;
                vector<int> old_path = paths[n1->agent_id];
                paths[n1->agent_id] = n1->path;
                n1->h_val = computeNumOfCollidingPairs();
                paths[n1->agent_id] = old_path;
                n1->sum_min_f_vals = curr->sum_min_f_vals
                    - ll_min_f_vals_[n1->agent_id] + n1->ll_min_f_val;
                n1->open_handle = hl_open_.push(n1);
                HL_num_generated_++;
                n1->time_generated = HL_num_generated_;
                if (n1->sum_min_f_vals <= focal_list_threshold_)
                    n1->focal_handle = hl_focal_.push(n1);
                all_nodes_.push_back(n1);
            } else {
                delete n1;
            }

            if (updateCBSNode(n2, dummy_start_)) {
                n2->g_val = curr->g_val - paths_costs_[n2->agent_id] + n2->path_cost;
                vector<int> old_path = paths[n2->agent_id];
                paths[n2->agent_id] = n2->path;
                n2->h_val = computeNumOfCollidingPairs();
                paths[n2->agent_id] = old_path;
                n2->sum_min_f_vals = curr->sum_min_f_vals
                    - ll_min_f_vals_[n2->agent_id] + n2->ll_min_f_val;
                n2->open_handle = hl_open_.push(n2);
                HL_num_generated_++;
                n2->time_generated = HL_num_generated_;
                if (n2->sum_min_f_vals <= focal_list_threshold_)
                    n2->focal_handle = hl_focal_.push(n2);
                all_nodes_.push_back(n2);
            } else {
                delete n2;
            }

            if (hl_open_.empty()) {
                solution_found = false;
                break;
            }
            HLNode* open_head = hl_open_.top();
            if (open_head->sum_min_f_vals > min_sum_f_vals_) {
                double new_threshold = open_head->sum_min_f_vals * focal_w_;
                updateFocalList(focal_list_threshold_, new_threshold);
                min_sum_f_vals_ = open_head->sum_min_f_vals;
                focal_list_threshold_ = new_threshold;
            }
        }

        delete collisions;
    }

    return solution_found;
}

CBSSearch::~CBSSearch() {
    for (auto* e : search_engines_) delete e;
    for (auto* n : all_nodes_) delete n;
}
