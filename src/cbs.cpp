#include "cbs.h"
#include <algorithm>
#include <memory>

// ============================================================
// LLNode constructor
// ============================================================

LLNode::LLNode(int l, double g, double h, LLNode* p, int t, bool io)
    : loc(l), timestep(t), g_val(g), h_val(h),
      tie_breaker(RandomTieBreaker::next()), parent(p), in_openlist(io) {}

// ============================================================
// HLNode constructor
// ============================================================

HLNode::HLNode() : agent_id(-1), g_val(0), h_val(0), path_cost(0),
                   tie_breaker(RandomTieBreaker::next()), parent(nullptr),
                   time_generated(0), time_expanded(0) {}

// ============================================================
// Section 1: SingleAgentECBS — Low-Level ECBS Search
// ============================================================

SingleAgentECBS::SingleAgentECBS(const vector<vector<int>>& cons_paths,
                                 const vector<int>& heuristic,
                                 const vector<bool>& grid,
                                 int ag_id, int start_loc, int goal_loc,
                                 int col, int curr_time, int max_time,
                                 int expansion_limit, bool wait_last)
    : cons_paths_(cons_paths), heuristic_(heuristic), grid_(grid),
      ag_id_(ag_id), start_loc_(start_loc), goal_loc_(goal_loc),
      curr_time_(curr_time), max_time_(max_time),
      expansion_limit_(expansion_limit),
      path_cost(0), min_f_val(0), num_expanded(0), num_generated(0)
{
    map_size_ = grid_.size();
    const int moves[5] = {0, -col, 1, col, -1};
    for (int d = 0; d < 5; d++) actions_[d] = moves[wait_last ? (d + 1) % 5 : d];
}

bool SingleAgentECBS::isConstrained(int curr, int next, int next_t) {
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
    return false;
}

bool SingleAgentECBS::canHoldGoal(int timestep) const {
    for (const auto& cp : cons_paths_)
        for (int t = curr_time_ + timestep + 1; t < (int)cp.size(); t++)
            if (cp[t] == goal_loc_) return false;
    return true;
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

void SingleAgentECBS::releaseNodes(map<unsigned int, LLNode*>& table) {
    for (auto& p : table) delete p.second;
    table.clear();
}

bool SingleAgentECBS::findPath() {
    const double f_weight = 1.0;
    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<LLNode::CompareOpen>> open_heap_t;
    typedef boost::heap::fibonacci_heap<LLNode*, boost::heap::compare<LLNode::CompareFocal>> focal_heap_t;

    open_heap_t open_list;
    focal_heap_t focal_list;
    map<unsigned int, LLNode*> allNodes;
    num_expanded = 0;
    num_generated = 0;

    LLNode* start = new LLNode(start_loc_, 0, heuristic_[start_loc_], nullptr, 0, true);
    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes.insert(make_pair((unsigned int)start_loc_, start));
    min_f_val = start->getFVal();
    double lower_bound = f_weight * min_f_val;

    // The default INT_MAX leaves the low-level search effectively uncapped.
    // A smaller positive limit may be supplied explicitly for experiments.
    while (!focal_list.empty()) {
        RuntimeDeadline::check_periodic(
            (uint64_t)num_expanded, "CBS low-level search");
        if (num_expanded >= expansion_limit_) {
            path.clear();
            releaseNodes(allNodes);
            return false;
        }

        LLNode* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        if (curr->loc == goal_loc_) {
            if (canHoldGoal(curr->timestep)) {
                updatePath(curr);
                releaseNodes(allNodes);
                return true;
            }
        }

        for (int d = 0; d < 5; d++) {
            int next_loc = curr->loc + actions_[d];
            int next_t = curr->timestep + 1;

            if (!isConstrained(curr->loc, next_loc, next_t)) {
                double next_g = curr->g_val + 1;
                double next_h = heuristic_[next_loc];

                unsigned int key = next_loc + (unsigned int)next_g * map_size_;
                auto it = allNodes.find(key);

                if (it == allNodes.end() && next_g < max_time_ - curr_time_) {
                    LLNode* next = new LLNode(next_loc, next_g, next_h, curr, next_t, true);
                    num_generated++;
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    if (next->getFVal() <= lower_bound)
                        next->focal_handle = focal_list.push(next);
                    allNodes.insert(make_pair(key, next));
                } else if (it != allNodes.end() && next_g < max_time_ - curr_time_) {
                    LLNode* existing = it->second;
                    if (existing->in_openlist) {
                        if (existing->getFVal() > next_g + next_h) {
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
                            if (update_open) open_list.increase(existing->open_handle);
                            if (add_to_focal) existing->focal_handle = focal_list.push(existing);
                            if (update_in_focal) focal_list.update(existing->focal_handle);
                        }
                    } else {
                        if (existing->getFVal() > next_g + next_h) {
                            existing->g_val = next_g;
                            existing->h_val = next_h;
                            existing->parent = curr;
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

CBSSearch::CBSSearch(int num_agents, const CBSLowLevel& low_level, int col,
                     double focal_w, int high_level_expansion_limit,
                     CBSConflictSelection conflict_selection, bool bypass,
                     bool target_reasoning, bool rectangle_reasoning)
    : solution_found(false), solution_cost(-1),
      num_agents_(num_agents), low_level_(low_level), focal_w_(focal_w),
      high_level_expansion_limit_(high_level_expansion_limit),
      conflict_selection_(conflict_selection), bypass_(bypass),
      target_reasoning_(target_reasoning),
      rectangle_reasoning_(rectangle_reasoning), col_(col),
      HL_num_expanded_(0), HL_num_generated_(0)
{
    paths.resize(num_agents_);
    paths_found_initially_.resize(num_agents_);
    paths_costs_found_initially_.resize(num_agents_);
    root_mdd_singletons_.resize(num_agents_);

    dummy_start_ = nullptr;
    for (int i = 0; i < num_agents_; i++) {
        if (!low_level_.find_path(i, {}, paths_found_initially_, paths_found_initially_[i]))
            return;
        paths_costs_found_initially_[i] = paths_found_initially_[i].size() - 1;
    }
    paths = paths_found_initially_;
    paths_costs_ = paths_costs_found_initially_;

    dummy_start_ = new HLNode();
    dummy_start_->agent_id = -1;
    dummy_start_->g_val = 0;
    for (int i = 0; i < num_agents_; i++)
        dummy_start_->g_val += paths_costs_[i];
    dummy_start_->h_val = computeNumOfCollidingPairs();
    dummy_start_->open_handle = hl_open_.push(dummy_start_);
    dummy_start_->focal_handle = hl_focal_.push(dummy_start_);
    HL_num_generated_++;
    dummy_start_->time_generated = HL_num_generated_;
    all_nodes_.push_back(dummy_start_);

    min_sum_f_vals_ = dummy_start_->g_val;
    focal_list_threshold_ = focal_w_ * min_sum_f_vals_;
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

void CBSSearch::updatePaths(HLNode* curr, HLNode* root) {
    paths = paths_found_initially_;
    paths_costs_ = paths_costs_found_initially_;
    vector<bool> updated(num_agents_, false);
    while (curr != root) {
        if (!updated[curr->agent_id]) {
            paths[curr->agent_id] = curr->path;
            paths_costs_[curr->agent_id] = curr->path_cost;
            updated[curr->agent_id] = true;
        }
        curr = curr->parent;
    }
}

// agent_id's constraints from node up to the root.
vector<tuple<int,int,int>> CBSSearch::collectConstraints(HLNode* node, int agent_id) {
    vector<tuple<int,int,int>> constraints;
    for (HLNode* curr = node; curr != dummy_start_; curr = curr->parent)
        if (curr->agent_id == agent_id)
            constraints.insert(constraints.end(), curr->constraints.begin(),
                               curr->constraints.end());
    return constraints;
}

bool CBSSearch::updateCBSNode(HLNode* leaf, HLNode* /*root*/) {
    if (!low_level_.find_path(leaf->agent_id, collectConstraints(leaf, leaf->agent_id),
                              paths, leaf->path))
        return false;
    leaf->path_cost = leaf->path.size() - 1;
    return true;
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

// An agent's MDD depends only on its constraints, which change only at nodes
// owned by that agent, so the MDD is cached on the nearest such ancestor.
const vector<bool>& CBSSearch::getMDDSingletons(HLNode* node, int agent_id) {
    HLNode* owner = node;
    while (owner != dummy_start_ && owner->agent_id != agent_id)
        owner = owner->parent;
    std::shared_ptr<const vector<bool>>& cached = owner == dummy_start_
        ? root_mdd_singletons_[agent_id] : owner->mdd_singletons;
    if (!cached)
        cached = std::make_shared<const vector<bool>>(low_level_.mdd_singletons(
            agent_id, collectConstraints(owner, agent_id), (int)paths[agent_id].size() - 1));
    return *cached;
}

// True when constraining agent_id at this conflict must raise its path cost.
bool CBSSearch::isSingletonSide(HLNode* node, int agent_id, int edge_to, int timestep) {
    int path_cost = (int)paths[agent_id].size() - 1;
    // The agent already holds its goal, so it must arrive later.
    if (timestep >= path_cost) return true;
    const vector<bool>& singletons = getMDDSingletons(node, agent_id);
    if (singletons.empty()) return false;
    if (edge_to < 0) return singletons[timestep];
    return singletons[timestep] && singletons[timestep + 1];
}

// ICBS conflict prioritization: cardinal, then semi-cardinal, then
// non-cardinal; the earliest conflict wins within a class.
void CBSSearch::selectCardinalConflict(HLNode* node,
        const vector<tuple<int,int,int,int,int>>& collisions) {
    int best_rank = INT_MAX;
    for (const auto& conflict : collisions) {
        int a1, a2, loc1, loc2, timestep;
        tie(a1, a2, loc1, loc2, timestep) = conflict;
        // For an edge conflict a1 moves loc1->loc2 and a2 moves loc2->loc1.
        int a2_edge_to = loc2 >= 0 ? loc1 : -1;
        int rank = 2 - (int)isSingletonSide(node, a1, loc2, timestep)
                     - (int)isSingletonSide(node, a2, a2_edge_to, timestep);
        if (rank < best_rank ||
            (rank == best_rank && timestep < get<4>(earliest_conflict_))) {
            best_rank = rank;
            earliest_conflict_ = conflict;
        }
    }
}

// Plans the child's agent under its constraints and computes its costs.
bool CBSSearch::generateChild(HLNode* child, HLNode* parent) {
    if (!updateCBSNode(child, dummy_start_)) return false;
    int agent_id = child->agent_id;
    child->g_val = parent->g_val - paths_costs_[agent_id] + child->path_cost;
    vector<int> old_path = paths[agent_id];
    paths[agent_id] = child->path;
    child->h_val = computeNumOfCollidingPairs();
    paths[agent_id] = old_path;
    return true;
}

// Rectangle reasoning for a vertex conflict at loc/timestep. Coordinates are
// mirrored so both agents move toward +x/+y. Each agent is anchored at its
// start at relative time 0, which every solution shares. When one agent enters
// the rectangle [Rs, Rg] through its left edge and the other through its
// bottom edge, both reaching Rs at the same time, any pair of monotone paths
// that cross the rectangle meet at the same vertex at the same time. So each
// conflict-free solution respects at least one barrier: the left-entering
// agent avoids the right edge, or the bottom-entering agent avoids the top
// edge, at the times a monotone path would reach those vertices. Returns false
// (no constraints added) unless both current paths violate their barrier,
// which guarantees progress.
bool CBSSearch::addRectangleBarriers(int a1, int a2, int loc, int timestep,
                                     HLNode* n1, HLNode* n2) {
    const vector<int>& p1 = paths[a1];
    const vector<int>& p2 = paths[a2];
    if ((int)p1.size() <= timestep || (int)p2.size() <= timestep) return false;
    int s1 = p1[0], s2 = p2[0];
    int cx = loc % col_, cy = loc / col_;
    int s1x = s1 % col_, s1y = s1 / col_, s2x = s2 % col_, s2y = s2 / col_;
    if ((cx - s1x) * (cx - s2x) < 0 || (cy - s1y) * (cy - s2y) < 0) return false;
    int dx = cx != s1x ? (cx > s1x ? 1 : -1) : (cx >= s2x ? 1 : -1);
    int dy = cy != s1y ? (cy > s1y ? 1 : -1) : (cy >= s2y ? 1 : -1);
    auto X = [&](int l) { return dx * (l % col_); };
    auto Y = [&](int l) { return dy * (l / col_); };

    // Longest prefix of each path that moves only in +X/+Y every step.
    auto monotone_end = [&](const vector<int>& p) {
        int k = 0;
        while (k + 1 < (int)p.size()) {
            int ddx = X(p[k + 1]) - X(p[k]), ddy = Y(p[k + 1]) - Y(p[k]);
            if (!((ddx == 1 && ddy == 0) || (ddx == 0 && ddy == 1))) break;
            k++;
        }
        return k;
    };
    int e1 = monotone_end(p1), e2 = monotone_end(p2);
    if (e1 < timestep || e2 < timestep) return false;

    // left: smaller X and larger Y at the start; it enters via the left edge.
    int left, bottom;
    if (X(s1) < X(s2) && Y(s1) > Y(s2)) { left = a1; bottom = a2; }
    else if (X(s2) < X(s1) && Y(s2) > Y(s1)) { left = a2; bottom = a1; }
    else return false;
    int rsX = max(X(s1), X(s2)), rsY = max(Y(s1), Y(s2));
    int t_rs1 = (rsX - X(s1)) + (rsY - Y(s1));
    int t_rs2 = (rsX - X(s2)) + (rsY - Y(s2));
    if (t_rs1 != t_rs2) return false;
    int rgX = min(X(p1[e1]), X(p2[e2])), rgY = min(Y(p1[e1]), Y(p2[e2]));
    if (rgX < rsX || rgY < rsY) return false;

    // Barrier vertex at mirrored (bx, by), reached at t_rs1 + d(Rs, v).
    auto barrier_loc = [&](int bx, int by) {
        return (dy * by) * col_ + dx * bx;
    };
    auto hits = [&](const vector<int>& p, int end, bool right_edge) {
        for (int k = 0; k <= end; k++) {
            if (right_edge && X(p[k]) == rgX && Y(p[k]) >= rsY && Y(p[k]) <= rgY)
                return true;
            if (!right_edge && Y(p[k]) == rgY && X(p[k]) >= rsX && X(p[k]) <= rgX)
                return true;
        }
        return false;
    };
    const vector<int>& pl = left == a1 ? p1 : p2;
    const vector<int>& pb = left == a1 ? p2 : p1;
    if (!hits(pl, left == a1 ? e1 : e2, true) ||
        !hits(pb, left == a1 ? e2 : e1, false))
        return false;

    n1->agent_id = left;
    n2->agent_id = bottom;
    for (int by = rsY; by <= rgY; by++)
        n1->constraints.push_back(make_tuple(
            barrier_loc(rgX, by), -1, t_rs1 + (rgX - rsX) + (by - rsY)));
    for (int bx = rsX; bx <= rgX; bx++)
        n2->constraints.push_back(make_tuple(
            barrier_loc(bx, rgY), -1, t_rs1 + (bx - rsX) + (rgY - rsY)));
    return true;
}

void CBSSearch::updateFocalList(double old_lb, double new_lb) {
    for (HLNode* n : hl_open_) {
        if (n->g_val > old_lb && n->g_val <= new_lb)
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
        RuntimeDeadline::check_periodic(
            (uint64_t)HL_num_expanded_, "CBS high-level search");
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
            if (conflict_selection_ == CBS_CONFLICT_CARDINAL)
                selectCardinalConflict(curr, *collisions);
            int a1, a2, loc1, loc2, timestep;
            tie(a1, a2, loc1, loc2, timestep) = earliest_conflict_;

            HLNode* n1 = new HLNode();
            HLNode* n2 = new HLNode();
            n1->agent_id = a1;
            n2->agent_id = a2;

            // Target conflict: one agent already holds its goal loc1 at
            // timestep. Either it arrives later, or the other agent never
            // enters loc1 from timestep on; together these cover every plan.
            int parked = -1;
            if (target_reasoning_ && loc2 == -1) {
                if (timestep >= (int)paths[a1].size() - 1 && paths[a1].back() == loc1)
                    parked = a1;
                else if (timestep >= (int)paths[a2].size() - 1 && paths[a2].back() == loc1)
                    parked = a2;
            }
            if (parked >= 0) {
                n1->agent_id = parked;
                n2->agent_id = parked == a1 ? a2 : a1;
                n1->constraints.push_back(make_tuple(loc1, -1, timestep));
                n2->constraints.push_back(make_tuple(loc1, -2, timestep));
            } else if (loc2 == -1 && rectangle_reasoning_ &&
                       addRectangleBarriers(a1, a2, loc1, timestep, n1, n2)) {
                // Barrier constraints were added for both children.
            } else if (loc2 == -1) {
                n1->constraints.push_back(make_tuple(loc1, -1, timestep));
                n2->constraints.push_back(make_tuple(loc1, -1, timestep));
            } else {
                n1->constraints.push_back(make_tuple(loc1, loc2, timestep));
                n2->constraints.push_back(make_tuple(loc2, loc1, timestep));
            }

            n1->parent = curr;
            n2->parent = curr;

            vector<HLNode*> children;
            for (HLNode* child : {n1, n2}) {
                if (generateChild(child, curr)) children.push_back(child);
                else delete child;
            }

            // Bypass: a child that keeps its agent's cost and has fewer
            // colliding pairs replaces the parent's path instead of branching.
            HLNode* bypass_child = nullptr;
            if (bypass_) {
                for (HLNode* child : children)
                    if (child->path_cost == paths_costs_[child->agent_id] &&
                        child->h_val < curr->h_val &&
                        (!bypass_child || child->h_val < bypass_child->h_val))
                        bypass_child = child;
            }
            if (bypass_child) {
                HLNode* bypass_node = new HLNode();
                bypass_node->agent_id = bypass_child->agent_id;
                bypass_node->path = bypass_child->path;
                bypass_node->path_cost = bypass_child->path_cost;
                bypass_node->g_val = bypass_child->g_val;
                bypass_node->h_val = bypass_child->h_val;
                bypass_node->parent = curr;
                for (HLNode* child : children) delete child;
                children.assign(1, bypass_node);
            }

            for (HLNode* child : children) {
                child->open_handle = hl_open_.push(child);
                HL_num_generated_++;
                child->time_generated = HL_num_generated_;
                if (child->g_val <= focal_list_threshold_)
                    child->focal_handle = hl_focal_.push(child);
                all_nodes_.push_back(child);
            }

            if (hl_open_.empty()) {
                solution_found = false;
                break;
            }
            HLNode* open_head = hl_open_.top();
            if (open_head->g_val > min_sum_f_vals_) {
                double new_threshold = open_head->g_val * focal_w_;
                updateFocalList(focal_list_threshold_, new_threshold);
                min_sum_f_vals_ = open_head->g_val;
                focal_list_threshold_ = new_threshold;
            }
        }

        delete collisions;
    }

    return solution_found;
}

CBSSearch::~CBSSearch() {
    for (auto* n : all_nodes_) delete n;
}
