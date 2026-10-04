#include "path_planners.h"
#include "simulation.h"
#include "cbs.h"
#include <climits>

bool PBSPlanner::solve() {
    return simulation_.pbs_solve_impl();
}

CBSConstraintTable::CBSConstraintTable(
        const std::vector<std::tuple<int,int,int>>& constraints,
        const std::vector<std::vector<int>>& others, int start) : start_(start) {
    for (const std::vector<int>& path : others)
        if (!path.empty()) others_.push_back(&path);
    for (const auto& constraint : constraints) {
        int loc = std::get<0>(constraint), to = std::get<1>(constraint);
        int time = std::get<2>(constraint);
        if (to == -2) {
            ranges_.emplace_back(loc, time);
            continue;
        }
        // A vertex constraint applies on arrival at t, an edge constraint on
        // the move that arrives at t + 1.
        int arrival = to == -1 ? time : time + 1;
        if (arrival >= (int)by_time_.size()) by_time_.resize(arrival + 1);
        by_time_[arrival].emplace_back(loc, to);
    }
}

bool CBSConstraintTable::blocks(int curr, int next, int t) const {
    for (const auto& range : ranges_)
        if (range.first == next && t >= range.second) return true;
    if (t < 0 || t >= (int)by_time_.size()) return false;
    for (const auto& constraint : by_time_[t])
        if (constraint.second == -1 ? constraint.first == next
                                    : constraint.first == curr && constraint.second == next)
            return true;
    return false;
}

int CBSConstraintTable::holdable_from(int loc) const {
    for (const auto& range : ranges_)
        if (range.first == loc) return INT_MAX;
    for (int t = (int)by_time_.size() - 1; t >= 0; t--)
        for (const auto& constraint : by_time_[t])
            if (constraint.second == -1 && constraint.first == loc) return t + 1;
    return 0;
}

std::vector<int> plan_ta_dummy_path(
        const std::vector<bool>& grid, const std::vector<int>& heuristic,
        const std::vector<std::vector<int>>& constraint_paths,
        int start_location, int goal_location, int columns,
        int current_time, int max_time) {
    SingleAgentECBS search(constraint_paths, heuristic, grid, -1,
                           start_location, goal_location, columns,
                           current_time, max_time, INT_MAX, true);
    if (!search.findPath()) return {};
    return search.path;
}
