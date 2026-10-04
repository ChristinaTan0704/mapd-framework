#include "path_planners.h"
#include "simulation.h"
#include "cbs.h"

bool PBSPlanner::solve() {
    return simulation_.pbs_solve_impl();
}

ECBSResult ECBSPlanner::solve(const ECBSRequest& request) const {
    CBSSearch search(
        request.grid, request.start_locations, request.goal_locations,
        request.goal_endpoint_indices, request.constraint_paths,
        request.current_time, request.columns, request.focal_weight,
        request.high_level_expansion_limit,
        request.low_level_expansion_limit,
        request.endpoints, request.max_time,
        request.conflict_selection, request.bypass,
        request.target_reasoning, request.rectangle_reasoning);

    ECBSResult result;
    result.solution_found = search.run();
    result.solution_cost = search.solution_cost;
    result.paths = search.paths;
    return result;
}

std::vector<int> plan_ta_dummy_path(
        const std::vector<bool>& grid, const std::vector<int>& heuristic,
        const std::vector<std::vector<int>>& constraint_paths,
        int start_location, int goal_location, int columns,
        int current_time, int max_time) {
    SingleAgentECBS search(constraint_paths, heuristic, grid, -1,
                           start_location, goal_location, columns,
                           current_time, max_time, INT_MAX, true);
    if (!search.findPath(1.0, nullptr, nullptr, 0)) return {};
    return search.path;
}
