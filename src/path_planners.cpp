#include "path_planners.h"
#include "simulation.h"
#include "cbs.h"

bool PBSPlanner::solve() {
    return simulation_.pbs_solve_impl();
}

void wPBSPlanner::solve() {
    simulation_.wpbs_windowed_solve_impl();
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
