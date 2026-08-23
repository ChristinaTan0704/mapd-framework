#include "path_planners.h"
#include "simulation.h"
#include "cbs.h"

bool PBSPlanner::solve() {
    return simulation_.pbs_solve_impl();
}

void wPBSPlanner::solve() {
    simulation_.wpbs_windowed_solve_impl();
}

std::vector<int> SIPPPlanner::solve(const SIPPRequest& request) {
    return simulation_.sipp_search_impl(
        request.agent_id, request.start_loc, request.start_time,
        request.goals, request.constraint_paths, request.old_paths,
        request.use_old_paths, request.skip_holding);
}

std::vector<int> MLAStarPlanner::solve(const MLAStarRequest& request) {
    return simulation_.mla_star_taskwise_impl(
        request.agent_id, request.start_loc, request.start_time,
        request.task_groups, request.constraint_paths, request.old_paths,
        request.use_old_paths);
}

std::pair<int,int> STAStarPlanner::solveTask(const STAStarRequest& request) {
    return simulation_.plan_task_sta_impl(
        request.agent, request.task, request.hidden_agent);
}

ECBSResult ECBSPlanner::solve(const ECBSRequest& request) const {
    CBSSearch search(
        request.grid, request.start_locations, request.goal_locations,
        request.goal_endpoint_indices, request.constraint_paths,
        request.current_time, request.columns, request.focal_weight,
        request.endpoints, request.max_time);

    ECBSResult result;
    result.solution_found = search.run();
    result.solution_cost = search.solution_cost;
    result.paths = search.paths;
    return result;
}
