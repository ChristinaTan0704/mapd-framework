#include "path_planners.h"
#include "simulation.h"

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
