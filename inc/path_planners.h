#pragma once

#include "types.h"
#include <utility>
#include <vector>

class Simulation;
struct Agent;
struct Task;

// TP/TPTS plan one task as sequential space-time A* searches through every
// ordered Task::goals location. The planner writes the committed path into the
// supplied agent and returns (first-goal arrival, final-goal arrival).
struct STAStarRequest {
    Agent& agent;
    Task& task;
    int hidden_agent;

    STAStarRequest(Agent& search_agent, Task& search_task, int hidden = -1)
        : agent(search_agent), task(search_task), hidden_agent(hidden) {}
};

// Explicit input passed from a MAPF planner to the MLA* low-level solver.
// The referenced containers only need to remain alive for the duration of
// MLAStarPlanner::solve().
struct MLAStarRequest {
    int agent_id;
    int start_loc;
    int start_time;
    const std::vector<std::vector<std::pair<int,int>>>& task_groups;
    const std::vector<std::vector<int>>& constraint_paths;
    const std::vector<std::vector<int>>& old_paths;
    bool use_old_paths;

    MLAStarRequest(int agent, int location, int time,
                   const std::vector<std::vector<std::pair<int,int>>>& groups,
                   const std::vector<std::vector<int>>& constraints,
                   const std::vector<std::vector<int>>& old,
                   bool use_old)
        : agent_id(agent), start_loc(location), start_time(time),
          task_groups(groups), constraint_paths(constraints), old_paths(old),
          use_old_paths(use_old) {}
};

// Explicit input passed from a MAPF planner to the MLSIPP low-level solver.
struct SIPPRequest {
    int agent_id;
    int start_loc;
    int start_time;
    const std::vector<std::pair<int,int>>& goals;
    const std::vector<std::vector<int>>& constraint_paths;
    const std::vector<std::vector<int>>& old_paths;
    bool use_old_paths;
    bool skip_holding;

    SIPPRequest(int agent, int location, int time,
                const std::vector<std::pair<int,int>>& search_goals,
                const std::vector<std::vector<int>>& constraints,
                const std::vector<std::vector<int>>& old,
                bool use_old, bool skip_hold = false)
        : agent_id(agent), start_loc(location), start_time(time),
          goals(search_goals), constraint_paths(constraints), old_paths(old),
          use_old_paths(use_old), skip_holding(skip_hold) {}
};

// Strategy objects used by Simulation's dispatchers. They receive the live
// Simulation explicitly because PBS/wPBS update the shared committed paths,
// while the low-level solvers receive their per-call data through the request
// objects above.
class PBSPlanner {
public:
    explicit PBSPlanner(Simulation& simulation) : simulation_(simulation) {}
    bool solve();
private:
    Simulation& simulation_;
};

class wPBSPlanner {
public:
    explicit wPBSPlanner(Simulation& simulation) : simulation_(simulation) {}
    void solve();
private:
    Simulation& simulation_;
};

class SIPPPlanner {
public:
    explicit SIPPPlanner(Simulation& simulation) : simulation_(simulation) {}
    std::vector<int> solve(const SIPPRequest& request);
private:
    Simulation& simulation_;
};

class MLAStarPlanner {
public:
    explicit MLAStarPlanner(Simulation& simulation) : simulation_(simulation) {}
    std::vector<int> solve(const MLAStarRequest& request);
private:
    Simulation& simulation_;
};

class STAStarPlanner {
public:
    explicit STAStarPlanner(Simulation& simulation) : simulation_(simulation) {}
    std::pair<int,int> solveTask(const STAStarRequest& request);
private:
    Simulation& simulation_;
};

struct ECBSRequest {
    const std::vector<bool>& grid;
    const std::vector<int>& start_locations;
    const std::vector<int>& goal_locations;
    const std::vector<int>& goal_endpoint_indices;
    const std::vector<std::vector<int>>& constraint_paths;
    int current_time;
    int columns;
    double focal_weight;
    int high_level_expansion_limit;
    int low_level_expansion_limit;
    const std::vector<Endpoint>& endpoints;
    int max_time;

    ECBSRequest(const std::vector<bool>& map_grid,
                const std::vector<int>& starts,
                const std::vector<int>& goals,
                const std::vector<int>& goal_endpoints,
                const std::vector<std::vector<int>>& constraints,
                int time, int map_columns, double weight,
                int high_expansion_limit, int low_expansion_limit,
                const std::vector<Endpoint>& map_endpoints, int horizon)
        : grid(map_grid), start_locations(starts), goal_locations(goals),
          goal_endpoint_indices(goal_endpoints), constraint_paths(constraints),
          current_time(time), columns(map_columns), focal_weight(weight),
          high_level_expansion_limit(high_expansion_limit),
          low_level_expansion_limit(low_expansion_limit),
          endpoints(map_endpoints), max_time(horizon) {}
};

struct ECBSResult {
    bool solution_found = false;
    double solution_cost = -1;
    std::vector<std::vector<int>> paths;
};

class ECBSPlanner {
public:
    ECBSResult solve(const ECBSRequest& request) const;
};
