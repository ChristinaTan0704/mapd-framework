#pragma once

#include <utility>
#include <vector>

class Simulation;

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
