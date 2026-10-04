#pragma once

#include "types.h"
#include "config.h"
#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>

class Simulation;
struct Agent;
struct Task;

// CBS constraints on one agent, in absolute timesteps: (loc, -1, t) it is not
// at loc at t, (loc, -2, t) it is never at loc from t on, and (from, to, t) it
// does not move from `from` at t to `to` at t + 1. The other CBS agents' paths
// (starting at `start`; referenced, so they must outlive the table) are soft:
// equally short paths that meet them less are preferred, as in ECBS.
class CBSConstraintTable {
public:
    CBSConstraintTable(const std::vector<std::tuple<int,int,int>>& constraints,
                       const std::vector<std::vector<int>>& others = {}, int start = 0);
    // How many of the other agents' paths are at loc at timestep t.
    int conflicts(int loc, int t) const {
        int step = t - start_;
        if (others_.empty() || step < 0) return 0;
        int count = 0;
        for (const std::vector<int>* path : others_)
            count += (*path)[std::min(step, (int)path->size() - 1)] == loc;
        return count;
    }
    // Whether moving from curr to next, arriving at timestep t, is forbidden.
    bool blocks(int curr, int next, int t) const;
    // First timestep from which the agent may stay at loc forever; INT_MAX
    // when it may never.
    int holdable_from(int loc) const;
    // Calls f(loc, begin, end) for every interval [begin, end) in which the
    // agent may not be at loc; range constraints end at `horizon`.
    template <typename F> void for_each_forbidden_interval(int horizon, F f) const {
        for (int t = 0; t < (int)by_time_.size(); t++)
            for (const auto& constraint : by_time_[t])
                if (constraint.second == -1) f(constraint.first, t, t + 1);
        for (const auto& range : ranges_) f(range.first, range.second, horizon);
    }
private:
    std::vector<std::vector<std::pair<int,int>>> by_time_;  // [t]: (loc, -1), (from, to)
    std::vector<std::pair<int,int>> ranges_;                // (loc, first timestep)
    std::vector<const std::vector<int>*> others_;
    int start_;
};

// Explicit input to the configured single-agent search (config.single_agent).
// Goals are (location, earliest arrival) pairs grouped by task, with the dummy
// endpoint as a trailing group; only the very last goal must be holdable. The
// referenced containers must outlive the call.
struct SingleAgentRequest {
    int agent_id;
    int start_loc;
    int start_time;
    const std::vector<std::vector<std::pair<int,int>>>& goal_groups;
    const std::vector<std::vector<int>>& constraint_paths;   // never collide
    const std::vector<std::vector<int>>& old_paths;          // PBS: previous plans
    bool use_old_paths;
    // Windowed planning: the search ends at this timestep and returns the best
    // partial path (the agent then holds its last cell).
    int stop_time = INT_MAX;
    // CBS: the constraints of the high-level node, or none.
    const CBSConstraintTable* constraints = nullptr;
};

// Strategy objects used by Simulation's dispatchers. They receive the live
// Simulation explicitly because PBS updates the shared committed paths.
class PBSPlanner {
public:
    explicit PBSPlanner(Simulation& simulation) : simulation_(simulation) {}
    bool solve();
private:
    Simulation& simulation_;
};

// Space-time A* from start_location at current_time to a goal the agent can
// hold forever, avoiding constraint_paths; moves are expanded in the order of
// the TA reference code. Used for TA-Hybrid dummy paths. Returns the path
// from current_time to the goal, or an empty vector.
std::vector<int> plan_ta_dummy_path(
    const std::vector<bool>& grid, const std::vector<int>& heuristic,
    const std::vector<std::vector<int>>& constraint_paths,
    int start_location, int goal_location, int columns,
    int current_time, int max_time);
