#pragma once
// Port of the ECBS used by Ma's CENTRAL (reference code "Centralized - ECBS":
// ecbs_search.cpp, single_agent_ecbs.cpp, node.h, ecbs_node.h). It plans
// agents that each go to one goal and stay there for good, against the
// committed paths of the agents it does not plan (hard constraints). The
// heaps, node orders and successor order are the reference's, so with
// focal weight 1 it returns the reference's paths.
#include <cstdint>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

namespace ecbs_reference {

struct AgentGoal {
    int start;                       // location at curr_time
    int goal;                        // stays there after arriving
    const std::vector<int>* h;       // distances to the goal
};

class ECBSSearch {
public:
    // map: true = free cell. cons_paths: full paths (indexed by absolute
    // time, at least max_time long) of the agents not planned here.
    ECBSSearch(const std::vector<bool>& map, const std::vector<AgentGoal>& agents,
               const std::vector<std::vector<int>>& cons_paths, int curr_time, int col,
               int max_time, double focal_w, std::uint64_t expansion_limit);
    // Whether a collision-free solution was found within the expansion limit;
    // paths[i] then runs from curr_time to agent i's arrival at its goal.
    bool run();
    std::vector<std::vector<int>> paths;

private:
    struct Impl;
    std::vector<bool> map_;
    std::vector<AgentGoal> agents_;
    const std::vector<std::vector<int>>& cons_paths_;
    int curr_time_, col_, max_time_;
    double focal_w_;
    std::uint64_t expansion_limit_;
};

}  // namespace ecbs_reference
