#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(const BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    double h = G.heuristics.at(goal_location[goal_id].first)[curr];
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        h += G.heuristics.at(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
        goal_id++;
    }
    return h;
}
// double SingleAgentSolver::compute_h_value(const BasicGraph& G, const State&  curr, int goal_id,
//                              const vector<pair<int, int> >& goal_location) const
// {
//     double h = G.heuristics.at(goal_location[goal_id].first)[curr.location];
//     h += max((double)0, (double)goal_location[goal_id].second - (double)curr.timestep - h);
//     goal_id++;
//     while (goal_id < (int) goal_location.size())
//     {
//         // double h_temp = G.heuristics.at(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
//         // h += h_temp;
//         h += G.heuristics.at(goal_location[goal_id].first)[goal_location[goal_id - 1].first];
//         h += max((double)0, (double)goal_location[goal_id].second - (double)curr.timestep - h);
//         goal_id++;
//     }
//     return h;
// }


