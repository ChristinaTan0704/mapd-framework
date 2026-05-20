#pragma once
#include "BasicGraph.h"
#include "ReservationTable.h"


class SingleAgentSolver
{
public:

	bool prioritize_start;
	double suboptimal_bound;
	bool hold_endpoints;


    uint64_t num_expanded;
    uint64_t num_generated;
	double runtime;

    // int map_size;
	int vis_goal_time;
    double path_cost;
	double temp_h_val;
    double min_f_val;  // min f-val seen so far
    int num_of_conf; // number of conflicts between this agent to all the other agents
	int goal_len;

	unordered_map<int, double> travel_times;

	double compute_h_value(const BasicGraph& G, int curr, int goal_id,
		const vector<pair<int, int> >& goal_location) const;
	double compute_h_value(const BasicGraph& G, const State& curr, int goal_id,
		const vector<pair<int, int> >& goal_location) const;

    virtual Path run(const BasicGraph& G, const State& start, const vector<pair<int, int> >& goal_location, ReservationTable& RT) = 0;
	virtual string getName() const = 0;
	SingleAgentSolver(): suboptimal_bound(1), num_expanded(0), num_generated(0), min_f_val(0), num_of_conf(0) {}


protected:
	double focal_bound;
};
