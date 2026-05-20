#pragma once
#include "BasicSystem.h"
#include "KivaGraph.h"
#include <chrono>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <cstdlib>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

class KivaSystem :
	public BasicSystem
{
public:
	KivaSystem(const KivaGrid& G, MAPFSolver& solver);
	~KivaSystem();

	void simulate(int simulation_time);
	bool load_task_assignments(string fname);
	int get_makespan();
	int get_flowtime() const;
	vector<int> path_len;
	vector<int> newly_finished_agents_idx;
	// vector<int> remained_agents; // 10 agents 
	int total_release_time = 0;
	int count=0;
	vector<vector<int>> agents_delivery_loc;
	vector<vector<int>> agents_pickup_loc;
	vector<vector<pair<int, int>>> agents_task_sequences;



private:
	const KivaGrid& G;
	unordered_set<int> held_endpoints;
	vector<list<tuple<int, int, int> > > task_sequences;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
};

