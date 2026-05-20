#pragma once
#include "BasicSystem.h"
#include "KivaGraph.h"
#include "LNS.h"
// #include "TasksLoader.h"
#include <chrono>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <cstdlib>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

class KivaSystemOnline :
	public BasicSystem
{
public:
	KivaSystemOnline(const KivaGrid& G, MAPFSolver& solver);
	~KivaSystemOnline();

	void simulate(int simulation_time);
	bool load_tasks(string fname);
	int get_makespan();
	int get_flowtime() const;
	int calculate_flowtime_tp(vector<vector<int>> finish_task_sequence);
	vector<int> path_len;
	vector<int> newly_finished_agents_idx;
	int total_release_time = 0;
	int num_finished_tasks = 0;
	double flowtime_init_tp = 0;
	vector<vector<int>> agents_delivery_loc;
	vector<vector<int>> agents_pickup_loc;
	vector<vector<Task>> agents_task_sequences;
	vector<vector<int>> agents_finish_sequence;
	vector<Task> all_tasks;
	std::map<int, vector<int>> all_tasks_list;
	std::map<int, Task> current_tasks;
	int task_num;
	bool finish_release = false;
	std::map<int, int> delivering_agents;
	std::map<int, int> agent_task_pair;
	vector<int> free_agent_set;
	bool all_agents_busy=false;
	bool finish_assign = false;
	bool new_agent_finish = false;
	vector<int> current_assigned_endpoints;
	bool apply_lns = true;

private:
	const KivaGrid& G;
	vector<vector<int> > task_sequences;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	int choose_good_endpoint(vector<int> current_assigned_endpoints, int last_task_endpoint);
	void update_goal_locations();
	void generate_tasks();
	void remove_from_system();
	void update_goal_locations(std::map<int, int> delivering_agents, std::map<int, int> agent_task_pair);
};

