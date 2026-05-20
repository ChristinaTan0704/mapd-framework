#pragma once
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <cstring>
#include <cassert>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <climits>

using namespace std;


#include<boost/tokenizer.hpp>
#include <dlib/optimization/max_cost_assignment.h>

#include "Endpoint.h"
#include "Agent.h"
#include "CostFLow.h"
#include "ICBSSearch.h"
#include "map_loader.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

class Simulation
{
public:

	Simulation(string map_name, string task_name, string tour_file, string out_file, string tsp_file, string par_file);
	~Simulation();
	

	//run
	void run( double focal_w);
	
	//save
	void ShowTask();
	void SavePath(string fname);
	void SaveTask(const string &fname, const string &instance_name);
	void SaveThroughput(const string &fname);
	void minCost();
	void test();
	bool Test(vector<Agent*> agents);

	double computation_time;
	int num_computations;

private:
	// initialize
	void LoadMap(string fname);
	void LoadTask(string fname);
	void AssignNewTask(int id);

	//void AssignTasks(vector<Agent*> &agents, const vector<vector<int> > &cons_paths);
	bool PathFinding(vector<Agent*> &agents, const vector<Agent*> &cons_agents); // using ICBS
	bool PathFinding(vector<Agent*> &agents, vector<Task*> &tasks, vector<vector<int> > &cons_paths); // using CostFLow
	void TaskAssignment();
	vector<int> CalcCost(Agent* ag);
	void BFS();
	int Cost(int t, queue<Task*> seq);
	void CalcFlow(vector<Agent*> &agents, vector<Task*> &tasks, const vector<vector<int> > &cons_paths, vector<int> len, int &flow, vector<vector<int> > &paths);
	bool TestConstraints();
	vector<int> MinCostAssignment(dlib::matrix<int> &cost);
	int GoHome(vector<Agent*> &ags);
	bool ReplanDummyPath(Agent* ag);
	int global_makespan;
	
private:
	int row, col;
	vector<bool> my_map;
	//vector<bool> DeliverGoal; //goals of DELIVER tasks
	double focal_w;
	//task
	string tour_file;
	string out_file;
	string tsp_file;
	string par_file;
	list<Task*> tasks_deliver;
	vector<queue<Task*> > TSP_seqs;
	vector<int> TSP_len;
	vector<int> TSP_agent;
	vector<vector<int> > real_assignments;
	vector<Task> tasks_total;
	vector<vector<int> > Dis;
	vector<vector<int> > TSPEdgeWeight;
	vector<bool> Hold;
	vector<int> prefer_agent;
	vector<int> finish_time;
	int* moves_offset; // = new int[5];
	bool* mymap;

	vector<Endpoint> endpoints;
	//agent
	vector<Agent> agents;

	int maxtime;//map_size * num_agents
	int timestep;
	
	
	int workpoint_num;
	int t_task;//timestep of last task

	vector<int> endpoint_hashtable;//loc->endpointID
};

