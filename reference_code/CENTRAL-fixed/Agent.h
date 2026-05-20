#pragma once
#include <vector>
#include <list>

#include <string>
#include <functional>  // for std::hash (c++11 and above)
#include <map>

#include "Node.h"
#include "Endpoint.h"

using namespace std;

class Task;

class Agent
{
public:
	Agent() {};
	Agent(int loc, int col,int row, int id, int maxtime);
	Agent(const Agent &ag);
	~Agent();
	void Set(int loc, int col, int row, int id, int maxtime);
	void reset(const Agent &ag);

	vector<int> path;  // a path that takes the agent from initial to start to goal location satisfing all constraints
	vector<int> non_dummy_path;
	vector<int> picktodelivery;
	vector<int> cost;
	//Endpoint* home;
	int loc, park_loc, goal_loc;
	Endpoint* next_ep;
	bool only_dummy;
	
	int id;
	int dummy_start_step;
	unsigned int maxtime;
	unsigned int timestep;//current timestep
	int start_time;
	unsigned int arrive_time;
	int release_time;
	Task *task;
	int row;
	int col;
	bool delivering;

};


class Task
{
public:
	Task() {};
	Task(int Id, int release, Endpoint *start, Endpoint *goal, int start_time, int goal_time)
		:Id(Id), release_time(release), start(start), goal(goal), start_time(start_time), goal_time(goal_time), delivering(false) {}
	~Task() {}

	Endpoint *start;
	Endpoint *goal;

	Agent* ag;
	unsigned int ag_arrive_start;
	unsigned int ag_arrive_goal;
	int release_time, hold_time;
	bool finished;
	int Id;
	int agent_id;
	int seq_id; // TSP sequences id 
	int start_time; //min time agent stay at start point
	int goal_time; //min time agent stay at goal point
	bool delivering;

};

