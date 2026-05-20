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
	vector<int> picktodelivery;
	//Endpoint* home;
	int loc, park_loc;
	Endpoint* next_ep;
	
	int id;
	unsigned int maxtime;
	unsigned int timestep;//current timestep
	unsigned int arrive_time;
	Task *task;
	int row;
	int col;
	int finishtime;
	bool delivering;

};


class Task
{
public:
	Task() {};
	Task(int Id, int release, Endpoint *start, Endpoint *goal, int start_time, int goal_time)
		:id(id), release_time(release), start(start), goal(goal), start_time(start_time), goal_time(goal_time), delivering(false) {}
	~Task() {}

	Endpoint *start;
	Endpoint *goal;
	void Set(int _id, int _release_time, Endpoint *_start, Endpoint *_goal, int _start_time, int _goal_time, bool _delivering) {
		id = _id;
		release_time = _release_time;
		start = _start;
		goal = _goal;
		start_time = _start_time;
		goal_time = _goal_time;
		delivering = _delivering;
	}

	Agent* ag;
	int ag_arrive_start;
	int ag_arrive_goal;
	int release_time, hold_time;
	int id;
	int agent_id;
	int seq_id; // TSP sequences id 
	int start_time; //min time agent stay at start point
	int goal_time; //min time agent stay at goal point
	bool delivering;

};

