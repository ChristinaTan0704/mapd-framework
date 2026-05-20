#include "Agent.h"

struct HeuristicNode
{
	HeuristicNode(int loc, Task *task, int h_val) : loc(loc), task(task), h_val(h_val) {};
	int loc;
	Task *task;
	int h_val;
};
struct CompareHeuristic
{
	// returns true if n1 > n2 (note -- this gives us *min*-heap).
	bool operator()(const HeuristicNode &n1, const HeuristicNode &n2) const
	{
		return n1.h_val > n2.h_val;
	}
};





Agent::Agent(int loc, int col, int row, int id, int maxtime)
	:loc(loc), col(col), row(row), id(id), timestep(1), maxtime(maxtime),task(NULL), delivering(false), park_loc(loc)
{ 
	only_dummy = false;
	dummy_start_step = 0;
	goal_loc = 0;
	for (int i = 0; i < maxtime; i++)
	{
		path.push_back(loc);//stay still all the tiem
	}
};
Agent::Agent(const Agent &ag)
{
	path.resize(ag.path.size());
	copy(ag.path.begin(), ag.path.end(), path.begin());
	loc = ag.loc;
	id = ag.id;
	maxtime = ag.maxtime;
	timestep = ag.timestep;
	task = ag.task;
	row = ag.row;
	col = ag.col;
	delivering = ag.delivering;
	only_dummy = ag.only_dummy;
	dummy_start_step = ag.dummy_start_step;
	goal_loc = ag.goal_loc;
}
Agent::~Agent()
{
}

void Agent::reset(const Agent &ag)
{
	for (unsigned int i = 0; i < path.size(); i++)
	{
		path[i] = ag.path[i];
	}
	loc = ag.loc;
	id = ag.id;
	maxtime = ag.maxtime;
	timestep = ag.timestep;
	task = ag.task;
	row = ag.row;
	col = ag.col;
	delivering = ag.delivering;
	only_dummy = ag.only_dummy;
	dummy_start_step = ag.dummy_start_step;
	goal_loc = ag.goal_loc;
}

void Agent::Set(int loc, int col, int row, int id, int maxtime)
{
	this->loc = loc;
	this->park_loc = loc;
	this->col = col;
	this->row = row;
	this->id = id;
	this->timestep = 0;
	this->task = NULL;
	this->delivering = false;
	this->maxtime = maxtime;
	this->only_dummy = false;
	this->dummy_start_step = 0;
	this->goal_loc = 0;
	for (int i = 0; i < maxtime; i++)
	{
		path.push_back(loc);//stay still all the tiem
		picktodelivery.push_back(-1);
	}
};


