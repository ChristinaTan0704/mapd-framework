#include "Simulation.h"
#include "CostFLow.h"

#include <ctime>
#include <cstdio>

Simulation::Simulation(string map_name, string task_name, string out_file): out_file(out_file)
{

	LoadMap(map_name);
	LoadTask(task_name);

}
Simulation::~Simulation()
{
}

void Simulation::LoadMap(string fname)
{
	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
	{
		printf("%s\n", fname.c_str());
		cerr << "Map file not found." << endl;
		return;
	}
	//read file
	getline(myfile, line);
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	row = atoi((*beg).c_str()) + 2; // read number of rows
	beg++;
	col = atoi((*beg).c_str()) + 2; // read number of cols

	stringstream ss;
	getline(myfile, line);
	ss << line;
	ss >> workpoint_num;

	int agent_num;
	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> agent_num;

	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> maxtime;

	this->agents.resize(agent_num);
	endpoints.resize(workpoint_num + agent_num);
	my_map.resize(row*col);
	DeliverGoal.resize(row*col, false);
	// read map
	int ep = 0, ag = 0;
	for (int i = 1; i<row - 1; i++)
	{
		getline(myfile, line);
		for (int j = 1; j<col - 1; j++)
		{
			my_map[col*i + j] = (line[j - 1] != '@'); // not a block
			if (line[j - 1] == 'e') //endpoint
			{
				endpoints[ep++].loc = i*col + j;
			}
			else if (line[j - 1] == 'r') //robot rest
			{
				endpoints[workpoint_num + ag].loc = i*col + j;
				agents[ag].Set(i*col + j, col, row, ag, maxtime);
				ag++;
			}
		}
	}
	myfile.close();

	//set the border of the map blocked
	for (int i = 0; i < row; i++)
	{
		my_map[i*col] = false;
		my_map[i*col + col - 1] = false;
	}
	for (int j = 1; j < col - 1; j++)
	{
		my_map[j] = false;
		my_map[row*col - col + j] = false;
	}

	//initial heuristic matrix for each endpoint
	for (unsigned int e = 0; e < endpoints.size(); e++)
	{
		endpoints[e].SetHVal(my_map, col);
		endpoints[e].id = e;
	}
}
void Simulation::LoadTask(string fname)
{

	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
	{
		cerr << "Task file not found." << endl;
		return;
	}
	//read file
	stringstream ss;
	int task_num;
	getline(myfile, line);
	ss << line;
	ss >> task_num;  // number of tasks
	//tasks_total.resize();
	for (int i = 0; i < task_num; i++)
	{
		int s, g, ts, tg;
		getline(myfile, line);
		ss.clear();
		ss << line;
		ss >> t_task >> s >> g >> ts >> tg; //time+start+goal+time at start+time at goal
		//printf("(%d)\n", t_task);
		Task task; // = Task(i, t_task, &endpoints[s], &endpoints[g], ts, tg);
		tasks_total.push_back(task);
		tasks_total[i].Set(i, t_task, &endpoints[s], &endpoints[g], ts, tg, false);
		//printf("%d %d\n", tasks_total[i].id, task.id);
	}
	myfile.close();
}

void Simulation::BFS() {
	for (int i = 0; i < my_map.size(); i++) {
		vector<int> D;
		D.resize(my_map.size(), INF);
		if (my_map[i]) {
			D[i] = 0;
			queue<int> Q;
			Q.push(i);
			while (!Q.empty()) {
				int u = Q.front();
				Q.pop();
				int offset[4] = {-1, 1, -col, col};
				for (int j = 0; j < 4; j++) {
					int v = u + offset[j];
					if (0 <= v && v < my_map.size() && abs(v % col - u % col) < 2 && my_map[v] && D[v] > D[u] + 1) {
						D[v] = D[u] + 1;
						Q.push(v);
					}
				}
			}
		}
		Dis.push_back(D);
	}
	return ;
}

vector<int> Simulation::MinCostAssignment(dlib::matrix<int> &cost) {
	int source = 0, sink = 1;
	CostFlow costflow(cost.nr() + cost.nc() + 2, source, sink);
	for (int i = 0; i < cost.nr(); i++) {
		costflow.AddEdges(source, 2 + i, 1, 0, i);
	}	
	for (int i = 0; i < cost.nc(); i++) {
		costflow.AddEdges(2 + cost.nr() + i, sink, 1, 0, -1);
	}	
	for (int i = 0; i < cost.nr(); i++) 
		for (int j = 0; j < cost.nc(); j++)
			costflow.AddEdges(2 + i, 2 + cost.nr() + j, 1, cost(i, j) , j);	
	vector<int> assignment(cost.nr(), 0);
	if (costflow.MinCostFlow() != cost.nr()) {
		printf("MaxCostAssignment ERROR!\n");
		while (1);
	}
	vector<vector<int> > paths = costflow.GetPath();
	for (int i = 0; i < paths.size(); i++) {
		assignment[paths[i][0]] = paths[i][1];
	}
	return assignment;
}

bool Simulation::PathFinding(vector<Agent*> &agents, vector<Task*> &tasks, vector<Endpoint*> parks, const vector<vector<int> > &cons_paths, int maxtimestep)
{
	vector<vector<int> > paths;
	int mincost = 0;
	dlib::matrix<int> cost(agents.size(), tasks.size() + parks.size());
	for (int i = 0; i < agents.size(); i++) {
		for (int j = 0; j < tasks.size(); j++) {
			cost(i, j) = Dis[agents[i]->loc][tasks[j]->start->loc];
			if (maxtimestep < cost(i,j))
				maxtimestep = cost(i,j);
		}
	}
	for (int i = 0; i < agents.size(); i++)
		for (int j = 0; j < parks.size(); j++) {
			cost(i, j + tasks.size()) = Dis[agents[i]->loc][parks[j]->loc] * 100000;
			if (maxtimestep < cost(i, j) / 100000)
				maxtimestep = cost(i, j) / 100000;
		}
	vector<int> assignment = MinCostAssignment(cost);
	for (int i = 0; i < agents.size(); i++)
		mincost += cost(i, assignment[i]);
	mincost %= 100000;
	for (int i = 0; i < tasks.size(); i++)
		if (maxtimestep < (tasks[i]->hold_time - timestep)) {
			maxtimestep = tasks[i]->hold_time - timestep;
		}
	maxtimestep++;
	//printf("%d\n", maxtimestep);
	while (true) {
		vector<vector<int> > in_node;
		vector<vector<int> > out_node;
		vector<int> task_node;
		int node_cnt = 0;
		int source = node_cnt++;
		int sink = node_cnt++;

		vector<vector<bool> > vis;
		for (int t = 0; t <= maxtimestep; t++) {
			vector<bool> v;
			v.resize(my_map.size(), false);
			vis.push_back(v);
		}

		for (int i = 0; i < cons_paths.size(); i++) {
			for (int t = 0; t < cons_paths[i].size(); t++) {
				int u = cons_paths[i][t];
				if (t >= timestep && t <= timestep + maxtimestep)
					vis[t - timestep][u] = true;
			}
		}

		for (int i = 0; i < my_map.size(); i++) {
			vector<int> in;
			vector<int> out;
			for (int j = 0; j <= maxtimestep; j++) {
				in.push_back(node_cnt++);
				out.push_back(node_cnt++);
			}
			in_node.push_back(in);
			out_node.push_back(out);
		}

		for (int i = 0; i < tasks.size(); i++)
			task_node.push_back(node_cnt++);

		vector<int> loc_task;
		loc_task.resize(my_map.size(), -1);

		for (int i = 0; i < tasks.size(); i++) {
			loc_task[tasks[i]->start->loc] = i;
		}

		CostFlow costflow(node_cnt, source, sink);
		
		for (int i = 0; i < agents.size(); i++) {
			costflow.AddEdges(source, out_node[agents[i]->loc][0], 1, 0, i);
		}

		for (int t = 0; t < maxtimestep; t++) {
			for (int u = 0; u < my_map.size(); u++) 
				if (my_map[u]) {
					if (loc_task[u] != -1 && timestep + t > tasks[loc_task[u]]->hold_time)
						continue;
					costflow.AddEdges(in_node[u][t], out_node[u][t], 1, 0, -1);
					if (loc_task[u] != -1 && timestep + t >= tasks[loc_task[u]]->hold_time)
						continue;
					costflow.AddEdges(out_node[u][t], in_node[u][t + 1], 1, 1, u);
					int offset[4] = {-1, 1, -col, col};
					for (int j = 0; j < 4; j++) {
						int v = u + offset[j];
						if (0 <= v && v < my_map.size() && abs(u % col - v % col) < 2 && my_map[v]) {
							if (loc_task[v] == -1)
								costflow.AddEdges(out_node[u][t], in_node[v][t + 1], 1, 1, v);
							else {
								int tt = t + 1 + timestep;
								if (tt <= tasks[loc_task[v]]->hold_time)
									costflow.AddEdges(out_node[u][t], in_node[v][t + 1], 1, 1, v);
								else if (vis[t + 1][v] == false)
									costflow.AddEdges(out_node[u][t], task_node[loc_task[v]], 1, 1, v);
							}
						}
					}
				}
		}

		for (int i = 0; i < parks.size(); i++) {
			costflow.AddEdges(in_node[parks[i]->loc][maxtimestep], sink, 1, 0, -1);
		}

		for (int i = 0; i < tasks.size(); i++) {
			Task* task = tasks[i]; 
			if (task->hold_time > timestep + maxtimestep)
				costflow.AddEdges(in_node[task->start->loc][maxtimestep], task_node[i], 1, 0, -1);
			else if (task->hold_time > timestep)
				costflow.AddEdges(out_node[task->start->loc][task->hold_time - timestep], task_node[i], 1, 0, -1);
			else
				costflow.AddEdges(out_node[task->start->loc][0], task_node[i], 1, 0, -1);
			costflow.AddEdges(task_node[i], sink, 1, -1000000, -1);
		}

		for (int i = 0; i < cons_paths.size(); i++) {
			for (int t = 1; t < maxtimestep; t++)
				if (timestep + t < cons_paths[i].size()) {
					int u = cons_paths[i][timestep + t];
					costflow.RemoveEdges(in_node[u][t], out_node[u][t]);
				}
		}

		for (int i = 0; i < cons_paths.size(); i++) {
			for (int t = 0; t < maxtimestep; t++)
				if (timestep + t + 1 < cons_paths[i].size()) {
					int u = cons_paths[i][timestep + t];
					int v = cons_paths[i][timestep + t + 1];
					int tt = t + 1 + timestep;
					if (loc_task[u] == -1)
						costflow.RemoveEdges(out_node[v][t], in_node[u][t + 1]);
					else if (tt <= tasks[loc_task[u]]->hold_time)
						costflow.RemoveEdges(out_node[v][t], in_node[u][t + 1]);
					else
						costflow.RemoveEdges(out_node[v][t], task_node[loc_task[u]]);
				}
		}
		int flow = costflow.MinCostFlow();
		int task_cnt = 0;
		while (costflow.cost < 0) {
			costflow.cost += 1000000;
			task_cnt++;
		}
		costflow.cost -= (flow - task_cnt) * maxtimestep;

		//printf("flow: %d costflow.cost: %d mincost: %d maxtimestep:%d\n",flow, costflow.cost, mincost, maxtimestep);
		if (flow != agents.size()) {
			mincost += maxtimestep;
			maxtimestep *= 2;
		}
		else {
			if (costflow.cost <= mincost) {
				//printf("%d\n", maxtimestep);
				paths = costflow.GetPath();
				break;
			}
			else {
				maxtimestep += costflow.cost - mincost;
				mincost = costflow.cost;
			}
		}
	}
	//vector<vector<int> > paths = costflow.GetPath();
	for (int i = 0; i < paths.size(); i++) {
		int id = paths[i][0];
		if (paths[i].size() == 1) 
			continue;
		for (int j = 1; j < paths[i].size(); j++)
		{
			agents[id]->path[timestep + j] = paths[i][j]; 
		}	

		//hold endpoint
		for (int j = paths[i].size() + timestep; j < maxtime; j++)
		{
			agents[id]->path[j] = paths[i][paths[i].size() - 1];
		}
	}
	while (true) {
		int edge_conflict = false;
		for (int i = 0; i < agents.size(); i++)
			for (int j = i + 1; j < agents.size(); j++)
				for (int t = 0; t < maxtime - 1; t++)
					if (agents[i]->path[t] == agents[j]->path[t + 1] && agents[j]->path[t] == agents[i]->path[t + 1]) {
						edge_conflict = true;
						for (int tt = t + 1; tt < maxtime; tt++)
							swap(agents[i]->path[tt], agents[j]->path[tt]);
					}
		if (!edge_conflict) break;
	}
	return true;
}

void Simulation::AssignTasks(vector<Agent*> &agents, const vector<vector<int> > &cons_paths)
{
	//hold goals of delivering tasks
	vector<bool> hold(col*row, false);
	for (list<Task*>::iterator it = tasks_deliver.begin(); it != tasks_deliver.end(); it++)
	{
		hold[(*it)->goal->loc] = true;
	}

	//pick off tasks that starts are not held and different with each other
	vector<Task*> tasks;
	vector<Endpoint*> starts;
	for (int i = 0; i < tasks_total.size(); i++)
		if (tasks_total[i].delivering == false)
		{
			//printf("%d %d\n", (*it)->start->loc, (int)hold[(*it)->start->loc]);
			if (hold[tasks_total[i].start->loc] == false /*&& hold[(*it)->goal->loc] == false*/)
			{
				tasks.push_back(&tasks_total[i]);
				//starts.push_back((*it)->start);
				hold[tasks_total[i].start->loc] = true; // once a task is chosen, hold its start and goal
				hold[tasks_total[i].goal->loc] = true;
			}
		}

	//if tasks are less than agents, add non-holding endpoints for each agent
	if (tasks.size() < agents.size())
	{
		//printf("%d %d\n", starts.size(), agents.size());
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			//choose nearest endpoint ep for agent i
			int ep = 0, dis = col*row;
			for (unsigned int j = 0; j < endpoints.size(); j++)
			{
				bool flag = true;
				for (int k = 0; k < cons_paths.size(); k++)
					if (cons_paths[k][cons_paths[k].size() - 1] == endpoints[j].loc)
						flag = false;
				if (!flag) continue;
				if (hold[endpoints[j].loc] == false && endpoints[j].loc == agents[i]->loc)
				{
					ep = j;
					dis = 0;
					break;

				}
				else if (hold[endpoints[j].loc] == false && 0 < endpoints[j].h_val[agents[i]->loc] && endpoints[j].h_val[agents[i]->loc] < dis)
				{
					ep = j;
					dis = endpoints[j].h_val[agents[i]->loc];
				}
			}
			hold[endpoints[ep].loc] = true;
			starts.push_back(&endpoints[ep]);
		}
	}
	//printf("!\n");
	int maxtimestep = 0;
	for (int i = 0; i < tasks.size(); i++) {
		tasks[i]->hold_time = tasks[i]->release_time;
		for (int j = 0; j < cons_paths.size(); j++)
			for (int t = tasks[i]->hold_time + 1; t < cons_paths[j].size(); t++)
				if (cons_paths[j][t] == tasks[i]->start->loc)
					tasks[i]->hold_time = t;
		//printf("%d %d\n", tasks[i]->id, tasks[i]->hold_time);
	}
	for (int i = 0; i < starts.size(); i++) {
		for (int j = 0; j < cons_paths.size(); j++)
			for (int t = maxtimestep + 1; t < cons_paths[j].size(); t++)
				if (cons_paths[j][t] == starts[i]->loc)
					maxtimestep = t;
	}
	PathFinding(agents, tasks, starts, cons_paths, maxtimestep - timestep);
}
bool Simulation::PathFinding(vector<Agent*> &agents, const vector<vector<int> > &cons_paths)
{
	EgraphReader egr;
	constraint_strategy s;
	s = constraint_strategy::ICBS;
	for (int i = 0; i < agents.size(); i++)
		agents[i]->park_loc = agents[i]->next_ep->loc;
	ICBSSearch icbs(my_map, agents, 1.0, egr, s, col, cons_paths, timestep);
	//ECBSSearch ecbs(my_map, agents, cons_paths, timestep, col, focal_w);
	if (icbs.runICBSSearch())
	{
		//update
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			//update searching path
			for (unsigned int j = 0; j < icbs.paths[i]->size(); j++)
			{
				agents[i]->path[timestep + j] = icbs.paths[i]->at(j).location;
			}
			//hold endpoint
			for (unsigned int j = icbs.paths[i]->size() + timestep; j < maxtime; j++)
			{
				agents[i]->path[j] = agents[i]->next_ep->loc;
			}
			if (icbs.paths[i]->size() > 100) {
				printf("%d %d\n", i, timestep);
				for (int j = 0; j < maxtime; j++)
					printf("%d:%d ", j, agents[i]->path[j]);
				for (int j = 0; j < cons_paths.size(); j++)
					if (cons_paths[j][684] == 492) {
						printf("!!!\n");
						for (int k = 0; k < 1000; k++)
							printf("%d:%d ", k, cons_paths[j][k]);
					}
			}
			//update task
			if (agents[i]->delivering == true)
			{
				agents[i]->task->ag_arrive_goal = timestep + icbs.paths[i]->size() - 1;
			}
			agents[i]->task = NULL;
		}
		return true;
	}	
	else
	{
		cout << "CBS fails" << endl;
		std::system("PAUSE");
		//Recovery. Let robots move along its original paths.
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			if (agents[i]->delivering == true)
			{
				agents[i]->delivering = false;
				DeliverGoal[agents[i]->task->goal->loc] = false;
				agents[i]->task->delivering = false;
				agents[i]->task->ag = NULL;
				//tasks_assign.push_back(agents[i]->task);
				tasks_deliver.pop_back();
			}
			agents[i]->task = NULL;
			agents[i]->next_ep = NULL;
		}
		return false;
	}
}

void Simulation::run(double focal_w)
{
	this->focal_w = focal_w;
	BFS();
	vector<bool> last_vis(agents.size(), false);
	for (timestep = 0; true; timestep++)
	{
		bool finish = true;
		for (int i = 0 ; i < tasks_total.size(); i++)
			if (tasks_total[i].delivering == false)
				finish = false;
		if (tasks_deliver.size() == 0 && finish)
			break;
		cout << endl << "Timestep " << timestep << endl;
		vector<Agent*> ag_pathfinding;
		vector<Agent*> ag_assign;
		vector<vector<int> > cons_paths;
		// delete FINISH tasks 	and update its agent's state	
		for (list<Task*>::iterator it = tasks_deliver.begin(); it != tasks_deliver.end();)
		{
			if (true == (*it)->delivering && timestep == (*it)->ag_arrive_goal)
			{
				(*it)->ag->finishtime = timestep;
				(*it)->ag->delivering = false;
				(*it)->ag->next_ep = NULL;
				(*it)->agent_id = (*it)->ag->id;
				DeliverGoal[(*it)->goal->loc] = false;
				list<Task*>::iterator done = it++;
				tasks_deliver.erase(done);
			}
			else
			{
				it++;
			}
		}

		//check non-package agents whether it is at a start that isn't held by other agent
		//if it is, assign task to it
		vector<int> ag_loc(my_map.size(), -1);
		vector<int> ag_hold(my_map.size(), -1);
		vector<int> hold(my_map.size(), 0);
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			agents[i].loc = agents[i].path[timestep];
			ag_hold[agents[i].path[maxtime - 1]] = i; //record every agent's holding point
			if (agents[i].delivering == false)
				ag_loc[agents[i].loc] = i; // record non_package agents current loc
			else
				cons_paths.push_back(agents[i].path); //record package agents' paths
			hold[agents[i].path[maxtime - 1]]++;
		}
		//printf("!\n");
		for (int i = 0; i < tasks_total.size(); i++)
			if (tasks_total[i].delivering == false && tasks_total[i].release_time <= timestep)
			{
				if (ag_loc[tasks_total[i].start->loc] >= 0 && hold[tasks_total[i].goal->loc] == 0 && DeliverGoal[tasks_total[i].goal->loc] == false)// assign agent to  deliver package
				{
					int id = ag_loc[tasks_total[i].start->loc];
					ag_loc[tasks_total[i].start->loc] = -1;
					//cout << "Agent " << id << " takes Task " << (*it)->start->loc << "-->" << (*it)->goal->loc << endl;
					tasks_total[i].ag_arrive_start = timestep;
					//update agent
					agents[id].task = &tasks_total[i];
					agents[id].next_ep = tasks_total[i].goal;
					agents[id].delivering = true;
					DeliverGoal[tasks_total[i].goal->loc] = true;
					//hold[(*it)->start->loc]--;
					ag_pathfinding.push_back(&agents[id]);
					//update task
					//list<Task*>::iterator done = it++;
					tasks_total[i].ag = &agents[id];
					tasks_total[i].delivering = true;
					
					tasks_deliver.push_back(&tasks_total[i]);
					//tasks_assign.erase(done);	
				}
			}
		//printf("!\n");
		num_computations++;
		clock_t start = std::clock();

		if (!ag_pathfinding.empty()) //path finding
		{
			//printf("%d\n", ag_pathfinding.size());
			cons_paths.clear();
			for (int i = 0; i < agents.size(); i++) {
				bool flag = true;
				for (int j = 0; j < ag_pathfinding.size(); j++)
					if (agents[i].id == ag_pathfinding[j]->id)
						flag = false;
				if (flag)
					cons_paths.push_back(agents[i].path);
			}
			printf("CBS [%d]\n", ag_pathfinding.size());
			PathFinding(ag_pathfinding, cons_paths);
		}
		ag_pathfinding.clear();

		//pick off non-package agents and 				
		//cout << "Non-package agents:	";
		bool changed = false;
		for (int i = 0; i < agents.size(); i++)
		{
			if (agents[i].delivering == false) {
				ag_assign.push_back(&agents[i]);
				if (last_vis[i] == false);
					changed = true;
				last_vis[i] = true;
			}
			else {
				if (last_vis[i] == true);
					changed = true;
				last_vis[i] = false;				
			}
		}
		if (!ag_assign.empty() && changed) //assign tasks
		{
			cons_paths.clear();
			for (int i = 0; i < agents.size(); i++) {
				bool flag = true;
				for (int j = 0; j < ag_assign.size(); j++)
					if (agents[i].id == ag_assign[j]->id)
						flag = false;
				if (flag)
					cons_paths.push_back(agents[i].path);
			}
			printf("CostFlow [%d]\n", ag_assign.size());
			AssignTasks(ag_assign, cons_paths); 
		}
		
		computation_time += (std::clock() - start);

		if (!TestConstraints()) //test correctness
		{
			std::system("PAUSE");
		}
		
	}
}

void Simulation::ShowTask()
{
	WaitingTime = 0;
	LastFinish = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		WaitingTime += tasks_total[i].ag_arrive_goal - tasks_total[i].release_time;
		LastFinish = LastFinish > tasks_total[i].ag_arrive_goal ? LastFinish : tasks_total[i].ag_arrive_goal;
	}
	//cout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//cout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	printf("%d ", LastFinish);
	freopen(out_file.c_str(), "w", stdout);
	printf("%d\n", tasks_total.size());
	for (int i = 0; i < tasks_total.size(); i++) 
		printf("%d %d %d\n", tasks_total[i].agent_id, tasks_total[i].ag_arrive_start, tasks_total[i].ag_arrive_goal);
}
void Simulation::SaveTask(const string &fname, const string &instance_name)
{
	/*/// write output file
	std::ofstream fout(fname, ios::app);
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//fout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc
				//	<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				WaitingTime += it->ag_arrive_goal - i;
				LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
	}
	//fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	fout << instance_name << " " << LastFinish << " " << WaitingTime << " " << computation_time / (double)LastFinish << endl;
	fout.close();*/
}

void Simulation::SaveThroughput(const string &fname)
{
	// write output file
	/*std::ofstream fout(fname + ".throughput");
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	vector<int> thpts(2500, 0);
	vector<int> inpts(2500, 0);

	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//fout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc
				//	<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				for (int time = 0; time < 100; time++) {
					thpts[it->ag_arrive_goal + time]++;
				}
				//WaitingTime += it->ag_arrive_goal - i;
				//LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
		for (int time = 0; time < 100; time++) {
			inpts[i + time] += tasks_total[i].size();
		}
	}
	//fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	for (int i = 0; i < thpts.size(); i++) {
		fout << thpts[i] << " " << inpts[i] << endl;
	}
	fout.close();*/
}

void Simulation::SavePath(string fname)
{
	// write output file
	/*std::ofstream fout(fname);
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	for (unsigned int i = 0; i < this->agents.size(); i++)
	{
		fout << maxtime << std::endl;
		for (unsigned int j = 0; j < maxtime; j++)
		{
			int x = this->agents[i].path[j] % col - 1;
			int y = this->agents[i].path[j] / col - 1;
			fout << x << "	" << y << endl;
		}
	}
	fout.close();*/
}

bool Simulation::TestConstraints()
{
	for (unsigned int ag = 0; ag < agents.size(); ag++)
	{
		for (unsigned int i = ag + 1; i < agents.size(); i++)
		{
			for (unsigned int j = timestep + 1; j < maxtime; j++)
			{
				if (agents[ag].path[j] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at location " 
						<< agents[ag].path[j] << " at time " << j << endl;
					printf("%d: ", ag);
					for (int t = 0; t < 400; t++)
						printf("%d ", agents[ag].path[t]);
					printf("\n\n");

					printf("%d: ", i);
					for (int t = 0; t < 400; t++)
						printf("%d ", agents[i].path[t]);
					printf("\n\n");
					while(1);
					return false;
				}
				else if (timestep > 0 && agents[ag].path[j] == agents[i].path[j - 1]
					&& agents[ag].path[j - 1] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at edge "
						<< agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
					while(1);
					return false;
				}
			}
		}
	}
	return true;
}
void Simulation::minCost()
{
	/*int cost = 0;
	for (int i = 0; i < t_task; i++)
	{
		if (!tasks_total[i].empty())
		{
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//cout << "(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")	";
				cost += it->start->h_val[it->goal->loc];
			}
		}
	}
	cout << "Min cost" << cost << endl;*/
}