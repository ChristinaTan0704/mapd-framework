#include "Simulation.h"

#include <ctime>
#include <algorithm>

#define INF 1000000000

Simulation::Simulation(string map_name, string task_name, string tour_file, string out_file, string tsp_file, string par_file) 
	: tour_file(tour_file), out_file(out_file), tsp_file(tsp_file), par_file(par_file)
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
	//DeliverGoal.resize(row*col, false);
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
				agents[ag].next_ep = NULL;
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
	//tasks_total.resize(task_num);
	for (int i = 0; i < task_num; i++)
	{
		int s, g, ts, tg;
		getline(myfile, line);
		ss.clear();
		ss << line;
		ss >> t_task >> s >> g >> ts >> tg; //time+start+goal+time at start+time at goal
		tasks_total.push_back(Task(i, t_task, &endpoints[s], &endpoints[g], ts, tg));
	}
	myfile.close();
}

void Simulation::TaskAssignment() {
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

// MinCostAssignment stub removed (dlib dependency)


vector<int> Simulation::CalcCost(Agent* ag) {
}


bool Simulation::PathFinding(vector<Agent*> &ags, const vector<Agent*> &cons_agents)
{
	EgraphReader egr;
	constraint_strategy s;
	s = constraint_strategy::ICBS;
	vector<Agent*> all_ags;
	for (int i = 0; i < ags.size(); i++) {
		ags[i]->goal_loc = ags[i]->next_ep->loc;
		ags[i]->start_time = timestep;
		ags[i]->only_dummy = false;
		all_ags.push_back(ags[i]);
	}
	for (int i = 0; i < cons_agents.size(); i++) {
		int dummy_step = cons_agents[i]->dummy_start_step;
		vector<int> cons;
		for (int j = 0; j <= dummy_step; j++) {
			cons.push_back(cons_agents[i]->path[j]);
		}
		cons_agents[i]->non_dummy_path = cons;
	}
	while (true) {
		vector<vector<int> > cons_paths;
		for (int i = 0; i < cons_agents.size(); i++)
			cons_paths.push_back(cons_agents[i]->path);
		ICBSSearch icbs(my_map, all_ags, 1.0, egr, s, col, cons_paths, timestep);
		if (icbs.runICBSSearch())
		{
			//update
			for (unsigned int i = 0; i < all_ags.size(); i++)
			{
				for (unsigned int j = 0; j < icbs.paths[i]->size(); j++)
				{
					all_ags[i]->path[timestep + j] = icbs.paths[i]->at(j).location;
				}
				//hold endpoint
				for (unsigned int j = icbs.paths[i]->size() + timestep; j < maxtime; j++)
				{
					all_ags[i]->path[j] = all_ags[i]->park_loc;
				}
				if (all_ags[i]->delivering == true && !all_ags[i]->only_dummy)
				{
					int t = all_ags[i]->task->ag_arrive_start;
					while(all_ags[i]->path[t] != all_ags[i]->next_ep->loc) t++;
					all_ags[i]->dummy_start_step = t;
					all_ags[i]->task->ag_arrive_goal = t; 
				}
			}
			bool ok = true;
			for (int i = 0; i < cons_agents.size(); i++) {
				if (!ReplanDummyPath(cons_agents[i])) {
					cons_agents[i]->non_dummy_path = cons_agents[i]->path;
					ok = false;
				}
			}
			if (ok)
				return true;
	}
	else
	{
		cout << "CBS fails" << endl;
		//Recovery. Let robots move along its original paths.
		/*for (int i = 0; i < ags.size(); i++) 
			printf("%d %d %d\n", i, ags[i]->id, ags[i]->next_ep->loc);
		for (unsigned int i = 0; i < ags.size(); i++)
		{
			if (ags[i]->delivering == true)
			{
				ags[i]->delivering = false;
				ags[i]->task->delivering = false;
				ags[i]->task->ag = NULL;
			}
			ags[i]->task = NULL;
			ags[i]->next_ep = NULL;
		}

		for (int i = 0; i < agents.size(); i++) {
			printf("%d: ", agents[i].id);
			for (int t = 0; t < timestep + 10; t++)
				printf("%d ", agents[i].path[t]);
			printf("\n");
		}
		while(1);
		return false;*/
	}
	}
}


void Simulation::CalcFlow(vector<Agent*> &agents, vector<Task*> &tasks, const vector<vector<int> > &cons_paths, vector<int> len, int &flow, vector<vector<int> > &paths) {
	int maxtimestep = 0;
	for (int i = 0; i < tasks.size(); i++)
		maxtimestep = max(maxtimestep, len[i] - timestep);

	maxtimestep++;

	vector<vector<int> > in_node;
	vector<vector<int> > out_node;
	vector<int> task_node;
	int node_cnt = 0;
	int source = node_cnt++;
	int sink = node_cnt++;

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
				costflow.AddEdges(in_node[u][t], out_node[u][t], 1, 0, -1);
				costflow.AddEdges(out_node[u][t], in_node[u][t + 1], 1, 1, u);
				int offset[4] = {-1, 1, -col, col};
				if (loc_task[u] != -1 && t + timestep >= tasks[loc_task[u]]->hold_time && tasks[loc_task[u]]->hold_time != 0) 
					continue;
				for (int j = 0; j < 4; j++) {
					int v = u + offset[j];
					if (0 <= v && v < my_map.size() && abs(u % col - v % col) < 2 && my_map[v]) {
						costflow.AddEdges(out_node[u][t], in_node[v][t + 1], 1, 1, v);
					}
				}
			}
	}

	for (int i = 0; i < tasks.size(); i++) {
		Task* task = tasks[i]; 
		int mint = max(task->hold_time - timestep, task->release_time - timestep);
		for (int t = max(mint, 0); t <= len[i] - timestep; t++)
			costflow.AddEdges(out_node[task->start->loc][t], task_node[i], 1, 0, -1);
		costflow.AddEdges(task_node[i], sink, 1, 0, -1);
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
				costflow.RemoveEdges(out_node[v][t], in_node[u][t + 1]);
			}
	}

	flow = costflow.MinCostFlow();
	paths = costflow.GetPath();
}

int Simulation::Cost(int t, queue<Task*> seq) {
}

bool Simulation::PathFinding(vector<Agent*> &agents, vector<Task*> &tasks, vector<vector<int> > &cons_paths)
{

	vector<vector<int> > paths;
	int l = 100;
	while (true) {
		l *= 2;
		vector<int> len;
		for (int i = 0; i < tasks.size(); i++) {
			len.push_back(l);
		}
		int flow;
		CalcFlow(agents, tasks, cons_paths, len, flow, paths);
		if (flow != agents.size()) {
			printf("no full flow\n");
		}
		else break;
	}

	vector<int> path_len(agents.size(), 0);
	for (int i = 0; i < paths.size(); i++) {
		int id = paths[i][0];
		path_len[id] = timestep + paths[i].size() - 1;
		if (paths[i].size() == 1) {
			for (int j = timestep + 1; j < maxtime; j++) {		
				agents[id]->path[j] = agents[id]->path[timestep];
			}
			continue;
		}
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
			for (int j = i + 1; j < agents.size(); j++) {
				int ai = paths[i][0], aj = paths[j][0];
				for (int t = timestep; t < min(path_len[ai], path_len[aj]); t++) {
					if (agents[ai]->path[t] == agents[aj]->path[t + 1] && agents[aj]->path[t] == agents[ai]->path[t + 1]) {
						edge_conflict = true;
						swap(path_len[ai], path_len[aj]);
						for (int tt = t + 1; tt < maxtime; tt++)
							swap(agents[ai]->path[tt], agents[aj]->path[tt]);
					}
				}
			}
		if (!edge_conflict) 
			break;
	}
	for (int i = 0; i < agents.size(); i++) {
		agents[i]->dummy_start_step = path_len[i];
		int loc = agents[i]->path[maxtime - 1];
		agents[i]->goal_loc = loc;
	}
	return true;
}

int Simulation::GoHome(vector<Agent*> &ags) {
	int* moves_offset; // = new int[5];
	moves_offset = new int[MapLoader::MOVE_COUNT];
	moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[MapLoader::valid_moves_t::NORTH] = -col;
	moves_offset[MapLoader::valid_moves_t::EAST] = 1;
	moves_offset[MapLoader::valid_moves_t::SOUTH] = col;
	moves_offset[MapLoader::valid_moves_t::WEST] = -1;
	bool* mymap = new bool [my_map.size()];
	for (int j = 0; j < my_map.size(); j++) {
		if (my_map[j]) {
			mymap[j] = false;
		}
		else {
			mymap[j] = true;
		}
	}
	for (int i = 0; i < ags.size(); i++) {
		int t = ags[i]->dummy_start_step;
		vector<vector<int> > cons_paths;
		for (int j = 0; j < agents.size(); j++) 
			if (agents[j].id != ags[i]->id) {
				cons_paths.push_back(agents[j].path);
			}
		int start_loc = ags[i]->path[t];
		int park_loc = ags[i]->park_loc;
		ComputeHeuristic ch(start_loc, start_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
		ComputeHeuristic ch1(start_loc, park_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
		vector<int> non_dummy_path;
		non_dummy_path.push_back(start_loc);
		SingleAgentICBS singleicbs(start_loc, start_loc, park_loc, mymap, my_map.size(),
			moves_offset, col, cons_paths, t, maxtime, true, non_dummy_path, t);
		ch.getHVals(singleicbs.my_heuristic);
		ch1.getHVals(singleicbs.my_heuristic1);
		vector<PathEntry> path;
		int goal_length;
		bool find_flag = singleicbs.findPath(path, goal_length, 1.0, NULL, NULL, 0, 0);
		if (find_flag) {
			for (int j = 0; j < path.size(); j++)
				ags[i]->path[t + j] = path.at(j).location;
			//hold endpoint
			for (unsigned int j = path.size() + t; j < maxtime; j++)
				ags[i]->path[j] = ags[i]->park_loc;
		}
		else {
			printf("GoHome Error! %d %d %d\n", i, start_loc, park_loc);
			return start_loc;
		}
		//delete singleicbs;
	}
	delete moves_offset;
	delete mymap;
	return -1;
}

bool Simulation::ReplanDummyPath(Agent* ag) {
	if (ag->dummy_start_step == maxtime - 1)
		return true;

	bool collision = false;
	for (int i = 0; i < agents.size() && !collision; i++)
	if (agents[i].id != ag->id)
	{
		for (int j = timestep + 1; !collision && j < maxtime; j++)
		{
			if (ag->path[j] == agents[i].path[j])
				collision = true;
			else if (ag->path[j] == agents[i].path[j - 1]
				&& ag->path[j - 1] == agents[i].path[j])
				collision = true;
		}
	}
	if (!collision)
		return true;
	vector<vector<int> > cons_paths;
	for (int i = 0; i < agents.size(); i++) 
		if (agents[i].id != ag->id) {
			cons_paths.push_back(agents[i].path);
		}
	int t = ag->dummy_start_step;
	int start_loc = ag->path[t];
	int park_loc = ag->park_loc;
	ComputeHeuristic ch(start_loc, start_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
	ComputeHeuristic ch1(start_loc, park_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
	vector<int> non_dummy_path;
	non_dummy_path.push_back(start_loc);
	SingleAgentICBS singleicbs(start_loc, start_loc, park_loc, mymap, my_map.size(),
		moves_offset, col, cons_paths, t, maxtime, true, non_dummy_path, t);
	ch.getHVals(singleicbs.my_heuristic);
	ch1.getHVals(singleicbs.my_heuristic1);
	vector<PathEntry> path;
	int goal_length;
	bool find_flag = singleicbs.findPath(path, goal_length, 1.0, NULL, NULL, 0, 0);
	if (find_flag) {
		printf("REPLAN SUCCESS\n");
		for (int j = 0; j < path.size(); j++)
			ag->path[t + j] = path.at(j).location;
			//hold endpoint
		for (int j = path.size() + t; j < maxtime; j++)
			ag->path[j] = ag->park_loc;
		return true;
	}
	else {
		printf("REPLAN Error!\n");
		return false;
	}
	delete moves_offset;
	delete mymap;
	return -1;
}

void Simulation::AssignNewTask(int id) {
}

void Simulation::run(double focal_w)
{
	BFS();
	moves_offset = new int[MapLoader::MOVE_COUNT];
	moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[MapLoader::valid_moves_t::NORTH] = -col;
	moves_offset[MapLoader::valid_moves_t::EAST] = 1;
	moves_offset[MapLoader::valid_moves_t::SOUTH] = col;
	moves_offset[MapLoader::valid_moves_t::WEST] = -1;
	mymap = new bool [my_map.size()];
	for (int j = 0; j < my_map.size(); j++) {
		if (my_map[j]) {
			mymap[j] = false;
		}
		else {
			mymap[j] = true;
		}
	}
	this->focal_w = focal_w;
	int cnt = 0;
	for (int i = 0; i < agents.size(); i++) {
		vector<int> v;
		real_assignments.push_back(v);
	}


	int makespan = 0;

	for (int i = 0; i < tasks_total.size(); i++)
		tasks_total[i].finished = false;

	for (timestep = 0; true; timestep++)
	{
		if (timestep > t_task) {
			bool finish = true;
			for (int i = 0; i < tasks_total.size(); i++)
				if (!tasks_total[i].finished)
					finish = false;
			if (finish) {
				makespan = timestep - 1;
				FILE *fp = freopen(out_file.c_str(), "w", stdout);
				printf("%d\n", (int)tasks_total.size());
				for (int i = 0; i < (int)tasks_total.size(); i++)
					printf("%d %d %d\n", tasks_total[i].agent_id, tasks_total[i].ag_arrive_start, tasks_total[i].ag_arrive_goal);
				fflush(fp);
				freopen("/dev/tty", "w", stdout);
				// Dump full agent paths for collision verification
				{
					string path_file = out_file + ".paths";
					ofstream pf(path_file);
					pf << agents.size() << " " << makespan << endl;
					for (int i = 0; i < (int)agents.size(); i++) {
						for (int t = 0; t <= makespan; t++) {
							if (t > 0) pf << " ";
							pf << agents[i].path[t];
						}
						pf << endl;
					}
					pf.close();
					fprintf(stderr, "Paths written to %s\n", path_file.c_str());
				}
				break;
			}
		}

		cout << endl << "Timestep " << timestep << endl;


		for (unsigned int i = 0; i < agents.size(); i++)
		{
			agents[i].loc = agents[i].path[timestep];
		}

		for (int i = 0; i < tasks_total.size(); i++)
		if (tasks_total[i].finished == false)
		{
			Task* task = &tasks_total[i];
			if (true == task->delivering && timestep == task->ag_arrive_goal) {	
				real_assignments[task->ag->id].push_back(task->Id);
				task->finished = true;
				task->agent_id = task->ag->id;
				task->delivering = false;
				task->ag->delivering = false;
				task->ag->next_ep = NULL;
				task->ag->task = NULL;
				cnt++;
				printf("%d Finish! %d\n", task->ag->id, cnt);
			}
		}
		//check non-package agents whether it is at a pickup location
		//if it is, assign task to it

		bool same_dest = false;
		vector<Agent*> ag_icbs;
		vector<bool> is_conflict(agents.size(), true);
		for (int i = 0; i < tasks_total.size(); i++)
		if (tasks_total[i].finished == false)
		{
			Task* task = &tasks_total[i];
			if (task->delivering == false && timestep >= task->release_time)// assign agent to deliver package
			{
				int id = -1;
				for (int j = 0; j < agents.size(); j++)
					if (!agents[j].delivering && agents[j].loc == task->start->loc)
						id = j;
				if (id == -1) 
					continue;
				task->ag_arrive_start = timestep;
				agents[id].task = task;
				agents[id].next_ep = task->goal;
				agents[id].delivering = true;
				ag_icbs.push_back(&agents[id]);
				task->ag = &agents[id];
				task->delivering = true;
				is_conflict[id] = false;
			}
		}
		if (!ag_icbs.empty()) //path finding
		{
			vector<Agent*> cons_agents;
			for (int i = 0; i < agents.size(); i++)
				if (is_conflict[i])
					cons_agents.push_back(&agents[i]);
			printf("ICBS: %d\n", ag_icbs.size());
			for (int i = 0; i < ag_icbs.size(); i++)
				printf("%d ", ag_icbs[i]->id);
			printf("\n");
			PathFinding(ag_icbs, cons_agents);
		}

		vector<bool> taskvis(tasks_total.size(), false);
		vector<bool> agentvis(agents.size(), false);
		vector<bool> in_costflow(agents.size(), false);
		while (true) {
			bool same_dest = false;
			vector<Agent*> ag_costflow;
			vector<Task*> task_costflow;
			for (int i = 0; i < tasks_total.size(); i++)
			if (tasks_total[i].finished == false)
			{
				Task* task = &tasks_total[i];
				if (taskvis[i] == false && task->delivering == false) {
					bool flag = false;
					for (int j = 0; j < task_costflow.size(); j++)
						if (task_costflow[j]->start->loc == task->start->loc) {
							flag = true;
							same_dest = true;
						}
					if (flag == true)
						continue;
					taskvis[i] = true;
					task_costflow.push_back(task);
				}
			}
			for (int i = 0; i < agents.size(); i++) {
				if (ag_costflow.size() < task_costflow.size() && agentvis[i] == false && agents[i].delivering == false) {
					ag_costflow.push_back(&agents[i]);
					agentvis[i] = true;
					in_costflow[i] = true;
				}
			}
			vector<int> hold_time;
			for (int i = 0; i < task_costflow.size(); i++) {
				task_costflow[i]->hold_time =  0;
			}

			for (int i = 0; i < agents.size(); i++) {
				vector<int> non_dummy_path;
				for (int j = 0; j <= agents[i].dummy_start_step; j++)
					non_dummy_path.push_back(agents[i].path[j]);
				agents[i].non_dummy_path = non_dummy_path;
			}
			while (true) {
				printf("COSTFLOW: %d\n", ag_costflow.size());
				for (int i = 0; i < ag_costflow.size(); i++)
					printf("%d ", ag_costflow[i]->id);
				printf("\n");
				vector<vector<int> > cons_paths;
				for (int i = 0; i < agents.size(); i++)
					if (!in_costflow[i]) {
						cons_paths.push_back(agents[i].path);
					}
				PathFinding(ag_costflow, task_costflow, cons_paths);
				int flag = GoHome(ag_costflow);
				if (flag == -1) break ;
				int task_id = -1;
				for (int i = 0; i < task_costflow.size(); i++)
					if (task_costflow[i]->start->loc == flag)
						task_id = i;
				if (task_id == -1) {
					printf("CANNOT FIND TASK\n");
					while (1);
				}
				task_costflow[task_id]->hold_time = task_costflow[task_id]->release_time;
				for (int i = 0; i < agents.size(); i++) 
					if (!in_costflow[i])
					for (int t = task_costflow[task_id]->hold_time + 1; t < agents[i].path.size(); t++)
						if (agents[i].path[t] == flag)
							task_costflow[task_id]->hold_time = t;
			}
			//if (!same_dest) 
			break;
		}
		if (!TestConstraints()) //test correctness
		{
			printf("TestConstraints ERROR\n");
			while(1);
		}
	}
	printf("%d\n", makespan);
}

void Simulation::ShowTask()
{
	int WaitingTime = 0;
	int LastFinish = 0;
	int completed = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].finished)
		{
			completed++;
			WaitingTime += tasks_total[i].ag_arrive_goal - tasks_total[i].release_time;
			if ((int)tasks_total[i].ag_arrive_goal > LastFinish)
				LastFinish = tasks_total[i].ag_arrive_goal;
		}
	}
	fprintf(stderr, "\nFinishing Timestep:\t%d\n", LastFinish);
	fprintf(stderr, "Sum of Task Waiting Time:\t%d\n", WaitingTime);
	fprintf(stderr, "Tasks completed:\t%d/%d\n", completed, (int)tasks_total.size());
}
void Simulation::SaveTask(const string &fname, const string &instance_name)
{
	/*// write output file
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
	/*// write output file
	std::ofstream fout(fname + ".throughput");
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
	/*// write output file
	std::ofstream fout(fname);
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
	for (int i = 0; i < agents.size(); i++) {
		for (int t = 0; t < maxtime; t++)
			if (my_map[agents[i].path[t]] == false) {
				printf("ILLEGAL POS! %d %d\n", i, agents[i].path[t]);
				while(1);
			}
	}
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
					printf("%d %d\n", agents[ag].dummy_start_step, agents[i].dummy_start_step);
					for (int k = timestep; k < timestep + 50; k++)
						printf("%d ", agents[ag].path[k]);
					printf("\n");
					for (int k = timestep; k < timestep + 50; k++)
						printf("%d ", agents[i].path[k]);
					printf("\n");
					return false;
				}
				else if ( agents[ag].path[j] == agents[i].path[j - 1]
					&& agents[ag].path[j - 1] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at edge "
						<< agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
					printf("%d %d\n", agents[ag].dummy_start_step, agents[i].dummy_start_step);
					for (int k = timestep; k < timestep + 50; k++)
						printf("%d:%d ", k, agents[ag].path[k]);
					printf("\n");
					for (int k = timestep; k < timestep + 50; k++)
						printf("%d:%d ", k, agents[i].path[k]);
					printf("\n");
					return false;
				}
			}
		}
	}
	return true;
}



void Simulation::minCost()
{
}