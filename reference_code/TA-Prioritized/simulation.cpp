#include "Simulation.h"

#include <ctime>
#include <dlib/optimization/max_cost_assignment.h>
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
	ss >> workpoint_num;  // start and goal locations, parking locations

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
	BFS();
	int task_cnt = tasks_total.size();
	int agent_cnt = agents.size();
	int node_cnt = task_cnt + agent_cnt; 
	freopen(tour_file.c_str(), "r", stdin);
	char st[256];
	st[0] = '\0';
	while (strcmp(st, "TOUR_SECTION"))
		scanf("%255s", st);
	queue<Task*> seq;
	int t = 0, seq_id = 0, loc = agents[0].park_loc;
	seq_agent.push_back(0);
	for (int i = 0; i < node_cnt; i++) {
		int u;
		scanf("%d", &u);
		if (u <= agent_cnt) {
			if (u != 1) {
				TSP_seqs.push_back(seq);
				//printf("Makespan: %d Task num: %d\n", t, seq.size());
				seq_length.push_back(t);
				while (seq.size() > 0) seq.pop();
				t = 0;
				loc = agents[u - 1].park_loc;
				seq_agent.push_back(u - 1);
				seq_id++;
			}
		} 
		else {
			Task* task = &tasks_total[u - agent_cnt - 1];
			t += Dis[loc][task->start->loc];
			if (t < task->release_time)
				t = task->release_time;
			t += Dis[task->start->loc][task->goal->loc];
			loc = task->goal->loc;
			task->seq_id = seq_id;
			seq.push(task);
		}
	}
	seq_length.push_back(t);
	//printf("Makespan: %d Task num: %d\n", t, seq.size());
	TSP_seqs.push_back(seq);
	return ;
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
}

bool Simulation::PathFinding(vector<Agent*> &ags, const vector<vector<int> > &cons_paths) {
}

bool Simulation::PathFinding(vector<Agent*> &agents, vector<Task*> &tasks, const vector<vector<int> > &cons_paths) {
}

int Simulation::SingleFindPath(Agent* ag, int t) {
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
	vector<vector<int> > cons_paths;
	for (int i = 0; i < agents.size(); i++)
		if (agents[i].id != ag->id)
			cons_paths.push_back(agents[i].path);
	int start_loc = ag->path[t];
	int goal_loc = ag->next_ep->loc;
	int park_loc = ag->park_loc;
	ComputeHeuristic ch(start_loc, goal_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
	ComputeHeuristic ch1(goal_loc, park_loc, mymap, row, col, moves_offset, NULL, 1.0, NULL);
	SingleAgentICBS singleicbs(start_loc, goal_loc, park_loc, mymap, my_map.size(),
		moves_offset, col, cons_paths, t, maxtime, ag->release_time - t);
	ch.getHVals(singleicbs.my_heuristic);
	ch1.getHVals(singleicbs.my_heuristic1);
	vector<PathEntry> path;
	int goal_length;
	//printf("%d %d\n", start_loc, goal_loc);
	bool find_flag = singleicbs.findPath(true, path, goal_length, 1.0, NULL, NULL, 0, 0);
	if (find_flag) {
		for (int j = 0; j < path.size(); j++)
			ag->path[t + j] = path.at(j).location;
		//hold endpoint
		for (unsigned int j = path.size() + t; j < maxtime; j++)
			ag->path[j] = ag->park_loc;
		return t + goal_length - 1;
	}
	else {
		bool find_flag = singleicbs.findPath(false, path, goal_length, 1.0, NULL, NULL, 0, 0);
		if (!find_flag) {
			fprintf(stderr, "Error: SingleFindPath failed for agent %d at t=%d, start=%d, goal=%d\n",
				ag->id, t, start_loc, goal_loc);
			for (unsigned int j = t; j < maxtime; j++)
				ag->path[j] = start_loc;
			delete[] moves_offset;
			delete[] mymap;
			return t;
		}
		else {
			for (int j = 0; j < path.size(); j++)
				ag->path[t + j] = path.at(j).location;
			//hold endpoint
			for (unsigned int j = path.size() + t; j < maxtime; j++)
				ag->path[j] = ag->park_loc;
		}
		return t + goal_length - 1;
	}
	//delete singleicbs;
	delete[] moves_offset;
	delete[] mymap;
}
 
void Simulation::run(double focal_w)
{
	int start_time = clock();
	TaskAssignment();
	vector<bool> seq_vis(agents.size(), false);
	vector<int> finish_time;
	int sumofcost = 0, makespan = 0;
	for (int i = 0; i < agents.size(); i++) {
		fprintf(stderr, "Planning agent %d/%d\n", i, (int)agents.size());
		fflush(stderr);
		int max_len = 0, id = -1;
		for (int j = 0; j < agents.size(); j++)
		if (!seq_vis[j])
		{
			Agent * ag = &agents[seq_agent[j]];
			queue<Task*> seq = TSP_seqs[j];
			int size = seq.size(), t = 0;
			int loc = ag->park_loc;
			for (int k = 0; k < size; k++) {
				Task * task = seq.front();
				seq.pop();
				t += Dis[loc][task->start->loc];
				if (t < task->release_time)
					t = task->release_time;
				t += Dis[task->start->loc][task->goal->loc];
				loc = task->goal->loc;
			}
			if (id == -1 || t > max_len) {
				max_len = t;
				id = j;
			}
		}
		seq_vis[id] = true;
		Agent * ag = &agents[seq_agent[id]];
		queue<Task*> seq = TSP_seqs[id];
		int size = seq.size(), t = 0;
		for (int k = 0; k < size; k++) {
			Task * task = seq.front();
			seq.pop();
			ag->next_ep = task->start;
			ag->release_time = task->release_time;
			t = SingleFindPath(ag, t);
			if (t < task->release_time)
				t = task->release_time;
			ag->next_ep = task->goal;
			t = SingleFindPath(ag, t);
			finish_time.push_back(t);
		}
		makespan = max(makespan, t);
		printf("%d\n", t);
		sumofcost += t;
	}
	timestep = 0;
	if (!TestConstraints()) //test correctness
	{
		printf("TestConstraints ERROR\n");
		exit(1);
	}
	printf("%d %d\n", sumofcost, makespan);
	FILE *fp = freopen(out_file.c_str(), "w", stdout);
	printf("%d\n", finish_time.size());
	for (int i = 0; i < finish_time.size(); i++)
		printf("%d\n", finish_time[i]);
	printf("sumofcost: %d\n", sumofcost);
	printf("makespan: %d\n", makespan);
	printf("runtime: %d\n", (clock() - start_time) / 1000000);
	fflush(stdout);
	freopen("/dev/tty", "w", stdout);

	// Save full paths for verification
	string path_file = out_file + ".paths";
	FILE *pf = fopen(path_file.c_str(), "w");
	if (pf) {
		fprintf(pf, "%d %d %d\n", (int)agents.size(), makespan + 1, col);
		for (int i = 0; i < agents.size(); i++) {
			fprintf(pf, "Agent %d:", i);
			for (int t = 0; t <= makespan; t++)
				fprintf(pf, " %d", agents[i].path[t]);
			fprintf(pf, "\n");
		}
		fclose(pf);
		fprintf(stderr, "Paths saved to %s\n", path_file.c_str());
	}

	// Save task delivery info for verification
	string task_verify_file = out_file + ".tasks";
	FILE *tf = fopen(task_verify_file.c_str(), "w");
	if (tf) {
		fprintf(tf, "%d\n", (int)tasks_total.size());
		for (int i = 0; i < tasks_total.size(); i++)
			fprintf(tf, "%d %d %d %d\n", tasks_total[i].Id, tasks_total[i].release_time,
				tasks_total[i].start->loc, tasks_total[i].goal->loc);
		fclose(tf);
	}

	return ;
}

void Simulation::ShowTask()
{
	/*int WaitingTime = 0;
	int LastFinish = 0;
	//cout << "TASK" << endl;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//cout << "Timestep " << i<<" :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//cout << "(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")	";
				WaitingTime += it->ag_arrive_goal - i;
				LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
	}
	cout << endl << "Finishing Timestep:	" << LastFinish << endl;
	cout << "Sum of Task Waiting Time:	" << WaitingTime << endl;*/
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
				exit(1);
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
					return false;
				}
				else if ( agents[ag].path[j] == agents[i].path[j - 1]
					&& agents[ag].path[j - 1] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at edge "
						<< agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
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