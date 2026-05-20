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
	BFS();
	int task_cnt = tasks_total.size();
	int agent_cnt = agents.size();
	int node_cnt = task_cnt + agent_cnt; 
	for (int i = 0; i < node_cnt; i++) {
		vector<int> W;
		W.resize(node_cnt, 1000000);
		TSPEdgeWeight.push_back(W);
	}
	for (int k = 0; k < agent_cnt; k++) {
		for (int i = 0; i < task_cnt; i++) {
			Task* task = &tasks_total[i];
			Agent* agent = &agents[k];
			TSPEdgeWeight[agent_cnt + i][k] = Dis[task->start->loc][task->goal->loc];
			TSPEdgeWeight[k][agent_cnt + i] = max(Dis[agent->loc][task->start->loc], task->release_time);
		}
	}
	for (int i = 0; i < task_cnt; i++)
		for (int j = 0; j < task_cnt; j++) 
			if (i != j) {
				Task* taski = &tasks_total[i], *taskj = &tasks_total[j];
				int d = Dis[taski->start->loc][taski->goal->loc] + Dis[taski->goal->loc][taskj->start->loc];
				TSPEdgeWeight[agent_cnt + i][agent_cnt + j] = d;
			}
	FILE *fp = freopen(tsp_file.c_str(), "w", stdout);
	printf("NAME: MAPD\n");
	printf("TYPE: TSPTW\n");
	printf("DIMENSION: %d\n", node_cnt);
	printf("EDGE_WEIGHT_TYPE: EXPLICIT\n");
	printf("DISPLAY_DATA_TYPE: NO_DISPLAY\n");
	printf("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n");
	printf("EDGE_WEIGHT_SECTION\n");
	for (int i = 0; i < node_cnt; i++) {
		for (int j = 0; j < node_cnt; j++)
			printf("%d ", TSPEdgeWeight[i][j]);
		printf("\n");
	}
	printf("TIME_WINDOW_SECTION\n");
	for (int i = 0; i < agent_cnt; i++) {
		printf("%d -1 %d 0 0 0\n", i + 1, 10000);
	}
	int cnt = 0;
	for (int i = 0; i < task_cnt; i++) {
		Task* task = &tasks_total[i];
		cnt += Dis[task->start->loc][task->goal->loc];
		printf("%d %d %d %d %d %d\n", i + 1 + agent_cnt, task->release_time, 10000, Dis[task->start->loc][task->goal->loc], task->start->loc, task->goal->loc);
	}
	fflush(stdout);
	freopen(par_file.c_str(), "w", stdout);
	printf("SPECIAL\n");
	printf("PROBLEM_FILE = %s\n", tsp_file.c_str());
	printf("MAX_TRIALS = 100000\nTIME_LIMIT = 30000\n");
	printf("RUNS = 1\n");
	printf("TRACE_LEVEL = 1\n");
	printf("TOUR_FILE = %s\n", tour_file.c_str());
	printf("MAKESPAN = YES\n");
	fflush(stdout);
	freopen("/dev/tty", "w", stdout);
	//system("./LKH3/LKH MAPD.par");
	freopen(tour_file.c_str(), "r", stdin);
	char st[256];
	st[0] = '\0';
	while (strcmp(st, "TOUR_SECTION"))
		scanf("%255s", st);
	queue<Task*> seq;
	int seq_id = 0, last = 1, t = 0, loc = agents[0].park_loc;
	TSP_agent.push_back(0);
	for (int i = 0; i < node_cnt; i++) {
		int u;
		scanf("%d", &u);
		if (u <= agent_cnt) {
			if (u != 1) {
				TSP_seqs.push_back(seq);
				TSP_len.push_back(t);
				TSP_agent.push_back(u - 1);
				printf("Makespan: %d Task num: %d\n", t, seq.size());
				while (seq.size() > 0) seq.pop();
				seq_id++;
				t = 0;
				loc = agents[u - 1].park_loc;
			}
		} 
		else {
			Task * task = &tasks_total[u - agent_cnt - 1];
			t += Dis[loc][task->start->loc];
			if (t < task->release_time)
				t = task->release_time;
			t += Dis[task->start->loc][task->goal->loc];
			loc = task->goal->loc;
			tasks_total[u - agent_cnt - 1].seq_id = seq_id;
			seq.push(&tasks_total[u - agent_cnt - 1]);
		}
		last = u;
	}
	printf("Makespan: %d Task num: %d\n", t, seq.size());
	TSP_len.push_back(t);
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


vector<int> Simulation::CalcCost(Agent* ag) {
	queue<Task*> seq;
	bool find_seq = false;
	for (int i = 0; i < agents.size(); i++) 
		if (TSP_agent[i] == ag->id) {
			find_seq = true;
			seq = TSP_seqs[i];
		}
	vector<int> cost;
	for (int t = timestep; t < timestep + 200; t++) {
		queue<Task*> tasks = seq;
		Task* task = tasks.front();
		tasks.pop();
		int loc = task->goal->loc, tt = t;
		while (!tasks.empty()) {
			task = tasks.front();
			tasks.pop();
			tt += Dis[loc][task->start->loc];
			if (tt < task->release_time)
				tt = task->release_time;
			tt += Dis[task->start->loc][task->goal->loc];
			loc = task->goal->loc;
		}
		cost.push_back(tt);
	}
	return cost;
}

bool Simulation::PathFinding(vector<Agent*> &ags, const vector<vector<int> > &cons_paths)
{
	EgraphReader egr;
	constraint_strategy s;
	s = constraint_strategy::ICBS;
	for (int i = 0; i < ags.size(); i++) {
		ags[i]->goal_loc = ags[i]->next_ep->loc;
		ags[i]->start_time = timestep;
		ags[i]->only_dummy = false;
		ags[i]->cost = CalcCost(ags[i]);
	}
	ICBSSearch icbs(my_map, ags, 1.0, egr, s, col, cons_paths, timestep);
	if (icbs.runICBSSearch())
	{
		//update
		for (unsigned int i = 0; i < ags.size(); i++)
		{
			//update searching path
			for (unsigned int j = 0; j < icbs.paths[i]->size(); j++)
			{
				ags[i]->path[timestep + j] = icbs.paths[i]->at(j).location;
			}
			//hold endpoint
			for (unsigned int j = icbs.paths[i]->size() + timestep; j < maxtime; j++)
			{
				ags[i]->path[j] = ags[i]->park_loc;
			}
			//update task
			if (ags[i]->delivering == true)
			{
				int t = ags[i]->task->ag_arrive_start;
				while(ags[i]->path[t] != ags[i]->next_ep->loc) t++;
				ags[i]->task->ag_arrive_goal = t; //timestep + icbs.paths[i]->size() - 1;
			}
			//agents[i]->task = NULL;
		}
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
		}*/
		return false;
	}
}



void Simulation::CalcFlow(vector<Agent*> &agents, vector<Task*> &tasks, const vector<vector<int> > &cons_paths, vector<int> len, int &flow, vector<vector<int> > &paths) {
}

int Simulation::Cost(int t, queue<Task*> seq) {
	int loc = -1;
	while (seq.size()) {
		Task *task = seq.front();
		seq.pop();
		if (!task->delivering) {
			if (loc != -1)
				t += Dis[loc][task->start->loc];
			if (t < task->release_time)
				t = task->release_time;
			t += Dis[task->start->loc][task->goal->loc];
		}
		loc = task->goal->loc;
	}
	return t;
}

bool Simulation::PathFinding(vector<Agent*> &agents, vector<Task*> &tasks, const vector<vector<int> > &cons_paths) {
}

int Simulation::GoHome(vector<Agent*> &ags) {

}

bool Simulation::ReplanDummyPath(Agent* ag) {
	
}

void Simulation::AssignNewTask(int id) {
	Task *chosetask = NULL;
	int tmax = 0, seq_id = -1;
	for (int i = 0; i < agents.size(); i++) 
		if (i != id && TSP_seqs[i].size() >= 2) {
			if (TSP_seqs[i].front()->delivering && TSP_seqs[i].size() == 1)
				continue;
			queue<Task*> seq = TSP_seqs[i];
			int t = timestep, loc = agents[TSP_agent[i]].path[timestep];
			Task* task;
			while (!seq.empty()) {
				task = seq.front();
				seq.pop();
				if (!task->delivering) {
					t += Dis[loc][task->start->loc];
					if (t < task->release_time)
						t = task->release_time;
					t += Dis[task->start->loc][task->goal->loc];
				}
				else
					t += Dis[loc][task->goal->loc];
				loc = task->goal->loc;
			}
			if (t > tmax && timestep + Dis[agents[TSP_agent[id]].loc][task->start->loc] > task->release_time - 10
				&& timestep + Dis[agents[TSP_agent[id]].loc][task->start->loc] + Dis[task->start->loc][task->goal->loc]  < t) {
				tmax = t;
				chosetask = task;
				seq_id = i;
			}
		}
	if (seq_id != -1) {
		queue<Task*> seq;
		while (TSP_seqs[seq_id].size()) {
			Task* task = TSP_seqs[seq_id].front();
			TSP_seqs[seq_id].pop();
			if (task != chosetask)
				seq.push(task);
		}
		TSP_seqs[seq_id] = seq;
		chosetask->seq_id = id;
		TSP_seqs[id].push(chosetask);
		global_makespan = 0;
		for (int i = 0; i < agents.size(); i++) {
			global_makespan = max(global_makespan, Cost(timestep, TSP_seqs[i]));
		}
	}
	return ;
}

void Simulation::run(double focal_w)
{
	int start_time = clock();
	TaskAssignment();
	this->focal_w = focal_w;
	for (int i = 0; i < agents.size(); i++) {
		vector<int> v;
		real_assignments.push_back(v);
	}

	int sumofcost = 0;
	vector<bool> last_vis(agents.size(), false);
	int tot = 0;
	for (timestep = 0; true; timestep++)
	{
		if (timestep > t_task) {
			bool finish = true;
			for (int i = 0; i < agents.size(); i++)
				if (TSP_seqs[i].size() > 0) finish = false;
			if (finish) {
				FILE *fp = freopen(out_file.c_str(), "w", stdout);
				printf("%d\n", tasks_total.size());
				for (int i = 0; i < tasks_total.size(); i++)
					printf("%d %d %d\n", tasks_total[i].agent_id, tasks_total[i].ag_arrive_start, tasks_total[i].ag_arrive_goal);
				printf("runtime: %d\n", (clock() - start_time) / 1000000);
				printf("makespan: %d\n", timestep - 1);
				fflush(fp);
				freopen("/dev/tty", "w", stdout); 
				break;
			}
		}

		cout << endl << "Timestep " << timestep << " " << tot << endl;

		vector<int> need_plan;
		if (timestep == 0) {
			for (int i = 0; i < agents.size(); i++)
				need_plan.push_back(i);
		}

		for (int i = 0; i < agents.size(); i++)
			agents[i].loc = agents[i].path[timestep];

		for (int i = 0; i < agents.size(); i++)
			if (!TSP_seqs[i].size())
				AssignNewTask(i);

		for (int i = 0; i < agents.size(); i++)
		{
			if (!TSP_seqs[i].size()) continue;
			Task* task = TSP_seqs[i].front();
			int id = TSP_agent[i];
			if (true == task->delivering && timestep == task->ag_arrive_goal) {	
				tot++;
				real_assignments[task->ag->id].push_back(task->Id);
				task->agent_id = task->ag->id;
				task->delivering = false;
				task->ag->delivering = false;
				task->ag->next_ep = NULL;
				task->ag->task = NULL;
				TSP_seqs[i].pop();
				need_plan.push_back(i);
				if (!TSP_seqs[i].size())
					sumofcost += timestep;
			}
		}
		vector<Agent*> ag_icbs;
		vector<bool> is_conflict(agents.size(), true);
		for (int j = 0; j < need_plan.size(); j++) {
			int i = need_plan[j];
			if (!TSP_seqs[i].size()) continue;
			Task* task = TSP_seqs[i].front();
			int id = TSP_agent[i];
			if (agents[id].loc != task->start->loc || timestep < task->release_time) {
				agents[id].next_ep = task->start;
				agents[id].release_time = task->release_time;
				ag_icbs.push_back(&agents[id]);
				is_conflict[id] = false;				
			}			
		}
		for (int i = 0; i < agents.size(); i++)
		{
			if (!TSP_seqs[i].size()) continue;
			Task* task = TSP_seqs[i].front();
			int id = TSP_agent[i];
			if (task->delivering == false && timestep >= task->release_time && agents[id].loc == task->start->loc)// assign agent to deliver package
			{
				task->ag_arrive_start = timestep;
				agents[id].task = task;
				agents[id].next_ep = task->goal;
				agents[id].delivering = true;
				ag_icbs.push_back(&agents[id]);
				//update task
				task->ag = &agents[id];
				task->delivering = true;
				is_conflict[id] = false;
			}
		}
		if (!ag_icbs.empty()) //path finding
		{
			for (int i = 0; i < agents.size(); i++) {
			if (!TSP_seqs[i].size()) continue;
				Task* task = TSP_seqs[i].front();
				int id = TSP_agent[i];
				bool flag = true;
				for (int j = 0; j < ag_icbs.size(); j++)
					if (ag_icbs[j]->id == id)
						flag = false;
				if (flag) {
					is_conflict[id] = false;
					if (agents[id].next_ep == NULL) {
						agents[id].next_ep = task->start;
						agents[id].release_time = task->release_time;
					}
					ag_icbs.push_back(&agents[id]);
				}
			}
			printf("ICBS %d\n", ag_icbs.size());
			for (int i = 0; i < ag_icbs.size(); i++)
				printf("%d ", ag_icbs[i]->id);
			printf("\n");
			vector<vector<int> > cons_paths;
			for (int i = 0; i < agents.size(); i++)
				if (is_conflict[i])
					cons_paths.push_back(agents[i].path);
			if (!PathFinding(ag_icbs, cons_paths))
				return ;
		}
		if (!TestConstraints()) //test correctness
		{
			printf("TestConstraints ERROR\n");
			exit(1);
		}
	}
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