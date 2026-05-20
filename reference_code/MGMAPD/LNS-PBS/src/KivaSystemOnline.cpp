#include "KivaSystemOnline.h"
#include "PBS.h"
#include <random>

KivaSystemOnline::KivaSystemOnline(const KivaGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}

KivaSystemOnline::~KivaSystemOnline()
{
}

bool KivaSystemOnline::load_tasks(string fname)
{
	string line;
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open()) {
		return false;
	}

    std::stringstream ss;
	getline(myfile, line);
	ss << line;
	ss >> task_num;
	bool have_freq = (bool)(ss >> task_frequency);
	bool have_period = (bool)(ss >> task_release_period);
	ss.clear();
    for (int i = 0; i < task_num; i++)
    {
        int release_time;
		getline(myfile, line);
		ss << line;
        ss >> release_time;
		vector<int> arr;
		arr.clear();
		int curr_loc;
		while (ss >> curr_loc)
		{
			if (arr.size() < 2)
				arr.push_back(G.endpoints[curr_loc]);
		}
        all_tasks.push_back(Task(i+1, release_time, arr));
		arr.push_back(release_time);
		all_tasks_list[i+1] = arr;
		total_release_time += release_time;
		ss.clear();
	}
	if (!have_freq || !have_period)
	{
		std::map<int, int> release_count;
		for (auto& t : all_tasks)
			release_count[t.release_time]++;
		if (release_count.size() <= 1)
		{
			task_release_period = 1;
			task_frequency = task_num;
		}
		else
		{
			auto it = release_count.begin();
			int first_time = it->first;
			int first_freq = it->second;
			++it;
			task_release_period = it->first - first_time;
			if (task_release_period <= 0) task_release_period = 1;
			task_frequency = first_freq;
		}
	}
	total_num_of_tasks = task_num;
	myfile.close();
	return true;
}

void KivaSystemOnline::initialize()
{
	initialize_solvers();
	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
	task_sequences.resize(num_of_drives);
	agents_task_sequences.resize(num_of_drives);
	agents_finish_sequence.resize(num_of_drives);

	for (int i = 0; i < num_of_drives; i++) {
		path_len.push_back(0);
		remained_agents.push_back(i);
	}
	bool succ = load_records(); // continue simulating from the records
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			initialize_start_locations();
		}
	}
}

void KivaSystemOnline::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	for (int k = 0; k < num_of_drives; k++)
	{
		int orientation = -1;
		if (consider_rotation)
		{
			orientation = rand() % 4;
		}
		starts[k] = State(G.agent_home_locations[k], 0, orientation);
		paths[k].emplace_back(starts[k]);
	}
}

void KivaSystemOnline::generate_tasks()
{
	int count = 0;
	for (int i = 0; i < task_release_period * task_frequency * look_ahead_horizon; i++)
	{
		if (all_tasks.empty())
			break;
		Task task = all_tasks.front();
		for (int i = 0; i < look_ahead_horizon; i++)
		{
			if (task.release_time == timestep + i * task_release_period)
			{
				current_tasks.insert(make_pair(task.task_id, task));
				all_tasks.erase(all_tasks.begin());
				count++;
			}
		}
	}
	if (count != 0)
		cout << "Generate " << count << " new tasks " << endl;
}

int KivaSystemOnline::choose_good_endpoint(vector<int> current_assigned_endpoints, int last_task_endpoint)
{
	std::map<int, int> distance;
	distance.clear();
	for (int i = 0; i < G.endpoints.size(); i++)
	{
		if (find(current_assigned_endpoints.begin(), current_assigned_endpoints.end(), G.endpoints[i]) != current_assigned_endpoints.end())
			continue;
		// if (G.endpoints[i] == last_task_endpoint)
		// 	continue;
		// int dist = G.get_Manhattan_distance(last_task_endpoint, G.endpoints[i]);
		int dist = G.heuristics.at(G.endpoints[i])[last_task_endpoint];
		distance[dist] = G.endpoints[i];
	}
	if (!distance.empty())
	{
		return distance.begin()->second;
	}
	else
	{
		return -1;
		// for (int i = 0; i < G.agent_home_locations.size(); i++)
		// {
		// 	if (find(current_assigned_endpoints.begin(), current_assigned_endpoints.end(), G.agent_home_locations[i]) != current_assigned_endpoints.end())
		// 		continue;
		// 	if (G.agent_home_locations[i] == last_task_endpoint)
		// 		continue;
		// 	// int dist = G.get_Manhattan_distance(last_task_endpoint,  G.agent_home_locations[i]);
		// 	int dist = G.heuristics.at(last_task_endpoint)[G.agent_home_locations[i]];
		// 	distance[dist] = G.agent_home_locations[i];
		// }
		// return distance.begin()->second;
	}
}

void KivaSystemOnline::update_goal_locations()
{	
	// run LNS for one second, given start pos and current release tasks
	vector<int> delivering_tasks;
	std::map<int, vector<int>> delivering_agents;
	std::map<int, pair<int,int>> agent_task_pair;
	new_agents.clear();
	assigned_agents.clear();
	free_agents.clear();
	// if agent is currently delivering a task, then we remove this task from LNS 
	if (REPLAN)
	{
		for (int i = 0; i < num_of_drives; i++)
		{
			if (task_sequences[i].empty())
			{
				goal_locations[i].clear();
				agents_task_sequences[i].clear();
				continue;
			}
			else
			{
				vector<int>::iterator iter = find(current_tasks[task_sequences[i].front()].goal_arr.begin(), 
					current_tasks[task_sequences[i].front()].goal_arr.end(), goal_locations[i].front().first);
				if ( goal_locations[i].size() > 1 && current_tasks[task_sequences[i].front()].goal_arr.size() != 1 &&  iter != current_tasks[task_sequences[i].front()].goal_arr.begin()
					&& iter != current_tasks[task_sequences[i].front()].goal_arr.end())
				{
					int idx = iter - current_tasks[task_sequences[i].front()].goal_arr.begin();
					delivering_tasks.push_back(task_sequences[i].front());
					vector<int> goal_subarr;
					while (iter != current_tasks[task_sequences[i].front()].goal_arr.end())
					{
						goal_subarr.push_back(*iter);
						iter++;
					}
					
					delivering_agents.insert(make_pair(i, goal_subarr));
					agent_task_pair.insert(make_pair(i, make_pair(task_sequences[i].front(), idx)));
				}
			}
			if ((!new_agent_finish || (new_agent_finish && !current_tasks.empty())) && apply_lns)
				task_sequences[i].clear();
			goal_locations[i].clear();
			agents_task_sequences[i].clear();
		}
	}
	
	if (timestep == 0)
	{
		for (int i =0; i < starts.size(); i++)
			current_assigned_endpoints.push_back(G.agent_home_locations[i]);
	}

	if (((REPLAN && !new_agent_finish) || (new_agent_finish && !current_tasks.empty())) && apply_lns)
	{
		TasksLoader tl(current_tasks, delivering_tasks, current_assigned_endpoints, deferred_task);
		AgentsLoader al(G, starts, delivering_agents, task_sequences);
		LNS lns(G, tl, al, 2, 1, 2, 2); 
		lns.run(lns_time);
		// lns.run_HBH_greedy();
		// lns.run_Hungarian_greedy();
		// flowtime_init_tp = calculate_flowtime_tp(task_sequences);
	}

	for (int i = 0; i < num_of_drives; i++)
	{
		int idx = remained_agents[i];
		int current_task_size = 0;
		if (delivering_agents.find(i) != delivering_agents.end())
		{
			int task_id = agent_task_pair[i].first;
			int task_idx = agent_task_pair[i].second;
			// if (REPLAN && !new_agent_finish)
			if (((REPLAN && !new_agent_finish) || (new_agent_finish && !current_tasks.empty())) && apply_lns)
				task_sequences[i].insert(task_sequences[i].begin(), task_id);
			int release_time = current_tasks[task_id].release_time;
			for (int idx = task_idx; idx < current_tasks[task_id].goal_arr.size(); idx++)
			{
				int loc = current_tasks[task_id].goal_arr[idx];
				goal_locations[i].push_back(make_pair(loc, release_time));
			}
			agents_task_sequences[i].push_back(current_tasks[task_id]);
			current_task_size++;
		}
		for (int j = 0; j < task_sequences[i].size(); j++)
		{
			if (current_task_size >= task_truncated_size)
				break;
			int task_id = task_sequences[i][j];
			if (task_id == agent_task_pair[i].first) {
				continue;
			}
			int release_time = current_tasks[task_id].release_time;
			for (int idx = 0; idx < current_tasks[task_id].goal_arr.size(); idx++)
			{
				int loc = current_tasks[task_id].goal_arr[idx];
				goal_locations[i].push_back(make_pair(loc, release_time));
			}
			agents_task_sequences[i].push_back(current_tasks[task_id]);
			current_task_size++;
		}
		if (goal_locations[i].empty())
			free_agents.push_back(i);
	}
	
	// Collect all task endpoints
	for (auto itr = current_tasks.begin();itr != current_tasks.end(); itr++)
    {
        Task task = itr->second;
        int task_id = itr->first;
        int i = 0;
        for (; i < task.goal_arr.size(); i++)
        {
            if (find(current_assigned_endpoints.begin(), current_assigned_endpoints.end(), task.goal_arr[i]) != current_assigned_endpoints.end()) {
                continue;
            }
			current_assigned_endpoints.push_back(task.goal_arr[i]);
        }
    }

	// Assign endpoints to non-free agents
	for (int i = 0; i < num_of_drives; i++)
	{
		if (goal_locations[i].size() == 0)
			continue;
		int loc = choose_good_endpoint(current_assigned_endpoints, goal_locations[i][goal_locations[i].size()-1].first);
		if (loc == -1)
			loc = G.agent_home_locations[i];
		goal_locations[i].push_back(make_pair(loc, 0));
		current_assigned_endpoints.push_back(loc);
	}

	// Assign endpoints to free agents
	for (int k : free_agents)
	{
		int loc = choose_good_endpoint(current_assigned_endpoints, starts[k].location);
		if (loc == -1)
			loc = G.agent_home_locations[k];
		current_assigned_endpoints.push_back(loc);
		goal_locations[k].push_back(make_pair(loc, 0));
	}

	// remember old endpoints
	current_assigned_endpoints.clear();
	for (int i = 0; i < num_of_drives; i++)
	{
		int	loc = goal_locations[i][goal_locations[i].size()-1].first;
		current_assigned_endpoints.push_back(loc);
	}
}

int KivaSystemOnline::calculate_flowtime_tp(vector<vector<int>> finish_task_sequence)
{
	int res = 0;
	int flowtime = 0;
	int makespan = 0;
	int cnt = 0;
	std::set<int> total;
	for (int l = 0; l < finish_task_sequence.size(); l++)
	{
		vector<int> i = finish_task_sequence[l];
		for (int t = 0 ; t < i.size(); t++)
		{
			total.insert(i[t]);
			vector<int> arr = all_tasks_list[i[t]];
			if (t == 0)
				res = G.get_Manhattan_distance(G.agent_home_locations[l], arr[0]);
			res = max(res, arr[arr.size()-1]);
			// last element of arr is the release time of task
			for (int j = 0; j < arr.size()-2; j++)
			{
				res += G.get_Manhattan_distance(arr[j], arr[j+1]);
			}
			cnt++;
			flowtime += res;
			makespan = max(makespan, res);
			if (t != i.size()-1)
				res += G.get_Manhattan_distance(arr[arr.size()-2], all_tasks_list[i[t+1]][0]);
		}
	}
	return flowtime;
}

void KivaSystemOnline::simulate(int simulation_time)
{
	clock_t t = clock();
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize();

	int mkspan = 0;
	int fltime = 0;
	int fltime_tp = 0;
	int last_plan_timestep = 0;
	int task_plan_time = 0;
	agents_finish_task_goal_arr.resize(num_of_drives);
	
	for (; timestep < simulation_time; timestep ++)
	{
		double task_planning_runtime = 0;
		double path_planning_runtime = 0;

		std::cout << "Timestep " << timestep << std::endl;
		if (REPLAN)
		{	
			new_agent_finish = false;
			if (all_tasks.size() != 0 && (timestep == 0 || timestep % task_release_period == 0))
			{
				generate_tasks(); // get current tasks
				update_start_locations();
				clock_t task_planning_time = clock();
				update_goal_locations(); // get task sequence, goal locations and agents_task_sequences
				task_planning_runtime = (double)(std::clock() - task_planning_time) * 1.0/ (CLOCKS_PER_SEC/1000);
				clock_t path_planning_time = clock();
				solve();
				path_planning_runtime = (double)(std::clock() - path_planning_time) * 1.0/ (CLOCKS_PER_SEC/1000);
				last_plan_timestep = timestep;
				path_planning_timestep.push_back(timestep);
				task_plan_time++;
			}
			else
			{
				for (int k = 0; k < num_of_drives; k++)
				{
					// any non-free agent finishes their current goals
					if ((find(free_agents.begin(), free_agents.end(), k)==free_agents.end() && goal_locations[k].size() == 1))
					{
						new_agent_finish = true;
						break;
					}
				}	
				if (new_agent_finish || deferred_task)
				{
					update_start_locations();
					apply_lns = true;
					clock_t task_planning_time = clock();
					update_goal_locations();
					task_planning_runtime = (double)(std::clock() - task_planning_time) * 1.0/ (CLOCKS_PER_SEC/1000);
					clock_t path_planning_time = clock();
					solve();
					path_planning_timestep.push_back(timestep);
					path_planning_runtime = (double)(std::clock() - path_planning_time) * 1.0/ (CLOCKS_PER_SEC/1000);
					last_plan_timestep = timestep;
					task_plan_time++;
					deferred_task = false;
				}
				apply_lns = true;
			}
		}
		
		// record time
		task_planning_time_list.push_back(task_planning_runtime);
		path_planning_time_list.push_back(path_planning_runtime);
		
		// move drives
		auto new_finished_tasks = move();
		int old = num_of_tasks;

		// update tasks
		int prev_finish_tasks = num_finished_tasks;
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			if (find(G.agent_home_locations.begin(), G.agent_home_locations.end(), loc) != G.agent_home_locations.end())
				continue;
			finished_tasks[id].emplace_back(loc, t);
			num_of_tasks++;
			if (agents_task_sequences[id].empty())
				continue;
			vector<int> curr_task_goal = agents_task_sequences[id].front().goal_arr;
			int num_of_curr_task_goal = curr_task_goal.size();
			int left_ptr = num_of_curr_task_goal-1;
			int right_ptr = finished_tasks[id].size()-1;
			int start = 0;
			while (start < num_of_curr_task_goal)
			{
				list<Key>::iterator it = finished_tasks[id].begin();
				std::advance(it, right_ptr--);
				if (curr_task_goal[left_ptr--] == it->first)
					start++;
				else
					break;	
			}
			// this task could be a pickup loc, or some dummy loc
			if (start == num_of_curr_task_goal)
			{
				agents_finish_task_goal_arr[id].push_back(curr_task_goal);
				fltime += t;
				mkspan = std::max(t, mkspan);
				num_finished_tasks++;
				current_tasks.erase(current_tasks.find(task_sequences[id].front()));
				agents_finish_sequence[id].push_back(task_sequences[id].front());
				task_sequences[id].erase(task_sequences[id].begin());
				agents_task_sequences[id].erase(agents_task_sequences[id].begin());
			}
		}
		
		if (screen > 0)
		{
			std::cout << num_of_tasks - old << " goals just finished" << std::endl;
			std::cout << num_of_tasks << " goals finished in total" << std::endl;
			std::cout << num_finished_tasks << " tasks finished in total" << std::endl;
		}
		throughput_accumulate.push_back(num_finished_tasks);
		if (num_finished_tasks == total_num_of_tasks) {
			break;
		}
		int curr_finish_tasks = num_finished_tasks - prev_finish_tasks;
		throughput_per_timestep.push_back(curr_finish_tasks);
	}
	update_start_locations();
	double runtime = (std::clock() - t) * 1.0/ (CLOCKS_PER_SEC/1000);
	std::cout << std::endl << "Done! " << std::endl;
	std::cout << num_finished_tasks << std::endl;
	cout<< "Makespan:  " << mkspan << " Flowtime:  " << (double)(fltime  - total_release_time)/total_num_of_tasks<< " Runtime: " << runtime/mkspan << " ms" << endl;
	save_results();
}
