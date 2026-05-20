#include "KivaSystem.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"

KivaSystem::KivaSystem(const KivaGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}

KivaSystem::~KivaSystem()
{
}

bool KivaSystem::load_task_assignments(string fname)
{
	string line;
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open()) {
		return false;
	}

	int id, start_loc, goal_loc, release_time;
	int seq_id = 0;
	vector<int> release_time_vec;
	for (int i = 0; i < 500; i++) {
		release_time_vec.emplace_back(499-i);
	}
	task_sequences.clear();
	agents_task_sequences.clear();
	while (getline(myfile, line)) {
		std::istringstream iss(line);
		iss >> id;
		if (id <= G.agent_home_locations.size()) {
			task_sequences.emplace_back();
			agents_task_sequences.emplace_back();
		}
		else {
			iss >> release_time >> start_loc >> goal_loc;
			task_sequences.back().emplace_back(G.endpoints[start_loc], release_time, 1); // 1 is pick up
			task_sequences.back().emplace_back(G.endpoints[goal_loc], release_time, 2); // 2 is delivery
			agents_task_sequences.back().emplace_back(G.endpoints[start_loc], G.endpoints[goal_loc]);
			total_release_time += release_time;
		}
	}
	myfile.close();
	return true;
}

void KivaSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
	if (task_sequences.empty()) {
		task_sequences.resize(num_of_drives);
		agents_task_sequences.resize(num_of_drives);
	}
	bool succ = load_records(); // continue simulating from the records
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			// cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();
		}
	}
}

void KivaSystem::initialize_start_locations()
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
		// finished_tasks[k].emplace_back(G.agent_home_locations[k], 0);
	}
}

void KivaSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
	// Choose random goal locations
	// Goal locations are not necessarily unique
	for (int k = 0; k < num_of_drives; k++)
	{
		// int goal = G.endpoints[rand() % (int)G.endpoints.size()];
		// goal_locations[k].emplace_back(goal, 0);
		if (task_sequences[k].empty()) {
			goal_locations[k].emplace_back(G.agent_home_locations[k], 0);
		}
		else {
			auto next = task_sequences[k].front();
			task_sequences[k].pop_front();
			goal_locations[k].emplace_back(make_pair(std::get<0>(next), std::get<1>(next)));
		}
	}
}

void KivaSystem::update_goal_locations()
{
	newly_finished_agents_idx.clear();
	if (hold_endpoints)
	{
		unordered_map<int, int> held_locations; // <location, agent id>
		for (int k = 0; k < num_of_drives; k++)
		{
			int curr = paths[k][timestep].location; // current location
			if (goal_locations[k].empty())
			{
				int next = G.endpoints[rand() % (int)G.endpoints.size()];
				while (next == curr || held_endpoints.find(next) != held_endpoints.end())
				{
					next = G.endpoints[rand() % (int)G.endpoints.size()];
				}
				goal_locations[k].emplace_back(next, 0);
				held_endpoints.insert(next);
			}
			if (paths[k].back().location == goal_locations[k].back().first &&  // agent already has paths to its goal location
				paths[k].back().timestep >= goal_locations[k].back().second) // after its relase time
			{
				int agent = k;
				int loc = goal_locations[k].back().first;
				auto it = held_locations.find(loc);
				while (it != held_locations.end()) // its start location has been held by another agent
				{
					int removed_agent = it->second;
					if (goal_locations[removed_agent].back().first != loc)
						cout << "BUG" << endl;
					new_agents.remove(removed_agent); // another agent cannot move to its new goal location
					cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
					held_locations[loc] = agent; // this agent has to keep holding this location
					agent = removed_agent;
					loc = paths[agent][timestep].location; // another agent's start location
					it = held_locations.find(loc);
				}
				held_locations[loc] = agent;
			}
			else // agent does not have paths to its goal location yet
			{
				if (held_locations.find(goal_locations[k].back().first) == held_locations.end()) // if the goal location has not been held by other agents
				{
					held_locations[goal_locations[k].back().first] = k; // hold this goal location
					new_agents.emplace_back(k); // replan paths for this agent later
					continue;
				}
				// the goal location has already been held by other agents 
				// so this agent has to keep holding its start location instead
				int agent = k;
				int loc = curr;
				cout << "Agent " << agent << " has to wait for agent " << held_locations[goal_locations[k].back().first] << " because of location " <<
					goal_locations[k].back().first << endl;
				auto it = held_locations.find(loc);
				while (it != held_locations.end()) // its start location has been held by another agent
				{
					int removed_agent = it->second;
					if (goal_locations[removed_agent].back().first != loc)
						cout << "BUG" << endl;
					new_agents.remove(removed_agent); // another agent cannot move to its new goal location
					cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
					held_locations[loc] = agent; // this agent has to keep holding its start location
					agent = removed_agent;
					loc = paths[agent][timestep].location; // another agent's start location
					it = held_locations.find(loc);
				}
				held_locations[loc] = agent;// this agent has to keep holding its start location
			}
		}
	}
	else
	{
		// newly_finished_agents_idx.clear();
		for (int k = 0; k < num_of_drives; k++)
		{
			int curr = paths[k][timestep].location; // current location
			if (useDummyPaths)
			{
				if (goal_locations[k].empty())
				{
					goal_locations[k].emplace_back(G.agent_home_locations[k], 0);
				}
				if (goal_locations[k].size() == 1)
				{
					int next;
					do {
						next = G.endpoints[rand() % (int)G.endpoints.size()];
					} while (next == curr);
					goal_locations[k].emplace(goal_locations[k].begin(), next, 0);
					new_agents.emplace_back(k);
				}
			}
			else
			{
				pair<int, int> goal; // The last goal location
				if (goal_locations[k].empty())
				{
					goal = make_pair(curr, timestep);
				}
				else
				{
					goal = goal_locations[k].back();
				}
				int min_timesteps = std::max(G.get_Manhattan_distance(goal.first, curr),
					goal.second - timestep); // G.heuristics.at(goal)[curr];
				while (min_timesteps <= simulation_window)
					// The agent might finish its tasks during the next planning horizon
				{
					// assign a new task
					tuple<int, int, int> next;
					if (goal.first == G.agent_home_locations[remained_agents[k]])
						break;
					if (task_sequences[k].empty()) {
						// next = make_tuple(G.agent_home_locations[remained_agents[k]], 0, 0);
						// goal_locations[k].emplace_back(make_pair(std::get<0>(next), std::get<1>(next)));
						break;
					}
					else {
						next = task_sequences[k].front();
						if (std::get<2>(next) == 2) {
							agents_delivery_loc[k].push_back(std::get<0>(next));
						}
						task_sequences[k].pop_front();
						goal_locations[k].emplace_back(make_pair(std::get<0>(next), std::get<1>(next)));
					}
					min_timesteps += G.get_Manhattan_distance(std::get<0>(next), goal.first); // G.heuristics.at(next)[goal];
					min_timesteps = std::max(min_timesteps, std::get<1>(next) - timestep);
					goal = make_pair(std::get<0>(next), std::get<1>(next));
				}
			}
		}
	}
}

void KivaSystem::simulate(int simulation_time)
{
	high_resolution_clock::time_point start_time = Time::now();
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize();
	for (int i = 0; i < num_of_drives; i++) {
		path_len.push_back(0);
		remained_agents.push_back(i);
		std::vector <int> row = {0};
		agents_delivery_loc.push_back(row);
	}
	int mkspan = 0;
	int fltime = 0;
	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "Timestep " << timestep << std::endl;
		std::cerr << "DBG: update_start_locations" << std::endl;
		update_start_locations();
		std::cerr << "DBG: update_goal_locations" << std::endl;
		update_goal_locations();
		std::cerr << "DBG: solve (drives=" << num_of_drives << " paths=" << paths.size() << " starts=" << starts.size() << " goals=" << goal_locations.size() << ")" << std::endl;
		for (int k = 0; k < num_of_drives; k++) {
			if ((int)paths[k].size() <= timestep) {
				std::cerr << "ERR: paths[" << k << "].size()=" << paths[k].size() << " but timestep=" << timestep << std::endl;
			}
		}
		solve();

		std::cerr << "DBG: move" << std::endl;
		auto new_finished_tasks = move();
		int old = num_of_tasks;
		std::cerr << "DBG: update tasks (" << new_finished_tasks.size() << " finished)" << std::endl;

		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			int prev_loc = -1;
			if (id < 0 || id >= (int)remained_agents.size()) {
				std::cerr << "ERR: id=" << id << " out of range (remained_agents.size=" << remained_agents.size() << ")" << std::endl;
				continue;
			}
			int ra = remained_agents[id];
			if (ra < 0 || ra >= (int)finished_tasks.size()) {
				std::cerr << "ERR: remained_agents[" << id << "]=" << ra << " out of range (finished_tasks.size=" << finished_tasks.size() << ")" << std::endl;
				continue;
			}
			if (!finished_tasks[ra].empty()) {
				prev_loc = finished_tasks[ra].back().first;
			}
			finished_tasks[ra].emplace_back(loc, t);
			num_of_tasks++;
			if (hold_endpoints)
				held_endpoints.erase(loc);
			mkspan = std::max(t, mkspan);
			pair<int, int> curr_finished_task = make_pair(prev_loc, loc);
			auto it = find(agents_task_sequences[ra].begin(), agents_task_sequences[ra].end(), curr_finished_task);
			if (it != agents_task_sequences[ra].end() && finished_tasks[ra].size() % 2 ==0)
			{
				fltime += t;
				count++;
			}
		}

		if (screen > 0)
		{
			std::cout << num_of_tasks - old << " tasks just finished" << std::endl;
			std::cout << num_of_tasks << " tasks finished in total" << std::endl;
		}

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}

		newly_finished_agents_idx.clear();
		for (int k = 0; k < num_of_drives; k++)
		{
			if (task_sequences[k].empty() && goal_locations[k].empty() && path_len[remained_agents[k]] == 0)
			{
				// goal_locations[k].emplace_back(G.agent_home_locations[remained_agents[k]], 0);
				path_len[remained_agents[k]] = paths[k].size() - 1;
				newly_finished_agents_idx.push_back(k);
				// cout << "Agent " << remained_agents[k] << " has finished its task sequence with " << 
				// 	finished_tasks[remained_agents[k]].size() << " tasks" << endl;
			}
		}
		if (find(path_len.begin(), path_len.end(), 0) == path_len.end()) {
			break;
		}
	}
	update_start_locations();
	// std::clock_t c_end = std::clock();
	double runtime = ((fsec)(Time::now() - start_time)).count();
	std::cout << std::endl << "Done! " << std::endl;
	std::cout << total_release_time << std::endl;
	cout<< "Makespan:  " << mkspan << " Flowtime:  " << fltime  - total_release_time<< " Runtime: " << runtime << " s" << endl;
	save_results();
}

// int KivaSystem::get_makespan()
// {
// 	while (paths[0].size() > 1)
// 	{
// 		int T = (int)paths[0].size() - 1;
// 		for (const auto& path : paths)
// 		{
// 			if (find(G.agent_home_locations.begin(), G.agent_home_locations.end(), path[T - 1].location) != G.agent_home_locations.end())
// 			{
// 				return T;
// 			}
// 		}
// 		for (int i = 0; i < int(paths.size()); i++)
// 		{
// 			paths[i].pop_back();
// 		}
// 	}
// 	return -1; // should never happen
// }