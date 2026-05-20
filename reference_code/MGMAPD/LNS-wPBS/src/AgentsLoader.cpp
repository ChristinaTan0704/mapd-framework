/*
 * This code is modified from https://github.com/Jiaoyang-Li/Flatland
*/
#include "AgentsLoader.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <vector>

using namespace boost;
using namespace std;


Agent::Agent(int start_location, int agent_id)
    : start_location(start_location), agent_id(agent_id)
{};

Agent::~Agent(){}

void Agent::Set(const State& starts, int agent_id, vector<int >* task_sequence)
{
    this->start_location = starts.location;
    this->start_timestep = starts.timestep;
    this->agent_id = agent_id;
    this->new_task_sequence = task_sequence;
};

AgentsLoader::AgentsLoader(const KivaGrid& G, const vector<State>& starts,
            std::map<int, vector<int>> delivering_agents,
            vector< vector<int > >& task_sequences)
{
    this->num_of_agents = starts.size();
    this->agents_all.resize(num_of_agents);
    for (int ag = 0; ag < num_of_agents; ag++)
    {
        this->agents_all[ag].Set(starts[ag], ag+1, &task_sequences[ag]);
        if (delivering_agents.find(ag) != delivering_agents.end())
        {
            this->agents_all[ag].is_delivering = true;
            this->agents_all[ag].start_location = delivering_agents[ag][0];
            // this->agents_all[ag].start_timestep = G.get_Manhattan_distance(starts[ag].location, delivering_agents[ag][0]);
            
            this->agents_all[ag].start_timestep = starts[ag].timestep + G.get_Manhattan_distance(starts[ag].location, delivering_agents[ag][0]);
            for (int i = 0; i < delivering_agents[ag].size()-1; i++)
            {
                this->agents_all[ag].start_timestep += G.get_Manhattan_distance(delivering_agents[ag][i], delivering_agents[ag][i+1]);
            }
        }
    }
}

AgentsLoader::AgentsLoader(const KivaGrid& G, const vector<State>& starts,
            vector<int> assigned_agents,
            vector< vector<int > >& task_sequences)
{
    this->num_of_agents = starts.size() - assigned_agents.size();
    this->agents_all.resize(num_of_agents);

    vector<int> new_agents_list;
    for (int k = 0; k < starts.size(); k++)
    {
        if (find(assigned_agents.begin(), assigned_agents.end(), k) != assigned_agents.end())
            continue;
        new_agents_list.push_back(k);
    }
    for (int ag = 0; ag < num_of_agents; ag++)
    {
        int idx = new_agents_list[ag];
        this->agents_all[ag].Set(starts[idx], ag+1, &task_sequences[idx]);
    }
}
AgentsLoader::~AgentsLoader(){}
