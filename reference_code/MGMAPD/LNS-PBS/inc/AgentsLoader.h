/*
 * This code is modified from https://github.com/Jiaoyang-Li/Flatland
*/

#include "States.h"
#include "BasicGraph.h"
#include "KivaGraph.h"
#include <vector>
#include <string>
#include <utility>
#include <map>

class Agent{
public:
    int agent_id;
    int start_location;
    int start_timestep;
    bool is_delivering = false;
    vector<int> task_sequence;
    vector<int>* new_task_sequence;
    int task_sequence_makespan;
    
    Agent() {};
    Agent(int start_location, int agent_id);
    ~Agent();
    void Set(const State& starts, int agent_id, vector<int >* task_sequence);
};

class AgentsLoader {
public:
    int num_of_agents;
    vector<Agent> agents_all; // agent_all store all the agent and agents_all[i] has agent_id i
    int curr_assignment_flowtime;
    
    // REPLAN
    AgentsLoader(const KivaGrid& G, const vector<State>& starts,
            std::map<int, vector<int>> delivering_agents,
            vector< vector<int> >& task_sequences);
    // IGNORE
    AgentsLoader(const KivaGrid& G, const vector<State>& starts,
            vector<int> assigned_agents,
            vector< vector<int > >& task_sequences);
    AgentsLoader();
    ~AgentsLoader();
};
