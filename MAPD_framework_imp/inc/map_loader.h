#pragma once
#include "types.h"
#include <fstream>
#include <sstream>
#include <queue>

struct MAPDMap {
    int row, col;
    int raw_row, raw_col;
    unsigned int maxtime;
    int num_agents;
    int workpoint_num;
    vector<bool> grid;
    vector<bool> is_endpoint;
    vector<Endpoint> endpoints;
    vector<int> agent_starts;

    void load(const string& fname);
    vector<int> get_neighbors(int loc) const;
};

vector<Task> load_tasks(const string& fname, const vector<Endpoint>& endpoints);
