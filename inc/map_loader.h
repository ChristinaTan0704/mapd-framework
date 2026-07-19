#pragma once
#include "types.h"
#include <fstream>
#include <sstream>
#include <queue>
#include <unordered_map>

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
    // O(1) location -> endpoint index lookup (built in load()).
    // Pure performance helper: returns the same value a linear scan over
    // `endpoints` would, or -1 if no endpoint sits on `loc`.
    unordered_map<int,int> loc2ep;

    void load(const string& fname);
    vector<int> get_neighbors(int loc) const;
    inline int ep_index(int loc) const {
        auto it = loc2ep.find(loc);
        return it == loc2ep.end() ? -1 : it->second;
    }
};

vector<Task> load_tasks(const string& fname, const vector<Endpoint>& endpoints);
