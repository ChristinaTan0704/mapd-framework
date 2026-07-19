#include "map_loader.h"

void MAPDMap::load(const string& fname) {
    ifstream myfile(fname);
    if (!myfile.is_open()) {
        cerr << "Map file not found: " << fname << endl;
        exit(1);
    }
    string line;
    getline(myfile, line);
    size_t comma = line.find(',');
    raw_row = stoi(line.substr(0, comma));
    raw_col = stoi(line.substr(comma + 1));
    row = raw_row + 2;
    col = raw_col + 2;

    getline(myfile, line);
    workpoint_num = stoi(line);

    getline(myfile, line);
    num_agents = stoi(line);

    getline(myfile, line);
    maxtime = stoi(line);

    grid.resize(row * col, false);
    is_endpoint.resize(row * col, false);
    endpoints.resize(workpoint_num + num_agents);
    agent_starts.clear();

    int ep = 0, ag = 0;
    for (int i = 1; i < row - 1; i++) {
        getline(myfile, line);
        for (int j = 1; j < col - 1; j++) {
            int loc = i * col + j;
            char ch = (j - 1 < (int)line.size()) ? line[j - 1] : '.';
            grid[loc] = (ch != '@');
            is_endpoint[loc] = (ch == 'e' || ch == 'r');
            if (ch == 'e') {
                endpoints[ep].loc = loc;
                endpoints[ep].id = ep;
                endpoints[ep].is_task_endpoint = true;
                ep++;
            } else if (ch == 'r') {
                endpoints[workpoint_num + ag].loc = loc;
                endpoints[workpoint_num + ag].id = workpoint_num + ag;
                endpoints[workpoint_num + ag].is_task_endpoint = false;
                agent_starts.push_back(loc);
                ag++;
            }
        }
    }
    myfile.close();

    for (int i = 0; i < row; i++) {
        grid[i * col] = false;
        grid[i * col + col - 1] = false;
        is_endpoint[i * col] = false;
        is_endpoint[i * col + col - 1] = false;
    }
    for (int j = 1; j < col - 1; j++) {
        grid[j] = false;
        grid[(row - 1) * col + j] = false;
        is_endpoint[j] = false;
        is_endpoint[(row - 1) * col + j] = false;
    }

    for (int e = 0; e < (int)endpoints.size(); e++) {
        endpoints[e].h_val.resize(row * col, INT_MAX);
        queue<int> Q;
        Q.push(endpoints[e].loc);
        endpoints[e].h_val[endpoints[e].loc] = 0;
        while (!Q.empty()) {
            int curr = Q.front(); Q.pop();
            for (int d : {1, -1, col, -col}) {
                int next = curr + d;
                if (next >= 0 && next < row * col && grid[next] && endpoints[e].h_val[next] == INT_MAX) {
                    endpoints[e].h_val[next] = endpoints[e].h_val[curr] + 1;
                    Q.push(next);
                }
            }
        }
    }

    // Build O(1) location -> endpoint index map. Mirrors the linear scans
    // `for (e ...) if (endpoints[e].loc == loc) ...` used throughout the
    // simulator. First-match semantics (lowest endpoint index wins), matching
    // those scans which break on the first hit.
    loc2ep.clear();
    loc2ep.reserve(endpoints.size() * 2);
    for (int e = 0; e < (int)endpoints.size(); e++) {
        loc2ep.emplace(endpoints[e].loc, e);  // emplace keeps the first index
    }
}

vector<int> MAPDMap::get_neighbors(int loc) const {
    vector<int> nbrs;
    for (int d : {1, -1, col, -col}) {
        int nloc = loc + d;
        if (nloc >= 0 && nloc < row * col && grid[nloc]) nbrs.push_back(nloc);
    }
    return nbrs;
}

vector<Task> load_tasks(const string& fname, const vector<Endpoint>& endpoints) {
    ifstream myfile(fname);
    if (!myfile.is_open()) {
        cerr << "Task file not found: " << fname << endl;
        exit(1);
    }
    string line;
    getline(myfile, line);
    int num_tasks = stoi(line);
    vector<Task> tasks;

    // Peek at the second line to detect format:
    // Standard: "release pickup delivery start_wait goal_wait" (5 integers)
    // Varying:  "release goal1 [goal2 ...]" (variable count, no trailing 0 0)
    streampos pos_after_header = myfile.tellg();
    string peek_line;
    if (getline(myfile, peek_line)) {
        stringstream ps(peek_line);
        vector<int> vals;
        int v;
        while (ps >> v) vals.push_back(v);

        bool is_varying = false;
        if ((int)vals.size() == 5 && vals[3] == 0 && vals[4] == 0) {
            // Could be standard format — check more lines to be sure
            // If any line has != 5 fields, it's varying
            streampos save = myfile.tellg();
            string check;
            for (int c = 0; c < min(5, num_tasks - 1); c++) {
                if (!getline(myfile, check)) break;
                stringstream cs(check);
                int cnt = 0;
                int cv;
                while (cs >> cv) cnt++;
                if (cnt != 5) { is_varying = true; break; }
            }
            myfile.seekg(pos_after_header);
        } else {
            is_varying = true;
            myfile.seekg(pos_after_header);
        }

        if (is_varying) {
            for (int i = 0; i < num_tasks; i++) {
                getline(myfile, line);
                stringstream ss(line);
                int release;
                ss >> release;
                vector<int> goal_eps;
                int ep;
                while (ss >> ep) goal_eps.push_back(ep);

                int pickup_ep = goal_eps.empty() ? 0 : goal_eps.front();
                int delivery_ep = goal_eps.size() >= 2 ? goal_eps.back() : pickup_ep;
                Task t(i, pickup_ep, delivery_ep,
                       endpoints[pickup_ep].loc, endpoints[delivery_ep].loc,
                       release, 0, 0);
                t.goals.clear();
                for (int e : goal_eps)
                    t.goals.push_back(endpoints[e].loc);
                if (t.goals.empty())
                    t.goals.push_back(endpoints[pickup_ep].loc);
                tasks.push_back(t);
            }
        } else {
            for (int i = 0; i < num_tasks; i++) {
                getline(myfile, line);
                stringstream ss(line);
                int release, s, g, sw, gw;
                ss >> release >> s >> g >> sw >> gw;
                tasks.push_back(Task(i, s, g, endpoints[s].loc, endpoints[g].loc, release, sw, gw));
            }
        }
    }

    myfile.close();
    return tasks;
}
