#include "KivaSystemOnline.h"
#include "PBS.h"
#include "StateTimeAStar.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
using namespace std;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " <map> <agents> [task_file] [lns_time] [sim_window] [plan_window] [sim_time] [seed]" << endl;
        return 1;
    }
    string map_file = argv[1];
    int num_agents = atoi(argv[2]);
    string task_file = (argc > 3) ? argv[3] : "";
    int lns_time = (argc > 4) ? atoi(argv[4]) : 1;
    int sim_window = (argc > 5) ? atoi(argv[5]) : 1073741823;
    int plan_window = (argc > 6) ? atoi(argv[6]) : 1073741823;
    int sim_time = (argc > 7) ? atoi(argv[7]) : 5000;
    int seed = (argc > 8) ? atoi(argv[8]) : 0;

    srand(seed);

    KivaGrid G;
    if (!G.load_Minghua_map(map_file)) return -1;

    StateTimeAStar path_planner;
    PBS pbs(G, path_planner);
    pbs.lazyPriority = false;

    KivaSystemOnline system(G, pbs);
    system.outfile = "/tmp/mgmapd_out";
    system.screen = 0;
    system.log = false;
    system.num_of_drives = num_agents;
    system.time_limit = 120;
    system.simulation_window = sim_window;
    system.planning_window = plan_window;
    system.travel_time_window = 0;
    system.consider_rotation = false;
    system.k_robust = 0;
    system.hold_endpoints = false;
    system.useDummyPaths = true;
    system.task_truncated_size = 1;
    system.REPLAN = true;
    system.look_ahead_horizon = 1;
    system.lns_time = lns_time;
    system.seed = seed;

    G.preprocessing(false);
    system.load_tasks(task_file);
    system.simulate(sim_time);
    return 0;
}
