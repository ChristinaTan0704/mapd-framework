#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "boost/program_options.hpp"
#include "simulation.h"
#include <boost/tokenizer.hpp>
#include <ctime>

using namespace std;

int main(int argc, char** argv) {
    string task_files[2] = {"Instances/small/kiva-500.task", "Instances/small/kiva-0.2.task"};
    string task_labels[2] = {"500", "0.2"};
    string map_file = "Instances/small/kiva-10-500-5.map";

    for (int j = 0; j < 2; j++) {
        printf("=== TA-Hybrid: map=kiva-10-500-5, task=%s ===\n", task_labels[j].c_str());
        fflush(stdout);
        int t = clock();
        string out_file = "output/offline/10-" + task_labels[j] + ".out";
        string tour_file = "tour/small-500-10-.tour";
        string tsp_file = "10-" + task_labels[j] + ".tsptw";
        string par_file = "10-" + task_labels[j] + ".par";
        Simulation simu(map_file, task_files[j], tour_file, out_file, tsp_file, par_file);
        simu.run(1);
        double elapsed = (double)(clock() - t) / CLOCKS_PER_SEC;
        printf("=== TA-Hybrid %s done, wall time: %.2f sec ===\n", task_labels[j].c_str(), elapsed);
        fflush(stdout);
    }
    return 0;
}
