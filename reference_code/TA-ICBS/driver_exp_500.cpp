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
    printf("=== TA-ICBS: map=kiva-10-500-5, task=500 ===\n");
    fflush(stdout);
    int t = clock();
    string map_file = "Instances/small/kiva-10-500-5.map";
    string task_file = "Instances/small/kiva-500.task";
    string out_file = "output/offline/10-500.out";
    string tour_file = "tour/10-500.tour";
    string tsp_file = "10-500.tsptw";
    string par_file = "10-500.par";
    Simulation simu(map_file, task_file, tour_file, out_file, tsp_file, par_file);
    simu.run(1);
    double elapsed = (double)(clock() - t) / CLOCKS_PER_SEC;
    printf("=== TA-ICBS 500 done, wall time: %.2f sec ===\n", elapsed);
    fflush(stdout);
    return 0;
}
