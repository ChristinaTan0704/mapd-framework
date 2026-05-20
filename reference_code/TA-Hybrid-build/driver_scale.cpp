#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "simulation.h"
#include <ctime>

using namespace std;

int main(int argc, char** argv) {
    if (argc < 4) {
        printf("Usage: %s <map_file> <task_file> <tour_file> [output_file]\n", argv[0]);
        return 1;
    }
    string map_file = argv[1];
    string task_file = argv[2];
    string tour_file = argv[3];
    string out_file = (argc > 4) ? argv[4] : "output.out";
    string tsp_file = "temp.tsptw";
    string par_file = "temp.par";

    printf("Map: %s\nTask: %s\nTour: %s\n", map_file.c_str(), task_file.c_str(), tour_file.c_str());
    fflush(stdout);
    int t = clock();
    Simulation simu(map_file, task_file, tour_file, out_file, tsp_file, par_file);
    simu.run(1);
    double elapsed = (double)(clock() - t) / CLOCKS_PER_SEC;
    printf("Wall time: %.2f seconds\n", elapsed);
    return 0;
}
