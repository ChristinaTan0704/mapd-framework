#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include "simulation.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <agents> <frequency>\n", argv[0]);
        return 1;
    }
    string ag = argv[1];
    string freq = argv[2];
    string base = "Instances/small/";
    string map_file = base + "kiva-" + ag + "-500-5.map";
    string task_file = base + "kiva-" + freq + ".task";
    string out_file = "/tmp/central_single_out.txt";

    clock_t t_start = clock();
    Simulation simu(map_file, task_file, out_file);
    simu.run(1);
    double elapsed = (double)(clock() - t_start) / CLOCKS_PER_SEC * 1000.0;

    // ShowTask computes LastFinish/WaitingTime and writes per-task data to out_file
    simu.ShowTask();
    // stdout is now redirected to out_file by ShowTask's freopen
    // Print results to stderr
    fprintf(stderr, "RESULT,%s,%s,%d,%d,%.0f\n", ag.c_str(), freq.c_str(),
            simu.LastFinish, simu.WaitingTime, elapsed);
    return 0;
}
