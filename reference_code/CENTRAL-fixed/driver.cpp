#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "simulation.h"
#include <boost/tokenizer.hpp>
#include <ctime>

using namespace std;

int main(int argc, char** argv) {
	if (argc < 3) {
		printf("Usage: %s <map_file> <task_file> [output_file]\n", argv[0]);
		return 1;
	}
	string map_file = argv[1];
	string task_file = argv[2];
	string out_file = (argc >= 4) ? argv[3] : "output.out";
	string tour_file = "tour.tour";
	string tsp_file = "dummy.tsptw";
	string par_file = "dummy.par";

	printf("Map: %s\n", map_file.c_str());
	printf("Task: %s\n", task_file.c_str());

	int t = clock();
	Simulation simu(map_file, task_file, tour_file, out_file, tsp_file, par_file);
	simu.run(1);
	simu.ShowTask();
	printf("Time: %d seconds\n", (clock() - t) / CLOCKS_PER_SEC);

	return 0;
}
