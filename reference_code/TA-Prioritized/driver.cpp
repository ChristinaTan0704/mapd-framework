#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <pthread.h>
#include <cstdlib>
#include <cmath>
#include "boost/program_options.hpp"
#include "simulation.h"
#include <boost/tokenizer.hpp>
#include <pthread.h>
#include <ctime>
#define NUM_THREADS 4

using namespace std;

string agents[5] = {"60", "90", "120", "150", "180"};


int main(int argc, char** argv) {

		for (int i = 0; i < 5; i++) {
			int t = clock();
			string map_file = "./Instances/large/kiva-" + agents[i] + ".map";
			string task_file = "./Instances/large/kiva-2000.task";
			string out_file = "./output/offline/" + agents[i] + "-large" + ".out";
			string tour_file = "./tour/middle-2000-" + agents[i] + "-" + ".tour";
			string tsp_file =  "./tsptw/large-2000-" + agents[i] + "-" + ".tsptw";
			string par_file =  "./par/large-2000-" + agents[i] + "-" + ".par";
			//cout << map_file << endl << task_file << endl << out_file << endl << tour_file <<  endl << tsp_file <<  endl << par_file <<  endl;
			Simulation simu(map_file, task_file, tour_file, out_file, tsp_file, par_file);
			simu.run(1);
			// string cmd = "./LKH3/LKH " + par_file;
			// system(cmd.c_str());
			printf("%d\n", (clock() - t) / 1000000);
	}
	return 0;
}
