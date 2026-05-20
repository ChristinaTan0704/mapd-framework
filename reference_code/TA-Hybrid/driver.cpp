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

string agents[5] = {"10", "20", "30", "40", "50"};
string tasks[6] = {"0.2", "0.5", "1", "2", "5", "10"};


int main(int argc, char** argv) {

		for (int i = 0; i < 1; i++) {
			int t = clock();
			string map_file = "Instances/small/kiva-" + agents[i] + "-500-5.map";
			string task_file = "Instances/small/kiva-"+ tasks[i] +".task";
			string out_file = "output/offline/" + agents[i] + "-small.out";
			string tour_file = "tour/small-500-" + agents[i] + "-" + ".tour";
			string tsp_file =  "tsptw/small-500-" + agents[i] + "-" + ".tsptw";
			string par_file =  "par/small-500-" + agents[i] + "-" + ".par";
			//cout << map_file << endl << task_file << endl << out_file << endl << tour_file <<  endl << tsp_file <<  endl << par_file <<  endl;
			Simulation simu(map_file, task_file, tour_file, out_file, tsp_file, par_file);
			simu.run(1);
			//string cmd = "./LKH3/LKH " + par_file;
			//system(cmd.c_str());
			// simu.ShowTask();
			printf("%d\n", (clock() - t) / 1000000);
	}
	return 0;
}
