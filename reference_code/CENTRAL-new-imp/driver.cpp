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
string frequency[5] = {"1", "2", "5", "10", "500"};

int main(int argc, char** argv) {

	for (int j = 0; j < 5; j++)
		for (int i = 0; i < 5; i++) {
			//i = 0; j = 0;
			printf("%d %d\n", i, j);
			int t = clock();
			string map_file = "Instances/small/kiva-" + agents[i] + "-500-5.map";
			string task_file = "Instances/small/kiva-" + frequency[j] + ".task";
			string out_file = "output/offline/" + agents[i] + "-" + frequency[j] + ".out";
			string tour_file = "tour/" + agents[i] + "-" + frequency[j] + ".tour";
			string tsp_file =  agents[i] + "-" + frequency[j] + ".tsptw";
			string par_file =  agents[i] + "-" + frequency[j] + ".par";
			//cout << map_file << endl << task_file << endl << out_file << endl << tour_file <<  endl << tsp_file <<  endl << par_file <<  endl;
			Simulation simu(map_file, task_file, out_file);
			simu.run(1);
			//string cmd = "./LKH3/LKH " + par_file;
			//system(cmd.c_str());
			printf("%d\n", (clock() - t) / 1000000);
	}
	return 0;
}
