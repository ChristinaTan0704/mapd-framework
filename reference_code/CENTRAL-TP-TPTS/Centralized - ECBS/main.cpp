#include <iostream>
#include "Simulation.h"

using namespace std;

int main(int argc, char** argv) {
	if (argc < 3) {
		cout << "Usage: central <map_file> <task_file> [ecbs_weight]" << endl;
		return 1;
	}
	string map_file = argv[1];
	string task_file = argv[2];
	double w = 1.0;
	if (argc >= 4) w = atof(argv[3]);

	double t = clock();
	Simulation simu(map_file.c_str(), task_file.c_str());
	simu.run(w);
	simu.FullCollisionCheck("CENTRAL");
	simu.ShowTask(t);
	return 0;
}
