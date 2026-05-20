#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include "simulation.h"

using namespace std;

int main() {
    string agents_arr[] = {"10", "20", "30", "40", "50"};
    string frequency[] = {"500", "10", "5", "2", "1", "0.5", "0.2"};
    string base = "Instances/small/";

    FILE* csv = fopen("/Users/jiaqit/Desktop/paper/output/central_ref_results.csv", "w");
    fprintf(csv, "algorithm,agents,frequency,makespan,swt,avg_service,tasks,runtime_ms\n");
    fflush(csv);

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            string map_file = base + "kiva-" + agents_arr[i] + "-500-5.map";
            string task_file = base + "kiva-" + frequency[j] + ".task";
            string out_file = "/tmp/central_ref_" + agents_arr[i] + "_" + frequency[j] + ".txt";

            fprintf(stderr, "[%d/35] CENTRAL-REF %sag freq%s... ",
                    i*7+j+1, agents_arr[i].c_str(), frequency[j].c_str());
            fflush(stderr);

            clock_t t_start = clock();
            // ShowTask() will freopen stdout to out_file
            // So we fork to isolate the freopen damage
            pid_t pid = fork();
            if (pid == 0) {
                // Child: run simulation
                Simulation simu(map_file, task_file, out_file);
                simu.run(1);
                // ShowTask already wrote to out_file via freopen
                _exit(0);
            }
            int status;
            waitpid(pid, &status, 0);
            double elapsed = (double)(clock() - t_start) / CLOCKS_PER_SEC * 1000.0;

            // Parse output file: first line = num_tasks, then agent_id ag_arrive_start ag_arrive_goal
            ifstream fin(out_file);
            int num_tasks = 0;
            fin >> num_tasks;
            int makespan = 0;
            int swt = 0;

            // We need release times from the task file
            // Parse task file to get release times
            ifstream tfin(task_file);
            int total_tasks_in_file;
            tfin >> total_tasks_in_file;
            vector<int> release_times(total_tasks_in_file);
            for (int k = 0; k < total_tasks_in_file; k++) {
                int rel, p, d, sw, gw;
                tfin >> rel >> p >> d >> sw >> gw;
                release_times[k] = rel;
            }

            for (int k = 0; k < num_tasks; k++) {
                int aid, arr_s, arr_g;
                fin >> aid >> arr_s >> arr_g;
                if (arr_g > makespan) makespan = arr_g;
                if (k < (int)release_times.size())
                    swt += arr_g - release_times[k];
            }

            double avg_service = num_tasks > 0 ? (double)swt / num_tasks : 0;

            fprintf(csv, "CENTRAL-REF,%s,%s,%d,%d,%.1f,%d,%.0f\n",
                   agents_arr[i].c_str(), frequency[j].c_str(),
                   makespan, swt, avg_service, num_tasks, elapsed);
            fflush(csv);
            fprintf(stderr, "makespan=%d swt=%d (%.0fms)\n", makespan, swt, elapsed);
            fflush(stderr);
        }
    }
    fclose(csv);
    return 0;
}
