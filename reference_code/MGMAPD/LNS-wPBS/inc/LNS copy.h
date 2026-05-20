/*
 * This code is modified from https://github.com/Jiaoyang-Li/Flatland
*/

#pragma once

#include "States.h"
#include "TasksLoader.h"
#include "AgentsLoader.h"
#include <chrono>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <random>
#include <ctime>
#include <cstdlib>
using std::vector;
using std::cout;
using std::endl;
using namespace std::chrono;
using std::string;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

class LNS
{
public:
    int flowtime = 0;
    int makespan = 0;
    int initial_flowtime = 0;
    int initial_makespan = 0;
    int runtime = 0;
    float initial_runtime = 0;
    vector<int> neighbors;

    LNS(TasksLoader& tl, AgentsLoader& al, int insertion_strategy, int removal_strategy,  int lns_insertion_strategy, int neighborhood_size):
            tl(tl), insertion_strategy(insertion_strategy), 
            removal_strategy(removal_strategy),
            lns_insertion_strategy(lns_insertion_strategy),
            neighborhood_size(neighborhood_size) {}
    
    bool run(int time_limit);
    bool getInitialSolution();

private:
    high_resolution_clock::time_point start_time;
    string outfile;
    string outStatFile;
    TasksLoader& tl;
    AgentsLoader& al;

    int num_of_agents;
    int num_of_tasks;
    int removal_strategy = 0; // 0: random; 1: shaw; 2: worst
    int insertion_strategy = 0; // 0: random; 1: basic_greedy; 2: regret
    int lns_insertion_strategy = 0; // 0: random; 1: shaw; 2: worst
    int neighborhood_size = 0;
    int updated_agent = 0;
    int removed_task = 0;
    float relatedness_weight1 = 9;
    float relatedness_weight2 = 3;
    int p = 6;
    float gamma = 0.01;

    double shaw_removal_weight = 1;
    double worst_removal_weight = 1;
    double random_removal_weight = 1;
    double random_insert_weight = 1;
    double greedy_insert_weight = 1;
    double regret_insert_weight = 1;
    
    std::unordered_map<int, vector<int>> best_task_sequence;
    std::unordered_map<int, vector<int>> curr_task_sequence;
    std::map<Key, TaskAssignment*>::iterator iter;

    void initializeAssignmentHeap();
    void sortNeighborsByStrategy(int lns_insertion_strategy);
    void addTaskAssignment();
    void updateAssignmentHeap();
    bool loadTasks(const vector<tuple<int, int, int>>& current_tasks);

    void generateNeighborsByShawRemoval();
    void generateNeighborsByWorstRemoval();
    void updateTaskSequenceAfterRemoval(vector<int> neighbors);

    // tool;
    int calculateMakespan(Agent agent, vector<int> task_sequence);
    int calculateFlowtime(Agent agent, vector<int> task_sequence);
    int calculateRegretValue(Task task, vector<int> task_sequence, int pos, Agent Agent);
    int calculateManhattanDistance(int loc1, int loc2);
    void quickSort(vector<int>& task_order, int low, int high, bool insert, int insertion_strategy, int removal_strategy);
    int randomfunc(int j) {return rand() % j;};
    int getFlowtime();
    int getMakespan();
};