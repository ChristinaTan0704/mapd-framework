#include "TasksLoader.h"
#include <vector>
#include <utility>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <fstream>

// TasksLoader::TasksLoader(const std::map<int, std::tuple<int, int, int>>& current_tasks, vector<int> undelivered_tasks, vector<int> assigned_endpoints){  
//     int idx = 0;
//     for (auto itr = current_tasks.begin();itr != current_tasks.end(); itr++)
//     {
//         std::tuple<int, int, int> task = itr->second;
//         int task_id = itr->first;
//         if (find(undelivered_tasks.begin(), undelivered_tasks.end(), task_id) != undelivered_tasks.end())
//             continue;
//         if (find(assigned_endpoints.begin(), assigned_endpoints.end(), std::get<1>(task))!=assigned_endpoints.end())
//             continue;
//         if (find(assigned_endpoints.begin(), assigned_endpoints.end(), std::get<2>(task))!=assigned_endpoints.end())
//             continue;
//         tasks_all.push_back(Task(task_id, std::get<0>(task), std::get<1>(task), std::get<2>(task), false));
//         tasks_table.insert(std::make_pair(task_id, idx));
//         idx++;
//     }
// }
TasksLoader::TasksLoader(const std::map<int, Task>& current_tasks, vector<int> undelivered_tasks, vector<int> assigned_endpoints)
{
    int idx = 0;
    for (auto itr = current_tasks.begin();itr != current_tasks.end(); itr++)
    {
        Task task = itr->second;
        int task_id = itr->first;
        if (find(undelivered_tasks.begin(), undelivered_tasks.end(), task_id) != undelivered_tasks.end())
            continue;
        int i = 0;
        for (; i < task.goal_arr.size(); i++)
        {
            if (find(assigned_endpoints.begin(), assigned_endpoints.end(), task.goal_arr[i])!=assigned_endpoints.end()) {
                break;
            }
        }
        if (i != task.goal_arr.size())
            continue;
        tasks_all.push_back(task);
        tasks_table.insert(std::make_pair(task_id, idx));
        idx++;
    }
}