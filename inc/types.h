#pragma once
#include <vector>
#include <list>
#include <deque>
#include <string>
#include <climits>
#include <iostream>

using namespace std;

// Forward declarations
struct Task;
struct Agent;

// ======================== Task ========================
struct Task {
    int id;
    int pickup;        // endpoint index for pickup
    int delivery;      // endpoint index for delivery
    int pickup_loc;    // grid location of pickup
    int delivery_loc;  // grid location of delivery
    int release_time;
    int status;        // -1=unassigned, agent_id=assigned, INT_MAX=finished
    int completion_time;
    int ag_arrive_start; // assigned agent's arrival at first ordered goal
    int start_wait_time; // min time at first ordered goal
    int goal_wait_time;  // min time at final ordered goal

    // Multi-goal: complete ordered sequence of grid locations to visit.
    // For standard MAPD: {pickup_loc, delivery_loc}
    // For MG-MAPD: {goal1_loc, goal2_loc, ..., goalN_loc}
    vector<int> goals;

    // TA-Hybrid specific fields
    int seq_id;         // which sequence this task belongs to
    int hold_time;      // hold time for cost flow (delay at pickup to avoid conflicts)
    int ag_arrive_goal; // timestep when agent arrives at delivery goal
    bool delivering;    // true if agent is currently delivering this task
    Agent* ag;          // pointer to agent delivering this task (TA-Hybrid)

    Task() : id(-1), pickup(-1), delivery(-1), pickup_loc(-1), delivery_loc(-1),
             release_time(0), status(-1), completion_time(-1), ag_arrive_start(-1),
             start_wait_time(0), goal_wait_time(0),
             seq_id(-1), hold_time(0), ag_arrive_goal(-1), delivering(false), ag(nullptr) {}
    Task(int id, int pickup, int delivery, int ploc, int dloc, int release, int sw=0, int gw=0)
        : id(id), pickup(pickup), delivery(delivery), pickup_loc(ploc), delivery_loc(dloc),
          release_time(release), status(-1), completion_time(-1), ag_arrive_start(-1),
          start_wait_time(sw), goal_wait_time(gw),
          seq_id(-1), hold_time(0), ag_arrive_goal(-1), delivering(false), ag(nullptr)
    { goals = {ploc, dloc}; }
};

// ======================== Agent ========================
enum AgentStatus { AG_FREE = 0, AG_MOVING_TO_PICKUP = 1, AG_CARRYING = 2 };

struct Agent {
    int id;
    int loc;           // current location
    int initial_loc;   // starting location (non-task endpoint)
    AgentStatus status;
    int current_task;  // task id or -1
    // Index of the next goal to visit in current_task. This must persist across
    // replans so an MG-MAPD task can resume after intermediate goals.
    int current_goal_index;
    unsigned int finish_time; // timestep when current path ends
    vector<unsigned int> path; // path[timestep] = location
    // Current assigned endpoint location for one-endpoint planning methods
    // such as CENTRAL; initialized to the agent's home location.
    int last_endpoint;
    deque<int> task_sequence; // ordered assigned-task queue

    // TA-Hybrid specific fields
    int dummy_start_step;      // timestep where dummy path begins
    int park_loc;              // parking location (= initial_loc for non-task endpoints)
    bool delivering;           // true if currently delivering a task
    Task* task_ptr;            // pointer to current task (for TA-Hybrid)
    int goal_loc;              // goal location for current task
    int release_time_agent;    // release time for the agent's current target
    vector<int> non_dummy_path; // path without dummy (for constraint in cost flow)

    Agent() : id(-1), loc(-1), initial_loc(-1), status(AG_FREE),
              current_task(-1), current_goal_index(0), finish_time(0), last_endpoint(-1),
              dummy_start_step(0), park_loc(-1), delivering(false),
              task_ptr(nullptr), goal_loc(-1), release_time_agent(0) {}

    void init(int _id, int _loc, int _col, int _row, unsigned int _maxtime) {
        id = _id;
        loc = _loc;
        initial_loc = _loc;
        status = AG_FREE;
        current_task = -1;
        current_goal_index = 0;
        finish_time = 0;
        last_endpoint = _loc;
        dummy_start_step = 0;
        park_loc = _loc;
        delivering = false;
        task_ptr = nullptr;
        goal_loc = -1;
        release_time_agent = 0;
        path.resize(_maxtime);
        for (unsigned int i = 0; i < _maxtime; i++) path[i] = _loc;
    }
};

// ======================== Endpoint ========================
struct Endpoint {
    int loc;
    int id;
    bool is_task_endpoint; // true = task endpoint (e), false = non-task endpoint (r)
    vector<int> h_val;     // h_val[cell_loc] = BFS distance from cell to this endpoint

    Endpoint() : loc(-1), id(-1), is_task_endpoint(false) {}
};

// ======================== Metrics ========================
struct Metrics {
    int makespan;
    double avg_service_time;
    double sum_task_wait;
    int tasks_completed;
    int total_tasks;
    double computation_time_ms;
};
