#!/usr/bin/env python3
"""Convert COBRA-format task files to MGMAPD task assignment format.

COBRA format:
  Line 1: total_tasks
  Lines 2+: release_time start_loc goal_loc 0 0

MGMAPD format:
  Lines with id <= num_agents start a new agent's task list
  Lines with id > num_agents: id release_time start_loc goal_loc
"""
import sys
import os

def convert(cobra_task_file, num_agents, output_file):
    with open(cobra_task_file) as f:
        lines = f.readlines()

    total_tasks = int(lines[0].strip())
    tasks = []
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        release_time = int(parts[0])
        start_loc = int(parts[1])
        goal_loc = int(parts[2])
        tasks.append((release_time, start_loc, goal_loc))

    tasks.sort(key=lambda t: t[0])

    agent_tasks = [[] for _ in range(num_agents)]
    for i, task in enumerate(tasks):
        agent_tasks[i % num_agents].append(task)

    with open(output_file, 'w') as f:
        for agent_id in range(num_agents):
            f.write(f"{agent_id + 1}\n")
            for task in agent_tasks[agent_id]:
                task_id = num_agents + 1
                f.write(f"{task_id} {task[0]} {task[1]} {task[2]}\n")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <cobra_task_file> <num_agents> <output_file>")
        sys.exit(1)
    convert(sys.argv[1], int(sys.argv[2]), sys.argv[3])
    print(f"Converted {sys.argv[1]} -> {sys.argv[3]} for {sys.argv[2]} agents")
