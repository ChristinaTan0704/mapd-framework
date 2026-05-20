import numpy as np
from re import split
#from spatial.fiducial_graph import make_fid_graph, get_compass_direction
def read_path_plan_timestep(fname):
    t = []
    with open(fname, "r") as f:
        line = f.readline()
        line = split(" ", line[:-1])
        rows = int(line[0])
        cols = int(line[1])
        drives = int(line[2])
        for r in range(drives):
            line = f.readline()
            line = split(",", line[:-1])
            for goal in line:
                t.append(goal)
    return t

def read_map_file(fname):
    with open(fname, "r") as f:
        # f.readline()  # skip "Grid size (rows, cols)"
        line = f.readline()
        line = split(",", line[:-1])
        rows = int(line[0])
        cols = int(line[1])
        f.readline()  # skip the headers
        num_of_drives = f.readline()
        f.readline()  # skip the headers
        my_map = []
        starts = []
        for r in range(rows):
            line = f.readline()
            my_map.append([])
            col = 0
            for cell in line:
                if cell == '@':
                    my_map[-1].append(0) # block cell
                elif cell == '.':
                    my_map[-1].append(1) # free cell
                elif cell == 'e':
                    my_map[-1].append(2) # task endpoints
                elif cell == 'r':
                    my_map[-1].append(3) # non-task endpoints
                    starts.append((r, col))
                col = col + 1
        return my_map, starts


def read_tasks_file(fname):
    goals = []
    realease_times = []
    with open(fname, "r") as f:
        line = f.readline()
        line = split(" ", line[:-1])
        rows = int(line[0])
        cols = int(line[1])
        drives = int(line[2])
        for r in range(drives):
            line = f.readline()
            line = split(";", line[:-1])
            goal_list = []
            time_list = []
            # print(line[-1])
            for goal in line:
                if goal == '':
                    continue
                goal_loc = split(",", goal)
                task_loc = []
                for i, goal_item in enumerate(goal_loc):
                    if i == len(goal_loc)-1:
                        continue
                    if i == "":
                        continue
                    gx = int(int(goal_item) / cols)
                    gy = int(int(goal_item) % cols)
                    task_loc.append((gx, gy))
                    # goal_list.append((gx, gy))
                    time_list.append(goal_loc[-1])
                goal_list.append(task_loc)
            goals.append(goal_list)
            realease_times.append(time_list)
    return goals, realease_times

def read_throughput_file(fname):
    throughput=[]
    throughput.append(0)
    with open(fname, "r") as f:
        line = f.readline()
        line = split(",", line[:-1])
        for l in line:
            if l == "":
                continue
            throughput.append(int(l))
        return throughput

'''def read_paths_file(fname, plot):
    if plot == 0:
        return
    with open(fname, "r") as f:
        drives = f.readline()[:-1]
        cols = 177
        rows = 56
        map = np.zeros(shape=(rows, cols), dtype=int)
        waits = np.zeros(shape=(rows, cols), dtype=int)
        for line in f.readlines():
            data = split(",", line)[:-1]
            for i in range(len(data)):
                loc = data[i]
                x = int(loc) % cols
                y = int(int(loc) / cols)
                map[y][x] += 1
                if i > 0 and data[i - 1] == loc:
                    waits[y][x] += 1
        if plot == 1:
            plt.subplot(2, 1, 1)
            plt.title("Traffic distribution")
            plt.imshow(map, cmap='hot', interpolation='nearest', origin='lower')
            plt.colorbar()
            plt.subplot(2, 1, 2)
            plt.title("Wait actions distribution")
            plt.imshow(waits, cmap='hot', interpolation='nearest', origin='lower')
            plt.colorbar()
            plt.show()
        return'''


def read_paths_file(fname, simulation_time):
    with open(fname, "r") as f:
        line = f.readline()
        line = split(" ", line[:-1])
        rows = int(line[0])
        cols = int(line[1])
        drives = int(line[2])
        paths = []
        # f.readline()  # skip the number of agents
        for line in f.readlines():
            data = split(";", line)[:-1]
            paths.append([])
            for tuple in data:
                tuple = split(",", tuple)
                if int(tuple[2]) > simulation_time:
                    break
                paths[-1].append((int(int(tuple[0])/cols),int(int(tuple[0])%cols)))
                # paths[-1].append({'location': (int(int(tuple[0])/cols),int(int(tuple[0])%cols)), 'timestep': int(tuple[2])})
                # paths[-1].append({'location': int(tuple[0]), 'orientation': int(tuple[1]), 'timestep': int(tuple[2])})
    return paths


def read_solver_file(fname, simulation_time):
    idx_runtime = 0
    idx_nodes = 1
    idx_cost = 5
    idx_bound = 6
    idx_length = 7
    idx_conflicts = 8
    idx_time = 9
    idx_drives = 10
    idx_window = 12

    num_drive = -1
    runtime = []
    node = []
    cost = []
    bound = []
    length = []
    timestep = []
    additional_cost = 0
    subopt = []
    num_failure = 0
    windows = []

    with open(fname + "/solver.csv", "r") as f:
        # f.readline() # skip the header line
        for line in f.readlines():
            data = split(",|\n| ", line[:-1])
            if num_drive == -1:
                num_drive = int(data[idx_drives])
            elif int(data[idx_time]) > simulation_time:
                break
            if float(data[idx_cost]) < 0:
                num_failure += 1
            if timestep != [] and timestep[-1] == int(data[idx_time]):  # the primary solver fails
                runtime[-1] += float(data[idx_runtime])
                node[-1] += int(data[idx_nodes])
            else:
                runtime.append(float(data[idx_runtime]))
                node.append(int(data[idx_nodes]))
                timestep.append(int(data[idx_time]))
                if len(data) > idx_window and data[idx_window].isnumeric():
                    windows.append(int(data[idx_window]))
            if float(data[idx_cost]) > 0:
                cost.append(float(data[idx_cost]))
                bound.append(float(data[idx_bound]))
                length.append(float(data[idx_length]))
                additional_cost += cost[-1] - bound[-1]
                subopt.append((cost[-1] - bound[-1]) * 100.0 / bound[-1])

        return num_drive, runtime, node, timestep, 1 - num_failure / len(timestep), additional_cost, subopt, cost, bound, length, windows


def read_results_file(fname):
    idx_runtime = 0
    idx_nodes = 1
    idx_cost = 5
    idx_bound = 7
    idx_drives = 9

    num_drives = []
    runtime = []
    nodes = []
    cost = []
    bound = []
    with open(fname, "r") as f:
        # f.readline() # skip the header line
        for line in f.readlines():
            data = split(",|\n| ", line)
            if len(num_drives) == 0 or int(data[idx_drives]) != num_drives[-1]:
                if len(num_drives) != 0:
                    runtime[-1] /= count
                    nodes[-1] /= count
                    bound[-1] /= count # (cost[-1] / bound[-1] - 1) * 100
                    cost[-1] /= count # * num_drives[-1]
                num_drives.append(int(data[idx_drives]))
                runtime.append(float(data[idx_runtime]))
                nodes.append(int(data[idx_nodes]))
                cost.append(float(data[idx_cost]))
                bound.append(float(data[idx_bound]))
                count = 1
            # elif int(data[idx_cost]) < 0:
            #     continue;
            else:
                runtime[-1] += float(data[idx_runtime])
                nodes[-1] += int(data[idx_nodes])
                cost[-1] += float(data[idx_cost])
                bound[-1] += float(data[idx_bound])
                count += 1
        runtime[-1] /= count
        nodes[-1] /= count
        bound[-1] /= count # (cost[-1] / bound[-1] - 1) * 100
        cost[-1] /= count # * num_drives[-1]

        print("#drives  runtime  suboptimal      nodes   sum-of-cost delta-cost path-cost")
        for i in range(len(num_drives)):
            print("{:4}     {:5.1f}     {:6.03f}        {:5.0f}     {:7.0f}     {:4.0f}         {:3.0f}\n".format( \
                num_drives[i], runtime[i],  (cost[i] - bound[i]) * 100 / bound[i], \
                nodes[i], cost[i], cost[i] - bound[i], cost[i] / num_drives[i]))