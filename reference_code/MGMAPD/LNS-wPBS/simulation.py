from visualize import Animation
import matplotlib.pyplot as plt
from matplotlib import animation
from read_file import read_tasks_file, read_paths_file, read_map_file, read_throughput_file

if __name__ == "__main__":
    map_fname = "./maps/Instances/small/kiva-1-2-5.map"
    paths_name = "../exp/test\paths.txt"
    tasks_name = "../exp/test\goals.txt"
    throughput_name = "../exp/test\-tasks.txt"
    my_map, starts = read_map_file(map_fname)
    goals = read_tasks_file(tasks_name)
    paths = read_paths_file(paths_name, 3000)
    throughput = read_throughput_file(throughput_name)
        
    animation = Animation(my_map, starts, goals, paths, throughput)
    # animation.save("output.gif", 1.0)

    # writergif = animation.PillowWriter(fps=30)
    # animation.save('10-20-offline.gif',writer=writergif)
    animation.show()
    # plt.show()