#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

Colors = ['green', 'blue', 'orange', 'red', 'yellow', 'purple', 'brown', 'pink', 'grey', 'cyan'
, 'lime', 'tomato', 'salmon', 'indigo', 'olive', 'wheat', 'silver', 'sienna', 'violet', 'magenta','teal']

class Animation:
    def __init__(self, my_map, starts, goals, realease_times, paths, throughput):
        self.my_map = np.flip(np.transpose(my_map), 1)
        self.starts = []
        for start in starts:
            self.starts.append((start[1], len(self.my_map[0]) - 1 - start[0]))
        
        self.goals = []
        self.realease_times = []
        self.throughput = throughput
        # self.plan_path_timestep = []
        
        # for t in plan_timestep:
        #     self.plan_path_timestep.append(t)
       
        # for goal_list in goals:
        #     ordered_goal = []
        #     for goal in goal_list:
        #         ordered_goal.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
        #     self.goals.append(ordered_goal)
        
        for time_list in realease_times:
            time = []
            for time_item in time_list:
                time.append(time_item)
            self.realease_times.append(time)

        # print(len(goals))   
        for goal_list in goals:
            task_goal = []
            # print(len(goal_list))   
            for task in goal_list:
                ordered_goal = []
                for goal in task:
                    ordered_goal.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
                task_goal.append(ordered_goal)
            self.goals.append(task_goal)

        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))
    

        aspect = len(self.my_map) / len(self.my_map[0])

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        self.goal_names = dict()
        # create boundary patch

        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j] == 0:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))
                elif self.my_map[i][j] == 1:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='none', edgecolor='gray'))
                elif self.my_map[i][j] == 2:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='lightblue', edgecolor='gray'))
                elif self.my_map[i][j] == 3:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gold', edgecolor='gray'))

        # create agents:
        self.T = 0
        
        # self.paths = []
        # self.paths_val = []
        # if paths:
        #     for i, path in enumerate(paths):
        #         self.paths_val.append([])
        #         curr_path = []
        #         self.paths.append(dict())
        #         for j, item in enumerate(path):
        #             loc = item['location']
        #             timestep = item['timestep']
        #             self.paths_val[-1].append({'location': (loc[1], len(self.my_map[0]) - 1 - loc[0]), 'timestep': timestep})
                    # self.paths[-1][j] = Rectangle((loc[1] - 0.25, len(self.my_map[0])- 1 - loc[0] - 0.25), 0.15, 0.15, facecolor=Colors[i % len(Colors)],
                    #                 edgecolor='none', alpha=1)
                    # self.paths[-1][j] = Circle((loc[1], len(self.my_map[0])- 1 - loc[0]), 0.1, facecolor=Colors[i % len(Colors)],
                                    # alpha = 1)
                    # self.paths[-1][j].original_face_color = Colors[i % len(Colors)]
                    # self.patches.append(self.paths[-1][j])  

        # self.mygoals = []
        # self.mygoals_val = []
        # draw goals first
        # for i, goal_list in enumerate(self.goals):
            # for l, task in enumerate(goal_list):
            # self.mygoals_val.append(dict())
            # self.mygoals.append(dict())
            # for j, goal in enumerate(goal_list):
            #     for p, path in enumerate(self.paths_val[i]):
            #         if (path['location'] == (goal[0], goal[1])):
            #             self.mygoals_val[-1][j] = path['timestep']
            #             self.mygoals[-1][j] = Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],
            #                             edgecolor='black', alpha=0)
            #             # self.mygoals[-1][j] = Circle((goal[0] - 0.25, goal[1] - 0.25), 0.5,facecolor=Colors[i % len(Colors)],
            #             #                 edgecolor='black', alpha=0.5)
            #             self.patches.append(self.mygoals[-1][j])
            #             break
                # g_name = str(j)
                # self.goal_names[j] = self.ax.text(goal[0], goal[1]+0.25, g_name)
                # self.goal_names[j].set_horizontalalignment('center')
                # self.goal_names[j].set_verticalalignment('center')
                # self.artists.append(self.goal_names[j]) 
                
        for i in range(len(self.paths)):
        # for i in range(len(self.paths_val)):
            name = str(i)
            self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                    edgecolor='black')
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(paths[i]) - 1)
            # self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.25, name)
            # self.agent_names[i].set_horizontalalignment('center')
            # self.agent_names[i].set_verticalalignment('center')
            # self.artists.append(self.agent_names[i])
    
        h = 0
        for i, goal_list in enumerate(self.goals):
            # print(len(goal_list))
            for l, task in enumerate(goal_list):
                for j, goal in enumerate(task):
                    self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[h],
                                          edgecolor='black', alpha=0.5))
                h = h +1
                # print(h)
                    # g_name = str(j)

                    # self.goal_names[j] = self.ax.text(goal[0], goal[1]+0.25, g_name)
                    # self.goal_names[j].set_horizontalalignment('center')
                    # self.goal_names[j].set_verticalalignment('center')
                    # self.artists.append(self.goal_names[j])

        ax = plt.gca()
        # ax.title.set_text('LNS-PBS', fontsize=15)
        plt.title('LNS-wPBS', fontdict = {'fontsize' : 22}, x=0.5, y=1.1)
        self.text = ax.text(0, 1.01, '', transform=ax.transAxes, fontsize=14)
        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1)*10,
                                                #  frames=int(self.T + 1) * 5,
                                                 interval=100,
                                                 blit=False, repeat=True)

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        # for i, goal_list in enumerate(self.goals):
        #     for j, item in enumerate(goal_list):
        #         if (int(t/10) >= int(self.realease_times[i][j])):
        #             self.mygoals[i][j].set_alpha(0.5)
        # for i, path in enumerate(self.mygoals_val):
        #     # for j, item in enumerate(path):
        #     for key, values in path.items():
        #         # print(int(t/10),  values)
        #         if int(t/10) >= values:
        #             self.mygoals[i][int(key)].set_alpha(0)
        # if int(t/10) == 0:
        #     for i, path in enumerate(self.paths_val):
        #         for j, item in enumerate(path):
        #             loc = item['location']
        #             if i == 0:
        #                 if j <= 2:
        #                     self.paths[i][j].center = (loc[0], loc[1])
        #             if i == 1:
        #                 if j <= 5:
        #                     self.paths[i][j].center = (loc[0], loc[1])

        # if int(t/10) == 0:
        #     for i, path in enumerate(self.paths_val):
        #         for j, item in enumerate(path):
        #             loc = item['location']
        #             if i == 0:
        #                 if j > 2:
        #                     self.paths[i][j].center = (-1,-1)
        #             if i == 1:
        #                 if j > 5:
        #                     self.paths[i][j].center = (-1,-1)
        # if int(t/10) == 2:
        #     for i, path in enumerate(self.paths_val):
        #         for j, item in enumerate(path):
        #             loc = item['location']
        #             if i == 0:
        #                 if j > 2:
        #                     self.paths[i][j].center = (loc[0], loc[1])
        # if int(t/10) == 5:
        #     for i, path in enumerate(self.paths_val):
        #         for j, item in enumerate(path):
        #             loc = item['location']
        #             if i == 1:
        #                 if j > 5:
        #                     self.paths[i][j].center = (loc[0], loc[1])
        # # if int(t/10) == 0:
        # #     for i, path in enumerate(self.paths_val):
        # #         for j, item in enumerate(path):
        # #             loc = item['location']
        # #             if i == 0:
        # #                 if j > 2:
        # #                     self.paths[i][j].center = (loc[0], loc[1])
        # #             if i == 1:
        # #                 if j > 5:
        # #                     self.paths[i][j].center = (loc[0], loc[1])

        for k in range(len(self.paths)):
        # for k in range(len(self.paths_val)):
            pos = self.get_state(t/10, self.paths[k])
            # pos = self.get_state(t/10, self.paths_val[k])
            self.agents[k].center = (pos[0], pos[1])
            # self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        self.text.set_text("Timestep {}: {} tasks finished.".format(t, self.throughput[int(t/10)]))
        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)
        

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        # if int(t) <= 0:
        #     return np.array(path[0])
        if int(t) == 0:
            # return np.array(path[0])
            pos_last = np.array(path[0])
            pos_next = np.array(path[1])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            # pos_last = np.array(path[int(t) - 1])
            # pos_next = np.array(path[int(t)])
            pos_last = np.array(path[int(t)])
            if int(t) == len(path)-1:
                pos_next = pos_last
            else:
                pos_next = np.array(path[int(t)+1])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos
    # def get_state(t, path):
    #     # if int(t) <= 0:
    #     #     return np.array(path[0])
    #     if int(t) == 0:
    #         # return np.array(path[0])
    #         pos_last = np.array(path[0]['location'])
    #         pos_next = np.array(path[1]['location'])
    #         pos = (pos_next - pos_last) * (t - int(t)) + pos_last
    #         return pos
    #     elif int(t) >= len(path):
    #         return np.array(path[-1]['location'])
    #     else:
    #         # pos_last = np.array(path[int(t) - 1])
    #         # pos_next = np.array(path[int(t)])
    #         pos_last = np.array(path[int(t)]['location'])
    #         if int(t) == len(path)-1:
    #             pos_next = pos_last
    #         else:
    #             pos_next = np.array(path[int(t)+1]['location'])
    #         pos = (pos_next - pos_last) * (t - int(t)) + pos_last
    #         return pos
