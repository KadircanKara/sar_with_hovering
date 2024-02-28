import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from Connectivity import dfs, connected_components, connected_nodes

class PathAnimation():

    def __init__(self, sol, path_tracing=False, connectivity=True):

        self.sol = sol
        print("-->", self.sol.path_matrix.shape[1])


        self.path_tracing = path_tracing
        self.connectivity = connectivity

        self.x_matrix = np.round(sol.x_matrix)
        self.y_matrix = np.round(sol.y_matrix)
        self.info = sol.info

        self.fig, self.axis = plt.subplots()
        self.axis = plt.axes( xlim=(-self.info.A, (self.info.grid_size+1)*self.info.A), ylim=(-self.info.A, (self.info.grid_size+1)*self.info.A) )
        self.axis.set_xticks(range(0, (self.info.grid_size+1)*self.info.A+1, self.info.A))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
        self.axis.set_yticks(range(0, (self.info.grid_size+1)*self.info.A+1, self.info.A))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
        plt.grid(linestyle='--')

        self.drone_colors = ['k', 'g', 'r', 'c', 'm', 'y', 'b']*3
        # if self.info.Nd > 7:
        #     self.drone_colors += self.drone_colors
        self.scatter_plots = [self.axis.scatter([], [], color=self.drone_colors[i], label=f'Drone {i}') for i in range(self.info.Nd+1)]
        if self.path_tracing:
            self.lines = [plt.plot([], color=self.drone_colors[i], label=f"Drone {i}") for i in range(self.info.Nd+1)]
        if self.connectivity:
            self.connectivity_lines = [plt.plot([], color='k', label=f"Drone {i}") for i in range(self.info.Nd + 1)]

        # self.create_figure()
        self.plot_animation()

    def create_figure(self):

        info = self.info

        fig, axis = plt.subplots()
        self.fig = fig

        self.scatter_plots = [axis.scatter([], [], color=self.drone_colors[i], label=f'Drone {i}') for i in range(info.Nd+1)]
        if self.path_tracing:
            self.lines = [plt.plot([], color=self.drone_colors[i], label=f"Drone {i}") for i in range(info.Nd+1)]
        if self.connectivity:
            self.connectivity_lines = [plt.plot([], color=self.drone_colors[i], label=f"Drone {i}") for i in range(info.Nd+1)]


        axis = plt.axes( xlim=(-info.A, (info.grid_size+1)*info.A), ylim=(-info.A, (info.grid_size+1)*info.A) )
        axis.set_xticks(range(0, (info.grid_size+1)*info.A+1, info.A))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
        axis.set_yticks(range(0, (info.grid_size+1)*info.A+1, info.A))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
        plt.grid(linestyle='--')

    def animate(self,frame):

        info=self.info

        if frame < self.sol.path_matrix.shape[1]:
            connectivity_matrix = self.sol.connectivity_matrix[frame]
            print(f"Step {frame} connected components:\n{connected_components(connectivity_matrix)}")

        for i in range(info.Nd+1):
            x_scatter = self.x_matrix[i, frame + 1]
            y_scatter = self.y_matrix[i, frame + 1]
            self.scatter_plots[i].set_offsets(np.column_stack([x_scatter, y_scatter]))
            x_line = self.x_matrix[i, :frame + 1]
            y_line = self.y_matrix[i, :frame + 1]
            # print("-->",lines[i])
            if self.path_tracing:
                self.lines[i][0].set_data((x_line, y_line))


    def plot_animation(self):
        print(self.fig, self.animate, self.x_matrix.shape)
        anim = FuncAnimation(self.fig, self.animate, frames=self.x_matrix.shape[1], interval=100, blit=False)
        plt.show()

'''
    COLORS
--------------
'b' or 'blue'
'g' or 'green'
'r' or 'red'
'c' or 'cyan'
'm' or 'magenta'
'y' or 'yellow'
'k' or 'black'
'w' or 'white'
'''