import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
from tempfile import TemporaryFile

from PathInfo import PathInfo
from Connectivity import connected_components, dfs

# info:PathInfo = sol.info
# x_matrix = sol.x_matrix
# y_matrix = sol.y_matrix

# def get_connectivity_matrix_from_xy_coords()

class PathAnimation():

    def __init__(self, sol):
        self.sol = sol

        self.create_figure()
        self.plot_animation()

    def create_figure(self):
        # Initialize figure
        fig, axis = plt.subplots()
        self.fig, self.axis = fig, axis
        # Set ticks and labels
        x_ticks_values = [i for i in range(-self.sol.info.A,(self.sol.info.grid_size+1)*self.sol.info.A,self.sol.info.A)]
        y_tick_values = x_ticks_values.copy()
        x_ticks_labels = [i for i in range(-1,self.sol.info.grid_size+1)]
        # print(len(x_ticks_labels), len(x_ticks_values))
        y_tick_labels = x_ticks_labels.copy()
        plt.xticks(x_ticks_values,x_ticks_labels)
        plt.yticks(y_tick_values,y_tick_labels)
        plt.grid(linestyle='--')
        drone_colors = ['k', 'g', 'r', 'c', 'm', 'y', 'b'] * 3
        self.drone_animation = [axis.scatter([], [], color=drone_colors[i], label=f'Drone {i}') for i in range(self.sol.info.Nn)]
        self.connectivity_lines = [plt.plot([], color='k', label=f"Drone {i}") for i in range(self.sol.info.Nn)]

    def animate(self, frame):
        # Draw drone path animation
        for i in range(self.sol.info.Nd + 1):
            # Plot drone animation
            x_scatter = self.sol.x_matrix[i, frame + 1]
            y_scatter = self.sol.y_matrix[i, frame + 1]
            self.drone_animation[i].set_offsets(np.column_stack([x_scatter, y_scatter]))
        # Draw connectivity lines for each frame
        # Get connectivity matrix for frame
        connectivity_matrix = np.zeros((self.sol.info.Nn,self.sol.info.Nn))
        for node_1 in range(self.sol.info.Nn):
            for node_2 in range(self.sol.info.Nn):
                if node_1 != node_2 and self.sol.info.D[self.sol.realtime_path_matrix[node_1,frame],self.sol.realtime_path_matrix[node_2,frame]] <= self.sol.info.rc*self.sol.info.A :
                    connectivity_matrix[node_1,node_2] = 1
        # Get connected components for frame
        conn_comp = connected_components( connectivity_matrix )
        # print( "-->", sum(sum([np.arange(len(comp)) for comp in conn_comp])) )
        num_lines = sum(sum([np.arange(len(comp)) for comp in conn_comp]))
        connectivity_lines = [plt.plot([], linestyle="--" ,color='k', label=f"Line {i}") for i in range(num_lines)]
        line_num = 0
        for comp in conn_comp :
            # [0,1,2,3,4,5] : 5+4+3+2+1 # sum([range(len)])
            # comp example: [0,1,3}: 0., 1., 3. nodes are connected
            # Lines: (0,1), (0,3), (1,3)
            for node_1 in range(len(comp)):
                node_1_x, node_1_y = self.sol.x_matrix[node_1, frame], self.sol.y_matrix[node_1, frame]
                for node_2 in range(node_1+1,len(comp)):
                    node_2_x, node_2_y = self.sol.x_matrix[node_2, frame], self.sol.y_matrix[node_2, frame]
                    print(f"x coords: {[node_1_x,node_2_x]} y coords: {[node_1_y,node_2_y]}")
                    connectivity_lines[line_num][0].set_data(([node_1_x,node_2_x],[node_1_y,node_2_y]))
                    # plt.plot([node_1_x,node_2_x],[node_1_y,node_2_y], linestyle='--', color='gray')
                    line_num += 1
        # Delete connectivity lines
        connectivity_lines[line_num][0].remove()

        # x_line = self.sol.x_matrix[i, :frame + 1]
            # y_line = self.sol.y_matrix[i, :frame + 1]
            # print("-->",lines[i])

            # if self.path_tracing:
            #     self.lines[i][0].set_data((x_line, y_line))

    def plot_animation(self): # fig, animate_func, num_frames
        # print(self.fig, self.animate, self.x_matrix.shape)
        anim = FuncAnimation(self.fig, self.animate, frames=self.sol.x_matrix.shape[1], interval=100, blit=False)
        plt.show()

sol = np.load('Results/X/alg_NSGA2_hovering_True_realtimeConnectivityCalculation_False_n_64_Ns_4_comm_4_nvisits_1_SolutionObjects.npy',allow_pickle=True)[0][0]
PathAnimation(sol)


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


# axis = fig.axes


# plt.scatter([50,100,150],[50,50,50])
# plt.show()
'''# Sample data for two drones
x_matrix = np.load('x_matrix.npy', allow_pickle=True)
y_matrix = np.load('y_matrix.npy', allow_pickle=True)


fig, axis = plt.subplots()

axis = plt.axes(xlim=(-50, 450), ylim=(-50, 450))
axis.set_xticks(range(0, 451, 50))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
axis.set_yticks(range(0, 451, 50))  # Set x-axis ticks at intervals of 2 (0, 2, 4, 6, 8, 10)
plt.grid(linestyle='--')

# np.random.rand(3,)
drone_colors = ['k','g','r','c','m','y','b']
scatter_plots = [axis.scatter([], [], color=drone_colors[i], label=f'Drone {i}') for i in range(5)]
lines = [plt.plot([], color=drone_colors[i], label=f"Drone {i}") for i in range(5)]
# lines = [axis.line([], [], color=drone_colors[i], label=f'Drone {i}') for i in range(5)]

def animate(frame):

    for i in range(5):
        x_scatter = x_matrix[i, frame+1]
        y_scatter = y_matrix[i, frame+1]
        scatter_plots[i].set_offsets(np.column_stack([x_scatter, y_scatter]))
        x_line = x_matrix[i, :frame+1]
        y_line = y_matrix[i, :frame+1]
        # print("-->",lines[i])
        lines[i][0].set_data((x_line, y_line))

anim = FuncAnimation(fig, animate, frames=x_matrix.shape[1], interval=100, blit=False)
plt.show()'''