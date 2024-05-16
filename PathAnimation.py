import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
from tempfile import TemporaryFile

from PathInfo import PathInfo
from Connectivity import connected_components, dfs
from PathSolution import *
from PathEvaluation import *
# from Distance import *

# info:PathInfo = sol.info
# x_matrix = sol.x_matrix
# y_matrix = sol.y_matrix

# def get_connectivity_matrix_from_xy_coords()

class PathAnimation():

    def __init__(self, sol):
        self.sol = sol

        self.create_figure()
        # self.plot_animation()

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
        drone_colors = ['k', 'g', 'r', 'c', 'm', 'y', 'b', 'w'] * 2
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
        connectivity_matrix = np.zeros((self.sol.info.Nn,self.sol.info.number_of_nodes))
        for node_1 in range(self.sol.info.number_of_nodes):
            for node_2 in range(self.sol.info.number_of_nodes):
                if node_1 != node_2 and self.sol.info.D[self.sol.real_time_path_matrix[node_1,frame],self.sol.real_time_path_matrix[node_2,frame]] <= self.sol.info.comm_cell_range*self.sol.info.cell_side_length :
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

sol = np.load('Results/X/Results/F/g_8_a_50_n_2_v_2.5_r_2_minv_1_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0_SolutionObjects.npy',allow_pickle=True)[0][0]
PathAnimation(sol)
        
def get_coords(sol:PathSolution, cell):

    grid_size = sol.info.grid_size
    A = sol.info.A

    if cell == -1:
        x = -A / 2
        y = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell % grid_size + 0.5) * A
        # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
        y = (cell // grid_size + 0.5) * A
    return np.array([x, y])


'''class PathAnimationTest():

        def __init__(self, colors=None, speed=500, max_comm_dist=2) -> None:

            self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen', 
                            'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                            'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                            'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                            'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                                'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                                'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen',
                                'b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen'] if colors is None else colors
            self.speed = speed

            self.x_values = []
            self.y_values = []
            self.intp_x_values = []
            self.intp_y_values = []
            self.x = []
            self.y = []

            self.sol = None
            self.n_drones = -1
            self.max_comm_dist=max_comm_dist

        def __call__(self, sol : PathSolution, n_drones=-1, speed=500, max_comm_dist=-1, nth_drone=-1, title='Multi UAV Path', subtitle='', show=True, show_prior_lines=True, max_time=100):

            self.sol = sol

            self.show_prior_lines = show_prior_lines

            self.speed = speed

            self.max_comm_dist= sol.info.rc * sol.info.A if max_comm_dist == -1 else max_comm_dist
            self.n_drones = sol.info.Nd + 1 if n_drones == -1 else n_drones + 1

            self.nth_drone = nth_drone

            self.color_index = 0
            self.path_start_points = [0]

            self.fig, self.ax = plt.subplots(figsize=(9, 7))
            
            # Since plotting a single graph
            line,  = self.ax.plot(0, 0) 

            self.ax.set_xlim([-1,self.sol.info.grid_size])
            self.ax.set_ylim([-1,self.sol.info.grid_size])

            # self.get_connectivity_lines()

            self.get_cartesion_drone_path()

            self.intp_x_values = get_real_time_path_interp(self.x_values, max_time=max_time)
            self.intp_y_values = get_real_time_path_interp(self.y_values, max_time=max_time)

            x_intp_df = pd.DataFrame(self.intp_x_values).to_string(index=False)
            y_intp_df = pd.DataFrame(self.intp_y_values).to_string(index=False)

            # print(self.intp_x_values, self.intp_y_values)
            print(x_intp_df, y_intp_df)

            # print(self.x_values, self.y_values)

                # ax.plot(x_values, y_values)
            self.path_start_points = self.path_start_points[0:len(self.path_start_points)-1]

            self.x = []
            self.y = []
            
            # self.plot_prior_path()

            # ani = FuncAnimation(self.fig, self.animate, frames=len(self.intp_x_values[:, 0]), interval=speed, repeat=False)

            self.ax.set_xlabel('X axis')
            self.ax.set_ylabel('Y axis')
            self.ax.set_title(subtitle, fontsize=10)
            plt.suptitle(title, fontsize=15)
            plt.grid(True, color='0.8')

            # if(show):
            #     plt.show()

            # return ani

        def animate(self, i):

            start = np.array([0,0])
            for ind in range(self.n_drones):
                x_val = self.intp_x_values[i, ind]
                y_val = self.intp_y_values[i, ind]

            # print(x_val, y_val, self.n_drones, ind, i, self.intp_x_values[i, :], self.intp_y_values[i, :])
            for ind2 in range(self.n_drones):
                new_x_val = self.intp_x_values[i, ind2]
                new_y_val = self.intp_y_values[i, ind2]

                coord1 = np.array([x_val, y_val])
                coord2 = np.array([new_x_val, new_y_val])
                x_vals = np.array([x_val, new_x_val])
                y_vals = np.array([y_val, new_y_val])

                if np.linalg.norm(coord1-coord2) <= np.sqrt(self.max_comm_dist) and ind != ind2:
                    print(self.connectivity_lines)
                    print(ind, ind2)
                    self.connectivity_lines[ind][ind2].set_xdata(x_vals)
                    self.connectivity_lines[ind][ind2].set_ydata(y_vals)
                    self.connectivity_lines[ind][ind2].set_color('k')
                    self.connectivity_lines[ind][ind2].set_linestyle(':')
                elif ind == ind2 and np.linalg.norm(coord1-start) <= np.sqrt(self.max_comm_dist):
                    x_vals = np.array([x_val, 0])
                    y_vals = np.array([y_val, 0])
                    self.connectivity_lines[ind][ind2].set_xdata(x_vals)
                    self.connectivity_lines[ind][ind2].set_ydata(y_vals)
                    self.connectivity_lines[ind][ind2].set_color('k')
                    self.connectivity_lines[ind][ind2].set_linestyle(':')
                else:
                    self.connectivity_lines[ind][ind2].set_xdata([])
                    self.connectivity_lines[ind][ind2].set_ydata([])

            self.lines[ind].set_xdata(self.intp_x_values[i, ind])
            self.lines[ind].set_ydata(self.intp_y_values[i, ind])
            self.lines[ind].set_color("k")

            #self.ax.clear()
            #self.ax.plot(self.x, self.y)

        def get_cartesion_drone_path(self):
            self.total_len = 0

            real_time_drone_mat = self.sol.real_time_path_matrix

            self.real_time_cartesian_drone_dict = dict()

            time_slot = len(real_time_drone_mat[0])+2

            drone_no = 0

            for drone_path in real_time_drone_mat:

                cartesian_path = [[-1, -1]]
                for city in drone_path:
                    cartesian_path.append(get_coords(self.sol, city))
            
            cartesian_path.append([-1,-1])
            self.real_time_cartesian_drone_dict[drone_no] = cartesian_path

            drone_no += 1

            self.x_values = np.zeros((time_slot, self.sol.info.Nd+1))
            self.y_values = np.zeros((time_slot, self.sol.info.Nd+1))

            for key in self.real_time_cartesian_drone_dict:
                path = self.real_time_cartesian_drone_dict[key]

                self.total_len += len(path)

                self.path_start_points.append(self.total_len)

                for time in range(time_slot):
                    coord = path[time]
                    self.x_values[time, key] = coord[0]
                    self.y_values[time, key] = coord[1]


        def get_connectivity_lines(self):
        
            self.lines = []
            self.prior_lines = []
            self.connectivity_lines = []

            for drone_no in range(self.sol.info.Nd + 1):
                line,  = self.ax.plot(0, 0) 
                prior_line, = self.ax.plot(0, 0)
                if self.nth_drone != -1 and drone_no != self.nth_drone:
                    line.set_visible(False)
                prior_line.set_visible(self.show_prior_lines)

                line.set_linewidth(3)
                line.set_marker("o")

                self.lines.append(line)
                self.prior_lines.append(prior_line)
                drone_connectivity_lines = []
                for drone_no2 in range(self.sol.info.Nd + 1):
                    line2, = self.ax.plot(0,0)

                if self.nth_drone != -1 and drone_no != self.nth_drone:
                    line2.set_visible(False)

                drone_connectivity_lines.append(line2)
                self.connectivity_lines.append(drone_connectivity_lines)
            

        def plot_prior_path(self):
            for ind in range(self.n_drones):
                self.prior_lines[ind].set_xdata(self.intp_x_values[:, ind])
                self.prior_lines[ind].set_ydata(self.intp_y_values[:, ind])
                self.prior_lines[ind].set_color(self.colors[ind])
                self.prior_lines[ind].set_alpha(0.4)



def interp1d(array: np.ndarray, new_len: int) -> np.ndarray:
    la = len(array)
    return np.interp(np.linspace(0, la - 1, num=new_len), np.arange(la), array)

def get_real_time_path_interp(path_matrix, max_time=50):

  real_time_interp_path = np.zeros((max_time, path_matrix.shape[1]))
    
  for ind in range(path_matrix.shape[1]):
    real_time_interp_path[:, ind] = interp1d(path_matrix[:, ind], max_time)

  return real_time_interp_path


def PathSolutionAnimationUnitTest():

    info = PathInfo(min_visits=5)
    path = np.random.permutation(info.min_visits * info.Nc).tolist()
    # Random start points
    start_points = sorted(random.sample([i for i in range(1, len(path))], info.Nd - 1))
    start_points.insert(0, 0)

    sol = PathSolution(path, start_points, info)

    # sol = PathSolution(np.array([i for i in range(info.Nc)]), np.array([0, 10, 20, 30]), info)

    print(sol.real_time_path_matrix % info.Nc)

    anim = PathAnimationTest()
    anim(sol, speed=20, n_drones=4, max_time=500)
    # animation = anim(sol, speed=20, n_drones=4, max_time=500)

    # for i in range(info.Nd):
    #     print(f"DRONE 1 CARTESIAN:\n{anim.real_time_cartesian_drone_dict[i]}")

    # print(anim.real_time_cartesian_drone_dict)

PathSolutionAnimationUnitTest()'''


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