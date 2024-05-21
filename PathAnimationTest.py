import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
from PathSolution import *
from Distance import get_total_distance
from Distance import *
from Connectivity import *

class PathAnimation:
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

        self.max_comm_dist= (sol.info.rc * sol.info.A)**2 if max_comm_dist == -1 else max_comm_dist
        self.n_drones = sol.info.Nd + 1 if n_drones == -1 else n_drones + 1

        self.nth_drone = nth_drone

        self.color_index = 0
        self.path_start_points = [0]

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        
        # Since plotting a single graph
        line,  = self.ax.plot(0, 0) 

        self.ax.set_xlim([-1,self.sol.info.grid_dims[0]])
        self.ax.set_ylim([-1,self.sol.info.grid_dims[1]])

        self.get_connectivity_lines()

        self.get_cartesion_drone_path()

        self.intp_x_values = get_real_time_path_interp(self.x_values, max_time=max_time)
        self.intp_y_values = get_real_time_path_interp(self.y_values, max_time=max_time)

        # print(self.intp_x_values, self.intp_y_values)

        # print(self.x_values, self.y_values)

            # ax.plot(x_values, y_values)
        self.path_start_points = self.path_start_points[0:len(self.path_start_points)-1]

        self.x = []
        self.y = []
        
        self.plot_prior_path()

        ani = FuncAnimation(self.fig, self.animate, frames=len(self.intp_x_values[:, 0]), interval=speed, repeat=False)

        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_title(subtitle, fontsize=10)
        plt.suptitle(title, fontsize=15)
        plt.grid(True, color='0.8')

        if(show):
          plt.show()

        return ani

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

        self.x_values = np.zeros((time_slot, self.sol.info.number_of_drones+1))
        self.y_values = np.zeros((time_slot, self.sol.info.number_of_drones+1))

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

        for drone_no in range(self.sol.info.number_of_drones + 1):
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
            for drone_no2 in range(self.sol.info.number_of_drones + 1):
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

