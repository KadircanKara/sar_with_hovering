import numpy as np
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial import distance
from typing import List, Dict
import itertools
from math import sin, cos, atan2, ceil
from scipy import io
# from scipy.stats import itemfreq
import subprocess
import time
import copy
import matplotlib.pyplot as plt

# from Distance import *
# from Connectivity import *
# from Time import *

# from PathAnimation import PathAnimation

default_input_parameters = {
    'grid_size': 8,
    'A': 50,
    'Nd': 8,
    'V': 2.5, # m/s
    'rc': 2,  # 2 cells
    'min_visits':2,
    'max_visits': 5,  # So that a cell is not visited more than this amount (Incorporate as a constraint)
    'Nt': 1,
    'p': 0.99,
    'q': 0.01,
    'Th': 0.9,
    'max_isolated_time': 0,
}

from PathInfo import *

def split_list(lst, val):
    return [list(group) for k,
    group in
            itertools.groupby(lst, lambda x: x == val) if not k]

class PathSolution():

    def __str__(self):
        info = self.info
        return f"Scenario: Nc_{info.Nc}_A_{info.A}_Nd_{info.Nd}_V_{info.V}_rc_{info.rc}_maxVisits_{info.min_visits}\n" \
               f"Objective Values: totalDistance_{self.total_distance}_longestSubtour_{self.longest_subtour}_percentageConnectivity_{self.percentage_connectivity}\n" \
               f"Chromosome: pathSequence_{self.path}_startPoints_{self.start_points}"

    def __init__(self, path, start_points, info:PathInfo):

        self.hovering = info.hovering
        self.realtime_connectivity = info.realtime_connectivity

        # Inputs
        self.path = path
        self.start_points = start_points
        self.info: PathInfo = info

        # cell - path
        self.drone_dict_generated = False
        self.drone_dict = dict()
        self.interpolated_path_matrix = None # Interpolated cell matrix (discrete, cell by cell)
        self.realtime_path_matrix = None # Interpolated cell matrix (seconds)


        # xy coordinate - path
        self.x_coords_dict = dict()
        self.y_coords_dict = dict()
        self.x_coords_list = [None]*info.Nd
        self.y_coords_list = [None]*info.Nd

        # self.x_matrix
        # self.y_matrix

        # Time
        self.time_slots = None  # Continuous
        self.drone_timeslots = None
        self.time_steps = None  # Discrete
        self.drone_timesteps = None
        self.occ_grid = np.full((self.info.Nn,self.info.Nc), 0.5) # Initialize occupancy grid for each node including BS
        self.cell_visit_steps = None
        self.tbv = None
        self.min_tbv = None
        # Distance
        self.subtour_lengths = None
        self.total_distance = None
        self.longest_subtour = None
        self.long_jump_violations = None
        self.long_jump_violations_constr = None
        self.cells_per_drone_constr = None
        self.max_visits_constr = None
        self.cell_nvisits = None

        self.time_slots = None
        self.time_steps = None

        # Connectivity
        self.connectivity_matrix = None
        self.num_connected_drones_to_base = None
        self.disconnected_time_steps = None
        # self.connectivity_to_base_matrix = None
        self.percentage_connectivity = None
        self.total_disconnected_timesteps = None
        self.max_disconnected_timesteps = None


        # Call PathSolution functions
        self.get_pathplan() # Calculates drone dict and path matrix (not interpolated, directly from the path sequence and start points)
        self.get_connectivity_matrix()
        self.calculate_cell_visit_steps()
        # self.calculate_percentage_connectivity()
        # self.calculate_disconnected_timesteps() # NOTE COMPLETE YET
        # self.calculate_total_distance_and_longest_subtour()
        # self.calculate_distance_constraints()

        # self.calculate_time_between_visits()
        #
        # min_time_between_visits(self)

    def calculate_cell_visit_steps(self):
        info = self.info
        path_matrix = self.interpolated_path_matrix[1:,:].transpose()
        cell_visit_steps = dict()
        for i in range(info.Nc):
            cell_visit_steps[i] = np.where(path_matrix==i)[0] # Steps at which the cell is visited

        self.cell_visit_steps = cell_visit_steps


    # def calculate_time_between_visits(self):
    #
    #     info = self.info
    #     path_matrix = self.interpolated_path_matrix[1:,:].transpose()
    #     cell_nvisits = np.zeros(info.Nc, dtype=int)
    #     cell_visit_steps = dict()
    #     time_between_visits = dict()
    #     # plt.figure()
    #     # plt.xticks( range(info.Nc) )  # Specify the locations of the ticks
    #     # xticklabels = ['A', 'B', 'C', 'D', 'E']
    #     # scatter_plot = plt.scatter([], [])
    #     for i in range(info.Nc):
    #         time_between_visits[i] = []
    #         cell_visit_steps[i] = np.where(path_matrix==i)[0] # Steps at which the cell is visited
    #         cell_nvisits[i] = len(cell_visit_steps[i]) # How many times the cell is visited
    #         # Calculate tbv
    #         for j in range(1,len(cell_visit_steps[i])):
    #             time_between_visits[i].append( cell_visit_steps[i][j] - cell_visit_steps[i][j-1] )
    #
    #         if len(time_between_visits[i])==0:
    #             time_between_visits[i].append(0)
    #             scatter_plot = plt.scatter(i, time_between_visits[i][-1])
    #             # plt.pause(0.01)  # Add a short pause to observe the animation
    #             # plt.draw()
    #     # plt.show()
    #
    #     self.cell_nvisits = cell_nvisits
    #     self.time_between_visits = time_between_visits

        # print(f"cell nvisits:\n{cell_nvisits}")
        # print(f"max: {max(cell_nvisits)}, min: {min(cell_nvisits)}")
        # print(f"cell visit steps:\n{cell_visit_steps}")
        # print(f"time between visits:\n{time_between_visits}")


    # def calculate_disconnected_timesteps(self):
    #
    #         # Finds the maximum disconnected time
    #
    #         info = self.info
    #
    #         time_steps = self.path_matrix.shape[1]
    #
    #         num_connected_drones = np.zeros((info.Nd, time_steps), dtype=int)
    #
    #         drone_total_disconnected_timesteps = np.zeros(info.Nd,dtype=int)
    #
    #         for i in range(info.Nd):
    #             num_connected_drones[i] = connected_nodes(self, i+1)  # To account for skipping the base station # 0,1 , 1,2 ... 7,8
    #             drone_total_disconnected_timesteps[i] = len(np.where(num_connected_drones[i] == 0)[0])
    #
    #         self.disconnected_time_steps = drone_total_disconnected_timesteps
    #
    #
    # def calculate_distance_constraints(self):
    #     limit_long_jumps(self)
    #     limit_cell_per_drone(self)
    #     # limit_max_visits(self)
    #
    #
    # def calculate_total_distance_and_longest_subtour(self):
    #     get_total_distance_and_longest_subtour(self)
    #     # print(f"Drone Distances:\n{self.drone_dists}")
    #     # print(f"Total Distance: {self.total_dist}, Longest Subtour: {self.longest_subtour}")
    #
    # def calculate_percentage_connectivity(self):
    #     self.num_connected_drones_to_base = connected_nodes(self, 0)
    #     info = self.info
    #     self.percentage_connectivity = ( sum(self.num_connected_drones_to_base) / (len(self.num_connected_drones_to_base) * info.Nd) ) * 100
    #     # print(f"percentage connectivity: {self.percentage_connectivity}")
    #
    #
    def get_connectivity_matrix(self):
        # Calculate in realtime
        info = self.info
        num_nodes = info.Nd+1
        realtime_path_matrix = self.realtime_path_matrix
        path_matrix = self.path_matrix
        comm_dist = info.rc * info.A

        if self.realtime_connectivity : # Calculate realtime connectivity
            self.connectivity_matrix = np.zeros((self.time_slots, num_nodes, num_nodes), dtype=int)  # At each timeslot, look at each drone connectivity including BS
            for i in range(self.time_slots):
                pos = realtime_path_matrix[:,i] # x-coords for every node including BS
                for j in range(num_nodes):
                    for k in range(num_nodes):
                        if j != k and info.D[pos[j],pos[k]] <= comm_dist:
                            self.connectivity_matrix[i,j,k] = 1

        else : # Calculate discrete connectivity (step-wise) with hovering
            self.connectivity_matrix = np.zeros((self.time_steps, num_nodes, num_nodes), dtype=int)  # At each timeslot, look at each drone connectivity including BS
            for i in range(self.time_steps):
                pos = path_matrix[:,i]
                for j in range(num_nodes):
                    for k in range(num_nodes):
                        if j != k and info.D[pos[j],pos[k]] <= comm_dist:
                            self.connectivity_matrix[i,j,k] = 1

                # df = pd.DataFrame(self.connectivity_matrix[i,:,:])
                # print(f"Step {i} Connectivity Matrix:\n{df.to_string(index=False)}")


    def get_pathplan(self):
        self.drone_dict_generated = True
        self.drone_dict = dict()
        self.time_steps = 0
        info = self.info

        # GET CELL DICT
        for i in range(info.Nd):
            if i < info.Nd - 1:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:self.start_points[i + 1]], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.Nc)[self.start_points[i]:self.start_points[i + 1]], np.array([-1])))
            else:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.Nc)[self.start_points[i]:], np.array([-1])))

            # Set longest "discrete" subtour
            if len(self.drone_dict[i]) > self.time_steps : self.time_steps = len(self.drone_dict[i]) # Set max subtour length

            # Add BS as a node to drone_dict (key=1)
            self.drone_dict[-1] = np.array([-1] * self.time_steps)

        # print(f"DRONE DICT: {self.drone_dict}")


        # GET CELL MATRIX
        self.path_matrix = np.zeros((info.Nd+1, self.time_steps), dtype=int) - 1 # Nd+1 bc. of BS (index 0)
        for i in range(info.Nd):
            if len(self.drone_dict[i]) == self.time_steps: # If this is the longest discrete tour drone
                self.path_matrix[i+1] = self.drone_dict[i]
            else : # If this is NOT the longest discrete tour drone
                len_diff = self.time_steps - len(self.drone_dict[i])
                filler = np.array([-1]*len_diff)
                self.path_matrix[i+1] = np.hstack( (self.drone_dict[i] , filler)  )

        # print(f"PATH MATRIX:\n{pd.DataFrame(self.path_matrix).to_string(index=False)}")


        # GET XY DICT
        for i in range(1, self.time_steps):
            current_step_cells , next_step_cells = self.path_matrix[1:,i-1].tolist() , self.path_matrix[1:,i].tolist()
            current_step_coords = list(map(self.get_coords, current_step_cells))
            next_step_coords = list(map(self.get_coords, next_step_cells))
            coord_diffs = [next_step_coords[j] - current_step_coords[j] for j in range(info.Nd)]
            thetas = [atan2(j[1],j[0]) for j in coord_diffs]
            current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], self.info.V * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]) for j in range(info.Nd) ]
            current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], self.info.V * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]) for j in range(info.Nd) ]
            for j in range(info.Nd):
                x_coords, y_coords = current_to_next_step_x_coords[j], current_to_next_step_y_coords[j]
                if len(x_coords) != len(y_coords):
                    xy_diff = abs(len(x_coords) - len(y_coords))
                    if len(x_coords) > len(y_coords): # Fill y
                        current_to_next_step_y_coords[j] = np.hstack((current_to_next_step_y_coords[j], np.array([y_coords[-1]]*xy_diff)))
                    else: # Fill x
                        current_to_next_step_x_coords[j] = np.hstack((current_to_next_step_x_coords[j], np.array([x_coords[-1]]*xy_diff)))
                else:
                    continue


            if self.hovering:
                step_lengths = [len(current_to_next_step_x_coords[j]) for j in range(info.Nd)]
                max_step_dist = max(step_lengths)
                longest_step_drone = step_lengths.index(max_step_dist)  # Identify drone with longest step
                current_to_next_step_x_coords = [np.hstack((current_to_next_step_x_coords[j] , np.array([current_to_next_step_x_coords[j][-1]]*(max_step_dist - len(current_to_next_step_x_coords[j]))))) if j != longest_step_drone else current_to_next_step_x_coords[j] for j in range(info.Nd)]
                current_to_next_step_y_coords = [np.hstack((current_to_next_step_y_coords[j] , np.array([current_to_next_step_y_coords[j][-1]]*(max_step_dist - len(current_to_next_step_y_coords[j]))))) if j != longest_step_drone else current_to_next_step_y_coords[j] for j in range(info.Nd)]
            # self.x_coords_dict[j].append(current_to_next_step_x_coords[j]) for j in range(info.Nd)
            self.x_coords_list = [current_to_next_step_x_coords[j] if i==1 else np.hstack((self.x_coords_list[j],current_to_next_step_x_coords[j])) for j in range(info.Nd)]
            self.y_coords_list = [current_to_next_step_y_coords[j] if i==1 else np.hstack((self.y_coords_list[j],current_to_next_step_y_coords[j])) for j in range(info.Nd)]

        self.drone_timeslots = [len(x) for x in self.x_coords_list]
        self.time_slots = max(self.drone_timeslots)

        # Initialize xy matrix
        x_sink,y_sink = self.get_coords(-1)
        self.x_matrix = np.full((info.Nd + 1, self.time_slots), x_sink)  # Nd+1 rows in order to incorporate base station
        self.y_matrix = self.x_matrix.copy()
        self.realtime_path_matrix = self.x_matrix.copy()
        self.realtime_path_matrix.astype(int)
        self.realtime_path_matrix[:, :] = -1
        interpolated_path_dict = dict()
        interpolated_path_max_len = 0

        for i in range(info.Nd):
            self.x_matrix[i + 1] = np.hstack((self.x_coords_list[i], np.array([x_sink] * (self.time_slots - self.drone_timeslots[i]))))
            self.y_matrix[i + 1] = np.hstack((self.y_coords_list[i], np.array([y_sink] * (self.time_slots - self.drone_timeslots[i]))))
            # drone_xy_coords = list(zip(self.x_matrix[j + 1], self.y_matrix[j + 1]))
            self.realtime_path_matrix[i + 1] = [self.get_city(k) for k in list(zip(self.x_matrix[i + 1], self.y_matrix[i + 1]))]
            interpolated_path_dict[i] = []
            for j in range(self.time_slots - 1):
                if self.realtime_path_matrix[i + 1][j] != self.realtime_path_matrix[i + 1][j + 1]:
                    interpolated_path_dict[i].append(self.realtime_path_matrix[i + 1][j])
            # print("----------------------------------------------------------------------------------------------------")
            # print(f"Drone {i+1}")
            # print("----------------------------------------------------------------------------------------------------")
            # print(f"matrix list: {self.realtime_path_matrix[i + 1]}\ndict list: {interpolated_path_dict[i]}")
            if  self.realtime_path_matrix[i + 1][-1] != interpolated_path_dict[i][-1]:
                interpolated_path_dict[i].append(self.realtime_path_matrix[i + 1][-1])
            if len(interpolated_path_dict[i]) > interpolated_path_max_len:
                interpolated_path_max_len = len(interpolated_path_dict[i])

        # Interpolated Path Matrix
        self.interpolated_path_matrix = np.full((info.Nd+1,interpolated_path_max_len), -1)
        for i in range(info.Nd):
            interpolated_drone_path = interpolated_path_dict[i]
            len_diff = abs(len(interpolated_drone_path) - interpolated_path_max_len)
            self.interpolated_path_matrix[i+1] = interpolated_drone_path + [interpolated_drone_path[-1]]*len_diff

        self.realtime_path_matrix = self.realtime_path_matrix.astype(int)


    def get_coords(self, cell):

        if cell == -1:
            x = -self.info.A / 2
            y = -self.info.A / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.A
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.A
        # return [x,y]
        return np.array([x, y])

    def get_city(self, coords):

        if coords[0] < 0 and coords[1] < 0:
            return -1
        else:
            x, y = coords
            return floor(y / self.info.A) * self.info.grid_size + floor(x / self.info.A)



    def get_x_coords(self, cell):

        if cell == -1:
            x = -self.info.A / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.A
        return x

    def get_y_coords(self, cell):

        if cell == -1:
            y = -self.info.A / 2
        else:
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.A
        return y



'''info = PathInfo(Nd=4, grid_size=8, min_visits=1)
# path = np.random.permutation(range(1,info.Nc))
path = np.random.permutation(info.Nc)
start_points = [0,16,32,48]
start_points = [0,12,22,30]
t = time.time()
sol = PathSolution(path, start_points, info)
'''
# print(f"time between visits:\n{sol.time_between_visits}")
# print(f"cell visit steps:\n{sol.cell_nvisits}")
# print("Hovering and realtime connectivity:", time.time()-t)

# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=True, realtime_connectivity=False)
# print("Hovering and discrete connectivity:", time.time()-t)
# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=False, realtime_connectivity=False)
# print("No Hovering and realtime connectivity:", time.time()-t)
# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=False, realtime_connectivity=False)
# print("No Hovering and discrete connectivity:", time.time()-t)


# animation = PathAnimation(sol.x_matrix, sol.y_matrix, sol.info)

# print("path sequence:", path)
# print("start points:", start_points)
# print("drone dict:", sol.drone_dict)
# print("path matrix:", sol.path_matrix)
# print("longest discrete subtour:", sol.longest_discrete_subtour)
# df = pd.DataFrame(sol.interpolated_realtime_path_matrix)
# print(f"intp realtime path matrix:\n{df.to_string(index=False)}")

# df = pd.DataFrame(sol.interpolated_path_matrix)
# print(f"intp path matrix:\n{df.to_string(index=False)}")