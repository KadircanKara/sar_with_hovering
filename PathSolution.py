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
import matplotlib.pyplot as plt # 1.20.3

# from distance import *
# from Conumber_of_nodesectivity import *
# from Time import *

# from PathAnimation import PathAnimation

default_input_parameters = {
    'grid_size': 8,
    'A': 50,
    'number_of_drones': 8,
    'V': 2.5, # m/s
    'rc': 2,  # 2 cells
    'min_visits':2,
    'max_visits': 5,  # So that a cell is not visited more than this amount (Inumber_of_cellsorporate as a constraint)
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
        return f"Scenario: number_of_cells_{info.number_of_cells}_A_{info.A}_number_of_drones_{info.number_of_drones}_V_{info.V}_rc_{info.rc}_maxVisits_{info.min_visits}\n" \
               f"Objective Values: totaldistance_{self.total_distance}_longestSubtour_{self.longest_subtour}_percentageConumber_of_nodesectivity_{self.percentage_conumber_of_nodesectivity}\n" \
               f"Chromosome: pathSequenumber_of_cellse_{self.path}_startPoints_{self.start_points}"

    def __init__(self, path, start_points, info:PathInfo):

        # self.hovering = info.hovering
        # self.realtime_conumber_of_nodesectivity = info.realtime_conumber_of_nodesectivity

        # Inputs
        self.path = path
        self.start_points = start_points
        self.info: PathInfo = info

        # print(f'path: {self.path}')
        # print(f"start points: {self.start_points}")

        # cell - path
        self.drone_dict_generated = False
        self.drone_dict = dict()
        self.interpolated_path_matrix = None # Interpolated cell matrix (discrete, cell by cell)
        self.realtime_path_matrix = None # Interpolated cell matrix (seconumber_of_droness)


        # xy coordinate - path
        self.x_coords_dict = dict()
        self.y_coords_dict = dict()
        self.x_coords_list = [None]*info.number_of_drones
        self.y_coords_list = [None]*info.number_of_drones

        # self.x_matrix
        # self.y_matrix

        # Time
        self.time_slots = None  # Continuous
        self.drone_timeslots = None
        self.time_steps = None  # Discrete
        self.drone_timesteps = None
        self.occ_grid = np.full((self.info.number_of_nodes,self.info.number_of_cells), 0.5) # Initialize occupanumber_of_cellsy grid for each node inumber_of_cellsluding BS
        self.cell_visit_steps = None
        self.tbv = None
        self.min_tbv = None
        # distance
        self.subtour_lengths = None
        self.total_distance = None
        self.longest_subtour = None
        self.shortest_subtour = None
        self.subtour_range = None
        self.long_jump_violations = None
        self.long_jump_violations_constr = None
        self.cells_per_drone_constr = None
        self.max_visits_constr = None
        self.cell_nvisits = None

        self.time_slots = None
        self.time_steps = None

        # Conumber_of_nodesectivity
        self.connectivity_matrix = None
        self.num_connected_drones_to_base = None
        self.disconnected_time_steps = None
        # self.conumber_of_nodesectivity_to_base_matrix = None
        self.percentage_connectivity = None
        self.total_disconnected_timesteps = None
        self.max_disconnected_timesteps = None


        # Call PathSolution funumber_of_cellstions
        self.get_pathplan() # Calculates drone dict and path matrix (not interpolated, directly from the path sequenumber_of_cellse and start points)
        # self.get_real_time_path_matrix_with_hovering()
        self.calculate_connectivity_matrix()
        self.calculate_connectivity_to_base_matrix()
        self.calculate_cell_visit_steps()
        # self.calculate_percentage_conumber_of_nodesectivity()
        # self.calculate_disconumber_of_nodesected_timesteps() # NOTE COMPLETE YET
        # self.calculate_total_distance_and_longest_subtour()
        # self.calculate_distance_constraints()

        # self.calculate_time_between_visits()
        #
        # min_time_between_visits(self)

# ISLAM'S CONNECTIVITY DEFINITIONS
        
    def calculate_connectivity_to_base_matrix(self):

        info = self.info

        connectivity = self.connectivity_matrix

        time_slots = self.time_slots

        connectivity_to_base = np.zeros((time_slots, info.number_of_drones + 1))

        #path_sums = np.zeros(info.number_of_drones + 1)

        for time in range(time_slots):
            adj_mat = connectivity[time, :, :]

            connectivity_to_base[time, BFS(adj_mat, self)] = 1

            """path_sums[:] = 0

            for pow in range(info.max_hops):
                path_sums += np.linalg.matrix_power(adj_mat, pow)[0, :]
            connectivity_to_base[time] = path_sums > 0
            """
        self.connectivity_to_base_matrix = connectivity_to_base

        return connectivity_to_base, time_slots
    
    def get_percentage_connectivity(self):

        connectivity_to_base_percentage = np.zeros(self.time_slots)

        for time in range(self.time_slots):
            connectivity_to_base_percentage[time] = sum(self.connectivity_to_base_matrix[time, 1:])/(self.info.number_of_drones)

        return np.mean(connectivity_to_base_percentage)


    def calculate_cell_visit_steps(self):
        info = self.info
        path_matrix = self.real_time_path_matrix[1:,:].transpose()
        cell_visit_steps = dict()
        for i in range(info.number_of_cells):
            cell_visit_steps[i] = np.where(path_matrix==i)[0] # Steps at which the cell is visited

        self.cell_visit_steps = cell_visit_steps

    def calculate_connectivity_matrix(self):

        info = self.info

        comm_dist = info.comm_cell_range * info.cell_side_length

        real_time_path_matrix = self.real_time_path_matrix % info.number_of_cells

        time_slots = real_time_path_matrix.shape[1]

        connectivity = np.zeros((time_slots, info.number_of_drones+1, info.number_of_drones+1))   

        # for drone_no in range(real_time_path_matrix.shape[0]):
        #     drone_path = real_time_path_matrix[drone_no, :]
        #     for drone_no_2 in range(real_time_path_matrix.shape[0]):
        #         drone_path_2 = real_time_path_matrix[drone_no_2, :]
        #         for time in range(time_slots):
        #             if info.D[drone_path[time], drone_path_2[time]] <= comm_dist:
        #                 connectivity[time, drone_no, drone_no_2] = 1

        for drone_no in range(real_time_path_matrix.shape[0]):
            drone_path = real_time_path_matrix[drone_no, :]
            for drone_no_2 in range(real_time_path_matrix.shape[0]):
                drone_path_2 = real_time_path_matrix[drone_no_2, :]
                for time in range(time_slots):
                    if drone_no != drone_no_2 and info.D[drone_path[time], drone_path_2[time]] <= comm_dist:
                        if drone_no == 0 and drone_no_2 > 0 and drone_path_2[time] != -1:
                            connectivity[time, drone_no, drone_no_2] = 1
                        elif drone_no > 0 and drone_no_2 == 0 and drone_path[time] != -1:
                            connectivity[time, drone_no, drone_no_2] = 1
                        elif drone_no > 0 and drone_no_2 > 0 and drone_path[time] != -1 and drone_path_2[time] != -1:
                            connectivity[time, drone_no, drone_no_2] = 1
                        else:
                            continue



        self.connectivity_matrix = connectivity
        self.time_slots = time_slots

        return connectivity, time_slots
                            
    def get_real_time_path_matrix_with_hovering(self):

        end = self.info.number_of_cells*self.info.min_visits
        start = self.start_points[-1]
        self.longest_path = end - start
        end = start

        for i in range(self.info.number_of_drones-2, -1, -1):
            start = self.start_points[i]
            current_path = end - start
            if current_path > self.longest_path:
                self.longest_path = current_path
            end = start

        self.real_time_path_matrix = np.zeros((self.info.number_of_nodes, self.longest_path), int) - 1


        for drone_no in range(self.info.number_of_drones):

            if drone_no < self.info.number_of_drones-1:
                end_point = self.start_points[drone_no+1]
            else:
                end_point = self.info.number_of_cells*self.info.min_visits

            city_counts = np.zeros(self.info.number_of_cells*self.info.min_visits, int)

            path_len = 0

            # print(self.start_points[drone_no], end_point)
            # print(self.path[self.start_points[drone_no]:end_point])

            sub_path = self.path[self.start_points[drone_no]:end_point]

            path_len += len(sub_path)

            city_counts[sub_path] = 1

            # print(f"sub_path: {sub_path}")

            # print("----------------------------------------------------------------------------------------------------------------------------------------------------------------")

            while not path_len == self.longest_path:
                # print(f"path_len: {path_len}, longest_path: {self.longest_path}")
                for city in sub_path:
                    city_counts[city] += 1
                    path_len += 1
                    if(path_len == self.longest_path):
                        break 

            i = 0
            for city in sub_path:
                for hover in range(city_counts[city]):
                    self.real_time_path_matrix[drone_no + 1, i] = city
                    i += 1
        bs_path_component = np.zeros((self.real_time_path_matrix.shape[0],1), dtype=int)-1
        self.real_time_path_matrix = np.hstack((bs_path_component, self.real_time_path_matrix, bs_path_component), dtype=int)

        # print("Real Time Path Matrix:\n",self.real_time_path_matrix)

        self.time_steps = self.real_time_path_matrix.shape[1]

        # print("path matrix with hovering:")
        # print(self.real_time_path_matrix % self.info.number_of_cells)



    def get_pathplan(self):

        self.drone_dict_generated = True
        self.drone_dict = dict()
        self.time_steps = 0
        info = self.info

        # GET CELL DICT
        for i in range(info.number_of_drones):
            if i < info.number_of_drones - 1:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:self.start_points[i + 1]], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.number_of_cells)[self.start_points[i]:self.start_points[i + 1]], np.array([-1])))
            else:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.number_of_cells)[self.start_points[i]:], np.array([-1])))

            # Set longest "discrete" subtour
            if len(self.drone_dict[i]) > self.time_steps : self.time_steps = len(self.drone_dict[i]) # Set max subtour length

            # Add BS as a node to drone_dict (key=1)
            self.drone_dict[-1] = np.array([-1] * self.time_steps)

        # print(f"DRONE DICT: {self.drone_dict}")


        # GET CELL MATRIX
        self.path_matrix = np.zeros((info.number_of_drones+1, self.time_steps), dtype=int) - 1 # number_of_drones+1 bc. of BS (inumber_of_dronesex 0)
        for i in range(info.number_of_drones):
            if len(self.drone_dict[i]) == self.time_steps: # If this is the longest discrete tour drone
                self.path_matrix[i+1] = self.drone_dict[i]
            else : # If this is NOT the longest discrete tour drone
                len_diff = self.time_steps - len(self.drone_dict[i])
                filler = np.array([-1]*len_diff)
                self.path_matrix[i+1] = np.hstack( (self.drone_dict[i] , filler)  )

        self.real_time_path_matrix = self.path_matrix

        # APPLY HOVERING TO DRONES WITH SHORTER PATHS
        path_lens = [len(path) for path in list(self.drone_dict.values())]
        hovering_drone_ids = [path_lens.index(i) for i in path_lens if i != min(path_lens)]
        print("----------------------------------------------------------")
        for drone in hovering_drone_ids:
            print(f"Drone {drone}:")
            # APPLY HOVERING
            path_without_hovering = self.real_time_path_matrix[i+1].copy()
            # Find last cell
            hovering_cell_idx = np.where(path_without_hovering==-1)[0][1] - 1
            hovering_cell = path_without_hovering[hovering_cell_idx]
            hovering_component = np.array([hovering_cell] * (len(path_without_hovering) - hovering_cell_idx - 1))
            path_with_hovering = path_without_hovering.copy()
            path_with_hovering[hovering_cell_idx:len(path_without_hovering)-1] = hovering_component
            self.real_time_path_matrix[i+1] = path_with_hovering
            print(f"Path Without Hovering: {path_without_hovering}\nPath with Hovring:{path_with_hovering}")
        print("----------------------------------------------------------")
            # self.real_time_path_matrix[i+1] = np.hstack((path_with_hovering[:hovering_cell_idx], np.array([hovering_cell]*(len(path_with_hovering)-hovering_cell_idx-1))))
        # print(f"PATH MATRIX:\n{pd.DataFrame(self.real_time_path_matrix).to_string(index=False)}")
        print(f"Path Matrix:\n{self.real_time_path_matrix}")

        # XY COORDINATES DON'T ENABLE

        # # GET XY DICT
        # for i in range(1, self.time_steps):
        #     current_step_cells , next_step_cells = self.path_matrix[1:,i-1].tolist() , self.path_matrix[1:,i].tolist()
        #     current_step_coords = list(map(self.get_coords, current_step_cells))
        #     next_step_coords = list(map(self.get_coords, next_step_cells))
        #     coord_diffs = [next_step_coords[j] - current_step_coords[j] for j in range(info.number_of_drones)]
        #     thetas = [atan2(j[1],j[0]) for j in coord_diffs]
        #     current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], self.info.V * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]) for j in range(info.number_of_drones) ]
        #     current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], self.info.V * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]) for j in range(info.number_of_drones) ]
        #     for j in range(info.number_of_drones):
        #         x_coords, y_coords = current_to_next_step_x_coords[j], current_to_next_step_y_coords[j]
        #         if len(x_coords) != len(y_coords):
        #             xy_diff = abs(len(x_coords) - len(y_coords))
        #             if len(x_coords) > len(y_coords): # Fill y
        #                 current_to_next_step_y_coords[j] = np.hstack((current_to_next_step_y_coords[j], np.array([y_coords[-1]]*xy_diff)))
        #             else: # Fill x
        #                 current_to_next_step_x_coords[j] = np.hstack((current_to_next_step_x_coords[j], np.array([x_coords[-1]]*xy_diff)))
        #         else:
        #             continue


        #     if self.hovering:
        #         step_lengths = [len(current_to_next_step_x_coords[j]) for j in range(info.number_of_drones)]
        #         max_step_dist = max(step_lengths)
        #         longest_step_drone = step_lengths.inumber_of_dronesex(max_step_dist)  # Identify drone with longest step
        #         current_to_next_step_x_coords = [np.hstack((current_to_next_step_x_coords[j] , np.array([current_to_next_step_x_coords[j][-1]]*(max_step_dist - len(current_to_next_step_x_coords[j]))))) if j != longest_step_drone else current_to_next_step_x_coords[j] for j in range(info.number_of_drones)]
        #         current_to_next_step_y_coords = [np.hstack((current_to_next_step_y_coords[j] , np.array([current_to_next_step_y_coords[j][-1]]*(max_step_dist - len(current_to_next_step_y_coords[j]))))) if j != longest_step_drone else current_to_next_step_y_coords[j] for j in range(info.number_of_drones)]
        #     # self.x_coords_dict[j].append(current_to_next_step_x_coords[j]) for j in range(info.number_of_drones)
        #     self.x_coords_list = [current_to_next_step_x_coords[j] if i==1 else np.hstack((self.x_coords_list[j],current_to_next_step_x_coords[j])) for j in range(info.number_of_drones)]
        #     self.y_coords_list = [current_to_next_step_y_coords[j] if i==1 else np.hstack((self.y_coords_list[j],current_to_next_step_y_coords[j])) for j in range(info.number_of_drones)]

        # self.drone_timeslots = [len(x) for x in self.x_coords_list]
        # self.time_slots = max(self.drone_timeslots)

        # # Initialize xy matrix
        # x_sink,y_sink = self.get_coords(-1)
        # self.x_matrix = np.full((info.number_of_drones + 1, self.time_slots), x_sink)  # number_of_drones+1 rows in order to inumber_of_cellsorporate base station
        # self.y_matrix = self.x_matrix.copy()
        # self.realtime_path_matrix = self.x_matrix.copy()
        # self.realtime_path_matrix.astype(int)
        # self.realtime_path_matrix[:, :] = -1
        # interpolated_path_dict = dict()
        # interpolated_path_max_len = 0

        # # print(f"path matrix: {self.path_matrix}")

        # for i in range(info.number_of_drones):
        #     self.x_matrix[i + 1] = np.hstack((self.x_coords_list[i], np.array([x_sink] * (self.time_slots - self.drone_timeslots[i]))))
        #     self.y_matrix[i + 1] = np.hstack((self.y_coords_list[i], np.array([y_sink] * (self.time_slots - self.drone_timeslots[i]))))
        #     # drone_xy_coords = list(zip(self.x_matrix[j + 1], self.y_matrix[j + 1]))
        #     self.realtime_path_matrix[i + 1] = [self.get_city(k) for k in list(zip(self.x_matrix[i + 1], self.y_matrix[i + 1]))]
        #     interpolated_path_dict[i] = []
        #     for j in range(self.time_slots - 1):
        #         if self.realtime_path_matrix[i + 1][j] != self.realtime_path_matrix[i + 1][j + 1]:
        #             interpolated_path_dict[i].append(self.realtime_path_matrix[i + 1][j])
        #     # print("----------------------------------------------------------------------------------------------------")
        #     # print(f"Drone {i+1}")
        #     # print("----------------------------------------------------------------------------------------------------")
        #     # print(f"matrix list: {self.realtime_path_matrix[i + 1]}\number_of_dronesict list: {interpolated_path_dict[i]}")

        #     if  self.realtime_path_matrix[i + 1][-1] != interpolated_path_dict[i][-1]:
        #         interpolated_path_dict[i].append(self.realtime_path_matrix[i + 1][-1])
        #     if len(interpolated_path_dict[i]) > interpolated_path_max_len:
        #         interpolated_path_max_len = len(interpolated_path_dict[i])

        # # Interpolated Path Matrix
        # self.interpolated_path_matrix = np.full((info.number_of_drones+1,interpolated_path_max_len), -1)
        # for i in range(info.number_of_drones):
        #     interpolated_drone_path = interpolated_path_dict[i]
        #     len_diff = abs(len(interpolated_drone_path) - interpolated_path_max_len)
        #     self.interpolated_path_matrix[i+1] = interpolated_drone_path + [interpolated_drone_path[-1]]*len_diff

        # self.realtime_path_matrix = self.realtime_path_matrix.astype(int)


    def get_coords(self, cell):

        if cell == -1:
            x = -self.info.cell_side_length / 2
            y = -self.info.cell_side_length / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.cell_side_length
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.cell_side_length
        # return [x,y]
        return np.array([x, y])

    def get_city(self, coords):

        if coords[0] < 0 and coords[1] < 0:
            return -1
        else:
            x, y = coords
            return floor(y / self.info.cell_side_length) * self.info.grid_size + floor(x / self.info.cell_side_length)



    def get_x_coords(self, cell):

        if cell == -1:
            x = -self.info.cell_side_length / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.cell_side_length
        return x

    def get_y_coords(self, cell):

        if cell == -1:
            y = -self.info.cell_side_length / 2
        else:
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.cell_side_length
        return y
    

def BFS(adj, sol:PathSolution):
      
    v = sol.info.number_of_nodes

    ctb = []
    start = 0
    # Visited vector to so that a
    # vertex is not visited more than
    # once Initializing the vector to
    # false as no vertex is visited at
    # the beginning
    visited = [False] * (sol.info.number_of_nodes)
    q = [start]

    # Set source as visited
    visited[start] = True

    while q:
        vis = q[0]

        # Print current node
        ctb.append(vis)

        q.pop(0)
          
        # For every adjacent vertex to
        # the current vertex
        for i in range(v):
            if (adj[vis][i] == 1 and
                  (not visited[i])):
                        
                # Push the adjacent node
                # in the queue
                q.append(i)
                  
                # set
                visited[i] = True
    
    return ctb


    
    

# info = PathInfo(min_visits=5)
# path = np.random.permutation(info.min_visits * info.number_of_cells).tolist()
# # Random start points
# start_points = sorted(random.sample([i for i in range(1, len(path))], info.number_of_drones - 1))
# start_points.insert(0, 0)
# sol = PathSolution(path, start_points, info)


# info = PathInfo(number_of_drones=4, grid_size=8, min_visits=3)
# # path = np.random.permutation(range(1,info.number_of_cells))
# path = np.random.permutation(info.number_of_cells)
# start_points = [0,16,32,48]
# start_points = [0,12,22,30]
# t = time.time()
# sol = PathSolution(path, start_points, info)

# print(f"time between visits:\n{sol.time_between_visits}")
# print(f"cell visit steps:\n{sol.cell_nvisits}")
# print("Hovering and realtime conumber_of_nodesectivity:", time.time()-t)

# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=True, realtime_conumber_of_nodesectivity=False)
# print("Hovering and discrete conumber_of_nodesectivity:", time.time()-t)
# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=False, realtime_conumber_of_nodesectivity=False)
# print("No Hovering and realtime conumber_of_nodesectivity:", time.time()-t)
# t = time.time()
# sol = PathSolution(path, start_points, info, hovering=False, realtime_conumber_of_nodesectivity=False)
# print("No Hovering and discrete conumber_of_nodesectivity:", time.time()-t)


# animation = PathAnimation(sol.x_matrix, sol.y_matrix, sol.info)

# print("path sequenumber_of_cellse:", path)
# print("start points:", start_points)
# print("drone dict:", sol.drone_dict)
# print("path matrix:", sol.path_matrix)
# print("longest discrete subtour:", sol.longest_discrete_subtour)
# df = pd.DataFrame(sol.interpolated_realtime_path_matrix)
# print(f"intp realtime path matrix:\n{df.to_string(inumber_of_dronesex=False)}")

# df = pd.DataFrame(sol.interpolated_path_matrix)
# print(f"intp path matrix:\n{df.to_string(inumber_of_dronesex=False)}")