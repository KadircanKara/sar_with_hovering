import numpy as np
from math import floor, sqrt
import copy

# from PathSolution import *


def number_of_long_jumps(sol):
    info = sol.info
    path_matrix = sol.path_matrix
    long_jump_violations = 0
    for i in range(info.Nd):
        for j in range(path_matrix.shape[1] - 1):
            if info.D[path_matrix[i+1,j],path_matrix[i+1,j+1]] > info.A * sqrt(2):
                long_jump_violations += 1
    return long_jump_violations


def visit_time_variance(sol):
    return np.var(sol.cell_nvisits)

def max_visits(sol):
    # return max(sol.cell_nvisits) - info.max_visits
    return max(sol.cell_nvisits)

def min_time_between_visits(sol):
    info = sol.info
    tbv = sol.time_between_visits
    min_tbv = [ min(tbv[cell]) for cell in range(info.Nc)]
    return min(min_tbv)

# def limit_max_visits(sol):
#     info = sol.info
#     path_matrix = sol.interpolated_path_matrix
#     cell_visits = np.zeros(info.Nc)
#     for i in range(info.Nc):
#         cell_visits[i] = np.count_nonzero(path_matrix==i)
#
#     # print(f"Max Visits: {max(cell_visits)}")
#
#     sol.max_visits_constr = max(cell_visits) - info.max_visits
#
#     return sol.max_visits_constr


def limit_cell_per_drone(sol):

    info = sol.info
    drone_dict = sol.drone_dict

    cells_per_drone = []

    constr = info.Nc * info.min_visits // info.Nd

    for i in range(info.Nd):
        drone_path = drone_dict[i][2:-2] # To eliminate -1 and 0 at the start and the end
        cells_per_drone.append(len(drone_path))

    sol.cells_per_drone_constr = max(cells_per_drone) - min(cells_per_drone) - constr

    # print("cell per drone cv:", max(cells_per_drone) - min(cells_per_drone) - constr)

    return sol.cells_per_drone_constr

'''    constr = info.number_of_cities // info.number_of_drones

    path_lens = []

    for i in range(info.number_of_drones):
        path_lens.append(len(sol.drone_dict[i]))

    return (max(path_lens) - min(path_lens)) - constr
'''

def limit_long_jumps(sol):

    info = sol.info

    path_matrix = sol.path_matrix

    long_jump_violations = 0

    for i in range(info.Nd):
        for j in range(path_matrix.shape[1] - 1):
            if info.D[path_matrix[i+1,j],path_matrix[i+1,j+1]] > info.A * sqrt(2):
                long_jump_violations += 1

    # print("long jump cv:", long_jump_violations)

    # sol.long_jump_violations = long_jump_violations - constr

    cofactor = 2
    # bias = 5
    constr = info.Nd * info.min_visits * cofactor      # 33 for Nd=8 min_visits=2 (cofactor=2.0625)
    #                                                      37.5 for N=8 min_visits=3 (cofactor=1.5625)
    #                                                      107 for Nd=16 min_visits=3 (cofactor=4.45)

    sol.long_jump_violations_constr = long_jump_violations - constr

    return sol.long_jump_violations_constr

def get_total_distance_and_longest_subtour(sol):

    info = sol.info

    path_matrix = sol.path_matrix

    Nd, time_steps = path_matrix.shape
    Nd -= 1 # Remove base station

    drone_total_distances = []

    for i in range(info.Nd):
        drone_path = path_matrix[i+1]
        drone_dist = 0
        for j in range(time_steps-1):
            drone_dist += info.D[drone_path[j],drone_path[j+1]]
        drone_total_distances.append(drone_dist)

    sol.drone_dists = drone_total_distances
    sol.total_dist = sum(drone_total_distances)
    sol.longest_subtour = max(drone_total_distances)

    return sol.total_dist, sol.longest_subtour












def get_coords(cell, grid_size, A):

    if cell == -1:
        x = -A / 2
        y = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell % grid_size + 0.5) * A
        # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
        y = (cell // grid_size + 0.5) * A
    return np.array([x, y])

def get_city(coords, grid_size, A):

    if coords[0] < 0 and coords[1] < 0:
        return -1
    else:
        x, y = coords
        return floor(y / A) * grid_size + floor(x / A)

def get_x_coords(cell, grid_size, A):

    if cell == -1:
        x = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell %grid_size + 0.5) * A
    return x

def get_y_coords(self, cell, grid_size, A):

    if cell == -1:
        y = -A / 2
    else:
        # y = ((cell % n) // grid_size + 0.5) * A
        y = (cell // grid_size + 0.5) * self.info.A
    return y

