import numpy as np
from math import floor, sqrt
import copy

from Time import *

from PathSolution import *

def get_total_distance(sol:PathSolution):
    if not sol.total_distance:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.total_distance = sum(sol.subtour_lengths.values())
    return sol.total_distance

def get_longest_subtour(sol:PathSolution):
    if not sol.longest_subtour:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.longest_subtour = max(sol.subtour_lengths)
    return sol.longest_subtour

def calculate_subtour_lengths(sol:PathSolution):

    if not sol.subtour_lengths:

        info = sol.info

        path_matrix = sol.path_matrix

        Nd, time_steps = path_matrix.shape
        Nd -= 1 # Remove base station

        subtour_lengths = dict()

        for i in range(info.Nd):
            drone_path = path_matrix[i+1]
            drone_dist = 0
            for j in range(time_steps-1):
                drone_dist += info.D[drone_path[j],drone_path[j+1]]
            subtour_lengths[i] = drone_dist

        sol.subtour_lengths = subtour_lengths

    return sol.subtour_lengths


def calculate_number_of_long_jumps(sol:PathSolution):

    if not sol.long_jump_violations :
        info = sol.info
        path_matrix = sol.path_matrix
        long_jump_violations = 0
        for i in range(info.Nd):
            for j in range(path_matrix.shape[1] - 1):
                if info.D[path_matrix[i+1,j],path_matrix[i+1,j+1]] > info.A * sqrt(2):
                    long_jump_violations += 1

        sol.long_jump_violations = long_jump_violations

    return sol.long_jump_violations


def cell_per_drone_constr(sol:PathSolution):

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

def long_jumps_constr(sol:PathSolution):

    if not sol.long_jump_violations :
        calculate_number_of_long_jumps(sol)

    return sol.long_jump_violations - sol.info.Nd * sol.info.min_visits * 2

    # print("long jump cv:", long_jump_violations)

    # sol.long_jump_violations = long_jump_violations - constr

    # cofactor = 2
    # # bias = 5
    # constr = info.Nd * info.min_visits * cofactor      # 33 for Nd=8 min_visits=2 (cofactor=2.0625)
    # #                                                      37.5 for N=8 min_visits=3 (cofactor=1.5625)
    # #                                                      107 for Nd=16 min_visits=3 (cofactor=4.45)
    #
    # sol.long_jump_violations_constr = long_jump_violations - constr
    #
    # return sol.long_jump_violations_constr

def longest_subtour_constr(sol:PathSolution):
    if not sol.longest_subtour :
        get_longest_subtour(sol)
    return sol.longest_subtour - sol.info.subtour_length_th

'''# def limit_max_visits(sol):
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
#     return sol.max_visits_constr'''









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

