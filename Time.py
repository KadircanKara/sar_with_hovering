from Connectivity import *
from PathOptimizationModel import *
from PathSolution import *

def calculate_time_penalty(sol:PathSolution):
    return get_visit_time_variance(sol) + get_max_visits(sol) + get_min_time_between_visits_variance(sol)

def get_visit_time_variance(sol:PathSolution):
    if not sol.cell_nvisits :
        calculate_visit_times(sol)
    return np.var(sol.cell_nvisits)

def get_max_visits(sol:PathSolution):
    if not sol.cell_nvisits :
        calculate_visit_times(sol)
    return max(sol.cell_nvisits) - sol.info.max_visits

def calculate_visit_times(sol:PathSolution):
    cell_nvisits = [len(sol.cell_visit_steps[i]) for i in range(sol.info.Nc)]
    sol.cell_nvisits = cell_nvisits
    return sol.cell_nvisits

def get_min_time_between_visits_variance(sol:PathSolution):
    if not sol.min_tbv:
        get_min_time_between_visits(sol)
    return np.var(sol.min_tbv)

def get_min_time_between_visits(sol:PathSolution):
    if not sol.tbv:
        calculate_time_between_visits(sol)
    sol.min_tbv = [min(sol.tbv[i]) for i in range(sol.info.Nc)]
    return sol.min_tbv

def calculate_time_between_visits(sol:PathSolution):
    info = sol.info
    tbv = dict()
    for i in range(info.Nc):
        tbv[i] = [] # Initialize tbv for every cell
        for j in range(1,len(sol.cell_visit_steps[i])):
            tbv[i].append( sol.cell_visit_steps[i][j] - sol.cell_visit_steps[i][j-1] )
        if len(tbv[i])==0: # For cells only visited once
            tbv[i].append(0)

    sol.tbv = tbv

    return sol.tbv

'''    info = sol.info
    path_matrix = sol.interpolated_path_matrix[1:, :].transpose()
    cell_nvisits = np.zeros(info.Nc, dtype=int)
    cell_visit_steps = dict()
    time_between_visits = dict()
    # plt.figure()
    # plt.xticks( range(info.Nc) )  # Specify the locations of the ticks
    # xticklabels = ['A', 'B', 'C', 'D', 'E']
    # scatter_plot = plt.scatter([], [])
    for i in range(info.Nc):
        time_between_visits[i] = []
        cell_visit_steps[i] = np.where(path_matrix == i)[0]  # Steps at which the cell is visited
        cell_nvisits[i] = len(cell_visit_steps[i])  # How many times the cell is visited
        # Calculate tbv
        for j in range(1, len(cell_visit_steps[i])):
            time_between_visits[i].append(cell_visit_steps[i][j] - cell_visit_steps[i][j - 1])

        if len(time_between_visits[i]) == 0:
            time_between_visits[i].append(0)
            scatter_plot = plt.scatter(i, time_between_visits[i][-1])
            # plt.pause(0.01)  # Add a short pause to observe the animation
            # plt.draw()
    # plt.show()

    sol.cell_nvisits = cell_nvisits
    sol.time_between_visits = time_between_visits

    # print(f"cell nvisits:\n{cell_nvisits}")
    # print(f"max: {max(cell_nvisits)}, min: {min(cell_nvisits)}")
    # print(f"cell visit steps:\n{cell_visit_steps}")
    # print(f"time between visits:\n{time_between_visits}")
'''

'''def calculate_mission_time(sol:PathSolution):

    # TODO
    # Calculate mission time based on the occupancy grid
    # Probability update calculation: average of connected components' occupancy grid
    # Update path dict and matrix attributes accordingly (finish search early - once # high confidence cells == Nt)
    # Introduce a new PathSolution attribute : sol.mission_time

    # True Detection Prob :
    # True Miss Prob :
    # False Detection Prob :
    # False Miss Prob :

    info = sol.info

    num_nodes = info.Nd+1

    Nt = info.Nt
    tar_pos = info.tar_pos
    assert len(tar_pos) == Nt

    occ_grid = info.occ_grid # Initial occ grid (every cell has 0.5 occ prob. for each node) - Includes BS (Nd+1 rows)

    realtime_path_matrix = sol.realtime_path_matrix # pathplan cells (realtime) dt=1 second
    x_matrix = sol.x_matrix # pathplan x-coords (realtime) dt=1 second
    y_matrix = sol.y_matrix # pathplan y-coords (realtime) dt=1 second
    path_matrix = sol.path_matrix # Nodes will share info at discrete timesteps (only works when there is hovering)

    if realtime_info_sharing:
        time_steps = x_matrix.shape[1]
        path_matrix = np.empty(x_matrix.shape, dtype=tuple)
        for i, y in enumerate(x_matrix):
            # i=0 y=[1,2,3]
            # i=1 y=[4,5,6]
            path_matrix[i] = list(zip(y, y_matrix[i]))


    else:
        time_steps = path_matrix.shape[1]
        for i in range(time_steps):
            node_positions = path_matrix[:,i]
            for j in range(info.Nd):
                # First update occ. grid based on sensor readings
                drone_position = node_positions[j+1]

            # Update occ. grid based on connected components
            step_connectivity_matrix = sol.connectivity_matrix[i]
            comp = connected_components(step_connectivity_matrix)
            prob_sum = 0
            for c in comp:
                num_conn_comp = len(c)
                # Calculate the sum of the occupancy probabilities
                for d in c:
                    prob_sum += sol.occ_grid[d]
                # Take the average of the occupancy grids of the connected components
                for d in c:
                    sol.occ_grid[d] = prob_sum/num_conn_comp'''