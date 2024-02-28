from PathSolution import *
from Connectivity import *
from PathOptimizationModel import realtime_info_sharing

def calculate_mission_time(sol:PathSolution):

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
                    sol.occ_grid[d] = prob_sum/num_conn_comp