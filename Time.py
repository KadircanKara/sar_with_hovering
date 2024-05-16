from Connectivity import *
from PathSolution import *
from Distance import *
from scipy.io import savemat
from statistics import median_low

# def get_coord_path(sol:PathSolution):
   
#    path_matrix = sol.real_time_path_matrix

#    coord_path_dict = dict()

#    for drone in range(sol.info.number_of_drones):
#       drone_coord_path = [ [-1,-1] ]
#       drone_cell_path = path_matrix[drone]
#       for cell in drone_cell_path:
#          drone_coord_path.append(get_coords(sol, cell))
#       coord_path_dict[drone] = drone_coord_path
sol_path = 'Results/X'
values_path = 'Results/F'
scenario = 'g_8_a_50_n_4_v_2.5_r_4_minv_5_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0'

sols= np.load(f"{sol_path}/{scenario}_SolutionObjects.npy",allow_pickle=True)
objs = np.load(f"{values_path}/{scenario}_ObjectiveValues.npy",allow_pickle=True)

conn_values = objs[:,1].tolist()
best_conn_idx = conn_values.index(min(conn_values))
mid_conn_idx = conn_values.index(median_low(conn_values))

dist_values = objs[:,0].tolist()
best_dist_idx = dist_values.index(min(dist_values))
mid_dist_idx = dist_values.index(median_low(dist_values))

best_conn_sol = sols[best_conn_idx][0]
mid_conn_sol = sols[mid_conn_idx][0]

best_dist_sol = sols[best_dist_idx][0]
best_conn_sol = sols[best_conn_idx][0]
mid_dist_sol = sols[mid_dist_idx][0]
mid_conn_sol = sols[mid_conn_idx][0]


def get_cartesion_drone_path(sol:PathSolution):

    real_time_drone_mat = sol.real_time_path_matrix

    real_time_cartesian_drone_dict = dict()

    time_slot = len(real_time_drone_mat[0])+2

    drone_no = 0

    for drone_path in real_time_drone_mat:

        cartesian_path = [[-1, -1]]
        for city in drone_path:
            cartesian_path.append(get_coords(sol, city))
        
        cartesian_path.append([-1,-1])
        real_time_cartesian_drone_dict[drone_no] = cartesian_path

        drone_no += 1

    x_values = np.zeros((time_slot, sol.info.number_of_drones+1))
    y_values = np.zeros((time_slot, sol.info.number_of_drones+1))

    total_len = 0

    path_start_points = [0]

    for key in real_time_cartesian_drone_dict:
        path = real_time_cartesian_drone_dict[key]

        total_len += len(path)

        path_start_points.append(total_len)

        for time in range(time_slot):
            coord = path[time]
            x_values[time, key] = coord[0]
            y_values[time, key] = coord[1]

    path_start_points.pop(-1)

    return x_values, y_values, path_start_points

def get_real_real_path(xs, ys, path_start_points):
  
  n_drones = len(path_start_points)

  lens = dict()

  x = xs[:, 0]

  for i in range(len(xs[1:])):
    lens[i] = []

  for n in range(n_drones):
    x      = xs[:,n]
    el_prev = x[0]
    interp_x = np.array([])

    for i, el in enumerate(x[1:]):

      step = 1 if el>=el_prev else -1
      interp_mid = np.arange(el_prev*20, el*20+1, step) / 20
      interp_x = np.concatenate((interp_x, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

    y = ys[:,n]
    el_prev = y[0]
    interp_y = np.array([])

    for i, el in enumerate(y[1:]):

      step = 1 if el>=el_prev else -1
      interp_mid = np.arange(el_prev*20, el*20+1, step) / 20
      interp_y = np.concatenate((interp_y, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

  max_lens = []

  for i, key in enumerate(lens):
    #print(lens[key], "\t \t", xs[i+1, :])
    max_lens.append(max(lens[key]))

  lens = dict()

  for i in range(len(x[1:])):
    lens[i] = []

  final_interp_x = []
  final_interp_y = []

  for n in range(n_drones):
    x = xs[:,n]
    el_prev = x[0]
    interp_x = np.array([])

    for i, el in enumerate(x[1:]):
      interp_mid = np.linspace(el_prev*20, el*20+1, max_lens[i]) / 20
      interp_x = np.concatenate((interp_x, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))
    
    final_interp_x.append(interp_x)

    y = ys[:,n]
    el_prev = y[0]
    interp_y = np.array([])

    for i, el in enumerate(y[1:]):
      # print("-->",max_lens[i])
      interp_mid = np.linspace(el_prev*20, el*20+1, max_lens[i]) / 20
      interp_y = np.concatenate((interp_y, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))
    
    final_interp_y.append(interp_y)


  return np.array(final_interp_x), np.array(final_interp_y)

def get_real_paths(sol:PathSolution):

    info = sol.info

    time_steps = sol.real_time_path_matrix.shape[1]

    path_matrix = sol.real_time_path_matrix % info.number_of_cells

    # print("-->",path_matrix)

    for i in range(1, time_steps):
        current_step_cells , next_step_cells = path_matrix[1:,i-1].tolist() , path_matrix[1:,i].tolist()
        # Calculate Drone Speeds Based On Distance
        drone_dists = np.array([info.D[current_step_cells[j],next_step_cells[j]] for j in range(info.number_of_drones)])# Calculate Distance for Each Drone
        max_dist = max(drone_dists)
        step_time = ceil(max_dist / info.max_drone_speed)
        # print("-->",drone_dists, step_time)
        drone_speeds = drone_dists / step_time
        # print(f"Drone Dists: {drone_dists}\nStep Time: {step_time}\nDrone Speeds: {drone_speeds}")
        current_step_coords = list(map(sol.get_coords, current_step_cells))
        next_step_coords = list(map(sol.get_coords, next_step_cells))
        coord_diffs = [next_step_coords[j] - current_step_coords[j] for j in range(info.number_of_drones)]
        thetas = [atan2(j[1],j[0]) for j in coord_diffs]
        # Changes in current_to_next_step !!!
        current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], drone_speeds[j] * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]*step_time) for j in range(info.number_of_drones) ]
        current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], drone_speeds[j] * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]*step_time) for j in range(info.number_of_drones) ]
        for j in range(info.number_of_drones):
            x_coords, y_coords = current_to_next_step_x_coords[j], current_to_next_step_y_coords[j]
            if len(x_coords) != len(y_coords):
                xy_diff = abs(len(x_coords) - len(y_coords))
                if len(x_coords) > len(y_coords): # Fill y
                    current_to_next_step_y_coords[j] = np.hstack((current_to_next_step_y_coords[j], np.array([y_coords[-1]]*xy_diff)))
                else: # Fill x
                    current_to_next_step_x_coords[j] = np.hstack((current_to_next_step_x_coords[j], np.array([x_coords[-1]]*xy_diff)))
            else:
                continue


        sol.x_coords_list = [current_to_next_step_x_coords[j] if i==1 else np.hstack((sol.x_coords_list[j],current_to_next_step_x_coords[j])) for j in range(info.number_of_drones)]
        sol.y_coords_list = [current_to_next_step_y_coords[j] if i==1 else np.hstack((sol.y_coords_list[j],current_to_next_step_y_coords[j])) for j in range(info.number_of_drones)]

    sol.drone_timeslots = [len(x) for x in sol.x_coords_list]
    sol.time_slots = max(sol.drone_timeslots)

    # Initialize xy matrix
    x_sink,y_sink = sol.get_coords(-1)
    sol.x_matrix = np.full((info.number_of_drones + 1, sol.time_slots), x_sink)  # Nd+1 rows in order to incorporate base station
    sol.y_matrix = sol.x_matrix.copy()
    # sol.realtime_real_time_path_matrix = sol.x_matrix.copy()
    # sol.realtime_real_time_path_matrix.astype(int)
    # sol.realtime_real_time_path_matrix[:, :] = -1
    # interpolated_path_dict = dict()
    # interpolated_path_max_len = 0

    # print(f"path matrix: {sol.real_time_path_matrix}")

    for i in range(info.number_of_drones):
        sol.x_matrix[i + 1] = np.hstack((sol.x_coords_list[i], np.array([x_sink] * (sol.time_slots - sol.drone_timeslots[i]))))
        sol.y_matrix[i + 1] = np.hstack((sol.y_coords_list[i], np.array([y_sink] * (sol.time_slots - sol.drone_timeslots[i]))))
    
    return sol.x_matrix, sol.y_matrix

sol = best_dist_sol

x_values, y_values, start_points = get_cartesion_drone_path(sol)
x_path, y_path = get_real_real_path(x_values, y_values, start_points)
x_path, y_path = get_real_paths(sol)
print(x_path.shape, y_path.shape)
savemat('/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults/sample_x_coords.mat',{'array':x_path})
savemat('/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults/sample_y_coords.mat',{'array':y_path})


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
    real_time_path_matrix = sol.interpolated_real_time_path_matrix[1:, :].transpose()
    cell_nvisits = np.zeros(info.Nc, dtype=int)
    cell_visit_steps = dict()
    time_between_visits = dict()
    # plt.figure()
    # plt.xticks( range(info.Nc) )  # Specify the locations of the ticks
    # xticklabels = ['A', 'B', 'C', 'D', 'E']
    # scatter_plot = plt.scatter([], [])
    for i in range(info.Nc):
        time_between_visits[i] = []
        cell_visit_steps[i] = np.where(real_time_path_matrix == i)[0]  # Steps at which the cell is visited
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

    realtime_real_time_path_matrix = sol.realtime_real_time_path_matrix # pathplan cells (realtime) dt=1 second
    x_matrix = sol.x_matrix # pathplan x-coords (realtime) dt=1 second
    y_matrix = sol.y_matrix # pathplan y-coords (realtime) dt=1 second
    real_time_path_matrix = sol.real_time_path_matrix # Nodes will share info at discrete timesteps (only works when there is hovering)

    if realtime_info_sharing:
        time_steps = x_matrix.shape[1]
        real_time_path_matrix = np.empty(x_matrix.shape, dtype=tuple)
        for i, y in enumerate(x_matrix):
            # i=0 y=[1,2,3]
            # i=1 y=[4,5,6]
            real_time_path_matrix[i] = list(zip(y, y_matrix[i]))


    else:
        time_steps = real_time_path_matrix.shape[1]
        for i in range(time_steps):
            node_positions = real_time_path_matrix[:,i]
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