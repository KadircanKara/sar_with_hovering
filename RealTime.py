import numpy as np

from PathSolution import *
from Distance import *
from Connectivity import *


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

    x_values = np.zeros((time_slot, sol.info.Nd+1))
    y_values = np.zeros((time_slot, sol.info.Nd+1))

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


def get_total_timesteps(sol : PathSolution):
  D = sol.info.D

  path_with_hovering = sol.real_time_path_matrix

  city_prevs = [-1] * (sol.info.Nd + 1)

  total_steps = 0

  for visit in range(sol.longest_path):
      max_movement = -1

      for drone in range(sol.info.Nd + 1):
          movement = D[path_with_hovering[drone, visit], city_prevs[drone]]
          max_movement = max(movement, max_movement)
        
      total_steps += max_movement
      city_prevs = path_with_hovering[:, visit]
  
  max_movement = -1

  for drone in range(sol.info.Nd + 1):
      movement = D[city_prevs[drone], -1]
      max_movement = max(movement, max_movement)
    
  total_steps += max_movement

  return int(total_steps * 50 / 2.5) / (sol.info.number_of_cities * 40)

def get_timestep_path_sequence(sol : PathSolution):
  D = sol.info.D

  path_with_hovering = sol.real_time_path_matrix

  city_prevs = [-1] * (sol.info.Nd + 1)

  steps = []

  for visit in range(sol.longest_path):
      max_movement = -1

      for drone in range(sol.info.Nd + 1):
          movement = D[path_with_hovering[drone, visit], city_prevs[drone]]
          max_movement = max(movement, max_movement)
        
      steps.append(max_movement)
      city_prevs = path_with_hovering[:, visit]
  
  max_movement = -1

  for drone in range(sol.info.Nd + 1):
      movement = D[city_prevs[drone], -1]
      max_movement = max(movement, max_movement)
    
  steps.append(max_movement)

  return path_with_hovering, steps
  

def get_real_real_path(xs, ys, path_start_points):
  n_drones = len(path_start_points)

  lens = dict()

  x = xs[:, 0]

  for i in range(len(xs[1:])):
    lens[i] = []

  for n in range(n_drones):
    x = xs[:,n]
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
      interp_mid = np.linspace(el_prev*20, el*20+1, max_lens[i]) / 20
      interp_y = np.concatenate((interp_y, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))
    
    final_interp_y.append(interp_y)


  return np.array(final_interp_x), np.array(final_interp_y)

def get_real_time_step_number(sol, xs, ys):
    return len(xs[0])

def calculate_real_time_connectivity_matrix(sol, xs, ys):

    info = sol.info

    comm_dist = np.sqrt(info.comm_dist_square)

    time_slots = len(xs[0])

    connectivity = np.zeros((time_slots, info.Nd+1, info.Nd+1))   

    for drone1 in range(xs.shape[0]):

        for drone2 in range(xs.shape[0]):

            for time in range(time_slots):
                if  np.sqrt((xs[drone2, time]-xs[drone1, time])**2 + (ys[drone2, time]-ys[drone1, time])**2) <= comm_dist:
                  connectivity[time, drone1, drone2] = 1

    return connectivity, time_slots


def calculate_real_time_connectivity_to_base_matrix(sol, xs, ys):

    info = sol.info

    connectivity, time_slots = calculate_real_time_connectivity_matrix(sol, xs, ys)

    connectivity_to_base = np.zeros((time_slots, info.Nd + 1))

    #path_sums = np.zeros(info.Nd + 1)

    for time in range(time_slots):
        adj_mat = connectivity[time, :, :]

        connectivity_to_base[time, BFS(adj_mat, sol)] = 1

    return connectivity_to_base, time_slots


def calculate_real_time_percentage_connectivity(sol, xs, ys):

    connectivity_to_base_matrix, time_slots = calculate_real_time_connectivity_to_base_matrix(sol, xs, ys)

    connectivity_to_base_percentage = np.zeros(time_slots)

    for time in range(time_slots):
        connectivity_to_base_percentage[time] = sum(connectivity_to_base_matrix[time, 1:])/(sol.info.Nd)

    return np.mean(connectivity_to_base_percentage)


def calculate_real_time_disconnected_time_matrix(sol, xs, ys):

    info = sol.info

    connectivity_to_base, time_slots = calculate_real_time_connectivity_to_base_matrix(sol, xs, ys)

    disconnection = np.zeros(info.Nd)

    for drone in range(1, info.Nd+1):
        disconnected = 0
        for time in range(time_slots):
            disconnected += not connectivity_to_base[time, drone]

            if connectivity_to_base[time, drone] or time==time_slots-1:
                if(disconnected > disconnection[drone-1]):
                    disconnection[drone-1] = disconnected
                disconnected = 0

    return disconnection, time_slots


def calculate_real_time_max_disconnectivity(sol, xs, ys):
    disconnection, time_slots = calculate_real_time_disconnected_time_matrix(sol, xs, ys)

    return max(disconnection) / time_slots


def calculate_real_time_mean_disconnectivity(sol, xs, ys):
    disconnection, time_slots = calculate_real_time_disconnected_time_matrix(sol, xs, ys)

    return np.mean(disconnection) / time_slots
