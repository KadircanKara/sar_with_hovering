import numpy as np
from collections import deque
from PathSolution import PathSolution
from Time import get_path_matrix

# At Least 1 Hovering Drone Connected to BS Constraint
def hovering_connectivity_constraint(sol:PathSolution):
    info = sol.info
    relay_conn_score = 0
    relay_dist_score = 0
    # Get path matrix
    drone_path_matrix = get_path_matrix(sol)[1:,:]
    print(f"Path Matrix:\n{drone_path_matrix}")
    # Get unique cell counts
    num_unique_cells = np.array([len(np.unique(row)) for row in drone_path_matrix])
    print(f"Number of Unique Cells:\n{num_unique_cells}")
    # Get drone id with the least unique cells (Most Hovering Drone)
    main_hovering_drone = np.argmin(num_unique_cells)
    hovering_drones = np.array( [i for i in range(len(num_unique_cells)) if num_unique_cells[i] < 10] )
    print(f"All Hovering Drone IDs: {hovering_drones}") # Use Later
    print(f"Main Hovering Drone ID: {main_hovering_drone}")
    # Calculate Main Hovering Drone's Connectivity to BS
    main_hovering_drone_path = drone_path_matrix[main_hovering_drone]
    print(f"Path Matrix:\n{drone_path_matrix}")
    print(f"Main Hovering Drone Path: {main_hovering_drone_path}")
    main_hovering_drone_unique_cells = np.unique(main_hovering_drone_path)
    for cell in main_hovering_drone_unique_cells:
        cell_to_bs_dist = info.D[-1,cell]
        if cell_to_bs_dist <= info.comm_cell_range * info.cell_side_length:
            relay_conn_score += 1 # Add 1 to relay_conn_score if all cells visited by the main hovering drone are connected to BS
        relay_dist_score += cell_to_bs_dist




# ISLAM'S DEFINITIONS

def calculate_connectivity_to_base_percentage_matrix(sol : PathSolution):

    info = sol.info

    connectivity_to_base= sol.connectivity_to_base_matrix

    time_slots = sol.time_slots

    connectivity_to_base_percentage = np.zeros(time_slots)

    for time in range(time_slots):
        connectivity_to_base_percentage[time] = sum(connectivity_to_base[time, 1:])/(info.number_of_drones)

    return connectivity_to_base_percentage


def calculate_percentage_connectivity(sol : PathSolution):

    info = sol.info

    connectivity_to_base_percentage = calculate_connectivity_to_base_percentage_matrix(sol)

    return -np.mean(connectivity_to_base_percentage[::1]) # sol.info.connectivity_period (after second :)

#####################################################################################################

def BFS(adj, sol:PathSolution):
      
    v = sol.info.Nd+1

    ctb = []
    start = 0
    # Visited vector to so that a
    # vertex is not visited more than
    # once Initializing the vector to
    # false as no vertex is visited at
    # the beginning
    visited = [False] * (sol.info.Nd+1)
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



# def calculate_percentage_connectivity(sol:PathSolution):
#     if not sol.percentage_connectivity :
#         num_connected_drones_to_base = connected_nodes(sol, 0)
#         sol.percentage_connectivity = 100 * sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * sol.info.number_of_drones)
#     return -sol.percentage_connectivity
#     # return (sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * info.Nd)) * 100

def calculate_total_maxDisconnectedTime(sol:PathSolution):
    if not sol.disconnected_time_steps:
        calculate_disconnected_timesteps(sol)
    sol.total_maxDisconnectedTime = sum(sol.max_disconnected_timesteps)
    return sol.total_maxDisconnectedTime

def calculate_max_maxDisconnectedTime(sol:PathSolution):
    if sol.disconnected_time_steps is not None:
    # if not sol.disconnected_time_steps:
        calculate_disconnected_timesteps(sol)
    sol.max_maxDisconnectedTime = max(sol.disconnected_time_steps)
    return sol.max_maxDisconnectedTime

def calculate_mean_maxDisconnectedTime(sol:PathSolution):
    if not sol.disconnected_time_steps:
        calculate_disconnected_timesteps(sol)
    sol.mean_maxDisconnectedTime = np.mean(sol.disconnected_time_steps)
    return sol.mean_maxDisconnectedTime

# --------------------------------------------------------------------------------------------------------------
# Functions below are used in calculating connectivity related objective functions and constraints written above
# --------------------------------------------------------------------------------------------------------------

def calculate_disconnected_timesteps(sol:PathSolution):

    # Finds the maximum disconnected timesteps for each drone

    info = sol.info

    time_steps = sol.real_time_path_matrix.shape[1]

    disconnected_timesteps_matrix = np.zeros((info.number_of_drones, sol.connectivity_matrix.shape[0]), dtype=int)

    drone_total_disconnected_timesteps = np.zeros(info.number_of_drones, dtype=int)

    for i in range(info.number_of_drones):
        # print(disconnected_timesteps_matrix[i].shape, connected_nodes(sol,i + 1))
        disconnected_timesteps_matrix[i] = connected_nodes(sol,i + 1)  # To account for skipping the base station # 0,1 , 1,2 ... 7,8
        drone_total_disconnected_timesteps[i] = len(np.where(disconnected_timesteps_matrix[i] == 0)[0])

    sol.disconnected_time_steps = drone_total_disconnected_timesteps

    return sol.disconnected_time_steps

    # sol.total_disconnected_timesteps = len(np.where(disconnected_timesteps_matrix == 0)[0])  # Total timesteps where a drone is disconnected
    # sol.max_disconnected_timesteps = max(drone_total_disconnected_timesteps)


def dfs(connectivity_matrix, node, visited, component):
    visited[node] = True
    component.append(node)
    for neighbor, connected in enumerate(connectivity_matrix[node]):
        if connected == 1 and not visited[neighbor]:
            dfs(connectivity_matrix, neighbor, visited, component)

def connected_components(connectivity_matrix):
    n = len(connectivity_matrix)
    visited = [False] * n
    components = []

    for node in range(n):
        if not visited[node]:
            component = []
            dfs(connectivity_matrix, node, visited, component)
            components.append(component)

    return components


def connected_nodes(sol:PathSolution, start_node):

    # start node: The node that we calculate connectivity to

    info = sol.info
    num_nodes = info.number_of_nodes

    num_connected_drones = np.zeros(sol.connectivity_matrix.shape[0], dtype=int)

    for i in range(sol.connectivity_matrix.shape[0]):

        connectivity_matrix = sol.connectivity_matrix[i]

        # num_nodes = len(connectivity_matrix)
        visited = [False] * num_nodes
        queue = deque([start_node])
        connected_count = 0

        # print(f"visited: {visited}")
        # print(f"start node: {start_node}")

        visited[start_node] = True  # Mark start node as visited

        while queue:
            node = queue.popleft()
            connected_count += 1

            for j in range(num_nodes):
                if connectivity_matrix[node][j] != 0 and not visited[j]:
                    queue.append(j)
                    visited[j] = True  # Mark the connected node as visited

        num_connected_drones[i] = connected_count - 1

        # print("---------------------------------------------------------")
        # print(f"step {i}")
        # print("---------------------------------------------------------")
        # print(f"Connectivity Matrix:\n{connectivity_matrix}")
        # print(f"Number of nodes connected to node {start_node}: {num_connected_drones[i]}")

    return num_connected_drones

    # return connected_count - 1  # Subtract 1 because start node is included in count