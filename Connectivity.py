import numpy as np
from collections import deque

def percentage_connectivity(sol):
    num_connected_drones_to_base = connected_nodes(sol, 0)
    sol.percentage_connectivity = 100 * sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * sol.info.Nd)
    return sol.percentage_connectivity
    # return (sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * info.Nd)) * 100


def max_disconnected_time(sol):
    sol.max_maxDisconnectedTime = max(sol.disconnected_time_steps)
    return sol.max_maxDisconnectedTime

def mean_disconnected_time(sol):
    sol.mean_maxDisconnectedTime = np.mean(sol.disconnected_time_steps)
    return sol.mean_maxDisconnectedTime


# def calculate_disconnected_timesteps(sol):
#     # Finds the maximum disconnected time
#
#     info = sol.info
#
#     time_steps = sol.path_matrix.shape[1]
#
#     disconnected_timesteps_matrix = np.zeros((info.Nd, time_steps), dtype=int)
#
#     drone_total_disconnected_timesteps = np.zeros(info.Nd, dtype=int)
#
#     for i in range(info.Nd):
#         disconnected_timesteps_matrix[i] = connected_nodes(sol,i + 1)  # To account for skipping the base station # 0,1 , 1,2 ... 7,8
#         drone_total_disconnected_timesteps[i] = len(np.where(disconnected_timesteps_matrix[i] == 0)[0])
#
#     sol.total_disconnected_timesteps = len(np.where(disconnected_timesteps_matrix == 0)[0])  # Total timesteps where a drone is disconnected
#     sol.max_disconnected_timesteps = max(drone_total_disconnected_timesteps)


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


def connected_nodes(sol, start_node):

    # start node: The node that we calculate connectivity to

    info = sol.info
    num_nodes = info.Nn

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