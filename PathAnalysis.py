import pandas as pd

from PathAnimation import *


scenario = "alg_NSGA2_n_64_Ns_16_comm_2_nvisits_5"
solutions = np.load(f"Results/X/{scenario}_SolutionObjects.npy",allow_pickle=True)

sol = solutions[0][0]

path_matrix = sol.path_matrix
drone_dict = sol.drone_dict
path = sol.path
start_points = sol.start_points

print(f"Drone Dict: {drone_dict}")
print(f"Path: {path}")
print(f"Start Points: {start_points}")

print(pd.DataFrame(path_matrix).to_string(index=False))

PathAnimation(sol=sol)