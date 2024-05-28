import os
os.system('clear')
import numpy as np
import pandas as pd
from scipy import io
from PathOptimizationModel import moo_model, distance_soo_model
from PathSolution import PathSolution
from Time import get_real_paths
from Connectivity import *

matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/ResultsTest'

model = moo_model

number_of_drones = 8
cell_comm_range = 4
min_visits = 1

# Get All Solutions and Objectives
scenario = f'g_8_a_50_n_{number_of_drones}_v_2.5_r_{cell_comm_range}_minv_{min_visits}_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0'
sols = np.load(f'Results/X/{scenario}_SolutionObjects.npy', allow_pickle=True)
objs = np.load(f'Results/F/{scenario}_ObjectiveValues.npy', allow_pickle=True)
objs = pd.DataFrame(objs,columns=model['F'])

print(objs)

# Extract Sol with Best Connectivity
best_conn_idx = objs['Percentage Connectivity'].idxmin()
best_mean_disconn_idx = objs['Mean Disconnected Time'].idxmin()
best_max_disconn_idx = objs['Max Disconnected Time'].idxmin()
best_distance_idx = objs['Total Distance'].idxmin()

conn_sol:PathSolution = sols[best_conn_idx][0]
mean_disconn_sol:PathSolution = sols[best_mean_disconn_idx][0]
max_disconn_sol:PathSolution = sols[best_max_disconn_idx][0]
distance_sol:PathSolution = sols[best_distance_idx][0]

enforce_hovering_connectivity(conn_sol)