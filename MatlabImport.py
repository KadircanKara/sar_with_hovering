from scipy import io
import numpy as np
from statistics import median_low

matlab_path = '/Users/kadircan/Documents/MATLAB/Thesis/'

scenarios = ['alg_NSGA2_hovering_True_realtimeConnectivityCalculation_False_n_64_Ns_8_comm_2_nvisits_1']

for scenario in scenarios:

    sols = np.load(f'Results/X/{scenario}_SolutionObjects.npy',allow_pickle=True)
    objs = np.load(f'Results/F/{scenario}_ObjectiveValues.npy',allow_pickle=True)

    # Export Distance Related Solutions to MATLAB
    dist_values = objs[:, 0].tolist()
    max_dist_idx = dist_values.index(max(dist_values))
    min_dist_idx = dist_values.index(min(dist_values))
    mid_dist_idx = dist_values.index(median_low(dist_values))
    max_dist_sol = sols[max_dist_idx][0]
    min_dist_sol = sols[min_dist_idx][0]
    mid_dist_sol = sols[mid_dist_idx][0]
    io.savemat(matlab_path + f'{scenario}-MaxDist-x_matrix.mat',{'array': max_dist_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MaxDist-y_matrix.mat',{'array': max_dist_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MinDist-x_matrix.mat',{'array': min_dist_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MinDist-y_matrix.mat',{'array': min_dist_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MidDist-x_matrix.mat',{'array': mid_dist_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MidDist-y_matrix.mat',{'array': mid_dist_sol.y_matrix})

    # Export Subtour Related Solutions to MATLAB
    subtour_values = objs[:, 1].tolist()
    max_subtour_idx = subtour_values.index(max(subtour_values))
    min_subtour_idx = subtour_values.index(min(subtour_values))
    mid_subtour_idx = subtour_values.index(median_low(subtour_values))
    max_subtour_sol = sols[max_subtour_idx][0]
    min_subtour_sol = sols[min_subtour_idx][0]
    mid_subtour_sol = sols[mid_subtour_idx][0]
    io.savemat(matlab_path + f'{scenario}-MaxSubtour-x_matrix.mat',{'array': max_subtour_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MaxSubtour-y_matrix.mat',{'array': max_subtour_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MinSubtour-x_matrix.mat',{'array': min_subtour_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MinSubtour-y_matrix.mat',{'array': min_subtour_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MidSubtour-x_matrix.mat',{'array': mid_subtour_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MidSubtour-y_matrix.mat',{'array': mid_subtour_sol.y_matrix})

    # Export Connectivity Related Solutions to MATLAB
    conn_values = objs[:, 2].tolist()
    max_conn_idx = conn_values.index(min(conn_values))
    min_conn_idx = conn_values.index(max(conn_values))
    mid_conn_idx = conn_values.index(median_low(conn_values))
    max_conn_sol = sols[max_conn_idx][0]
    min_conn_sol = sols[min_conn_idx][0]
    mid_conn_sol = sols[mid_conn_idx][0]
    io.savemat(matlab_path + f'{scenario}-MaxConn-x_matrix.mat',{'array': max_conn_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MaxConn-y_matrix.mat',{'array': max_conn_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MinConn-x_matrix.mat',{'array': min_conn_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MinConn-y_matrix.mat',{'array': min_conn_sol.y_matrix})
    io.savemat(matlab_path + f'{scenario}-MidConn-x_matrix.mat',{'array': mid_conn_sol.x_matrix})
    io.savemat(matlab_path + f'{scenario}-MidConn-y_matrix.mat',{'array': mid_conn_sol.y_matrix})
