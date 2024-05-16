from PathSolution import *
from Time import *
from scipy.io import savemat

sol = np.load("Results/X/Dist_Opt_alg_GA_hovering_True_realtimeConnectivityCalculation_False_n_64_Ns_4_comm_4_nvisits_1_SolutionObjects.npy",allow_pickle=True)[0]
sol.real_time_path_matrix = np.hstack((sol.real_time_path_matrix, np.zeros((sol.real_time_path_matrix.shape[0],1))-1))
# get_cartesian_coords(sol)
print(sol.real_time_path_matrix)
# print(sol.x_matrix)
# print(sol.y_matrix)

'''info = PathInfo(min_visits=1)
path = np.random.permutation(info.min_visits * info.Nc).tolist()
# Random start points
start_points = sorted(random.sample([i for i in range(1, len(path))], info.Nd - 1))
start_points.insert(0, 0)

sol = PathSolution(path, start_points, info)

get_cartesian_coords(sol)

savemat("/Users/kadircan/Documents/MATLAB/Thesis/PathResults/sample_sol_with_hovering_x_matrix.mat",{'array':sol.x_matrix})
savemat("/Users/kadircan/Documents/MATLAB/Thesis/PathResults/sample_sol_with_hovering_y_matrix.mat",{'array':sol.y_matrix})

print(sol.x_matrix)
print(sol.y_matrix)
'''
