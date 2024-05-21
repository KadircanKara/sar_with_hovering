from statistics import median_low
from scipy import io
from Time import *
from PathOptimizationModel import *


def export_to_matlab(path, model, scenario, obj_name, obj_values):

    obj_name.replace(" ","_")

    sols = np.load(f"Results/X/{scenario}_SolutionObjects.npy", allow_pickle=True)

    if path[-1] != '/':
        path += '/'

    if model == distance_soo_model:
        sol = sols[0]
        info = sol.info
        xs, ys, start_points = get_cartesion_drone_path(sol)
        real_x_path, real_y_path = get_real_real_path(xs, ys, start_points)
        x_matrix, y_matrix = get_real_paths(sol)
        io.savemat(f'{path}{scenario}-x_matrix.mat',{'array': x_matrix})
        io.savemat(f'{path}{scenario}-y_matrix.mat',{'array': y_matrix})
    else:
        info = sols[0][0].info
        best_idx = obj_values.index(min(obj_values))
        worst_idx = obj_values.index(max(obj_values))
        mid_idx = obj_values.index(median_low(obj_values))
        best_sol = sols[best_idx][0]
        worst_sol = sols[worst_idx][0]
        mid_sol = sols[mid_idx][0]

        xs, ys, start_points = get_cartesion_drone_path(best_sol)
        real_x_path, real_y_path = get_real_real_path(xs, ys, start_points)
        x_matrix, y_matrix = get_real_paths(best_sol)
        io.savemat(f'{path}{scenario}-Best{obj_name}-x_matrix.mat',{'array': x_matrix})
        io.savemat(f'{path}{scenario}-Best{obj_name}-y_matrix.mat',{'array': y_matrix})

        xs, ys, start_points = get_cartesion_drone_path(worst_sol)
        real_x_path, real_y_path = get_real_real_path(xs, ys, start_points)
        x_matrix, y_matrix = get_real_paths(worst_sol)
        io.savemat(f'{path}{scenario}-Worst{obj_name}-x_matrix.mat',{'array': x_matrix})
        io.savemat(f'{path}{scenario}-Worst{obj_name}-y_matrix.mat',{'array': y_matrix})

        xs, ys, start_points = get_cartesion_drone_path(mid_sol)
        real_x_path, real_y_path = get_real_real_path(xs, ys, start_points)
        x_matrix, y_matrix = get_real_paths(mid_sol)
        io.savemat(f'{path}{scenario}-Mid{obj_name}-x_matrix.mat',{'array': x_matrix})
        io.savemat(f'{path}{scenario}-Mid{obj_name}-y_matrix.mat',{'array': y_matrix})

path = '/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults'
model = moo_model
scenario = 'g_8_a_50_n_4_v_2.5_r_4_minv_5_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0'
obj_name = "Percentage Connectivity"
obj_values = np.load(f"Results/F/{scenario}_ObjectiveValues.npy", allow_pickle=True)[:,1].tolist()

export_to_matlab(path, model, scenario, obj_name, obj_values)

print("DONE !!!")









'''        
        # scenario = f'alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}'
        max_idx = obj_values.index(max(obj_values))
        min_idx = obj_values.index(min(obj_values))
        mid_idx = obj_values.index(median_low(obj_values))
        # if inv_rel:
        #     min_idx, max_idx = max_idx, min_idx
        max_sol = sols[max_idx][0]
        min_sol = sols[min_idx][0]
        mid_sol = sols[mid_idx][0]
        if inv_rel:
            max_sol, min_sol = min_sol, max_sol
        get_cartesian_coords(max_sol)
        get_cartesian_coords(min_sol)
        get_cartesian_coords(mid_sol)
        io.savemat(f'{path}{scenario}-Max{obj_name}-x_matrix.mat',{'array': max_sol.x_matrix})
        io.savemat(f'{path}{scenario}-Max{obj_name}-y_matrix.mat',{'array': max_sol.y_matrix})
        io.savemat(f'{path}{scenario}-Min{obj_name}-x_matrix.mat',{'array': min_sol.x_matrix})
        io.savemat(f'{path}{scenario}-Min{obj_name}-y_matrix.mat',{'array': min_sol.y_matrix})
        io.savemat(f'{path}{scenario}-Mid{obj_name}-x_matrix.mat',{'array': mid_sol.x_matrix})
        io.savemat(f'{path}{scenario}-Mid{obj_name}-y_matrix.mat',{'array': mid_sol.y_matrix})
'''