from statistics import median_low
from scipy import io

# matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/PathResults'

def export_to_matlab(path, scenario, sols, obj_name, obj_values, inv_rel=False):

    if path[-1] != '/':
        path += '/'

    info = sols[0][0].info
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

    io.savemat(f'{path}{scenario}-Max{obj_name}-x_matrix.mat',{'array': max_sol.x_matrix})
    io.savemat(f'{path}{scenario}-Max{obj_name}-y_matrix.mat',{'array': max_sol.y_matrix})

    io.savemat(f'{path}{scenario}-Min{obj_name}-x_matrix.mat',{'array': min_sol.x_matrix})
    io.savemat(f'{path}{scenario}-Min{obj_name}-y_matrix.mat',{'array': min_sol.y_matrix})

    io.savemat(f'{path}{scenario}-Mid{obj_name}-x_matrix.mat',{'array': mid_sol.x_matrix})
    io.savemat(f'{path}{scenario}-Mid{obj_name}-y_matrix.mat',{'array': mid_sol.y_matrix})