from statistics import median, median_low, median_high
from PathAlgorithm import *
from PathOutput import *
from pymoo.operators.crossover.nox import NoCrossover
from pymoo.operators.mutation.nom import NoMutation
from pymoo.core.duplicate import NoDuplicateElimination
from pymoo.optimize import minimize
from PathResults import export_to_matlab

def save_to_file(verbose_output, filename):
    with open(filename, "w") as f:
        f.write(verbose_output)

matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/PathResults'

algorithm_list = ['NSGA2','NSGA3']
number_of_drones_list = [4] # 8
r_comm_list = [4]
min_visits_list = [1] # 4,5
hovering_states = [True]
realtime_connectivity_states = [False]

for algorithm in algorithm_list:
    for hovering in hovering_states:
        # info.hovering = hovering
        for realtime_connectivity in realtime_connectivity_states:
            # info.realtime_connectivity = realtime_connectivity
            for min_visits in min_visits_list:
                # info.Nd = number_of_drones
                for r_comm in r_comm_list:
                    # info.rc = r_comm
                    for number_of_drones in number_of_drones_list:

                        # info.min_visits = min_visits
                        info = PathInfo(hovering=hovering, realtime_connectivity=realtime_connectivity, Nd=number_of_drones, rc=r_comm, min_visits=min_visits)

                        scenario = f"alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}"

                        print(f"Algorithm: {algorithm}, Hovering: {info.hovering}, Realtime Connectivity: {info.realtime_connectivity}, Number of Drones: {info.Nd}, Number of Nodes: {info.Nn}, Communication Range: {info.rc}, Min Visits: {info.min_visits}")

                        t = time.time()

                        res = minimize(problem=PathProblem(info),
                                      algorithm=algorithm_dict[algorithm],
                                      termination=('n_gen',4000),
                                      seed=1,
                                      # output=PathOutput(PathProblem(info)),
                                      verbose=True,
                                      # termination=path_termination
                                      )

                        print(res)

                        elapsed_time = time.time() - t

                        X = res.X
                        F = res.F
                        print('F:',abs(F))

                        print(f"Elapsed time: {round(elapsed_time/60)} minutes")

                        # Save verbose
                        # df = pd.DataFrame(res)
                        # df.to_excel(f"Results/Verbose/{scenario}_verbose.txt", index=False)
                        # np.save(f"{scenario}_Pop",[individual._X[0]  for individual in res.pop])
                        # np.save(f"{scenario}_Opt",[individual._X[0]  for individual in res.opt])
                        # print(f"X: {[individual._X[0]  for individual in res.pop]}")
                        # print(f"opt: {[individual._X[0]  for individual in res.opt]}")
                        # save_to_file(res, f"Results/Verbose/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_verbose.txt")

                        # Save Solution Objects and Objective Values
                        np.save(f"Results/X/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_SolutionObjects",X)
                        np.save(f"Results/F/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_ObjectiveValues",F)
                        np.save(f"Results/Time/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_Runtime",elapsed_time)

                        sols = np.load(f'Results/X/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_SolutionObjects.npy',allow_pickle=True)
                        objs = np.load(f'Results/F/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_ObjectiveValues.npy',allow_pickle=True)

                        dist_values, conn_values, maxDisconn_values, timePenalty_values = objs.transpose().tolist()

                        export_to_matlab(matlab_filepath, scenario, sols, "Dist", dist_values, inv_rel=False)
                        export_to_matlab(matlab_filepath, scenario, sols, "Conn", conn_values, inv_rel=True)
                        export_to_matlab(matlab_filepath, scenario, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)
                        export_to_matlab(matlab_filepath, scenario, sols, "DistPenalties", timePenalty_values, inv_rel=False)

'''
# Export Distance Related Solutions to MATLAB
dist_values = objs[:, 0].tolist()
max_dist_idx = dist_values.index(max(dist_values))
min_dist_idx = dist_values.index(min(dist_values))
mid_dist_idx = dist_values.index(median_low(dist_values))
max_dist_sol = sols[max_dist_idx][0]
min_dist_sol = sols[min_dist_idx][0]
mid_dist_sol = sols[mid_dist_idx][0]
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-x_matrix.mat',
    {'array': max_dist_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-y_matrix.mat',
    {'array': max_dist_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-x_matrix.mat',
    {'array': min_dist_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-y_matrix.mat',
    {'array': min_dist_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-x_matrix.mat',
    {'array': mid_dist_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-y_matrix.mat',
    {'array': mid_dist_sol.y_matrix})

# Export Subtour Related Solutions to MATLAB
subtour_values = objs[:, 1].tolist()
max_subtour_idx = subtour_values.index(max(subtour_values))
min_subtour_idx = subtour_values.index(min(subtour_values))
mid_subtour_idx = subtour_values.index(median_low(subtour_values))
max_subtour_sol = sols[max_subtour_idx][0]
min_subtour_sol = sols[min_subtour_idx][0]
mid_subtour_sol = sols[mid_subtour_idx][0]
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxSubtour-x_matrix.mat',
    {'array': max_subtour_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxSubtour-y_matrix.mat',
    {'array': max_subtour_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinSubtour-x_matrix.mat',
    {'array': min_subtour_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinSubtour-y_matrix.mat',
    {'array': min_subtour_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidSubtour-x_matrix.mat',
    {'array': mid_subtour_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidSubtour-y_matrix.mat',
    {'array': mid_subtour_sol.y_matrix})

# Export Connectivity Related Solutions to MATLAB
conn_values = objs[:, 2].tolist()
max_conn_idx = conn_values.index(min(conn_values))
min_conn_idx = conn_values.index(max(conn_values))
mid_conn_idx = conn_values.index(median_low(conn_values))
max_conn_sol = sols[max_conn_idx][0]
min_conn_sol = sols[min_conn_idx][0]
mid_conn_sol = sols[mid_conn_idx][0]
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxConn-x_matrix.mat',
    {'array': max_conn_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxConn-y_matrix.mat',
    {'array': max_conn_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinConn-x_matrix.mat',
    {'array': min_conn_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinConn-y_matrix.mat',
    {'array': min_conn_sol.y_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidConn-x_matrix.mat',
    {'array': mid_conn_sol.x_matrix})
io.savemat(
    f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidConn-y_matrix.mat',
    {'array': mid_conn_sol.y_matrix})
'''