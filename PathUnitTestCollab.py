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

model_list = [moo_model,distance_soo_model]
# algorithm_list = ['NSGA2','NSGA3']
number_of_drones_list = [12,16] # 8
r_comm_list = [2,2*sqrt(2),4]
min_visits_list = [1,2,3,4,5] # 4,5
hovering_states = [True]
realtime_connectivity_states = [True]

for model in model_list:
    for alg in model['Alg']:
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

                            scenario = f"{model['Exp']}_Opt_alg_{alg}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}"

                            print(f"Opt: {model['Exp']}, Algorithm: {alg}, Hovering: {info.hovering}, Realtime Connectivity: {info.realtime_connectivity}, Number of Drones: {info.Nd}, Number of Nodes: {info.Nn}, Communication Range: {info.rc}, Min Visits: {info.min_visits}")

                            t = time.time()

                            res = minimize(problem=PathProblem(info,model=model),
                                          algorithm=algorithm_dict[alg],
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
                            np.save(f"Results/X/{scenario}_SolutionObjects",X)
                            np.save(f"Results/F/{scenario}_ObjectiveValues",F)
                            np.save(f"Results/Time/{scenario}_Runtime",elapsed_time)

                            sols = np.load(f"Results/X/{scenario}_SolutionObjects.npy",allow_pickle=True)
                            objs = np.load(f"Results/F/{scenario}_ObjectiveValues.npy",allow_pickle=True)

                            dist_values, conn_values, maxDisconn_values, timePenalty_values = objs.transpose().tolist()

                            export_to_matlab(matlab_filepath, scenario, sols, "Dist", dist_values, inv_rel=False)
                            export_to_matlab(matlab_filepath, scenario, sols, "Conn", conn_values, inv_rel=True)
                            export_to_matlab(matlab_filepath, scenario, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)
                            export_to_matlab(matlab_filepath, scenario, sols, "DistPenalties", timePenalty_values, inv_rel=False)