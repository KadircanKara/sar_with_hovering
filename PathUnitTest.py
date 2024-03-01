import time
from statistics import median, median_low, median_high
import pickle
from scipy import io
import numpy as np

from PathInfo import *
from PathProblem import PathProblem
from PathSampling import PathSampling
from PathSolution import *
# from PathRepair import *
from PathAlgorithm import *
# from PathResult import *
# from PathOutput import *


from pymoo.operators.crossover.nox import NoCrossover
from pymoo.operators.mutation.nom import NoMutation
from pymoo.core.duplicate import NoDuplicateElimination


from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.optimize import minimize

from pymoo.termination.default import DefaultMultiObjectiveTermination

import matplotlib.pyplot as plt
import pandas as pd

# Hovering yazılacak
# Hovering olmazsa disconnecitivty min. edilebilir. (Bağlanıp çıkıp bağlanıp çıkabilir)
# BS 0. cell'e çekilecek.

# import tkinter
# import fractions as f

# root   = tkinter.Tk()

# width  = root.winfo_screenwidth()
# height = root.winfo_screenheight()
# frac   = f.Fraction(width,height)

# print ("WIDTH-HEIGHT:",width, height)

# if width==1280 and height==800 :
#     {"terminal.integrated.fontSize": 12,
#     "editor.fontSize": 12}
# else :
#     {"terminal.integrated.fontSize": 16,
#     "editor.fontSize": 16}


# -------------    AUTO    -----------------------------------

# class PathUnitTest:
#
#     def __init__(self, alg_list=None, setups=None):
#
#         # self.path = "/Users/kadircan/Documents/MATLAB/Thesis/"
#         self.alg_list = alg_list if alg_list else ['NSGA2']
#         self.setups = setups if setups else [PathInfo()]
#         self.algorithm_list = []
#         self.result = dict()
#         self.result["alg_name"] = []
#         self.result["res_objs"] = []
#         self.result["info"] = []
#
#         for alg in self.alg_list:
#             self.algorithm_list.append(algorithm_dict[alg])
#
#     def ExecuteUnitTest(self):
#
#         for algorithm in self.algorithm_list:
#             for setup in self.setups:
#                 res = minimize(PathProblem(setup),
#                        algorithm,
#                        termination=('n_gen', 10000),
#                        seed=1,
#                        # output=PathOutput(),
#                        verbose=True,
#                        # termination=path_termination
#                        )
#                 self.result["alg_name"].append(self.alg_list[self.algorithm_list.index(algorithm)])
#                 self.result["res_objs"].append(res)
#                 self.result["info"].append(setup)
#
#
#     def SaveResults(self, path):
#
#         self.path = path if path else "/Users/kadircan/Documents/MATLAB/Thesis/"
#
#         for i in range(len(self.result["res_objs"])):
#
#             res = self.result["res_objs"][i]
#             alg_name = self.result["alg_name"][i]
#
#             X = res.X
#             F = res.F
#             print('F:', abs(F))
#
#             min_obj_values = []
#             min_obj_indexes = []
#             endpoint_sols = []
#
#             for i in range(F.shape[1]):
#
#                 min_obj_values.append(abs(min(F[:, i])))
#                 min_obj_indexes.append(np.where(F[:, i] == min(F[:, i]))[0][0])
#                 endpoint_sols.append(X[min_obj_indexes[-1], 0])
#
#                 sol = endpoint_sols[-1]
#                 info = sol.info
#
#                 io.savemat(f"{self.path}{alg_name}_Ns_{info.number_of_drones}_comm_{info.r_comm}_nvisits_{info.max_visits}_path_matrix-{F_descriptions[i]}.mat",{'array': sol.path_matrix})
#                 io.savemat(f"{self.path}{alg_name}_Ns_{info.number_of_drones}_comm_{info.r_comm}_nvisits_{info.max_visits}_x_coords-{F_descriptions[i]}.mat",{'array': sol.x_coords_matrix})
#                 io.savemat(f"{self.path}{alg_name}_Ns_{info.number_of_drones}_comm_{info.r_comm}_nvisits_{info.max_visits}_y_coords-{F_descriptions[i]}.mat",{'array': sol.y_coords_matrix})
#
#
#     def __call__(self, *args, **kwargs):
#
#         self.ExecuteUnitTest()
#         self.SaveResults()
#
#
# alg_list = ['NSGA2','NSGA3']
# setup = PathInfo()
#
# number_of_drones = [8,12]
# r_comm = [100*sqrt(2),200*sqrt(2)]
#
# for Ns in number_of_drones:
#     setup.number_of_drones = Ns
#     for comm_dist in r_comm:
#         setup.r_comm = comm_dist
#
#         unittest = PathUnitTest(alg_list=alg_list, setups=[setup])
#         unittest()




# ------------  MANUAL  -------------------------

path_termination = DefaultMultiObjectiveTermination(
    # xtol=1e-8,
    # cvtol=0,    # 1e-6,
    # ftol=0.01, #0.0025,
    # # period=30,
    n_max_gen=10000,
    # n_max_evals=100000
)

matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/'

algorithm_list = ['NSGA2']
number_of_drones_list = [8]
r_comm_list = [2]
min_visits_list = [4,5]
hovering_states = [False]
realtime_connectivity_states = [False]

for algorithm in algorithm_list:
    for hovering in hovering_states:
        # info.hovering = hovering
        for realtime_connectivity in realtime_connectivity_states:
            # info.realtime_connectivity = realtime_connectivity
            for number_of_drones in number_of_drones_list:
                # info.Nd = number_of_drones
                for r_comm in r_comm_list:
                    # info.rc = r_comm
                    for min_visits in min_visits_list:
                        # info.min_visits = min_visits
                        info = PathInfo(hovering=hovering, realtime_connectivity=realtime_connectivity, Nd=number_of_drones, rc=r_comm, min_visits=min_visits)

                        print(f"Algorithm: {algorithm}, Hovering: {info.hovering}, Realtime Connectivity: {info.realtime_connectivity}, Number of Drones: {info.Nd}, Number of Nodes: {info.Nn}, Communication Range: {info.rc}, Min Visits: {info.min_visits}")

                        t = time.time()

                        res = minimize(problem=PathProblem(info),
                                      algorithm=algorithm_dict[algorithm],
                                      termination=('n_gen',4000),
                                      seed=1,
                                      # output=PathOutput(),
                                      verbose=True,
                                      # termination=path_termination
                                      )

                        elapsed_time = time.time() - t

                        X = res.X
                        F = res.F
                        print('F:',abs(F))

                        print(f"Elapsed time: {round(elapsed_time/60)} minutes")

                        # Save Solution Objects and Objective Values
                        np.save(f"Results/X/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_SolutionObjects",X)
                        np.save(f"Results/F/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_ObjectiveValues",F)
                        np.save(f"Results/Time/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_Runtime",elapsed_time)

                        sols = np.load(f'Results/X/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_SolutionObjects.npy',allow_pickle=True)
                        objs = np.load(f'Results/F/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_ObjectiveValues.npy',allow_pickle=True)

                        # Export Distance Related Solutions to MATLAB
                        dist_values = objs[:,0].tolist()
                        max_dist_idx = dist_values.index(max(dist_values))
                        min_dist_idx = dist_values.index(min(dist_values))
                        mid_dist_idx = dist_values.index(median_low(dist_values))
                        max_dist_sol = sols[max_dist_idx][0]
                        min_dist_sol = sols[min_dist_idx][0]
                        mid_dist_sol = sols[mid_dist_idx][0]
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-x_matrix.mat',{'array': max_dist_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-y_matrix.mat',{'array': max_dist_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-x_matrix.mat',{'array': min_dist_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-y_matrix.mat',{'array': min_dist_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-x_matrix.mat',{'array': mid_dist_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-y_matrix.mat',{'array': mid_dist_sol.y_matrix})

                        # Export Subtour Related Solutions to MATLAB
                        subtour_values = objs[:,1].tolist()
                        max_subtour_idx = subtour_values.index(max(subtour_values))
                        min_subtour_idx = subtour_values.index(min(subtour_values))
                        mid_subtour_idx = subtour_values.index(median_low(subtour_values))
                        max_subtour_sol = sols[max_subtour_idx][0]
                        min_subtour_sol = sols[min_subtour_idx][0]
                        mid_subtour_sol = sols[mid_subtour_idx][0]
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxSubtour-x_matrix.mat',{'array': max_subtour_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxSubtour-y_matrix.mat',{'array': max_subtour_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinSubtour-x_matrix.mat',{'array': min_subtour_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinSubtour-y_matrix.mat',{'array': min_subtour_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidSubtour-x_matrix.mat',{'array': mid_subtour_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidSubtour-y_matrix.mat',{'array': mid_subtour_sol.y_matrix})

                        # Export Connectivity Related Solutions to MATLAB
                        conn_values = objs[:, 2].tolist()
                        max_conn_idx = conn_values.index(min(conn_values))
                        min_conn_idx = conn_values.index(max(conn_values))
                        mid_conn_idx = conn_values.index(median_low(conn_values))
                        max_conn_sol = sols[max_conn_idx][0]
                        min_conn_sol = sols[min_conn_idx][0]
                        mid_conn_sol = sols[mid_conn_idx][0]
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxConn-x_matrix.mat',{'array': max_conn_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxConn-y_matrix.mat',{'array': max_conn_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinConn-x_matrix.mat',{'array': min_conn_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinConn-y_matrix.mat',{'array': min_conn_sol.y_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidConn-x_matrix.mat',{'array': mid_conn_sol.x_matrix})
                        io.savemat(f'{matlab_filepath}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidConn-y_matrix.mat',{'array': mid_conn_sol.y_matrix})


def export_to_matlab(path, algorithm, sols, name, values, inv_prop):

    if path[-1] != '/':
        path += '/'

    info = sols[0][0].info
    scenario = f'alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}'

    max_idx = values.index(max(dist_values))
    min_idx = dist_values.index(min(dist_values))
    mid_idx = dist_values.index(median_low(dist_values))

    if inv_prop:
        min_idx, max_idx = max_idx, min_idx

    max_sol = sols[max_dist_idx][0]
    min_sol = sols[min_dist_idx][0]
    mid_sol = sols[mid_dist_idx][0]

    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-x_matrix.mat',{'array': max_dist_sol.x_matrix})
    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MaxDist-y_matrix.mat',{'array': max_dist_sol.y_matrix})
    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-x_matrix.mat',{'array': min_dist_sol.x_matrix})
    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MinDist-y_matrix.mat',{'array': min_dist_sol.y_matrix})
    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-x_matrix.mat',{'array': mid_dist_sol.x_matrix})
    io.savemat(f'{path}alg_{algorithm}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}-MidDist-y_matrix.mat',{'array': mid_dist_sol.y_matrix})
