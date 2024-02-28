from pymoo.core.sampling import Sampling
import numpy as np
from math import floor

from PathProblem import *

'''
TODO

Redo the path sampling i.e. start at -1. cell return to -1.cell (start and end the tours at BS)
Visit each cell (0.-63.) at least "min_visits" times at most "max_visits" times

UPDATES tick
DONE not yet

'''

class PathSampling(Sampling):

    def __init__(self) -> None:
        super().__init__()

    def _do(self, problem: PathProblem, n_samples, **kwargs):


        # X = np.full((n_samples, problem.n_var), 0, dtype=int)
        X = np.full((n_samples, 1), None, dtype=PathSolution)

        for i in range(n_samples):

            # Create a cell list (include every cell max_visits times)
            # Return PathSolution(chromosome, problem.info)
            # Calculate drone dict, connectivity, etc. in PathSolution.py script

            # path = np.random.permutation(problem.info.nvisits_th * problem.info.Nc).tolist()
            # for j in range(problem.info.nvisits_th):
            #     path.remove(j * problem.info.Nc) # Remove cell 0

            # path = []
            # for j in range(problem.info.min_visits):
            #     # new_seq = np.random.permutation(np.arange(1,problem.info.Nc)).tolist()
            #     new_seq = np.random.permutation(problem.info.Nc).tolist()
            #     path.extend(new_seq)

            path = np.random.permutation(problem.info.min_visits * problem.info.Nc).tolist()
            # for j in range(problem.info.min_visits):
                # path.remove(j * problem.info.Nc)  # Remove cell 0

            start_points = []
            cpd = problem.info.Nc * problem.info.min_visits // problem.info.Nd  # Cells per Drone
            cpd_bias = 2  # Cells per Drone Bias
            for d in range(problem.info.Nd - 1):
                if d == 0:
                    start_points.append(random.randrange(cpd - cpd_bias, cpd + cpd_bias))  # Generate random index
                else:
                    start_points.append(start_points[-1] + random.randrange(cpd - cpd_bias, cpd + cpd_bias))
            start_points.insert(0, 0)

            # print(f"Path: {path}\nStart Points: {start_points}")
            # print("----------------------------------------------------------------------------------------------------")
            # print(f"Sample {i}")
            # print("----------------------------------------------------------------------------------------------------")
            # print(f"Path: {path}\nStart Points: {start_points}")

            X[i, :] = PathSolution(path, start_points, problem.info)

        return X


'''        for i in range(n_samples):

            print("--------------------------------------------")
            print(f"Sample {i} :\n")

            drone_dict = {k:[] for k in range(problem.info.number_of_drones)}
            cells_per_drone = problem.info.number_of_cities // problem.info.number_of_drones
            random_cell_sequence = []

            for _ in range(problem.info.max_visits):
                cell_sequence = np.random.permutation(problem.info.number_of_cities-1)+1
                print(f"Cell Sequence {_}: {cell_sequence}")
                for drone_no in range(problem.info.number_of_drones):
                    start = drone_no*cells_per_drone
                    if drone_no < problem.info.number_of_drones - 1 :
                        end = (drone_no+1)*cells_per_drone
                    else :
                        end = len(cell_sequence)
                    drone_dict[drone_no].extend(cell_sequence[start:end])
                    if _ == problem.info.max_visits-1 :
                        drone_dict[drone_no] = np.hstack((np.array([-1,0]) , drone_dict[drone_no] , np.array([0,-1]))).tolist()


            # for drone_no in range(problem.info.number_of_drones):
            #     drone_dict[drone_no] = []
            #     for visit_no in range(problem.info.max_visits):
            #         cell_sequence = np.random.permutation(problem.info.number_of_cities)
            #         # print(f"Cell Sequence {visit_no}: {cell_sequence}")
            #         start = drone_no*cells_per_drone
            #         if drone_no < problem.info.number_of_drones - 1 :
            #             end = (drone_no+1)*cells_per_drone
            #         else :
            #             end = len(cell_sequence)
            #         drone_dict[drone_no].extend(cell_sequence[start:end])

            # for visit_no in range(problem.info.max_visits):

            #     random_cell_sequence.append(np.random.permutation(problem.info.number_of_cities))
            #     current_sequence = random_cell_sequence[-1]

            #     for drone in range(problem.info.number_of_drones-1):
            #         drone_dict[i] = random_cell_sequence[]





            # start_points = np.hstack(( np.array([0]) , np.random.choice(range(1,problem.info.number_of_cities), problem.info.number_of_drones-1, replace=False) ))
            # start_points.sort()
            # start_points = start_points[np.newaxis, :]

            # path = np.random.permutation(problem.info.number_of_cities)
            # path = path[np.newaxis, :]

            # for i in range(problem.info.max_visits - 1):
            #     path = np.vstack((path, np.random.permutation(problem.info.number_of_cities)))
            #     start_points = np.vstack((start_points, np.hstack(( np.array([0]) , np.random.choice(range(1,problem.info.number_of_cities), problem.info.number_of_drones-1, replace=False) ))))


            print(f"Drone Dict: {drone_dict}\n")
            X[i, :] = PathSolution(drone_dict, problem.info)

        # print('Path:', X[0].path)
        # print('Start Points:', X[0].start_points)

        return X
'''