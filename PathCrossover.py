# Comprehensive Sequential Constructive Crossover (CSCX)

import numpy as np

from PathSolution import *

from pymoo.core.crossover import Crossover
from pymoo.core.variable import Real, get


def random_sequence(n):
    start, end = np.sort(np.random.choice(n, 2, replace=False))
    return tuple([start, end])


def ox(receiver, donor, seq=None, shift=False):
    """
    The Ordered Crossover (OX) as explained in http://www.dmi.unict.it/mpavone/nc-cs/materiale/moscato89.pdf.

    Parameters
    ----------
    receiver : numpy.array
        The receiver of the sequence. The array needs to be repaired after the donation took place.
    donor : numpy.array
        The donor of the sequence.
    seq : tuple (optional)
        Tuple with two problems defining the start and the end of the sequence. Please note in our implementation
        the end of the sequence is included. The sequence is randomly chosen if not provided.

    shift : bool
        Whether during the repair the receiver should be shifted or not. Both version of it can be found in the
        literature.

    Returns
    -------

    y : numpy.array
        The offspring which was created by the ordered crossover.

    """
    assert len(donor) == len(receiver)

    # print(f"donor: {donor}\nreceiver: {receiver}")

    # the sequence which shall be use for the crossover
    seq = seq if not None else random_sequence(len(receiver))
    start, end = seq

    # print(f"seq: {seq}")

    # the donation and a set of it to allow a quick lookup
    donation = np.copy(donor[start:end + 1])
    donation_as_set = set(donation)

    # the final value to be returned
    y = []

    for k in range(len(receiver)):

        # print(f"iteration {k}: {y}")

        # do the shift starting from the swapped sequence - as proposed in the paper
        i = k if not shift else (start + k) % len(receiver)
        v = receiver[i]

        if v not in donation_as_set:
            y.append(v)

    # now insert the donation at the right place
    y = np.concatenate([y[:start], donation, y[start:]]).astype(copy=False, dtype=int).tolist()

    # print(f"Offspring: {y}")

    return y


def scx(parent1:PathSolution, parent2:PathSolution):

    info = parent1.info

    # path_1, path_2 = parent1.drone_dict[1:,2:-2], parent2.path_matrix[1:,2:-2]

    num_drones = info.Nd

    offspring_path = []
    offspring_start_points = [0]
    offspring_info = info

    for i in range(num_drones):
        # print("---------------------------------------------------------------------------------------------------------")
        # print(f"Drone {i}")
        # print("---------------------------------------------------------------------------------------------------------")
        drone_path_1, drone_path_2 = parent1.drone_dict[i][1:-2], parent2.drone_dict[i][1:-2] # To always start with node 0
        # print(f"Parent 1: {drone_path_1}\nParent 2: {drone_path_2}")
        starting_cell = 0
        offspring_drone_path = [starting_cell]

        num_steps = max([len(drone_path_1),len(drone_path_2)])

        for j in range(1,num_steps):
            taken_from = 'Drone 1'
            # Check if j exceeds the length of either drone path
            if j > len(drone_path_1) - 1 and j <= len(drone_path_2) - 1:
                taken_from = 'Drone 2'
                offspring_drone_path.append(drone_path_2[j])
            elif j > len(drone_path_2) - 1 and j <= len(drone_path_1) - 1:
                offspring_drone_path.append(drone_path_1[j])
            # elif j > len(drone_path_2) - 1 and j > len(drone_path_1) - 1:
            #     break

            # If there is no index violation, apply SCX
            else :
                # print(f"path 1 length: {len(drone_path_1)}, path 2 length: {len(drone_path_2)}, num_steps: {num_steps}, j: {j}")
                if info.D[starting_cell, drone_path_1[j]] <= info.D[starting_cell, drone_path_2[j]] :
                    offspring_drone_path.append(drone_path_1[j])
                else :
                    taken_from = 'Drone 2'
                    offspring_drone_path.append(drone_path_2[j])

                # Exchange choice in the case of duplicate cell
                if offspring_drone_path.count(offspring_drone_path[-1]) > 1:
                    if taken_from == 'Drone 1':
                        offspring_drone_path[-1] = drone_path_2[j]
                    else :
                        offspring_drone_path[-1] = drone_path_1[j]

            # if j == 0:
            #     offspring_drone_path = offspring_drone_path[1:] # Delete 0

            # print(f"current cell: {starting_cell}, updated offspring drone path: {offspring_drone_path}")

            starting_cell = offspring_drone_path[-1]

        # print("---------------------------------------------------------------------------------------------------------")

        # print(f"Offspring Path: {offspring_drone_path}, Offspring Path Length: {len(offspring_drone_path)}")

        offspring_path.extend(offspring_drone_path)  # Assign generated drone path to corresponding key
        if i < num_drones - 1 :
            # print(f"Start Points: {offspring_start_points}")
            offspring_start_points.append(offspring_start_points[-1] + len(offspring_drone_path))

    # print(f"of path: {offspring_path}\noffspring startpoints: {offspring_start_points}")

    return PathSolution(offspring_path, offspring_start_points, offspring_info)


class PathCrossover(Crossover):

    # rnd = random.uniform(0, 1)
    #
    # rnd = 0.7 # ENFORCE SCX FOR NOW !!!
    #
    # if rnd < 0.5:
    #     n_offsprings = 2 # OX
    #     print("OX")
    # else :
    #     n_offsprings = 1 # SCX
    #     print("SCX")


    def __init__(self, shift=False, Pc=1, empty=False, n_offsprings=2, total_dist_weight=0.5, **kwargs):
        super().__init__(n_parents=2, n_offsprings=n_offsprings, **kwargs)

        self.Pc = Pc  # Crossover Prob. (100% by default)
        self.empty = empty
        self.total_dist_weight = total_dist_weight
        self.shift = shift

        self.n_offsprings = n_offsprings

    def _do(self, problem, X, **kwargs):

        _, n_matings, n_var = X.shape

        # print("-->", X.shape) = (2, 50, 1) !

        # DELETE HERE JUST TRYING IF IT WORKS !

        # self.n_offsprings = 2
        Y = np.full((self.n_offsprings, n_matings, n_var), None, dtype=PathSolution)
        for i in range(n_matings):
            if self.empty:
                if self.n_offsprings == 1:
                    PathSolution(np.copy(path_1), np.copy(start_points_1),
                                 problem.info)
                else:
                    Y[0, i, 0], Y[1, i, 0] = PathSolution(np.copy(path_1), np.copy(start_points_1),problem.info), PathSolution(np.copy(path_2),np.copy(start_points_2), problem.info)
            else:
                path_1, path_2 = X[0, i, 0].path, X[1, i, 0].path # NOT USED
                # print(f"path 1: {path_1} path 2: {path_2}")
                # print(f"path 1 len: {len(path_1)} path 2 len: {len(path_2)}")
                start_points_1, start_points_2 = X[0, i, 0].start_points, X[1, i, 0].start_points # NOT USED
                # print(f"start points 1: {start_points_1}, start points 2: {start_points_2}")
                # print(f"n_offsprings: {self.n_offsprings}")
                # SCX (1 offspring)
                if self.n_offsprings == 1 :
                    # SCX
                    Y[0,i,0] = scx(X[0, i, 0],X[1, i, 0])
                else :
                    # OX
                    start, end = random_sequence(problem.info.Nc)
                    path_cross_1 = ox(path_1, path_2, seq=(start, end), shift=self.shift)
                    path_cross_2 = ox(path_2, path_1, seq=(start, end), shift=self.shift)
                    # print(f"path cross 1: {path_cross_1}, path cross 2: {path_cross_2}")
                    # print(f"path cross 1 len: {len(path_cross_1)}, path cross 2 len: {len(path_cross_2)}")
                    Y[0, i, 0], Y[1, i, 0] = PathSolution(path_cross_1, start_points_1, problem.info), PathSolution(path_cross_2, start_points_2, problem.info)

        # print("DEFAULT OX PASSED !")

                    # print(f"Offspring 1 path: {Y[0,i,0].path}\nOffspring 2 path: {Y[1,i,0].path}")
                    # print(f"Offspring 1 start points: {Y[0, i, 0].start_points}, Offspring 2 start points: {Y[1, i, 0].start_points}")

        return Y


# DEFAULT OX (GOES under for i in range(n_matings))
'''                path_1, path_2 = X[0, i, 0].path, X[1, i, 0].path
                start_points_1, start_points_2 = X[0, i, 0].start_points, X[1, i, 0].start_points

                start, end = random_sequence(problem.info.number_of_cities)

                path_cross_1 = ox(path_1, path_2, seq=(start, end), shift=self.shift)
                path_cross_2 = ox(path_2, path_1, seq=(start, end), shift=self.shift)

                Y[0, i, 0], Y[1, i, 0] = PathSolution(path_cross_1, start_points_1, problem.info), PathSolution(
                    path_cross_2, start_points_2, problem.info)
'''
