import numpy as np
from pymoo.core.mutation import Mutation
from pymoo.operators.crossover.ox import random_sequence
from pymoo.operators.mutation.inversion import inversion_mutation
from scipy.spatial import distance
from typing import List, Dict
import random

from PathSolution import *
from PathInfo import *

class PathMutation(Mutation):

    def __init__(self, prob=1) -> None:
        super().__init__()
        self.prob = prob

    def _do(self, problem, X, **kwargs):
        Y = X.copy()

        for i, y in enumerate(X):
            sol : PathSolution = y[0]

            start_points = sol.start_points.copy()
            path = sol.path.copy()

            mut_path = path.copy()
            mut_start_points = start_points.copy()

            # start, end = sorted(random.randrange(path,2))
            # while end-start == 1:
            #     print("IN WHILE LOOP !")
            #     start, end = sorted(random.randrange(path, 2))

            start, end = seq = random_sequence(len(mut_path))
            # print(f"original path: {path}\nchosen sequence: {seq}")
            # start = mut_path.index(seq[0])
            # end = mut_path.index(seq[-1])

            # Swap Mutation
            if random.uniform(0,1) <= 0.5:
                # print("Swap Mutation")
                mut_path[start],mut_path[end] = mut_path[end],mut_path[start] # Apply swap
            # Inversion Mutation
            else:
                # print("Inversion Mutation")
                mut_path = inversion_mutation(path, seq, inplace=True)

            # print(f"Mutated path: {mut_path}")
            # print(f"Mutated start points: {mut_start_points}")
            # print(f"Mutated start points: {sorted_mut_start_points}")

            Y[i][0] = PathSolution(mut_path, mut_start_points, problem.info)

        return Y


'''
seq = random_sequence(len(path))
mut_path = inversion_mutation(path, seq, inplace=True)
# mut_start_points = np.copy(start_points)
mut_start_points = start_points.copy()
prob = 1 / (len(start_points) - 1) if len(start_points) > 1 else 0
# random_start_points = random_start_points_from_ranges(problem.start_points_ranges, problem.number_of_drones)
for j in range(1, len(start_points)):
    if np.random.random() <= prob:
        randomStart = np.random.randint(1, len(path))
        if randomStart not in mut_start_points:
            mut_start_points[j] = randomStart
# sorted_mut_start_points = np.sort(mut_start_points)
mut_start_points.sort()
sorted_mut_start_points = mut_start_points
'''