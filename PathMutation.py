import numpy as np
from pymoo.core.mutation import Mutation
from pymoo.operators.crossover.ox import random_sequence
from pymoo.operators.mutation.inversion import inversion_mutation
from scipy.spatial import distance
from typing import List, Dict
import random
from copy import copy, deepcopy

from PathSolution import *
from PathInfo import *
from PathProblem import *

class PathMutation(Mutation):

    def __init__(self, prob_random_swap=0.6, num_random_swaps=1,  
                            prob_inversion=1, num_inversions=1, 
                            prob_scramble=0.3, num_scrambles=1) -> None:
        super().__init__()
        self.prob_random_swap = prob_random_swap
        self.num_random_swaps = num_random_swaps
        self.prob_inversion = prob_inversion
        self.num_inversions = num_inversions
        self.prob_scramble = prob_scramble
        self.num_scrambles = num_scrambles

    def _do(self, problem : PathProblem, X, **kwargs):
        Y = X.copy()

        for i, y in enumerate(X):
            sol : PathSolution = y[0]

            start_points = sol.start_points
            path = np.copy(sol.path)
            mut_path = path

            # print("Original Start Points:",start_points)

            if np.random.random() <= self.prob_scramble:
                for _ in range(self.num_scrambles):
                    seq = random_sequence(len(path))
                    random.shuffle(mut_path[seq[0]:seq[1]])

            if np.random.random() <= self.prob_inversion:
                for _ in range(self.num_inversions):
                    seq = random_sequence(len(path))
                    mut_path = inversion_mutation(mut_path, seq, inplace=True)

            if np.random.random() <= self.prob_random_swap:
                for _ in range(self.num_random_swaps):
                        seq = random_sequence(len(path))
                        temp = path[seq[0]]
                        mut_path[seq[0]] = mut_path[seq[1]]
                        mut_path[seq[1]] = temp

            mut_start_points = np.copy(start_points)

            prob = 1 / (len(start_points)-1) if len(start_points) > 1 else 0

            # random_start_points = random_start_points_from_ranges(problem.start_points_ranges, problem.number_of_drones)

            for j in range(1, len(start_points)):
                if np.random.random() <= prob:

                    randomStart = np.random.randint(1, len(path))

                    if randomStart not in mut_start_points:
                        mut_start_points[j] = randomStart

            sorted_mut_start_points = np.sort(mut_start_points)

            # print("Mutated Start Points:",sorted_mut_start_points)
            
            Y[i][0] = PathSolution(mut_path, sorted_mut_start_points, problem.info)

        return Y


'''
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

            start, end = seq = random_sequence(len(mut_path))

            # Path Mutations

            # Swap Mutation
            if random.uniform(0,1) <= 0.5:
                mut_path[start],mut_path[end] = mut_path[end],mut_path[start] # Apply swap
            # Inversion Mutation
            else:
                mut_path = inversion_mutation(path, seq, inplace=True)

            # Start Points Mutation
            if random.uniform(0, 1) <= 0.5:
                random_index = random.randint(1,len(start_points)-2)
                # print(f"random index: {random_index}")
                # print(f"pre mutation: {mut_start_points[random_index]}")
                if mut_start_points[random_index] - mut_start_points[random_index-1] < int(round(problem.info.number_of_cells/problem.info.number_of_drones)):
                    mut_start_points[random_index] = mut_start_points[random_index-1] + int(round((mut_start_points[random_index+1] - mut_start_points[random_index-1])/2))
                # print(f"post mutation: {mut_start_points[random_index]}")
                if mut_start_points[random_index] - mut_start_points[random_index-1] >= int(round(problem.info.number_of_cells/problem.info.number_of_drones)):
                    mut_start_points[random_index] = mut_start_points[random_index-1] + int(round((mut_start_points[random_index]-mut_start_points[random_index-1])/2))

            # print(f"Mutated path: {mut_path}")
            # print(f"Mutated start points: {mut_start_points}")
            # print(f"Mutated start points: {sorted_mut_start_points}")

            Y[i][0] = PathSolution(mut_path, mut_start_points, problem.info)

        return Y
        
        '''


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