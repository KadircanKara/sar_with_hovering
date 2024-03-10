import numpy as np
from math import sqrt
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial import distance
from typing import List, Dict
import random
from math import floor

import pandas as pd

default_input_parameters = {
    'grid_size': 8,
    'A': 50,
    'Nd': 4,
    'V': 2.5, # m/s
    'rc': 2,  # 2 cells
    'min_visits': 2,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'Nt': 1,
    'tar_cell':12,
    'p': 0.99,
    'q': 0.01,
    'Th': 0.9,
    'max_isolated_time': 0,
    'hovering':True,
    'realtime_connectivity':False,
    'realtime_connectivity_stepsize':10
}

class PathInfo:
    def __init__(self, grid_size=None, A=None, Nd=None, V=None, rc=None, min_visits=None, max_visits=None, Nt=None, tar_cell=None, p=None, q=None,
                 Th=None, max_isolated_time=None, hovering=True, realtime_connectivity=False, realtime_connectivity_stepsize=None):

        self.grid_size = grid_size if grid_size else default_input_parameters['grid_size']
        self.Nc = self.grid_size ** 2
        self.A = A if A else default_input_parameters['A']
        self.Nd = Nd if Nd else default_input_parameters['Nd']
        self.Nn = self.Nd + 1
        self.V = Nd if V else default_input_parameters['V']
        self.rc = rc if rc else default_input_parameters['rc']
        self.min_visits = min_visits if min_visits else default_input_parameters['min_visits']
        self.max_visits = max_visits if max_visits else default_input_parameters['max_visits']
        self.Nt = Nt if Nt else default_input_parameters['Nt']
        self.tar_cell = tar_cell if tar_cell else default_input_parameters['tar_cell']
        self.p = p if p else default_input_parameters['p']
        self.q = q if q else default_input_parameters['q']
        self.Th = Th if Th else default_input_parameters['Th']
        self.hovering = hovering if hovering else default_input_parameters['hovering']
        self.realtime_connectivity = realtime_connectivity if realtime_connectivity else default_input_parameters['realtime_connectivity']
        self.realtime_connectivity_stepsize = realtime_connectivity_stepsize if realtime_connectivity_stepsize else default_input_parameters['realtime_connectivity_stepsize']

        self.subtour_length_th = (int(round(self.Nc * self.max_visits/self.Nd)^2))*self.A



        P = [[i, j] for i in range(self.grid_size) for j in range(self.grid_size)]
        P.append([-1, -1])
        self.D = distance.cdist(P, P) * self.A


# x = PathInfo()
# print(pd.DataFrame(x.D))