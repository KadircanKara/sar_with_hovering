from PathProblem import *
from pymoo.core.repair import Repair

class NoRepair(Repair):

    def _do(self, problem: PathProblem, X, **kwargs):
        return X
