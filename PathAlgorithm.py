from PathSampling import *
from PathMutation import *
from PathCrossover import *
from PathRepair import *
from PathProblem import *

from pymoo.operators.crossover.nox import NoCrossover
from PathCrossover import PathCrossover
from pymoo.operators.mutation.nom import NoMutation
from pymoo.core.duplicate import NoDuplicateElimination

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.ctaea import CTAEA
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.constraints.as_obj import ConstraintsAsObjective
from pymoo.algorithms.moo.moead import MOEAD
from pymoo.constraints.eps import AdaptiveEpsilonConstraintHandling
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.algorithms.moo.unsga3 import UNSGA3
# from pymoo.algorithms.moo.age import AGEMOEA
from pymoo.algorithms.moo.sms import SMSEMOA
from pymoo.algorithms.soo.nonconvex.ga import GA

# from pymoo.termination.default import PathTermination


path_sampling = PathSampling()
path_mutation = PathMutation()
path_crossover = PathCrossover()
path_eliminate_duplicates = NoDuplicateElimination()
path_repair = NoRepair()

# path_termination = PathTermination(
#     # xtol=1e-8,
#     # cvtol=0,    # 1e-6,
#     ftol=0.01, #0.0025,
#     # period=30,
#     # n_max_gen=10000,
#     # n_max_evals=100000
# )



algorithm_dict = {

    'GA': GA(
        pop_size=100,
        sampling=path_sampling,
        mutation=path_mutation,
        crossover=path_crossover,
        eliminate_duplicates=path_eliminate_duplicates,
        repair=path_repair
    ),

    'NSGA2' : NSGA2(
                        pop_size=100,
                        sampling=path_sampling,
                        mutation=path_mutation,
                        crossover=path_crossover,
                        eliminate_duplicates=path_eliminate_duplicates,
                        repair=path_repair
                ),

    'MOEAD' : MOEAD(
                    # pop_size=40,
                    ref_dirs=get_reference_directions("das-dennis", PathProblem(PathInfo()).n_obj, n_partitions=12),
                    n_neighbors=15, # 5
                    prob_neighbor_mating=0.7, # 0.3
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    # eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair
                ),

    'NSGA3' : NSGA3(
                    pop_size=100,
                    ref_dirs=get_reference_directions("das-dennis", PathProblem(PathInfo()).n_obj, n_partitions=12),
                    n_neighbors=15,
                    prob_neighbor_mating=0.7,
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair,
                    # termination=path_termination
                        ),

    'USNGA3' : UNSGA3(
                    ref_dirs=np.array([[0.7, 0.9, 0.2, 0], [0.5, 0.7, 0.4, 0], [0.8, 0.9, 0.4, 0]]),
                    pop_size=126,
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair
                        ),

    # 'AGEMOEAD' : AGEMOEA(
    #                 pop_size=126,
    #                 sampling=PathSampling(),
    #                 mutation=path_mutation,
    #                 crossover=path_crossover,
    #                 eliminate_duplicates=path_eliminate_duplicates,
    #                 repair=path_repair
    #                     ),

    'SMSEMOA' : SMSEMOA(
                    pop_size=126,
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair
                        )


}


