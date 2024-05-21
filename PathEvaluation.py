from Distance import *
from Connectivity import *
from Time import *


evaluation_metrics = {
    "longest subtour length":get_longest_subtour, 
    "total distance":get_total_distance, 
    "speed violations":long_jumps_eq_constr, 
    "percentage connectivity":calculate_percentage_connectivity,
    "maximum disconnected time":calculate_max_maxDisconnectedTime,
    "mean disconnected time":calculate_mean_maxDisconnectedTime,
    # "total_timesteps":get_total_timesteps,
    "total distance in meters":get_total_distance,
    "longest subtour length in meters":get_longest_subtour,
    # "avg_hops":get_average_num_of_hops,
    }

# evaluation_metrics_real_time = {
#     "real time step size":get_real_time_step_number,
#     "real time percentage connectivity":calculate_real_time_percentage_connectivity,
#     "real time maximum disconnected time":calculate_real_time_max_disconnectivity,
#     "real time mean disconnected time":calculate_real_time_mean_disconnectivity,
#     }


class PathEvaluation:

    def __str__(self) -> str:
        return str(self.sol) + "\n" + "\n".join(str(self.evaluation_results).split(","))

    def __init__(self, sol : PathSolution):
        self.sol = sol
        self.evaluation_results = dict()

        xs, ys, sp = get_cartesion_drone_path(sol)

        real_xs, real_ys = get_real_real_path(xs, ys, sp)

        for metric, func in evaluation_metrics.items():
            self.evaluation_results[metric] = func(sol)

        # for metric, func in evaluation_metrics_real_time.items():
        #     self.evaluation_results[metric] = func(sol, real_xs, real_ys)

