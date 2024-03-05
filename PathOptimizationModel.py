from PathSolution import *
from Distance import min_time_between_visits, max_visits, visit_time_variance, number_of_long_jumps
from Connectivity import max_disconnected_time, mean_disconnected_time

# hovering = True
# realtime_connectivity = False
# realtime_info_sharing = False # If drone can merge occ grids every second or not

# hovering_and_connectivity_scenarios = {
#     'Hovering':True,
#     'Realtime Connectivity':True
# }


moo_model = {
    'F':['Total Distance', 'Distance Penalties', 'Percentage Connectivity','Avg Disconnected Time'],
    'G':['Limit Long Jumps'], # 'Limit Cell per Drone'
    'H':[] # 'Limit Long Jumps'
}

soo_model = {
    'F':['Total Distance'],
    'H': ['Limit Long Jumps'],
    'G': ['Limit Cell per Drone']
}

def get_model_function_values(sol:PathSolution):

    model_functions = {
        'Total Distance': sol.total_dist,
        'Distance Penalties': max_visits(sol) - min_time_between_visits(sol) + visit_time_variance(sol) + number_of_long_jumps(sol),
        'Longest Subtour': sol.longest_subtour,
        'Percentage Connectivity': -sol.percentage_connectivity,
        'Total Disconnected Time': sol.total_disconnected_timesteps,
        'Max Disconnected Time': max_disconnected_time(sol),
        'Mean Disconnected Time': avg_disconnected_time(sol),
        'Limit Long Jumps': sol.long_jump_violations_constr,
        'Limit Cell per Drone': sol.cells_per_drone_constr,
        'Limit Max Visits': sol.max_visits_constr
    }

    return model_functions