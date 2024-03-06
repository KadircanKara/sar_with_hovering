from PathSolution import *
from Distance import *
from Connectivity import *

# hovering = True
# realtime_connectivity = False
# realtime_info_sharing = False # If drone can merge occ grids every second or not

# hovering_and_connectivity_scenarios = {
#     'Hovering':True,
#     'Realtime Connectivity':True
# }


moo_model = {
    'F':['Total Distance', 'Percentage Connectivity','Mean Disconnected Time', 'Time Penalties'],
    'G':['Limit Long Jumps','Limit Cell per Drone'], # 'Limit Cell per Drone'
    'H':[] # 'Limit Long Jumps'
}

soo_model = {
    'F':['Total Distance'],
    'H': ['Limit Long Jumps'],
    'G': ['Limit Cell per Drone']
}

model_metric_info = {
    'Total Distance': get_total_distance,
    # 'Max Visits': get_max_visits,
    'Time Penalties': calculate_time_penalty,
    'Longest Subtour': get_longest_subtour,
    'Percentage Connectivity': calculate_percentage_connectivity,
    'Total Disconnected Time': calculate_total_maxDisconnectedTime,
    'Max Disconnected Time': calculate_max_maxDisconnectedTime,
    'Mean Disconnected Time': calculate_mean_maxDisconnectedTime,
    'Limit Long Jumps': long_jumps_constr,
    'Limit Cell per Drone': cell_per_drone_constr
    # 'Limit Max Visits': sol.max_visits_constr # Not Neccessary alongside limit long jumps cv
}