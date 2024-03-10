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
    'Exp':'Dist_PercConn_MeanDisconn,TimePenalties',
    'Alg':['NSGA2','NSGA3'],
    'F': ['Total Distance', 'Percentage Connectivity','Mean Disconnected Time', 'Time Penalties'],
    'G': ['Limit Long Jumps','Limit Longest Subtour'],
    'H': [] # 'Limit Long Jumps'
}

soo_model = {
    'Total Distance':{
        'Alg': ['GA'],
        'F': ['Total Distance'],
        'G': ['Limit Long Jumps', 'Limit Cell per Drone', 'Limit Longest Subtour'],
        'H': []  # 'Limit Long Jumps'

    },
    'Percentage Connectivity': {
        'Alg': ['GA'],
        'F': ['Percentage Connectivity'],
        'G': ['Limit Long Jumps', 'Limit Cell per Drone', 'Limit Longest Subtour'],
        'H': []  # 'Limit Long Jumps'

    },
    'Mean Disconnected Time': {
        'Alg': ['GA'],
        'F': ['Mean Disconnected Time'],
        'G': ['Limit Long Jumps', 'Limit Cell per Drone', 'Limit Longest Subtour'],
        'H': []  # 'Limit Long Jumps'

    }

}

distance_soo_model = {
    'Exp': 'Dist',
    'Alg':['GA'],
    'F': ['Total Distance'],
    'G': ['Limit Long Jumps','Limit Cell per Drone','Limit Longest Subtour'],
    'H': []  # 'Limit Long Jumps'
}
connectivity_soo_model = {
    'Exp': 'Conn',
    'Alg':['GA'],
    'F': ['Percentage Connectivity'],
    'G': ['Limit Long Jumps','Limit Cell per Drone','Limit Longest Subtour'],
    'H': []  # 'Limit Long Jumps'
}
meanMaxDisconnectivity_soo_model = {
    'Exp': 'MeanDisconn',
    'Alg':['GA'],
    'F': ['Mean Disconnected Time'],
    'G': ['Limit Long Jumps','Limit Cell per Drone','Limit Longest Subtour'],
    'H': []  # 'Limit Long Jumps'
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
    'Limit Cell per Drone': cell_per_drone_constr,
    'Limit Longest Subtour': longest_subtour_constr
    # 'Limit Max Visits': sol.max_visits_constr # Not Neccessary alongside limit long jumps cv
}