from PathSolution import *

# hovering = True
# realtime_connectivity = False
# realtime_info_sharing = False # If drone can merge occ grids every second or not

# hovering_and_connectivity_scenarios = {
#     'Hovering':True,
#     'Realtime Connectivity':True
# }


moo_model = {
    'F':['Total Distance','Longest Subtour','Percentage Connectivity','Max Disconnected Time'],
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
        'Total Distance':sol.total_dist,
        'Longest Subtour':sol.longest_subtour,
        'Percentage Connectivity':-sol.percentage_connectivity,
        'Total Disconnected Time':sol.total_disconnected_timesteps,
        'Max Disconnected Time':sol.max_disconnected_timesteps,
        'Limit Long Jumps':sol.long_jump_violations_constr,
        'Limit Cell per Drone':sol.cells_per_drone_constr,
        'Limit Max Visits':sol.max_visits_constr
    }

    return model_functions