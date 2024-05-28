from Distance import *
from Connectivity import *
from Time import *

model_metric_info = {
    'Total Distance': get_total_distance,
    # 'Max Visits': get_max_visits,
    'Time Penalties': calculate_time_penalty,
    'Longest Subtour': get_longest_subtour,
    'Percentage Connectivity': calculate_percentage_connectivity,
    'Total Disconnected Time': calculate_total_maxDisconnectedTime,
    'Max Disconnected Time': calculate_max_maxDisconnectedTime,
    'Mean Disconnected Time': calculate_mean_maxDisconnectedTime,
    'Limit Long Jumps': long_jumps_ieq_constr,
    'No Long Jumps': long_jumps_eq_constr,
    'Limit Cell per Drone': cell_per_drone_constr,
    'Limit Max Longest Subtour': max_longest_subtour_constr,
    'Limit Min Longest Subtour': min_longest_subtour_constr,
    'Limit Subtour Range': max_subtour_range_constr,
    'Enforce Hovering Connectivity':enforce_hovering_connectivity
    # 'Limit Max Visits': sol.max_visits_constr # Not Neccessary alongside limit long jumps cv
}