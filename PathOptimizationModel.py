from PathSolution import *
from Distance import *
from Connectivity import *
from Time import *




moo_model = {
    'Exp':'MOO',
    'Alg':['NSGA2'],
    'F': ['Total Distance','Percentage Connectivity','Mean Disconnected Time', 'Max Disconnected Time'],
    'G': ['Limit Long Jumps','Limit Max Longest Subtour'], # Limit Subtour Range
    'H': [] # 'Limit Long Jumps'
}

distance_soo_model = {
    'Exp': 'SOO',
    'Alg':['GA'],
    'F': ['Total Distance'],
    'G': ['Limit Longest Subtour'],
    'H': []  # 'No Long Jumps'
}