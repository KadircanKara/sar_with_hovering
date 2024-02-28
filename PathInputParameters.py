# grid_size=8
# A=50
# Nd=8
# V=2.5
# rc=2
# nvisits_th=5
# Nt=1
# p=0.99
# q=0.01
# Th=0.9
# max_isolated_time=0

scenario_parameters = {
    'grid_size': 8,
    'A': 50,
    'Nd': 16,
    'V': 2.5, # m/s
    'rc': 2,  # 2 cells
    'nvisits_th': 5,  # So that a cell is not visited more than this amount (Incorporate as a constraint)
    'Nt': 1,
    'p': 0.99,
    'q': 0.01,
    'Th': 0.9,
    'max_isolated_time': 0,
}

'''
    moo_model = {
    'F':['Distance','Connectivity'],
    # 'G':['Limit Long Jumps','Limit Cell per Drone'],
    'H':['Limit Long Jumps','Limit Cell per Drone']
}

soo_model = {
    'F':['Distance'],
    'H': ['Limit Long Jumps', 'Limit Cell per Drone']
}

model_functions = {
    'Distance':0,
    'Connectivity':1,
    'Limit Long Jumps':2,
    'Limit Cell per Drone':3,
}
'''