from statistics import median, median_low, median_high
from typing import Any
from PathAlgorithm import *
from PathOutput import *
from pymoo.operators.crossover.nox import NoCrossover
from pymoo.operators.mutation.nom import NoMutation
from pymoo.core.duplicate import NoDuplicateElimination
from pymoo.optimize import minimize
from PathResults import export_to_matlab
from PathAnimation import *
from Time import *
from Abbreviations import abbreviations

matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults'

def save_to_file(verbose_output, filename):
    with open(filename, "w") as f:
        f.write(verbose_output)


default_scenario = {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 4,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
}

test_scenario = {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 2,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
}
class PathUnitTest(object):

    def __init__(self, scenario=None, model=None, algorithm=None) -> None:
        test_min_visits = test_scenario['min_visits']
        self.test_path = np.random.permutation(64*test_min_visits)
        self.test_start_points = [0,16,32,48]*test_min_visits
        self.test_info = PathInfo(test_scenario)

        self.scenario = scenario if scenario else default_scenario
        self.scenario_info = PathInfo(self.scenario)
        self.scenario_text = ""
        for key in self.scenario.keys():
            self.scenario_text += (abbreviations[key] + "_" + str(self.scenario[key]) + "_")
        self.scenario_text = self.scenario_text[:-1] # Delete the last underscore (_)
        print("-->", self.scenario_text)

        self.model = model if model else moo_model

        self.algorithm = algorithm if algorithm else 'NSGA2'

    def __call__(self, type, *args: Any, **kwds: Any) -> Any:

        return self.Optimize()
        
        # if type=='algorithm':
        #     self.Optimize()
        # if type=='animation':
        #     self.Animate
        # else:
        #     self.Optimize()
    
    # def Animate(self):

    #     path = np.random.permutation(64)
    #     start_points = [0,16,32,48]
    #     Nd = 8
    #     rc = 2*sqrt(2)
    #     info = PathInfo(Nd=Nd, rc=rc)

    #     # Run PathAnimation

    #     sol = PathSolution(path, start_points, info)

    #     # anim = PathAnimation(sol)
    #     # anim()

    def Optimize(self, save_to_matlab=True):

        t = time.time()

        res = minimize(problem=PathProblem(self.scenario_info, self.model),
                        algorithm=PathAlgorithm(self.algorithm)(),# algorithm_dict[alg],
                        termination=('n_gen',4000),
                        seed=1,
                        # output=PathOutput(PathProblem(info)),
                        verbose=True,
                        # termination=path_termination
                        )
        
        elapsed_time = time.time() - t
        
        # try:
        X = res.X
        F = res.F
        # print('F:',abs(F))

        print(f"Elapsed time: {round(elapsed_time/60)} minutes")
        
        scenario = self.scenario_text
        model = self.model
        # Convert F (np array to dataframe)
        F = pd.DataFrame(F,columns=model['F'])
        print('F:',abs(F).to_string(index=False))

        # SAVE OBJS AND SOL OBJECTS AND RUNTIMES
        np.save(f"Results/X/{self.scenario_text}_SolutionObjects",X)
        np.save(f"Results/F/{self.scenario_text}_ObjectiveValues",F)
        np.save(f"Results/Time/{self.scenario_text}_Runtime",elapsed_time)

        # LOAD OBJS AND SOL OBJECTS
        sols = np.load(f"Results/X/{self.scenario_text}_SolutionObjects.npy",allow_pickle=True)
        objs = np.load(f"Results/F/{self.scenario_text}_ObjectiveValues.npy",allow_pickle=True)

        # EXPORT TO MATLAB
        for ind, obj_name in enumerate(self.model['F']):
            obj_values = objs[:,ind].tolist()
            export_to_matlab(matlab_filepath, model, scenario, obj_name, obj_values)

        return pd.DataFrame(F,columns=model['F']) , X

            # # sol = sols[0][0]

            # if self.model == distance_soo_model:
            #     dist_values = objs[0]
            # else:
            #     dist_values, subtour_values, conn_values, meanDisconn_values, maxDisconn_values = objs.transpose().tolist()

            # export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Dist", dist_values, inv_rel=False)
            # if self.model==moo_model:
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Subtour", subtour_values, inv_rel=False)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Conn", conn_values, inv_rel=True)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "MeanDisconn", meanDisconn_values, inv_rel=False)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)

        # except:
            # print("NO SOLUTION FOUND !!!")


test = PathUnitTest()
F,X = test('algorithm')

anim = PathAnimation()
anim('simple')



'''matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults'

model_list = [moo_model]
# algorithm_list = ['NSGA2','NSGA3']
number_of_drones_list = [8] # 8
r_comm_list = [2*sqrt(2)]
min_visits_list = [1] # 4,5
hovering_states = [True]
realtime_connectivity_states = [False]

for model in model_list:
    for alg in model['Alg']:
        for hovering in hovering_states:
            # info.hovering = hovering
            for realtime_connectivity in realtime_connectivity_states:
                # info.realtime_connectivity = realtime_connectivity
                for min_visits in min_visits_list:
                    # info.Nd = number_of_drones
                    for r_comm in r_comm_list:
                        # info.rc = r_comm
                        for number_of_drones in number_of_drones_list:

                            # info.min_visits = min_visits
                            info = PathInfo(hovering=hovering, realtime_connectivity=realtime_connectivity, Nd=number_of_drones, rc=r_comm, min_visits=min_visits)

                            scenario = f"{model['Exp']}_Opt_alg_{alg}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}"

                            print(f"Opt: {model['Exp']}, Algorithm: {alg}, Hovering: {info.hovering}, Realtime Connectivity: {info.realtime_connectivity}, Number of Drones: {info.Nd}, Number of Nodes: {info.Nn}, Communication Range: {info.rc}, Min Visits: {info.min_visits}")

                            t = time.time()

                            res = minimize(problem=PathProblem(info,model=model),
                                          algorithm=algorithm_dict[alg],
                                          termination=('n_gen',1000),
                                          seed=1,
                                          # output=PathOutput(PathProblem(info)),
                                          verbose=True,
                                          # termination=path_termination
                                          )

                            print(res)

                            elapsed_time = time.time() - t

                            X = res.X
                            F = res.F
                            print('F:',abs(F))

                            print(f"Elapsed time: {round(elapsed_time/60)} minutes")

                            # Save verbose
                            # df = pd.DataFrame(res)
                            # df.to_excel(f"Results/Verbose/{scenario}_verbose.txt", index=False)
                            # np.save(f"{scenario}_Pop",[individual._X[0]  for individual in res.pop])
                            # np.save(f"{scenario}_Opt",[individual._X[0]  for individual in res.opt])
                            # print(f"X: {[individual._X[0]  for individual in res.pop]}")
                            # print(f"opt: {[individual._X[0]  for individual in res.opt]}")
                            # save_to_file(res, f"Results/Verbose/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_verbose.txt")

                            # Save Solution Objects and Objective Values
                            np.save(f"Results/X/{scenario}_SolutionObjects",X)
                            np.save(f"Results/F/{scenario}_ObjectiveValues",F)
                            np.save(f"Results/Time/{scenario}_Runtime",elapsed_time)

                            sols = np.load(f"Results/X/{scenario}_SolutionObjects.npy",allow_pickle=True)
                            objs = np.load(f"Results/F/{scenario}_ObjectiveValues.npy",allow_pickle=True)

                            # sol = sols[0][0]

                            if model == distance_soo_model:
                                dist_values = objs[0]
                            else:
                                dist_values, subtour_values, conn_values, meanDisconn_values, maxDisconn_values = objs.transpose().tolist()

                            export_to_matlab(matlab_filepath, model, scenario, sols, "Dist", dist_values, inv_rel=False)
                            if model==moo_model:
                                export_to_matlab(matlab_filepath, model, scenario, sols, "Subtour", subtour_values, inv_rel=False)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "Conn", conn_values, inv_rel=True)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "MeanDisconn", meanDisconn_values, inv_rel=False)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)

                            # anim = PathAnimationTest()
                            # anim(sol, speed=20, title='random', subtitle='random random randomness')'''