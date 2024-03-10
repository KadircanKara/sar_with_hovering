import autograd.numpy as anp

from PathOptimizationModel import *
from PathSolution import *
from PathInfo import *
from Distance import *
from Connectivity import *

from pymoo.core.problem import ElementwiseProblem

class PathProblem(ElementwiseProblem):

    def __init__(self, info:PathInfo, model=moo_model, elementwise=True, **kwargs):
        self.model = model # My addition
        self.info = info
        self.n_var = 1
        self.n_obj = len(self.model['F'])
        self.n_ieq_constr = len(self.model['G'])
        self.n_eq_constr = len(self.model['H'])


        super().__init__(n_var = self.n_var, n_obj=self.n_obj, n_ieq_constr=self.n_ieq_constr, n_eq_constr=self.n_eq_constr, elementwise=True, **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):

        sol:PathSolution = x[0]
        model = self.model
        # model_functions = get_model_function_values(sol)
        f,g,h=[],[],[]

        # if model == 'moo':
        #     model_var = moo_model
        # elif model == 'distance_soo':
        #     model_var = distance_soo_model
        # elif model == 'meanMaxDisconnectivity_soo':
        #     model_var = meanMaxDisconnectivity_soo_model
        # elif model == 'connectivity_soo':
        #     model_var = connectivity_soo_model


        for i in range(self.n_obj):
            obj_name = self.model['F'][i]
            obj_calc = model_metric_info[obj_name]
            f.append(obj_calc(sol))
        for j in range(self.n_ieq_constr):
            ieq_constr_name = self.model['G'][j]
            ieq_constr_calc = model_metric_info[ieq_constr_name]
            g.append(ieq_constr_calc(sol))
        for k in range(self.n_eq_constr):
            eq_constr_name = self.model['H'][k]
            eq_constr_calc = model_metric_info[eq_constr_name]
            h.append(eq_constr_calc(sol))

        if f:
            out['F'] = anp.column_stack(f)
            # print(f"F:{out['F']}")
        if g:
            out['G'] = anp.column_stack(g)
            # for i,y in enumerate(out['G'][0]):
            #     print(f"{model['G'][i]} CV: {y}")
            # print(f"G:{out['G']}")
        if h:
            out['H'] = anp.column_stack(h)
            # print(f"H:{out['H']}")