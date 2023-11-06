import numpy as np
from pyidto.problem_definition import ProblemDefinition

# Construct a problem definition object
problem_definition = ProblemDefinition()

# Make sure we can set the various parameters
problem_definition.num_steps = 10
assert problem_definition.num_steps == 10

problem_definition.q_init = np.array([1, 2, 3])
assert problem_definition.q_init.shape == (3,)

problem_definition.v_init = np.array([2,3,4,5])
assert problem_definition.v_init.shape == (4,)

problem_definition.Qq = np.eye(3)
assert problem_definition.Qq.shape == (3,3)

problem_definition.Qv = np.eye(4)
assert problem_definition.Qv.shape == (4,4)

problem_definition.Qf_q = np.eye(3)
assert problem_definition.Qf_q.shape == (3,3)

problem_definition.Qf_v = np.eye(4)
assert problem_definition.Qf_v.shape == (4,4)

problem_definition.R = np.eye(4)
assert problem_definition.R.shape == (4,4)

problem_definition.q_nom = [np.array([2,3,4.5]) for i in range(10)]
assert(len(problem_definition.q_nom) == 10)
assert(problem_definition.q_nom[0][2] == 4.5)

problem_definition.v_nom = [np.array([2,3,4.5,5.5]) for i in range(10)]
assert(len(problem_definition.v_nom) == 10)
assert(problem_definition.v_nom[0][3] == 5.5)
