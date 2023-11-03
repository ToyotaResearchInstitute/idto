import numpy as np

from bindings.trajectory_optimizer import MakeOptimizer
from bindings.problem_definition import ProblemDefinition
from bindings.solver_parameters import SolverParameters

# TODO(vincekurtz): use a relative path, or bind FindIdtoResource
model_file = "/home/vkurtz/build/drake/examples/acrobot/Acrobot.urdf"

# Set solver parameters
params = SolverParameters()

# Define the optimization problem
problem = ProblemDefinition()
problem.num_steps = 5
problem.q_init = np.array([0.1, 0.2])
problem.v_init = np.array([0.3, 0.4])
problem.Qq = 0.1 * np.eye(2)
problem.Qv = 0.2 * np.eye(2)
problem.Qf_q = 0.3 * np.eye(2)
problem.Qf_v = 0.4 * np.eye(2)
problem.R = 0.01 * np.eye(2)

q_nom = []   # Can't use list comprehension here because of Eigen conversion
v_nom = []
for i in range(problem.num_steps + 1):
    q_nom.append(np.array([0.1, 0.2]))
    v_nom.append(np.array([0.2, 0.1]))
problem.q_nom = q_nom
problem.v_nom = v_nom

# Create the optimizer object
time_step = 0.05
opt = MakeOptimizer(model_file, problem, params, time_step)

assert opt.time_step() == time_step
assert opt.num_steps() == problem.num_steps

#opt.Solve(None, None, None)