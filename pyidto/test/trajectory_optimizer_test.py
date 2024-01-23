import numpy as np

from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.problem_definition import ProblemDefinition
from pyidto.solver_parameters import SolverParameters
from pyidto.trajectory_optimizer_solution import TrajectoryOptimizerSolution
from pyidto.trajectory_optimizer_stats import TrajectoryOptimizerStats
from pyidto.find_idto_resource import FindIdtoResourceOrThrow

# Get the absolute path to a model file
model_file = FindIdtoResourceOrThrow(
    "idto/examples/models/spinner_friction.urdf")

# Define the optimization problem
problem = ProblemDefinition()
problem.num_steps = 40
problem.q_init = np.array([0.3, 1.5, 0.0])
problem.v_init = np.array([0.0, 0.0, 0.0])
problem.Qq = 1.0 * np.eye(3)
problem.Qv = 0.1 * np.eye(3)
problem.R = np.diag([0.1, 0.1, 1e3])
problem.Qf_q = 10 * np.eye(3)
problem.Qf_v = 0.1 * np.eye(3)

q_nom = []   # Can't use list comprehension here because of Eigen conversion
v_nom = []
for i in range(problem.num_steps + 1):
    q_nom.append(np.array([0.3, 1.5, 2.0]))
    v_nom.append(np.array([0.0, 0.0, 0.0]))
problem.q_nom = q_nom
problem.v_nom = v_nom

# Set solver parameters
params = SolverParameters()
params.max_iterations = 200
params.scaling = True
params.equality_constraints = True
params.Delta0 = 1e1
params.Delta_max = 1e5
params.num_threads = 1

params.contact_stiffness = 200
params.dissipation_velocity = 0.1
params.smoothing_factor = 0.01
params.friction_coefficient = 0.5
params.stiction_velocity = 0.05

params.verbose = False

# Define an initial guess
q_guess = []
for i in range(problem.num_steps + 1):
    q_guess.append(np.array([0.3, 1.5, 0.0]))

# Create the optimizer object
time_step = 0.05
opt = TrajectoryOptimizer(model_file, problem, params, time_step)

assert opt.time_step() == time_step
assert opt.num_steps() == problem.num_steps

# Solve the optimization problem
solution = TrajectoryOptimizerSolution()
stats = TrajectoryOptimizerStats()
opt.Solve(q_guess, solution, stats)

solve_time = np.sum(stats.iteration_times)
print("Solved in ", solve_time, "seconds")

assert len(solution.q) == problem.num_steps + 1
expected_qN = np.array([0.287, 1.497, 1.995])  # from CPP version
assert np.linalg.norm(solution.q[-1]-expected_qN) < 1e-3
