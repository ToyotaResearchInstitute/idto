from bindings.trajectory_optimizer_solution import TrajectoryOptimizerSolution
import numpy as np

solution = TrajectoryOptimizerSolution()

solution.q = [np.array([0.1, 0.2]), np.array([0.3, 0.4]), np.array([0.5, 0.6])]
solution.v = [np.array([0.1, 0.2]), np.array([0.3, 0.4]), np.array([0.5, 0.6])]
solution.tau = [np.array([0.1, 0.2]), np.array([0.3, 0.4]), np.array([0.5, 0.6])]

assert len(solution.q) == 3
assert len(solution.v) == 3
assert len(solution.tau) == 3
