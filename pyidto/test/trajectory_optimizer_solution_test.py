from pyidto.trajectory_optimizer_solution import TrajectoryOptimizerSolution
import numpy as np

solution = TrajectoryOptimizerSolution()

assert len(solution.q) == 0
assert len(solution.v) == 0
assert len(solution.tau) == 0
