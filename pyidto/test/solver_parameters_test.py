import numpy as np
from pyidto.solver_parameters import SolverParameters

params = SolverParameters()

assert params.max_iterations == 100  # default value
params.max_iterations = 1
assert params.max_iterations == 1

assert params.normalize_quaternions == False
params.normalize_quaternions = True
assert params.normalize_quaternions == True

assert params.verbose == True
params.verbose = False
assert params.verbose == False

assert params.contact_stiffness == 100
params.contact_stiffness = 1.2
assert params.contact_stiffness == 1.2

assert params.dissipation_velocity == 0.1
params.dissipation_velocity = 0.2
assert params.dissipation_velocity == 0.2

assert params.stiction_velocity == 0.05
params.stiction_velocity = 0.1
assert params.stiction_velocity == 0.1

assert params.friction_coefficient == 0.5
params.friction_coefficient = 0.6
assert params.friction_coefficient == 0.6

assert params.smoothing_factor == 0.1
params.smoothing_factor = 0.2
assert params.smoothing_factor == 0.2

assert params.scaling == True
params.scaling = False
assert params.scaling == False

assert params.equality_constraints == True
params.equality_constraints = False
assert params.equality_constraints == False

assert params.Delta0 == 1e-1
params.Delta0 = 1e-2
assert params.Delta0 == 1e-2

assert params.Delta_max == 1e5
params.Delta_max = 1e6
assert params.Delta_max == 1e6

assert params.num_threads == 1
params.num_threads = 4
assert params.num_threads == 4
