import numpy as np

from pyidto import TrajectoryOptimizer
from pyidto import ProblemDefinition
from pyidto import SolverParameters
from pyidto import TrajectoryOptimizerSolution
from pyidto import TrajectoryOptimizerStats
from pyidto import FindIdtoResource

from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser


def test_optimizer():
    """Test that we can solve a small optimization problem."""
    # Define the system model
    time_step = 0.05
    model_file = FindIdtoResource("idto/models/spinner_friction.urdf")
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()
    diagram = builder.Build()

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

    q_nom = []  # Can't use list comprehension here because of Eigen conversion
    v_nom = []
    for i in range(problem.num_steps + 1):
       q_nom.append(np.array([0.3, 1.5, 2.0]))
       v_nom.append(np.array([0.0, 0.0, 0.0]))
    problem.q_nom = q_nom
    problem.v_nom = v_nom
    assert len(problem.q_nom) == problem.num_steps + 1
    assert len(problem.v_nom) == problem.num_steps + 1

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

    params.verbose = True

    # Define an initial guess
    q_guess = []
    for i in range(problem.num_steps + 1):
        q_guess.append(np.array([0.3, 1.5, 0.0]))

    # Create the optimizer object
    opt = TrajectoryOptimizer(diagram, plant, problem, params)
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

    # Get the problem definition
    problem = opt.prob()
    assert problem.num_steps == 40

    # Get the solver parameters
    params = opt.params()
    assert params.max_iterations == 200

    # Make sure we can change the nominal trajectory
    assert np.all(problem.q_nom[10] == np.array([0.3, 1.5, 2.0]))
    assert np.all(problem.v_nom[10] == np.array([0.0, 0.0, 0.0]))
    new_q_nom = problem.q_nom.copy()
    new_v_nom = problem.v_nom.copy()
    new_q_nom[10] = np.array([0.4, 1.6, 2.1])
    new_v_nom[10] = np.array([0.1, 0.1, 0.1])

    opt.UpdateNominalTrajectory(new_q_nom, new_v_nom)
    problem = opt.prob()
    assert np.all(problem.q_nom[10] == np.array([0.4, 1.6, 2.1]))
    assert np.all(problem.v_nom[10] == np.array([0.1, 0.1, 0.1]))
