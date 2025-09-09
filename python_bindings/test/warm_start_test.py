import numpy as np

from pyidto import (
    TrajectoryOptimizer,
    ProblemDefinition,
    SolverParameters,
    TrajectoryOptimizerSolution,
    TrajectoryOptimizerStats,
    FindIdtoResource
)

from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser


def define_problem_parameters():
    """Define some optimization parameters for a test problem."""
    # Get the absolute path to a model file
    model_file = FindIdtoResource(
        "idto/models/spinner_friction.urdf")

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
    params.max_iterations = 10
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
    for _ in range(problem.num_steps + 1):
        q_guess.append(np.array([0.3, 1.5, 0.0]))

    return model_file, problem, params, q_guess


def solve_once():
    """Solve the test problem in one fell swoop."""
    # Problem setup
    model_file, problem, params, q_guess = define_problem_parameters()

    # Create the optimizer object
    time_step = 0.05
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()
    diagram = builder.Build()
    opt = TrajectoryOptimizer(diagram, plant, problem, params)

    # Solve the optimization problem
    solution = TrajectoryOptimizerSolution()
    stats = TrajectoryOptimizerStats()
    opt.Solve(q_guess, solution, stats)

    solve_time = np.sum(stats.iteration_times)
    print("Solved in ", solve_time, "seconds")

    return solution, stats


def solve_step_by_step():
    """Solve the test problem step-by-step with warm starts."""
    # Problem setup
    model_file, problem, params, q_guess = define_problem_parameters()
    params.max_iterations = 1

    # Create the optimizer object
    time_step = 0.05
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()
    diagram = builder.Build()
    opt = TrajectoryOptimizer(diagram, plant, problem, params)

    # Solve the optimization problem
    solution = TrajectoryOptimizerSolution()
    stats = TrajectoryOptimizerStats()
    warm_start = opt.CreateWarmStart(q_guess)
    for _ in range(10):
        opt.SolveFromWarmStart(warm_start, solution, stats)

    solve_time = np.sum(stats.iteration_times)
    print("Solved in ", solve_time, "seconds")
    return solution, stats


def test_reset_initial_conditions():
    """Test resetting the initial conditions."""
    model_file, problem, params, q_guess = define_problem_parameters()

    time_step = 0.05
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()
    diagram = builder.Build()
    opt = TrajectoryOptimizer(diagram, plant, problem, params)

    new_q_init = np.array([0.5, 1.2, -0.1])
    new_v_init = np.array([0.04, 0.3, 0.2])
    opt.ResetInitialConditions(new_q_init, new_v_init)
    new_solution = TrajectoryOptimizerSolution()
    new_stats = TrajectoryOptimizerStats()
    q_guess[0] = new_q_init
    opt.Solve(q_guess, new_solution, new_stats)
    assert np.linalg.norm(new_solution.q[0] - new_q_init) < 1e-8
    assert np.linalg.norm(new_solution.v[0] - new_v_init) < 1e-8


def test_set_q():
    """Test setting the joint trajectory stored in the warm start."""
    model_file, problem, params, q_guess = define_problem_parameters()
    time_step = 0.05
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()
    diagram = builder.Build()
    opt = TrajectoryOptimizer(diagram, plant, problem, params)

    warm_start = opt.CreateWarmStart(q_guess)

    assert np.all(q_guess[10] == np.array([0.3, 1.5, 0.0]))
    assert np.all(warm_start.get_q()[10] == np.array([0.3, 1.5, 0.0]))

    new_guess = q_guess.copy()
    new_guess[10] = np.array([0.5, 1.2, -0.1])
    warm_start.set_q(new_guess)

    assert np.all(warm_start.get_q()[10] == np.array([0.5, 1.2, -0.1]))


def test_compare_warm_start():
    """Make sure open-loop and warm-start solutions are the same."""
    one_shot_solution, one_shot_stats = solve_once()
    warm_stat_solution, warm_start_stats = solve_step_by_step()

    # Solutions should be the same
    assert np.linalg.norm(
        one_shot_solution.q[-1] - warm_stat_solution.q[-1]) < 1e-8
    assert np.linalg.norm(
        one_shot_solution.v[-1] - warm_stat_solution.v[-1]) < 1e-8

    # Stats should be the same
    assert np.linalg.norm(np.array(one_shot_stats.iteration_costs)
                          - np.array(warm_start_stats.iteration_costs)) < 1e-8
    assert np.linalg.norm(np.array(one_shot_stats.trust_region_radii)
                          - np.array(warm_start_stats.trust_region_radii)) < 1e-8
    assert np.linalg.norm(np.array(one_shot_stats.gradient_norms)
                          - np.array(warm_start_stats.gradient_norms)) < 1e-8
