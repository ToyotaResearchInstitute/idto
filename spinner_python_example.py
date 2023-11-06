#!/usr/bin/env python

##
#
# An example of using the python bindings to solve the spinner optimization 
# problem. The project must be compiled with bazel first:
#
#  bazel build //...
#
##

# Note: this could be added to the PYTHONPATH environment variable instead, 
# as a better long-term solution
import os
import sys
sys.path.insert(-1, os.getcwd() + "/bazel-bin/")

import numpy as np
import time

from pydrake.all import (StartMeshcat, DiagramBuilder,
        AddMultibodyPlantSceneGraph, AddDefaultVisualization, Parser)

from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.problem_definition import ProblemDefinition
from pyidto.solver_parameters import SolverParameters
from pyidto.trajectory_optimizer_solution import TrajectoryOptimizerSolution
from pyidto.trajectory_optimizer_stats import TrajectoryOptimizerStats

def define_spinner_optimization_problem():
    """
    Create a problem definition for the spinner.
    """
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

    return problem

def define_spinner_solver_parameters():
    """
    Create a set of solver parameters for the spinner.
    """
    params = SolverParameters()

    params.max_iterations = 200
    params.scaling = True
    params.equality_constraints = True
    params.Delta0 = 1e1
    params.Delta_max = 1e5
    params.num_threads = 4

    params.contact_stiffness = 200
    params.dissipation_velocity = 0.1
    params.smoothing_factor = 0.01
    params.friction_coefficient = 0.5
    params.stiction_velocity = 0.05

    params.verbose = True

    return params

def define_spinner_initial_guess(num_steps):
    """
    Create an initial guess for the spinner
    """
    q_guess = []
    for i in range(num_steps + 1):
        q_guess.append(np.array([0.3, 1.5, 0.0]))

    return q_guess

def visualize_trajectory(q, time_step, model_file, meshcat=None):
    """
    Display the given trajectory (list of configurations) on meshcat
    """
    # Create a simple Drake diagram with a plant model
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    Parser(plant).AddModels(model_file)
    plant.Finalize()

    # Connect to the meshcat visualizer
    AddDefaultVisualization(builder, meshcat)

    # Build the system diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    plant.get_actuation_input_port().FixValue(plant_context,
            np.zeros(plant.num_actuators()))
    
    # Step through q, setting the plant positions at each step
    meshcat.StartRecording()
    for k in range(len(q)):
        diagram_context.SetTime(k * time_step)
        plant.SetPositions(plant_context, q[k])
        diagram.ForcedPublish(diagram_context)
        time.sleep(time_step)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    
if __name__=="__main__":
    # Start up meshcat (for viewing the result)
    meshcat = StartMeshcat()

    # Relative path to the model file that we'll use
    model_file = "./examples/models/spinner_friction.urdf"

    # Specify a cost function and target trajectory
    problem = define_spinner_optimization_problem()

    # Specify solver parameters, including contact modeling parameters
    params = define_spinner_solver_parameters()

    # Specify the timestep we'll use to discretize the trajectory
    time_step = 0.05

    # Specify an initial guess
    q_guess = define_spinner_initial_guess(problem.num_steps)

    # Create the optimizer object
    opt = TrajectoryOptimizer(model_file, problem, params, time_step)

    # Allocate some structs that will hold the solution
    solution = TrajectoryOptimizerSolution()
    stats = TrajectoryOptimizerStats()

    # Solve the optimization problem
    opt.Solve(q_guess, solution, stats)

    solve_time = np.sum(stats.iteration_times)
    print(f"Solved in {solve_time:.4f} seconds")
   
    # Play back the solution on meshcat
    visualize_trajectory(solution.q, time_step, model_file, meshcat)

