#!/usr/bin/env python

##
#
# An example of using the python bindings to perform Model Predictive Control
# on the spinner. The project must be compiled with bazel first:
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

from pydrake.all import (AddDefaultVisualization, AddMultibodyPlantSceneGraph, 
                         BasicVector, DiagramBuilder, LeafSystem, Parser, 
                         Simulator, StartMeshcat)

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
        q_nom.append(np.array([0.3, 1.5, np.pi/2]))
        v_nom.append(np.array([0.0, 0.0, 0.0]))
    problem.q_nom = q_nom
    problem.v_nom = v_nom

    return problem

def define_spinner_solver_parameters():
    """
    Create a set of solver parameters for the spinner.
    """
    params = SolverParameters()

    params.max_iterations = 1
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

class SpinnerMPCController(LeafSystem):
    """
    A Drake system that implements a simple MPC controller for the spinner.
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Specify a cost function and target trajectory
        self.problem = define_spinner_optimization_problem()

        # Specify solver parameters, including contact modeling parameters
        self.params = define_spinner_solver_parameters()

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = 0.05

        # Create the optimizer object
        model_file = "./examples/models/spinner_friction.urdf"
        self.opt = TrajectoryOptimizer(model_file, self.problem, self.params, self.time_step)

        # Allocate a warm-start
        q_guess = define_spinner_initial_guess(self.problem.num_steps)
        self.warm_start = self.opt.MakeWarmStart(q_guess)

        # Allocate some structs that will hold the solution
        self.solution = TrajectoryOptimizerSolution()
        self.stats = TrajectoryOptimizerStats()

        # Set up the input and output ports
        self.state_input_port = self.DeclareVectorInputPort("x", BasicVector(6))
        self.control_output_port = self.DeclareVectorOutputPort(
            "tau", BasicVector(2), self.CalcOutput)

    def CalcOutput(self, context, output):
        # Get the current state
        x = self.state_input_port.Eval(context)

        # Set the initial state
        q0 = x[:3]
        v0 = x[3:]
        self.opt.ResetInitialConditions(q0, v0)

        # Solve the optimization problem
        self.opt.SolveFromWarmStart(self.warm_start, self.solution, self.stats)

        # Get the first control input
        # N.B. IDTO returns generalized forces for all DoFs, but we only have
        # actuators on the first two.
        # Note also that some better interpolation would be nice here.
        u = self.solution.tau[1][0:2]
        q_nom = self.solution.q[1][0:2]
        v_nom = self.solution.v[1][0:2]
        q = q0[0:2]
        v = v0[0:2]
        u = u - 10 * (q - q_nom) - 1 * (v - v_nom)

        # Return the control input
        output.SetFromVector(u)


if __name__ == "__main__":
    # Set up a Drake diagram for simulation
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-2)
    Parser(plant).AddModels("examples/models/spinner_friction.urdf")
    plant.Finalize()

    # Create the MPC controller
    controller = builder.AddSystem(SpinnerMPCController())

    # Connect the controller to the plant
    builder.Connect(controller.control_output_port,
            plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(),
            controller.state_input_port)
    
    # Connect the plant to meshcat for visualization
    meshcat = StartMeshcat()
    AddDefaultVisualization(builder, meshcat)

    # Build the system diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # Set the initial state
    plant.SetPositions(plant_context, [0.3, 1.5, 0.0])
    plant.SetVelocities(plant_context, [0.0, 0.0, 0.0])

    # Simulate and play back on meshcat
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording()
    simulator.AdvanceTo(5.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
