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
                         Simulator, StartMeshcat, Value, EventStatus, PiecewisePolynomial,
                         PdControllerGains, JointActuatorIndex, DiscreteContactApproximation)

from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.problem_definition import ProblemDefinition
from pyidto.solver_parameters import SolverParameters
from pyidto.trajectory_optimizer_solution import TrajectoryOptimizerSolution
from pyidto.trajectory_optimizer_stats import TrajectoryOptimizerStats

from mpc_utils import StoredTrajectory, Interpolator

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


class ModelPredictiveController(LeafSystem):
    """
    A Drake system that implements an MPC controller.
    """
    def __init__(self, mpc_rate):
        """
        Construct the MPC controller system, which takes the current state as
        input and sends an optimial StoredTrajectory as output. 

                         -------------------------------
                         |                             |
            state  --->  |  ModelPredictiveController  |  --->  trajectory
                         |                             |
                         -------------------------------

        Args:
            optimizer: A TrajectoryOptimizer object that can solve the MPC
                       problem.
        """
        LeafSystem.__init__(self)

        # Specify a cost function and target trajectory
        self.problem = define_spinner_optimization_problem()

        # Specify solver parameters, including contact modeling parameters
        self.params = define_spinner_solver_parameters()

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = 0.05

        # Create the optimizer object
        model_file = "../examples/models/spinner_friction.urdf"
        self.optimizer = TrajectoryOptimizer(model_file, self.problem, self.params, self.time_step)

        self.nq = 3
        self.nv = 3
        self.nu = 2
        self.B = np.array([[1, 0], [0, 1], [0, 0]]).T

        # Allocate a warm-start
        q_guess = define_spinner_initial_guess(self.problem.num_steps)
        self.warm_start = self.optimizer.MakeWarmStart(q_guess)

        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        state = self.StoreOptimizerSolution(solution, 0.0)



        # Declare an abstract-valued state that will hold the optimal trajectory
        self.stored_trajectory = self.DeclareAbstractState(Value(state))

        # Define a periodic update event that will trigger the optimizer to resolve
        # the MPC problem.
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            1. / mpc_rate, 0, self.UpdateAbstractState)

        # Declare the input and output ports
        self.state_input_port = self.DeclareVectorInputPort(
            "state", BasicVector(self.nq + self.nv))
        self.trajectory_output_port = self.DeclareStateOutputPort(
            "optimal_trajectory", self.stored_trajectory)
        
    def StoreOptimizerSolution(self, solution, start_time):
        """
        Store a solution to the optimization problem in a StoredTrajectory object.

        Args:
            solution: A TrajectoryOptimizerSolution object
            start_time: The time at which the trajectory starts

        Returns:
            A StoredTrajectory object containing an interpolation of the solution.
        """
        # Create numpy arrays with knot points for iterpolation of the solution
        # along the actuated DoFs
        time_steps = []
        q_knots = []
        v_knots = []
        u_knots = []

        for t in range(self.problem.num_steps + 1):
            time_steps.append(t * self.time_step)
            q_knots.append(self.B @ solution.q[t])
            v_knots.append(self.B @ solution.v[t])

            if t == self.problem.num_steps:
                # Repeat the last control input
                u_knots.append(self.B @ solution.tau[t - 1])
            else:
                u_knots.append(self.B @ solution.tau[t])

        time_steps = np.array(time_steps)
        print(time_steps)
        q_knots = np.array(q_knots).T
        v_knots = np.array(v_knots).T
        u_knots = np.array(u_knots).T
        print(u_knots[0,:])

        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = start_time
        trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, q_knots)
        trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, v_knots)
        trajectory.u = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, u_knots)
        
        return trajectory
        
    def UpdateAbstractState(self, context, state):
        """
        Resolve the MPC problem and store the optimal trajectory in the abstract
        state.
        """
        print(f"Resolving at t={context.get_time()}")

        # Get the current state
        x0 = self.state_input_port.Eval(context)
        q0 = x0[:self.nq]
        v0 = x0[self.nq:]
        self.optimizer.ResetInitialConditions(q0, v0)

        # TODO: shift the warm-start based on the stored interpolation and time elapsed

        # TODO: shift the nominal trajectory as needed

        # Solve the optimization problem
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        # Store the solution in the abstract state
        state.get_mutable_abstract_state(0).SetFrom(
            Value(self.StoreOptimizerSolution(solution, context.get_time())))
        
        print(f"At solve time {context.get_time()}, u0 = {solution.tau[0]}")

        return EventStatus.Succeeded()

if __name__ == "__main__":
    # Set up a Drake diagram for simulation
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kTamsi)
    models = Parser(plant).AddModels("../examples/models/spinner_friction.urdf")

    # Add implicit PD controllers (must use kLagged or kSimilar)
    Kp = 1e-4 * np.ones(plant.num_actuators())
    Kd = 1e-5 * np.ones(plant.num_actuators())
    actuator_indices = [JointActuatorIndex(i) for i in range(plant.num_actuators())]
    for actuator_index, Kp, Kd in zip(actuator_indices, Kp, Kd):
        plant.get_joint_actuator(actuator_index).set_controller_gains(
            PdControllerGains(p=Kp, d=Kd))

    plant.Finalize()

    # Create the MPC controller and interpolator systems
    mpc_rate = 200  # Hz
    controller = builder.AddSystem(ModelPredictiveController(mpc_rate))
    interpolator = builder.AddSystem(Interpolator(plant.num_actuators()))

    # Wire the systems together
    builder.Connect(
        plant.get_state_output_port(), 
        controller.GetInputPort("state"))
    builder.Connect(
        controller.GetOutputPort("optimal_trajectory"), 
        interpolator.GetInputPort("trajectory"))
    builder.Connect(
        interpolator.GetOutputPort("control"), 
        plant.get_actuation_input_port())
    builder.Connect(
        interpolator.GetOutputPort("state"), 
        plant.get_desired_state_input_port(models[0])
    )
    
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
