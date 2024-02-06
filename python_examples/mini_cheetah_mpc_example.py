#!/usr/bin/env python

##
#
# An example of using the python bindings to perform Model Predictive Control
# on the mini cheetah quadruped. The project must be compiled with bazel first:
#
#  bazel build //...
#
##

# Note: this could be added to the PYTHONPATH environment variable instead, 
# as a better long-term solution
import os
import sys
sys.path.insert(-1, os.getcwd() + "../bazel-bin/")

import numpy as np
import time

from pydrake.all import (AddDefaultVisualization, AddMultibodyPlantSceneGraph,
                         DiagramBuilder, Parser, Simulator, StartMeshcat, 
                         PdControllerGains, JointActuatorIndex, 
                         DiscreteContactApproximation, LeafSystem, BasicVector,
                         RollPitchYaw, Quaternion, RotationMatrix)

from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.problem_definition import ProblemDefinition
from pyidto.solver_parameters import SolverParameters

from mpc_utils import Interpolator, ModelPredictiveController

def create_optimizer():
    """
    Create a trajectory optimizer object for the mini cheetah.
    """
    # Problem definition
    problem = ProblemDefinition()
    problem.num_steps = 20
    problem.q_init = np.array([
           1.0, 0.0, 0.0, 0.0,    # base orientation
           0.0, 0.0, 0.29,        # base position
           0.0,-0.8, 1.6,
           0.0,-0.8, 1.6,
           0.0,-0.8, 1.6,
           0.0,-0.8, 1.6])
    problem.v_init = np.zeros(18)
    problem.Qq = np.diag(np.concatenate([
        10 * np.ones(4),   # base orientation
        10 * np.ones(3),  # base position
        0 * np.ones(12),  # legs
    ]))
    problem.Qv = np.diag(np.concatenate([
        1 * np.ones(3),     # base orientation
        1 * np.ones(3),     # base position
        0.1 * np.ones(12),  # legs
    ]))
    problem.R = np.diag(np.concatenate([
        100 * np.ones(3),    # base orientation
        100 * np.ones(3),    # base position
        0.01 * np.ones(12),  # legs
    ]))
    problem.Qf_q = np.diag(np.concatenate([
        10 * np.ones(4),  # base orientation
        10 * np.ones(3),  # base position
        1 * np.ones(12),  # legs
    ]))
    problem.Qf_v = np.diag(np.concatenate([
        1 * np.ones(3),    # base orientation
        1 * np.ones(3),    # base position
        0.1 * np.ones(12), # legs
    ]))

    q_nom = []
    v_nom = []
    for i in range(problem.num_steps + 1):
        x_nom = 0.4 * i / problem.num_steps
        q_nom.append(np.array([
            1.0, 0.0, 0.0, 0.0,    # base orientation
            x_nom, 0.0, 0.29,      # base position
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6
        ]))
        v_nom.append(np.zeros(18))
    problem.q_nom = q_nom
    problem.v_nom = v_nom

    # Solver parameters
    params = SolverParameters()

    params.max_iterations = 1
    params.scaling = True
    params.equality_constraints = True
    params.num_threads = 4

    params.contact_stiffness = 2000
    params.dissipation_velocity = 0.1
    params.smoothing_factor = 0.01
    params.friction_coefficient = 1.0
    params.stiction_velocity = 0.5

    params.verbose = True

    # Create the optimizer 
    time_step = 0.05
    model_file = "../examples/models/mini_cheetah_with_ground.urdf"
    optimizer = TrajectoryOptimizer(model_file, problem, params, time_step)

    return optimizer


def create_initial_guess(num_steps):
    """
    Create an initial guess for the mini cheetah problem.
    """
    q_guess = []
    for _ in range(num_steps + 1):
        q_guess.append(np.array([
            1.0, 0.0, 0.0, 0.0,  # base orientation
            0.0, 0.0, 0.29,      # base position
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6,
            0.0,-0.8, 1.6
        ]))

    return q_guess

class GamepadCommand(LeafSystem):
    """
    A system that reads commands from a gamepad controller conencted to meshcat.
    """
    def __init__(self, meshcat):
        """
        Construct the gamepad command system.

        Args:
            meshcat: The meshcat visualizer object.
        """
        LeafSystem.__init__(self)
        
        self._meshcat = meshcat
        self.DeclareVectorOutputPort("command", 
                                     BasicVector(3), 
                                     self.CalcOutput)

    def CalcOutput(self, context, output):
        """
        Output x velocity, y velocity, and z angular velocity commands.
        """
        gamepad = self._meshcat.GetGamepad()

        if gamepad.index == None:
            # If the gamepad is not connected, send zero commands
            print("Gamepad not connected, sending zero commands.")
            output.SetFromVector(np.zeros(3))
        else:
            output[0] = - gamepad.axes[1]  # x velocity
            output[1] = - gamepad.axes[0]  # y velocity
            output[2] = - gamepad.axes[2]  # z angular velocity


class MiniCheetahMPC(ModelPredictiveController):
    """
    Model predictive controller for the Mini Cheetah quadruped.
    """
    def __init__(self, optimizer, q_guess, nq, nv, mpc_rate):
        ModelPredictiveController.__init__(self, optimizer, q_guess, nq, nv, mpc_rate)

        # Declare an input port for the gamepad commands
        self.gamepad_port = self.DeclareVectorInputPort("gamepad_command", 
                                                        BasicVector(3))
        
    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to move the robot forward.
        """
        # Get the current state
        x0 = self.state_input_port.Eval(context)
        q0 = x0[:self.nq]
        v0 = x0[self.nq:]

        # Quit if the robot has fallen down
        base_height = q0[6]
        assert base_height > 0.0, "Oh no, the robot fell over!"

        # Get the current gamepad command
        gamepad_command = self.gamepad_port.Eval(context)
        vx = 0.5 * gamepad_command[0]
        vy = 0.2 * gamepad_command[1]
        wz = 0.5 * gamepad_command[2]

        # Translate velocity commands in the base frame to the world frame
        current_quat = q0[:4]
        current_quat = Quaternion(current_quat / np.linalg.norm(current_quat))
        R = RotationMatrix(current_quat)
        vx, vy, _ = R.multiply([vx, vy, 0.0])

        # Get the current nominal trajectory (most dimensions won't change)
        prob = self.optimizer.prob()
        q_nom = prob.q_nom
        v_nom = prob.v_nom

        # Update the nominal trajectory to match the gamepad command
        dt = self.optimizer.time_step()
        for i in range(self.num_steps + 1):
            # Forward/backward
            q_nom[i][4] = q0[4] + vx * i * dt
            v_nom[i][4] = vx

            # Side-to-side
            q_nom[i][5] = q0[5] + vy * i * dt
            v_nom[i][5] = vy

            # Orientation
            current_rpy = RollPitchYaw(current_quat).vector()
            target_rpy = np.array([0.0, 0.0, current_rpy[2] + wz * i * dt])
            target_quat = RollPitchYaw(target_rpy).ToQuaternion().wxyz()

            if np.dot(current_quat.wxyz(), target_quat) < 0:
                # Ensure the quaternion is always the shortest path
                target_quat = -target_quat
            q_nom[i][:4] = target_quat
            v_nom[i][3] = wz

        self.optimizer.UpdateNominalTrajectory(q_nom, v_nom)

if __name__ == "__main__":
    meshcat = StartMeshcat()

    # Set up a Drake diagram for simulation
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)
    models = Parser(plant).AddModels("../examples/models/mini_cheetah_with_ground.urdf")

    # Add implicit PD controllers (must use kLagged or kSimilar)
    Kp = 50 * np.ones(plant.num_actuators())
    Kd = 2 * np.ones(plant.num_actuators())
    actuator_indices = [JointActuatorIndex(i) for i in range(plant.num_actuators())]
    for actuator_index, Kp, Kd in zip(actuator_indices, Kp, Kd):
        plant.get_joint_actuator(actuator_index).set_controller_gains(
            PdControllerGains(p=Kp, d=Kd))

    plant.Finalize()

    # Set up the trajectory optimization problem
    optimizer = create_optimizer()
    q_guess = create_initial_guess(optimizer.num_steps())

    # Create the MPC controller and interpolator systems
    mpc_rate = 60  # Hz
    nq, nv = plant.num_positions(), plant.num_velocities()
    controller = builder.AddSystem(MiniCheetahMPC(
        optimizer, q_guess, nq, nv, mpc_rate))

    Bv = plant.MakeActuationMatrix()
    N = plant.MakeVelocityToQDotMap(plant.CreateDefaultContext())
    Bq = N@Bv
    interpolator = builder.AddSystem(Interpolator(Bq.T, Bv.T))
    
    # Add a gamepad command system
    gamepad = builder.AddSystem(GamepadCommand(meshcat))

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
    builder.Connect(
        gamepad.get_output_port(),
        controller.GetInputPort("gamepad_command")
    )
    
    # Connect the plant to meshcat for visualization
    AddDefaultVisualization(builder, meshcat)

    # Build the system diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # Set the initial state
    q0 = np.array([
           1.0, 0.0, 0.0, 0.0,  # base orientation
           0.0, 0.0, 0.29,      # base position
           0.0,-0.8, 1.6,       # legs
           0.0,-0.8, 1.6,
           0.0,-0.8, 1.6,
           0.0,-0.8, 1.6])
    v0 = np.zeros(plant.num_velocities())
    plant.SetPositions(plant_context, q0)
    plant.SetVelocities(plant_context, v0)

    # Simulate and play back on meshcat
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)
