#!/usr/bin/env python

##
#
# An example of using the python bindings to perform Model Predictive Control
# on the spinner.
#
##


import numpy as np

from pydrake.all import (AddDefaultVisualization, AddMultibodyPlantSceneGraph,
                         DiagramBuilder, Parser, Simulator, StartMeshcat,
                         PdControllerGains, JointActuatorIndex,
                         DiscreteContactApproximation)

from pyidto import (
    TrajectoryOptimizer,
    SolverParameters,
    ProblemDefinition,
    FindIdtoResource,
)

from mpc_utils import Interpolator, ModelPredictiveController


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
        q_nom.append(np.array([0.3, 1.5, 2.0 * i / problem.num_steps]))
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
    for _ in range(num_steps + 1):
        q_guess.append(np.array([0.3, 1.5, 0.0]))

    return q_guess


class SpinnerMPC(ModelPredictiveController):
    """
    Model predictive controller for the spinner.
    """

    def __init__(self, optimizer, q_guess, nq, nv, mpc_rate):
        ModelPredictiveController.__init__(
            self, optimizer, q_guess, nq, nv, mpc_rate)

    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to account for the current state,
        so that the spinner's reference position is always ahead of the
        current position.
        """
        # Get the current configuration
        x0 = self.state_input_port.Eval(context)
        q0 = x0[:self.nq]

        # Update the nominal trajectory
        prob = self.optimizer.prob()
        q_nom = prob.q_nom
        v_nom = prob.v_nom
        q0_nom_old = prob.q_nom[0]
        for i in range(self.num_steps + 1):
            q_nom[i][2] += q0[2] - q0_nom_old[2]
        self.optimizer.UpdateNominalTrajectory(q_nom, v_nom)


if __name__ == "__main__":
    model_file = FindIdtoResource("idto/models/spinner_friction.urdf")

    # Set up a Drake diagram for simulation
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    plant.set_discrete_contact_approximation(
        DiscreteContactApproximation.kLagged)
    models = Parser(plant).AddModels(model_file)

    # Add implicit PD controllers (must use kLagged or kSimilar)
    Kp = 100 * np.ones(plant.num_actuators())
    Kd = 10 * np.ones(plant.num_actuators())
    actuator_indices = [JointActuatorIndex(
        i) for i in range(plant.num_actuators())]
    for actuator_index, Kp, Kd in zip(actuator_indices, Kp, Kd):
        plant.get_joint_actuator(actuator_index).set_controller_gains(
            PdControllerGains(p=Kp, d=Kd))

    plant.Finalize()

    # Set up the trajectory optimization problem. Note that this uses a different
    # plant with a larger timestep than the one used for simulation.
    problem = define_spinner_optimization_problem()
    params = define_spinner_solver_parameters()
    q_guess = define_spinner_initial_guess(problem.num_steps)
    ctrl_builder = DiagramBuilder()
    ctrl_plant, _ = AddMultibodyPlantSceneGraph(ctrl_builder, 0.05)
    Parser(ctrl_plant).AddModels(model_file)
    ctrl_plant.Finalize()
    ctrl_diagram = ctrl_builder.Build()
    optimizer = TrajectoryOptimizer(ctrl_diagram, ctrl_plant, problem, params)

    # Create the MPC controller and interpolator systems
    mpc_rate = 200  # Hz
    nq, nv = 3, 3
    controller = builder.AddSystem(SpinnerMPC(
        optimizer, q_guess, nq, nv, mpc_rate))

    Bq = np.array([[1, 0, 0], [0, 1, 0]])
    Bv = np.array([[1, 0, 0], [0, 1, 0]])
    interpolator = builder.AddSystem(Interpolator(Bq, Bv))

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
