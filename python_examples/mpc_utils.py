##
#
# Helper utilities for performing MPC in simulation with pyidto.
#
##

import numpy as np
from pydrake.all import (
    LeafSystem,
    BasicVector,
    EventStatus,
    PiecewisePolynomial,
    Value,
)

from pyidto import TrajectoryOptimizerSolution, TrajectoryOptimizerStats


class StoredTrajectory:
    """
    A simple class for storing a polynomial representation of a trajectory
    that we might get from pyidto.
    """
    start_time = None  # The time at which the trajectory starts
    q = None           # A PiecewisePolynomial representing the generalized coordinates
    v = None           # A PiecewisePolynomial representing the generalized velocities
    tau = None         # A PiecewisePolynomial representing the generalized forces


class Interpolator(LeafSystem):
    """
    A simple Drake system that interpolates a StoredTrajectory to provide the
    actuated state reference x(t) and input u(t) at a given time t. This is
    useful for passing a reference trajectory to a low-level controller.
    """

    def __init__(self, Bq, Bv):
        """
        Construct the interpolator system, which takes StoredTrajectory as input
        and produces the state and input at a given time.

                             ------------------
                             |                | --->  x(t)
            trajectory --->  |  Interpolator  |
                             |                | --->  u(t)
                             ------------------

        Args:
            Bq: Actuated DoF selection matrix for generalized coordinates
            Bv: Actuator selection matrix for generalized velocities and forces
        """
        LeafSystem.__init__(self)

        # Check that the actuated selection matrices are the right size
        num_actuators = Bq.shape[0]
        assert Bv.shape[0] == num_actuators

        self.Bq = Bq
        self.Bv = Bv

        # Declare the input and output ports
        trajectory_input_port = self.DeclareAbstractInputPort("trajectory",
                                                              Value(StoredTrajectory()))
        state_output_port = self.DeclareVectorOutputPort("state",
                                                         BasicVector(
                                                             2 * num_actuators),
                                                         self.SendState)
        control_output_port = self.DeclareVectorOutputPort("control",
                                                           BasicVector(
                                                               num_actuators),
                                                           self.SendControl)

    def SendState(self, context, output):
        """
        Send the state at the current time.
        """
        trajectory = self.EvalAbstractInput(context, 0).get_value()
        t = context.get_time() - trajectory.start_time
        q = self.Bq @ trajectory.q.value(t)
        v = self.Bv @ trajectory.v.value(t)
        output.SetFromVector(np.concatenate((q, v)))

    def SendControl(self, context, output):
        """
        Send the control input at the current time.
        """
        trajectory = self.EvalAbstractInput(context, 0).get_value()
        u = self.Bv @ trajectory.tau.value(context.get_time() -
                                           trajectory.start_time)
        output.SetFromVector(u)


class ModelPredictiveController(LeafSystem):
    """
    A Drake system that implements an MPC controller.
    """

    def __init__(self, optimizer, q_guess, nq, nv, mpc_rate):
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
            q_guess: An initial guess for the trajectory optimization problem.
            nq: The number of generalized coordinates
            nv: The number of generalized velocities
            mpc_rate: The rate at which the MPC problem is to be solved (Hz)
        """
        LeafSystem.__init__(self)

        self.optimizer = optimizer
        self.nq = nq

        # Allocate a warm-start
        self.q_guess = q_guess
        self.warm_start = self.optimizer.CreateWarmStart(self.q_guess)

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = self.optimizer.time_step()
        self.num_steps = self.optimizer.num_steps()

        # Solve the optimization problem to get the initial trajectory
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        # Declare an abstract-valued state that will hold the optimal trajectory
        state = self.StoreOptimizerSolution(solution, 0.0)
        self.stored_trajectory = self.DeclareAbstractState(Value(state))

        # Define a periodic update event that will trigger the optimizer to
        # resolve the MPC problem.
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            1. / mpc_rate, 0, self.UpdateAbstractState)

        # Declare the input and output ports
        self.state_input_port = self.DeclareVectorInputPort(
            "state", BasicVector(nq + nv))
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
        time_steps = np.linspace(
            0, self.time_step * self.num_steps, self.num_steps + 1)
        q_knots = np.array(solution.q).T
        v_knots = np.array(solution.v).T
        tau_knots = solution.tau
        tau_knots.append(solution.tau[-1])  # Repeat the last control input
        tau_knots = np.array(tau_knots).T

        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = start_time
        trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, q_knots)
        trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, v_knots)
        trajectory.tau = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, tau_knots)

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

        # Shift the warm-start based on the stored interpolation and time elapsed
        last_trajectory = state.get_abstract_state(0).get_value()
        self.q_guess[0] = q0
        start_time = context.get_time() - last_trajectory.start_time
        for i in range(1, self.num_steps + 1):
            t = start_time + i * self.time_step
            self.q_guess[i] = last_trajectory.q.value(t).flatten()
        self.warm_start.set_q(self.q_guess)

        # Shift the nominal trajectory as needed
        self.UpdateNominalTrajectory(context)

        # Solve the optimization problem
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        # Store the solution in the abstract state
        state.get_mutable_abstract_state(0).SetFrom(
            Value(self.StoreOptimizerSolution(solution, context.get_time())))

        return EventStatus.Succeeded()

    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to account for the current state.
        This may or may not be useful depending on the specifics of the task.
        """
        pass
