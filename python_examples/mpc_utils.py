##
#
# Helper utilities for performing MPC in simulation with pyidto.
#
##

import numpy as np
from pydrake.all import LeafSystem, Value, BasicVector


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
                                                         BasicVector(2 * num_actuators), 
                                                         self.SendState)
        control_output_port = self.DeclareVectorOutputPort("control",
                                                         BasicVector(num_actuators),
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
