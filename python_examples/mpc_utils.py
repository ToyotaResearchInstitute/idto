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
    q = None           # A PiecewisePolynomial representing the actuated joint positions
    v = None           # A PiecewisePolynomial representing the actuated joint velocities
    u = None           # A PiecewisePolynomial representing the control inputs (torques)


class Interpolator(LeafSystem):
    """
    A simple Drake system that interpolates a StoredTrajectory to provice the
    actuated state reference x(t) and input u(t) at a given time t. This is
    useful for passing a reference trajectory to a low-level controller.
    """
    def __init__(self, num_actuators):
        """
        Construct the interpolator system, which takes StoredTrajectory as input
        and produces the state and input at a given time.

                             ------------------
                             |                | --->  x(t)
            trajectory --->  |  Interpolator  |
                             |                | --->  u(t)
                             ------------------

        Args:
            nq: The number of generalized coordinates
            nv: The number of generalized velocities
            nu: The number of control inputs
        """
        LeafSystem.__init__(self)

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
        raise NotImplementedError
        #t = context.get_time()
        #trajectory = self.EvalAbstractInput(context, 0).get_value()
        #q = trajectory.q.value(t)
        #v = trajectory.v.value(t)

        #output.SetFromVector(np.concatenate((q, v)))

    def SendControl(self, context, output):
        """
        Send the control input at the current time.
        """
        trajectory = self.EvalAbstractInput(context, 0).get_value()
        u = trajectory.u.value(context.get_time() - trajectory.start_time)

        print(f"At send time {context.get_time()}, t0 = {trajectory.start_time}, u = {u}")

        output.SetFromVector(u)
