# SPDX-License-Identifier: MIT-0

"""
Provides an example of using pybind11-bound Drake C++ system with pydrake.
"""

from __future__ import print_function

from simple_adder import SimpleAdder, SimpleAdder_

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import (
    ConstantVectorSource,
    VectorLogSink,
)


def main():
    # Simulate with doubles.
    builder = DiagramBuilder()
    source = builder.AddSystem(ConstantVectorSource([10.]))
    adder = builder.AddSystem(SimpleAdder(100.))
    builder.Connect(source.get_output_port(0), adder.get_input_port(0))
    logger = builder.AddSystem(VectorLogSink(1))
    builder.Connect(adder.get_output_port(0), logger.get_input_port(0))
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.AdvanceTo(1)

    x = logger.FindLog(simulator.get_context()).data()
    print("Output values: {}".format(x))
    assert np.allclose(x, 110.)

    # Compute outputs with AutoDiff and Symbolic.
    for T in (AutoDiffXd, Expression):
        adder_T = SimpleAdder_[T](100.)
        context = adder_T.CreateDefaultContext()
        adder_T.get_input_port().FixValue(context, [10.])
        output = adder_T.AllocateOutput()
        adder_T.CalcOutput(context, output)
        # N.B. At present, you cannot get a reference to existing AutoDiffXd
        # or Expression numpy arrays, so we will explictly copy the vector:
        # https://github.com/RobotLocomotion/drake/issues/8116
        value, = output.get_vector_data(0).CopyToVector()
        assert isinstance(value, T)
        print("Output from {}: {}".format(type(adder_T), repr(value)))


if __name__ == "__main__":
    main()
