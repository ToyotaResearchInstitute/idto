# SPDX-License-Identifier: MIT-0

"""
Provides an example of using pybind11-bound Drake C++ system with pydrake.
"""

from __future__ import print_function

from simple_bindings import SimpleAdder

import numpy as np

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    DiagramBuilder,
)
from pydrake.systems.primitives import (
    ConstantVectorSource,
    VectorLogSink,
)


def main():
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


if __name__ == "__main__":
    main()
