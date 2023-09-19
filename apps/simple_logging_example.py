# SPDX-License-Identifier: MIT-0

"""
Provides an example of using pydrake from a Bazel external.
"""

from __future__ import print_function

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
    logger = builder.AddSystem(VectorLogSink(1))
    builder.Connect(source.get_output_port(0), logger.get_input_port(0))
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.AdvanceTo(1)

    x = logger.FindLog(simulator.get_context()).data()
    print("Output values: {}".format(x))
    assert np.allclose(x, 10.)


if __name__ == "__main__":
    main()
