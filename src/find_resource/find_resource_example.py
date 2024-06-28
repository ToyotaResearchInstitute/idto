# SPDX-License-Identifier: MIT-0

"""Test the Drake find resource API."""

from pydrake.common import FindResourceOrThrow

FindResourceOrThrow('drake/examples/pendulum/Pendulum.urdf')
