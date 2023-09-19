#!/bin/bash
# SPDX-License-Identifier: MIT-0

# Proxy script for things such as passing a `py_binary` to a `sh_test`, since
# we cannot list the Python binary in `srcs` for the test.
exec "$@"
