# Bazel Project with Drake as an External

This pulls in Drake via the Bazel workspace mechanism.

For an introduction to Bazel, refer to
[Getting Started with Bazel](https://docs.bazel.build/versions/master/getting-started.html).

## Instructions

First, install the required Ubuntu packages:

```
sudo ../scripts/setup/linux/ubuntu/focal/install_prereqs
```

Then, to build and test all apps:
```
bazel test //...
```

As an example to run a binary directly:
```
bazel run //apps:simple_logging_example
```

You may also run the binary directly per the `bazel-bin/...` path that the
above command prints out; however, be aware that your working directories may
cause differences.  This is important when using tools like
`drake::FindResource` / `pydrake.common.FindResource`.
You may generally want to stick to using `bazel run` when able.

## Python Versions

By default, Python 3 is the Python interpreter that Drake will use when built
with Bazel. To see which Python versions are supported, see the
[supported configurations](https://drake.mit.edu/developers.html#supported-configurations).
