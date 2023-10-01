# Inverse Dynamics Trajectory Optimization 

Implements the contact-implicit trajectory optimization algorithm described in
the paper

[Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive
Control](https://idto.github.io/) by Vince Kurtz, Alejandro Castro, Aykut Özgün
Önol, and Hai Lin. https://arxiv.org/abs/2309.01813.

## Dependencies

This software is built on [Drake](https://drake.mit.edu). You do not need a
separate Drake installation, but all the requirements for [building Drake from
source](https://drake.mit.edu/from_source.html) apply. Most notably, that
includes Bazel and a C++17 compiler. 

The easiest way to install these dependencies is with Drake's
[`install_prereqs.sh`](https://drake.mit.edu/from_source.html#mandatory-platform-specific-instructions)
script.

## Installation

Clone this repository:
```
git clone https://github.com/ToyotaResearchInstitute/idto
cd idto
```

Install the dependencies (see above).

Compile the package:
```
bazel build //...
```

## Examples

The `examples` folder contains various examples, including those described in
[our paper](https://idto.github.io). Run them with, e.g.,
```
bazel run //examples/spinner:spinner
```
A link will appear (e.g., `http://localhost:7000`), which you can use to open
the Meshcat visualizer in a web browser. 

Most of the examples (e.g., `spinner`) run a simulation with contact-implicit
model predictive control. Some others (e.g., `kuka`) perform a single open-loop
trajectory optimization.

Problem definitions, solver parameters, whether to run MPC, etc. are set in YAML
config files, e.g., `spinner.yaml`.

## Other Tips and Tricks

### Enable OpenMP parallelization

Our solver supports parallel derivative computations with OpenMP. Enable OpenMP
with the build flag `--config=omp` and set `num_threads : N` in the YAML config
file for each example. 

You may want to want to add
```
build --config=omp
```
to `user.bazelrc` (create this file in `/path/to/idto/` if it does not exist) to
enable OpenMP by default. 

### Use an existing Drake installation

By default, Bazel pulls in a copy of Drake as an external and compiles it. If you have an existing local checkout at `/home/user/stuff/drake` that you would like to use instead, set the environment variable
```
export IDTO_LOCAL_DRAKE_PATH=/home/user/stuff/drake
```
before building.

### Run unit tests

Run a (fairly minimal) set of unit tests and lint checks with
```
bazel test //...
```
