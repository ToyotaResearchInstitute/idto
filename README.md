# Inverse Dynamics Trajectory Optimization 

Implements the contact-implicit trajectory optimization algorithm described in

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
script:

```
git clone https://github.com/RobotLocomotion/drake.git
cd drake
sudo ./setup/ubuntu/install_prereqs.sh
```

For Mac OS, replace the last line with `./setup/mac/install_prereqs.sh`.

## Installation

Install the dependencies (see above).

Clone this repository:
```
git clone https://github.com/ToyotaResearchInstitute/idto
cd idto
```

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
config files, e.g., `spinner.yaml`. Here are some common options:

- `mpc : {true, false}` choose whether or not to run MPC. If this is set to
  true, Meshcat will show and record a simulation where IDTO is used as an MPC
  controller. 
- `num_threads : N` sets the number of threads used for parallel derivative
  computations.
- `play_target_trajectory : {true, false}` whether to play an animation of the
  target trajectory over Meshcat. 
- `play_initial_guess : {true, false}` whether to play an animation of the
  initial guess over Meshcat.
- `play_optimal_trajectory : {true, false}` whether to play an animation of the
  optimal trajectory over Meshcat. This is not a simulation: the generated
  trajectory may or may not be dynamically feasible. 

**NOTE** 
If Meshcat plays multiple things, only the last one will be recorded for
playback via the dropdown menu. For example, if `play_target_trajectory`,
`play_optimal_trajectory`, and `mpc` are all set to `true`, Meshcat will first
play the target trajectory, followed by the open-loop solution, followed by a
simulation with MPC. Only the simulation will be saved for playback.

## Other Tips and Tricks

### Use an existing Drake installation

By default, Bazel pulls in a copy of Drake as an external and compiles it. If
you have an existing local checkout at `/home/user/stuff/drake` that you would
like to use instead, set the environment variable
```
export IDTO_LOCAL_DRAKE_PATH=/home/user/stuff/drake
```
before building.

### Run unit tests

Run a (fairly minimal) set of unit tests and lint checks with
```
bazel test //...
```

## Contributing

We welcome your contributions, whether they are bug fixes, solver improvements,
or new features! To make a contribution:

1. Open a new pull request
2. Make sure all the unit tests and lint checks pass
3. Obtain a review from a code owner (e.g., @vincekurtz, @amcastro-tri, or
   @aykut-tri)
4. Once the review is approved, we'll merge it into the `main` branch!

Since this is research code, we will not review to the same production-quality
standards as Drake. Nonetheless, new contributions should be clean and
well-documented, and unit tests are encouraged. The standard of review should be
*improving the health of the codebase* rather than perfection.
