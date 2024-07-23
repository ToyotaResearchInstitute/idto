# Inverse Dynamics Trajectory Optimization 

Implements the contact-implicit trajectory optimization algorithm described in

[Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive
Control](https://idto.github.io/) by Vince Kurtz, Alejandro Castro, Aykut Özgün
Önol, and Hai Lin. https://arxiv.org/abs/2309.01813.

## Prerequisites

The only dependency is [Drake](https://drake.mit.edu/installation.html). 
We recommend Ubuntu 22.04 and Drake v1.30.0. Other configurations may work
but are untested.

For example, for a binary Drake installation at `$HOME/drake`:

Download and extract the binaries:

```bash
cd $HOME
wget https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-1.30.0-jammy.tar.gz
tar -xvzf drake-1.30.0-jammy.tar.gz
```

Install Drake dependencies and gflags:

```bash
cd $HOME/drake
sudo ./share/drake/setup/install_prereqs
sudo apt-get install libgflags-dev
```

Update environment variables:

```bash
echo 'export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${HOME}/drake' >> ~/.bashrc
echo 'export PYTHONPATH=${PYTHONPATH}:${HOME}/drake/lib/python3.10/site-packages' >> ~/.bashrc
source ~/.bashrc
```

## Installation

### C++

Clone this repository:

```bash
git clone https://github.com/ToyotaResearchInstitute/idto/
```

Configure and build:

```bash
cd idto
mkdir build && cd build
cmake ..
make -j
```

(Optional) run unit tests:

```bash
ctest
```

### Python

A limited subset of functionality is available via python bindings.

Build and install with pip:

```bash
git clone https://github.com/ToyotaResearchInstitute/idto/
cd idto
pip install .
```

(Optional) run unit tests:

```bash
pytest
```

**Note:** The python bindings assume that the `pydrake` version you are using 
is the one used to build `pyidto`. A separate `pydrake` installation (e.g.,
from `pip install drake`) may not work.

## Examples

All of the examples use meshcat for visualization. A link will appear in the
terminal, or just go ahead and open http://localhost:7000 in a browser.

### C++

After building, run the spinner example with:

```
./build/examples/spinner/spinner
```

### Python

After installing with pip, run the spinner example with:

```
./python_examples/spinner_open_loop.py
```
