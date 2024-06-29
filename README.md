# Inverse Dynamics Trajectory Optimization 

Implements the contact-implicit trajectory optimization algorithm described in

[Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive
Control](https://idto.github.io/) by Vince Kurtz, Alejandro Castro, Aykut Özgün
Önol, and Hai Lin. https://arxiv.org/abs/2309.01813.

## Supported Configurations

Ubuntu 22.04 and Drake 1.30.0.

## Installation

### C++

1. [Install](https://drake.mit.edu/installation.html) Drake 1.30.0. For example,
   for a binary installation at `$HOME/drake`:
  
```bash
cd $HOME
wget https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-1.30.0-jammy.tar.gz
tar -xvzf drake-1.30.0-jammy.tar.gz
echo 'export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${HOME}/drake' >> ~/.bashrc
echo 'export PYTHON_PATH=${PYTHONPATH}:${HOME}/drake/lib/python3.10/site-packages' >> ~/.bashrc
source ~/.bashrc
```

2. Install Drake dependencies:

```bash
cd $HOME/drake
sudo ./share/drake/setup/install_prereqs
```

3. Clone this repository:

```bash
cd $HOME
git clone https://github.com/ToyotaResearchInstitute/idto/
```

4. Configure and build the repository:

```bash
cd $HOME/idto
mkdir build && cd build
cmake ..
make -j
```

5. (Optional) run unit tests:

```bash
cd $HOME/idto/build
ctest
```

### Python

1. Install Drake 1.30.0 and clone this repository (steps 1-3 above).

2. Build and install with pip:

```bash
cd $HOME/idto
pip install .
```

3. (Optional) run unit tests:

```bash
cd $HOME/idto
pytest
```
