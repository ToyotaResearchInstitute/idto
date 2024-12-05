# syntax=docker/dockerfile:1

FROM ubuntu:jammy
SHELL ["/bin/bash", "-c", "-i"]
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

WORKDIR /home

# Set up python 3.10 and alias it with 'python'
RUN apt update -y
RUN apt install -y \
    build-essential \
    vim \
    cmake \
    git \
    python3.10 \
    python-is-python3 \
    wget \ 
    python3-pip

# Run drake installation
RUN wget https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-1.30.0-jammy.tar.gz && \
    tar -xvzf drake-1.30.0-jammy.tar.gz && \
    rm drake-1.30.0-jammy.tar.gz
WORKDIR /home/drake
RUN ./share/drake/setup/install_prereqs -y && \
    apt-get install -y libgflags-dev && \
    pip install scipy==1.12 && \ 
    pip install scikit-learn #Scipy install is after the numpy installation from Drake, for some python examples
RUN echo 'export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/home/drake' >> ~/.bashrc && \
    echo 'export PYTHONPATH=${PYTHONPATH}:/home/drake/lib/python3.10/site-packages' >> ~/.bashrc && \
    source ~/.bashrc
 
# Copy the current directory contents into the container, then build IDTO
COPY . /home/idto
WORKDIR /home/idto
RUN mkdir build 
WORKDIR /home/idto/build
RUN cmake .. && make -j
WORKDIR /home/idto/
