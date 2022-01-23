FROM ubuntu:20.04
ENV DEBIAN_FRONTEND noninteractive

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    gdb \
    git \
    ca-certificates \
    libssl-dev \
    pkg-config \
    wget \
    unzip \
    vim \
    tmux && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install GTest
RUN apt update && apt install -y --no-install-recommends \
    libgtest-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install Boost
RUN apt update && apt install -y --no-install-recommends \
    libboost-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install latest CMake
RUN git clone -b release --depth=1 https://github.com/Kitware/CMake.git && cd CMake && \
    ./bootstrap && make -j "$(nproc)" && make install && \
    cd ../ && rm -rf CMake

# Install Eigen 3.3.9
ENV EIGEN_VERSION="3.3.9"
RUN mkdir -p /tmp/eigen && cd /tmp/eigen && \
    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip && \
    unzip eigen-${EIGEN_VERSION}.zip -d . && \
    mkdir /tmp/eigen/eigen-${EIGEN_VERSION}/build && cd /tmp/eigen/eigen-${EIGEN_VERSION}/build/ && \
    cmake .. && \
    make install && \
    cd /tmp && rm -rf eigen
