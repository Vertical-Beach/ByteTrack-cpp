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

# Install OpenCV 3.4.16
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

ENV OPENCV_VERSION="3.4.16"
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip && \
    unzip ${OPENCV_VERSION}.zip -d . && \
    mkdir /tmp/opencv/opencv-${OPENCV_VERSION}/build && cd /tmp/opencv/opencv-${OPENCV_VERSION}/build/ && \
    cmake -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D WITH_FFMPEG=ON -D WITH_TBB=ON .. | tee /tmp/opencv_cmake.log && \
    make -j "$(nproc)" | tee /tmp/opencv_build.log && \
    make install | tee /tmp/opencv_install.log && \
    cd /tmp && rm -rf opencv

WORKDIR /usr/src/app
