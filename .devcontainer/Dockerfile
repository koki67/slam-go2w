FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

# Build GTSAM from source — libgtsam-dev is not available in default Ubuntu 22.04 repos
# and the old PPA (mpickering/gtsam) is unreliable.
RUN apt-get update && apt-get install -y \
        cmake \
        git \
        libeigen3-dev \
        libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --branch 4.2.0 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && mkdir /tmp/gtsam/build && cd /tmp/gtsam/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
    && make -j$(nproc) && make install \
    && ldconfig \
    && rm -rf /tmp/gtsam

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgoogle-glog-dev \
    libomp-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
