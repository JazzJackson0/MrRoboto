FROM debian:bullseye

ENV DEBIAN_FRONTEND=noninteractive

# Add ARM64 architecture
RUN dpkg --add-architecture arm64

# Install essential tools
RUN apt-get update && apt-get install -y --fix-missing \
    crossbuild-essential-arm64 \
    cmake \
    git \
    wget \
    rsync \
    vim \
    pkg-config \
    && apt-get clean

# Install ARM64 dependencies (Eigen, CppAD, OpenCV, SDL2)
RUN apt-get update && apt-get install -y \
    libcppad-dev:arm64 \
    libopencv-dev:arm64 \
    libsdl2-dev:arm64 \
    && apt-get clean

# Install Google Test & Ceres Solver dependencies (ARM64)
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev:arm64 \
    libabsl-dev:arm64 \
    libgflags-dev:arm64 \
    libatlas-base-dev:arm64 \
    libsuitesparse-dev:arm64 \
    && apt-get clean

# Create ARM64 CMake toolchain file
RUN mkdir -p /toolchains && echo "\
set(CMAKE_SYSTEM_NAME Linux)\n\
set(CMAKE_SYSTEM_PROCESSOR aarch64)\n\
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)\n\
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)\n\
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)\n" > /toolchains/arm64-toolchain.cmake
# BUILD CMAKE WITH: cmake .. -DCMAKE_TOOLCHAIN_FILE=/toolchains/arm64-toolchain.cmake

# Build Eigen (for ARM64, with CMake config files)
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
    tar xf eigen-3.4.0.tar.gz && cd eigen-3.4.0 && mkdir build && cd build && \
    cmake .. -DCMAKE_TOOLCHAIN_FILE=/toolchains/arm64-toolchain.cmake -DCMAKE_INSTALL_PREFIX=/usr && \
    make install && \
    cd ../../ && rm -rf eigen-3.4.0*

# Build Google Test (cross-compiled for ARM64)
RUN git clone https://github.com/google/googletest.git && \
    cd googletest && mkdir build && cd build && \
    cmake .. -DBUILD_GMOCK=ON -DCMAKE_TOOLCHAIN_FILE=/toolchains/arm64-toolchain.cmake && \
    make -j$(nproc) && make install && \
    cd ../../ && rm -rf googletest

RUN find /usr -name Eigen3Config.cmake

# Build Ceres Solver for ARM64
RUN git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. -DBUILD_SHARED_LIBS=ON \
             -DCMAKE_INSTALL_PREFIX=/usr \
             -DCMAKE_TOOLCHAIN_FILE=/toolchains/arm64-toolchain.cmake \
             -DEigen3_DIR=/usr/lib/aarch64-linux-gnu/cmake/eigen3 \
             -DCMAKE_CXX_FLAGS="-DABSL_RANDOM_INTERNAL_AES_DISPATCH=0 -DABSL_RANDOM_INTERNAL_AES_IMPL=0" && \
    make -j$(nproc) && make install && \
    cd ../../ && rm -rf ceres-solver

# Set default working directory
WORKDIR /workspace

CMD ["/bin/bash"]


# docker build --memory=12g --memory-swap=16g -t rpi-cc-img .







# # Use a base image suitable for cross-compilation (a base image with ARM64 toolchain)
# FROM debian:bullseye

# # Install essential tools
# RUN apt-get update && apt-get install -y --fix-missing \
#     crossbuild-essential-arm64 \
#     cmake \
#     git \
#     wget \
#     rsync \
#     vim \
#     && apt-get clean

# # Install dependencies (Eigen, CppAD, Ceres Solver, OpenCV, SDL2)
# RUN apt-get update && apt-get install -y \
#     libeigen3-dev \
#     libcppad-dev \
#     libopencv-dev \
#     libsdl2-dev \
#     && apt-get clean

# # Install Google Test & Ceres Solver
# RUN apt-get update && apt-get install -y \
#     libgoogle-glog-dev \
#     libabsl-dev \
#     libgflags-dev \
#     libatlas-base-dev \
#     libsuitesparse-dev \
#     && apt-get clean \
#     && git clone https://github.com/google/googletest.git \
#     && cd googletest \
#     && mkdir build && cd build \
#     && cmake .. -DBUILD_GMOCK=ON \
#     && make -j3 \
#     && make install \
#     && cd ../../ && rm -rf googletest \
#     && git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver \
#     && cd ceres-solver \
#     && mkdir build && cd build \
#     && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr \
#     && make -j1 \
#     && make install \
#     && cd ../../ && rm -rf ceres-solver

# # Set default working directory
# WORKDIR /workspace

# # Entry point for the container
# CMD ["/bin/bash"]
