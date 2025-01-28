# Use a base image suitable for cross-compilation (a base image with ARM64 toolchain)
FROM debian:bullseye

# Install essential tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    qemu-user-static \
    rsync \
    vim \
    && apt-get clean

# Install dependencies (Eigen, CppAD, Ceres Solver, OpenCV)
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libcppad-dev \
    libopencv-dev \
    && apt-get clean

# Install Ceres Solver
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    && apt-get clean \
    && git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver \
    && cd ceres-solver \
    && mkdir build && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr \
    && make -j$(nproc) \
    && make install \
    && cd ../../ && rm -rf ceres-solver

# Set default working directory
WORKDIR /workspace

# Entry point for the container
CMD ["/bin/bash"]
