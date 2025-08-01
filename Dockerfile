# Use a base image suitable for cross-compilation (a base image with ARM64 toolchain)
FROM debian:bullseye

# Install essential tools
RUN apt update && apt install -y \
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

# Install dependencies (Eigen, CppAD, Ceres Solver, OpenCV, SDL2)
RUN apt update && apt install -y \
    libeigen3-dev \
    libcppad-dev \
    libopencv-dev \
    libsdl2-dev \
    && apt-get clean

# Install Google Test & Ceres Solver
RUN apt update && apt install -y \
    libgoogle-glog-dev \
    libabsl-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    && apt-get clean \
    && git clone https://github.com/google/googletest.git \
    && cd googletest \
    && mkdir build && cd build \
    && cmake .. -DBUILD_GMOCK=ON \
    && make -j4 \
    && make install \
    && cd ../../ && rm -rf googletest \
    && git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver \
    && cd ceres-solver \
    && mkdir build && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr \
    && make -j4 \
    && make install \
    && cd ../../ && rm -rf ceres-solver

# Set default working directory
WORKDIR /workspace

# Entry point for the container
CMD ["/bin/bash"]
