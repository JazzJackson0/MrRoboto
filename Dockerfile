# Use an x86 Debian base to avoid QEMU crashes
FROM debian:bullseye

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install essential tools and cross-compilers
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    rsync \
    vim \
    pkg-config \
    python3 \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for OpenCV, Ceres, Google Test
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create ARM64 CMake toolchain file
RUN mkdir -p /toolchain && echo "\
set(CMAKE_SYSTEM_NAME Linux)\n\
set(CMAKE_SYSTEM_PROCESSOR aarch64)\n\
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)\n\
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)\n\
set(CMAKE_FIND_ROOT_PATH /usr /usr/aarch64-linux-gnu)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)\n\
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)\n\
" > /toolchain/arm64-toolchain.cmake

# Build Google Test for ARM64
RUN git clone https://github.com/google/googletest.git /tmp/googletest \
    && mkdir /tmp/googletest/build \
    && cd /tmp/googletest/build \
    && cmake .. \
        # Use custom toolchain file for cross-compilation
        -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake \
        # Also build Google Mock libraries
        -DBUILD_GMOCK=ON \
    && make -j$(nproc) \
    && make install \
    && cd / && rm -rf /tmp/googletest

# -------- SDL2 (ARM64) ----------
# Pin to a stable release for reproducible builds
RUN git clone --branch release-2.30.9 --depth=1 https://github.com/libsdl-org/SDL.git sdl2 \
 && cmake -S sdl2 -B sdl2/build \
      # Use custom toolchain file for cross-compilation
      -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake \
      # Type of build to generate: Debug, Release, RelWithDebInfo, MinSizeRel
      -DCMAKE_BUILD_TYPE=Release \
      # Set installation location
      -DCMAKE_INSTALL_PREFIX=/usr/aarch64-linux-gnu \
      # Where libraries are installed when you run make install
      -DCMAKE_INSTALL_LIBDIR=lib \
 && cmake --build sdl2/build -j \
 && cmake --install sdl2/build \
 && rm -rf sdl2

 # Set PKG_CONFIG_PATH so CMake/pkg-config can find SDL2
ENV PKG_CONFIG_PATH=/usr/aarch64-linux-gnu/lib/pkgconfig:$PKG_CONFIG_PATH

 # ---------------- CppAD (for ARM64. Header-only, like apt's libcppad-dev)
RUN git clone --depth 1 https://github.com/coin-or/CppAD.git /tmp/cppad \
 && cd /tmp/cppad \
 && mkdir build && cd build \
 && cmake .. \
      # Use custom toolchain file for cross-compilation
      -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake \
      # Set installation location
      -DCMAKE_INSTALL_PREFIX=/usr/aarch64-linux-gnu \
      # install headers only
      -Dcppad_prefix=/usr/aarch64-linux-gnu \
      # Build static libs only (no shared) -> [https://cppad.readthedocs.io/latest/cmake.html#cmake-name]
      -Dcppad_static_lib=true \
      # Don't add tests to build (CMake-wide)
      -DBUILD_TESTING=OFF \
 && make install \
 && rm -rf /tmp/cppad

# Build Ceres Solver for ARM64
RUN git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver /tmp/ceres-solver \
    && mkdir /tmp/ceres-solver/build \
    && cd /tmp/ceres-solver/build \
    && cmake .. \
        # Use custom toolchain file for cross-compilation
        -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake \
        # Build static libs only (no shared/dynamic) (CMake-wide)
        -DBUILD_SHARED_LIBS=OFF \
        # Set installation location
        -DCMAKE_INSTALL_PREFIX=/usr \
    && make -j$(nproc) \
    && make install \
    && cd / && rm -rf /tmp/ceres-solver

# Build OpenCV for ARM64
RUN git clone https://github.com/opencv/opencv.git /tmp/opencv \
    && git clone https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib \
    && mkdir /tmp/opencv/build \
    && cd /tmp/opencv/build \
    && cmake .. \
       # Use custom toolchain file for cross-compilation
       -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake \
       # Set installation location
       -DCMAKE_INSTALL_PREFIX=/usr \
       # Location of Extra OpenCV Contrib Modules
       -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
       # Build as static libs instead of shared/dynamic
       -DBUILD_SHARED_LIBS=OFF \
       # Skip compiling OpenCV’s internal test suites
       -DBUILD_TESTS=OFF \
       # Skip OpenCV’s performance benchmarking binaries.
       -DBUILD_PERF_TESTS=OFF \
       # Prevent compiling all the sample programs in samples/.
       -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) \
    && make install \
    && cd / && rm -rf /tmp/opencv /tmp/opencv_contrib


# Set default working directory
WORKDIR /workspace

# Default entrypoint
CMD ["/bin/bash"]




# Compile with: docker build -t rpi-cc-img .
# Run with: docker run --rm -it -v $(pwd):/workspace rpi-cc-img

# Build Steps (FROM INSIDE THE WORKSPACE): 
    # mkdir -p build && cd build && cmake .. -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake
    # make -j$(nproc)