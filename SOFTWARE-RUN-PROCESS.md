# Software Run Process

## Hardware Set-up Checklist
1. Ensure Power sources are Charged.
   1. And other components that need to be charged.
2. Ensure Peripherals (sensors, actuators) are connected.
3. Ensure Components are wired properly
4. Ensure Hardware Structure is stable (i.e. No breaks, no loose components, etc)


## Debugger Connection Process
### Debugger Tool
**Process**: Connect Pi to Host with USB cable

## Software Compilation
### PRE-Cross-Compilation Process [ONLY DO ONCE]
**Pre-Condition**: Dockerfile is available (& Docker installed)  
**Pre-Condition**: CMakeLists.txt is available (& CMake installed)  
1. Build a Docker image (rpi-cc-img) from the **Dockerfile** in your workspace directory.  
   a) `docker build --no-cache -t rpi-cc-img .`
2. **Create** a build directory, **Enter** it, and **Run** CMake to configure your project using an ARM64 cross-compilation toolchain:  
   a) `mkdir -p build && cd build && cmake .. -DCMAKE_TOOLCHAIN_FILE=/toolchain/arm64-toolchain.cmake`
3. Compile your project  
   a) `make -j$(nproc)`


### Cross-Compilation Process
**Pre-Condition**: Docker engine is running.  
1. Enter the project root directory
2. Start a Docker container from the *rpi-cc-img* image (with an interactive terminal) and mount your current directory into it.  
   a) `docker run --rm -it -v $(pwd):/workspace rpi-cc-img`
3. cd into the `build/` directory you set up for ARM cross compilation.
4. Build the image: `make`


## Image Download Process
**Pre-Condition**: Raspberry Pi is powered, running and connected to host  
**Pre-Condition**: You are on the same wifi as the Raspberry Pi  
1. Find Raspberry Pi Information:  
   a) Obtain Raspberry Pi IP Address. Run: `ping [your-pi-hostname].local`  
   b) Save IP Address  
1. Go to image location in `build/` directory
2. Copy the image over to the Raspberry Pi.  
   a) `scp ./image-name [your-pi-username]@[pi-ip-address]:~`

## Image Running Process
**Pre-Condition**: Raspberry Pi is powered, running and connected to host  
**Pre-Condition**: You are on the same wifi as the Raspberry Pi  
1. Remote into Raspberry Pi: `ssh [your-pi-username]@[pi-ip-address]`
2. Go to image location
3. Run the image `./image`


## Post-Run Actions


## Useful Tools