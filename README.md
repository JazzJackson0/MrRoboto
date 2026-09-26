# Project: Mr Roboto

## Summary


## Features
- **Pose Graph Optimization** – Some Features, ...
- **Particle Filter** – Some Features, ...
- **Etc** – Some Features, ...


## Requirements

### Hardware Requirements
**Board**: Raspberry Pi  
- Power source for Pi 
**Sensor:** RpLidar A1 
**Sensor**: Wireless Bluetooth dongle + Sony Dualshock PS4 Controller**  
**Raspberry Pi Pico**

### Software Requirements

### Other


## Dependencies
### **dependency name** - `version`
**Uses**
- Use A
- Use B

**Files Using Dependency**
- File A
- File B
- File C

**Notes**
- Note A
- Note B

## Controller Buttons

- **Forward**: D-Pad Up
- **Backward**: D-Pad Down
- **Right**: CIRCLE
- **Left**: SQUARE
- **Accelerate**: R0
- **Decelerate**: L0


## Other???
Live Output View 
(But first, ensure point cloud data is being output to stdout)
`./demo | python3 display.py`


## Build Notes
### Some CMakeLists.txt Alteration
```cmake
cmake_minimum_required(VERSION 3.10)
project( MyProject )
add_executable( MyProject main.cpp )
```

### Some Makefile Alteration
```makefile
CC = gcc
CFLAGS = -Wall -g

main: main.o
	(CC) (CFLAGS) -o main main.o
```


