cmake_minimum_required(VERSION 3.16.3)
project(RPLidar)

set(RPLidar_VERSION 0.0.0)
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(TARGET demo)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads REQUIRED)

set(RPLIDAR_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/sdk/)

FILE(GLOB RPLIDAR_SDK_SRC
      "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp")

set(SOURCES ${SRC}/main.cpp ${SRC}/Robot.cpp ${SRC}/ICP.cpp 
    ${SRC}/PoseGraphOptSLAM.cpp ${SRC}/kd_tree.cpp ${SRC}/utils.cpp 
    ${RPLIDAR_SDK_SRC} ${SRC}/EKFSLAM.cpp ${SRC}/FeatureExtraction.cpp
    ${SRC}/AStar.cpp ${SRC}/RRT.cpp ${SRC}/MapBuilder.cpp 
    ${SRC}/OccupancyGrid.cpp ${SRC}/ParticleFilter.cpp ${SRC}/DynamicWindow.cpp
    ${SRC}/FrontierExploration.cpp ${SRC}/PID.cpp ${SRC}/Serial.cpp ${SRC}/Odometry.cpp
    ${SRC}/PathUtil.cpp)

include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} PRIVATE Threads::Threads)
