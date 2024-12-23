cmake_minimum_required(VERSION 3.16.3)
project(RPLidar)

set(RPLidar_VERSION 0.0.0)
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(TARGET demo)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)           # Specify C++17 standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Make C++17 standard required

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


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
    ${SRC}/PathUtil.cpp
    ${SRC}/camera/Calibration.cpp ${SRC}/camera/VisualOdometry ${SRC}/camera/VSLAM.cpp
    )


include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS} # Include OpenCV headers
    ${CERES_INCLUDE_DIRS} # Include Ceres headers
)

add_executable(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} PRIVATE Threads::Threads ${OpenCV_LIBS} ${CERES_LIBRARIES})
