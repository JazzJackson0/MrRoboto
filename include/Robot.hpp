#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "unsupported/Eigen/CXX11/Tensor"
#include "rplidar.h"
#include "MapBuilder.hpp"
#include "PoseGraphOptSLAM.hpp"
#include "EKFSLAM.hpp"
#include "ParticleFilter.hpp"
#include "AStar.hpp"
#include "RRT.hpp"
#include "OccupancyGrid.hpp"
#include "FrontierExploration.hpp"
#include "DynamicWindow.hpp"
#include "PID.hpp"
#include "Odometry.hpp"
#include "Serial.hpp"
#include "PathUtil.hpp"
#include "camera/Calibration.hpp"
#include "camera/VSLAM.hpp"
#define POSE_GRAPH 0
#define EKF 1
#define A_STAR 2
#define RRT_VANILLA 3
#define RRT_STAR 4
#define BROADCAST 5
#define NO_BROADCAST 6
#define UART_NUM 0

// Pose Graph SLAM Parameters
#define MAX_NODES 500
#define POSE_DIMENSION 3
#define GUESS_VARIATION 0.01
#define N_RECENT_POSES 5
#define LOOP_CLOSURE_DIST 1.0

// EKF SLAM Parameters
#define POSE_DIM 3
#define LANDMARK_DIM 2
#define PROCESS_UNCERTAINTY 0.01
#define MEASUREMENT_UNCERTAINTY 0.001

// Occupancy Grid MAp Parameters
#define ALPHA 0.01
#define BETA 2 * M_PI
#define MAX_SCAN_RANGE 6

// Particle Filter Parameters
#define MAX_PARTICLES 200
#define P_FILTER_POSE_DIM 3
#define P_FILTER_TIME_INTERVAL 0.01

// Dynamic Window Parameters
#define MIN_TRANS_VEL -20
#define MAX_TRANS_VEL 20
#define TRANS_VEL_INTERVAL 0.1
#define MAX_TRANS_ACCEL 1000

#define MIN_ROT_VEL -10
#define MAX_ROT_VEL 10
#define ROT_VEL_INTERVAL 0.1
#define MAX_ROT_ACCEL 1000

#define HEADING_WEIGHT 0.04
#define DISTANCE_WEIGHT 0.2
#define VELOCITY_WEIGHT 0.1
#define SMOOTHING_VAL 1

// PID Parameters
#define SAMPLE_TIME_MS 1
#define KP 0.1
#define KI 0.01 
#define KD 0.001
#define MIN_OUTPUT_VAL -100
#define MAX_OUTPUT_VAL 100
#define MAX_OUTOUT_VAL_INV (1 / MAX_OUTPUT_VAL)

// Odometry Parameters
#define ODOM_TRACKWIDTH 1
#define ODOM_TIMESTEP 0.1

#define MAX_WHEEL_VEL 30



using namespace rp::standalone::rplidar;

namespace diffdrive {

    class Robot {

        private:
            std::fstream broadcast_output;

            // Shared Data
            VectorXf current_pos;
            PointCloud current_cloud;
            std::vector<VectorXf> current_scan;
            Eigen::Tensor<float, 2> current_map;
            bool map_data_available;
            std::mutex pos_mutex;
            std::mutex cloud_mutex;
            std::mutex scan_mutex;
            std::mutex map_mutex;
            std::mutex map_ready_mutex;
            std::mutex odom_read_mutex;

            RPlidarDriver *lidar;
            // IChannel *_channel;
            sl_lidar_response_measurement_node_hq_t nodes[8192]; // List of nodes, each representing a Lidar Beam
            size_t nodeCount; // Number of Nodes (i.e. Lidar Beams)

            MapBuilder *map_builder;
            PoseGraphOptSLAM *slam1; // LATER: Make this a generic SLAM Interface that can also take EKF 
            EKFSlam *slam2;
            ParticleFilter *pfilter;
            A_Star *astar_path;
            RRT *rrt_path;
            OccupancyGridMap *og_map;
            DynamicWindowApproach *d_window;
            FrontierExplorer *frontier_explorer;
            PID *pid_right;
            PID *pid_left;
            Odom *odom;
            Serial *serial;
            PathUtil *path_util;

            // Robot Physical Dimensions
            float trackwidth;
            float wheel_radius;
            bool physical_set;

            // Actuation
            int8_t serial_bus;

            // Map
            int map_height;
            int map_width;
            bool map_set;

            // Camera
            CameraCalibrator *calibrator;
            vSLAM *v_slam;
            cv::VideoCapture cap;
            cv::VideoCapture cap_left;
            cv::VideoCapture cap_right;
            

            bool map_ready();

            /**
             * @brief 
             * 
             * @return true 
             * @return false 
             */
            bool map_params_set();

            /**
             * @brief 
             * 
             * @return true 
             * @return false 
             */
            bool physical_params_set();

            /**
             * @brief 
             * 
             */
            void Callibrate_Camera(const std::string& images_path);

            /**
             * @brief 
             * 
             */
            void StartScanner();

            /**
             * @brief 
             * 
             */
            void StopScanner();

            /**
             * @brief 
             * 
             */
            void RawScan();

            /**
             * @brief Convert lidar beam nodes from RPLidar into a Point Cloud
             * 
             * @param broadcast_state 
             * @return PointCloud 
             */
            PointCloud GetCloud(int broadcast_state); 


            /**
             * @brief 
             * 
             * @return std::vector<VectorXf> 
             */
            std::vector<VectorXf> GetScan();


            /**
             * @brief Overloaded version of public CreatePath function
             * 
             * @param algorithm 
             * @param map 
             * @param start 
             * @param goal 
             * @return std::vector<VectorXi> 
             */
            std::vector<VectorXi> CreatePath(int algorithm, Eigen::Tensor<float, 2> map, VectorXi start, VectorXi goal);


            void FindFrontier();



            void FollowLocalPath(std::vector<VectorXf> smooth_waypoints, Eigen::Tensor<float, 2> map, PointCloud cloud, VectorXf pos);


            void RunSLAM(int algorithm);


            cv::Mat RunVSLAM(bool stereo);


            void RunLocalizer(Eigen::Tensor<float, 2> map);


            void RunMapper();

            /**
             * @brief 
             * 
             * @param output_filename 
             */
            void Save_Map(std::string output_filename);


            /**
             * @brief Construct a new Robot object
             * 
             */
            Robot();

        public:
  

            /**
             * @brief Create an Robot Instance. This interface should be invoked FIRST before any other operation
             * 
             * @return Robot* 
             */
            static Robot * CreateRobot();


            /**
             * @brief 
             * 
             * @param robot_trackwidth 
             * @param robot_wheel_radius 
             */
            void Set_PhysicalParameters(float robot_trackwidth, float robot_wheel_radius);

            /**
             * @brief (Must be called AFTER the RobobtStart() function)
             * 
             * @param height 
             * @param width 
             */
            void Set_MapDimensions(int height, int width);

            
            /**
             * @brief 
             */
            void RobotStart();

            /**
             * @brief 
             * 
             * @param output_filename name of ouptut map file
             */
            void RobotStop(std::string output_filename);

            /**
             * @brief 
             * 
             */
            void MapEnv();

            /**
             * @brief 
             * 
             * @param map_filename 
             */
            void Localize(std::string map_filename);

            /**
             * @brief 
             * 
             * @param algorithm 
             */
            void MapAndLocalize(int algorithm);

            /**
             * @brief 
             * 
             * @param algorithm 
             * @param map_filename 
             * @param start 
             * @param goal 
             * @return std::vector<VectorXi> 
             */
            std::vector<VectorXi> CreatePath(int algorithm, std::string map_filename, VectorXi start, VectorXi goal);

            /**
             * @brief Broadcast the point cloud points to stdout
             * 
             */
            void BroadcastPointCloud();
    };
}
