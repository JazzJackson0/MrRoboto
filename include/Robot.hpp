#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <memory>
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
#include "Controller.hpp"
#include "ext/nlohmann/json.hpp"
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


using json = nlohmann::json;
using namespace rp::standalone::rplidar;

namespace diffdrive {

    class Robot {

        private:

            bool autonomous;

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

            std::unique_ptr<MapBuilder> map_builder;
            std::unique_ptr<PoseGraphOptSLAM> slam1; // LATER: Make this a generic SLAM Interface that can also take EKF 
            std::unique_ptr<EKFSlam> slam2;
            std::unique_ptr<ParticleFilter> pfilter;
            std::unique_ptr<AStar> astar_path;
            std::unique_ptr<RRT> rrt_path;
            std::unique_ptr<OccupancyGridMap> og_map;
            std::unique_ptr<DynamicWindowApproach> d_window;
            std::unique_ptr<FrontierExplorer> frontier_explorer;
            std::unique_ptr<PID> pid_right;
            std::unique_ptr<PID> pid_left;
            std::unique_ptr<Odom> odom;
            std::unique_ptr<Serial> serial;
            std::unique_ptr<PathUtil> path_util;
            std::unique_ptr<Controller> controller;

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
            std::unique_ptr<vSLAM> v_slam;
            cv::VideoCapture cap;
            cv::VideoCapture cap_left;
            cv::VideoCapture cap_right;

            const std::string map_param_path = "cfg/map-params.json";
            const std::string robot_param_path = "cfg/config.json";
            

            bool map_ready();

            /**
             * @brief 
             * 
             * @param robot_wheel_radius 
             * @param robot_trackwidth 
             */
            void set_physical_parameters(float robot_wheel_radius, float robot_trackwidth);

            /**
             * @brief 
             * 
             * @param width 
             * @param height 
             */
            void set_map_dimensions(int width, int height);

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
            void set_parameters();

            /**
             * @brief 
             * 
             */
            void callibrate_camera(const std::string& images_path);

            /**
             * @brief 
             * 
             */
            void start_scanner();

            /**
             * @brief 
             * 
             */
            void stop_scanner();

            /**
             * @brief 
             * 
             */
            void raw_scan();

            /**
             * @brief Convert lidar beam nodes from RPLidar into a Point Cloud
             * 
             * @param broadcast_state 
             * @return PointCloud 
             */
            PointCloud get_cloud(int broadcast_state); 


            /**
             * @brief 
             * 
             * @return std::vector<VectorXf> 
             */
            std::vector<VectorXf> get_scan();


            /**
             * @brief Overloaded version of public CreatePath function
             * 
             * @param algorithm 
             * @param map 
             * @param start 
             * @param goal 
             * @return std::vector<VectorXi> 
             */
            std::vector<VectorXi> create_path(int algorithm, Eigen::Tensor<float, 2> map, VectorXi start, VectorXi goal);


            void find_frontier();



            void follow_local_path(std::vector<VectorXf> smooth_waypoints, Eigen::Tensor<float, 2> map, PointCloud cloud, VectorXf pos);


            void run_slam(int algorithm);


            cv::Mat run_vslam(bool stereo);


            void run_localizer(Eigen::Tensor<float, 2> map);


            void run_mapper();

            /**
             * @brief 
             * 
             * @param output_filename 
             */
            void save_map(std::string output_filename);


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
             * @param autonomous 
             */
            void robotStart(bool autonomous);

            /**
             * @brief 
             * 
             * @param output_filename name of ouptut map file
             */
            void robotStop(std::string output_filename);

            /**
             * @brief 
             * 
             */
            void mapEnv();

            /**
             * @brief 
             * 
             * @param map_filename 
             */
            void localize(std::string map_filename);

            /**
             * @brief 
             * 
             * @param algorithm 
             */
            void mapAndLocalize(int algorithm);

            /**
             * @brief 
             * 
             * @param algorithm 
             * @param map_filename 
             * @param start 
             * @param goal 
             * @return std::vector<VectorXi> 
             */
            std::vector<VectorXi> createPath(int algorithm, std::string map_filename, VectorXi start, VectorXi goal);

            /**
             * @brief Broadcast the point cloud points to stdout
             * 
             */
            void broadcastPointCloud();
    };
}
