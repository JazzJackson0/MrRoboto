#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
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
#define POSE_GRAPH 0
#define EKF 1
#define A_STAR 2
#define RRT_VANILLA 3
#define RRT_STAR 4
#define BROADCAST 5
#define NO_BROADCAST 6

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
            std::mutex cloud_mutex;
            std::mutex scan_mutex;
            std::mutex map_mutex;
            std::mutex pos_mutex;

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

            // Robot Physical Dimensions
            float trackwidth;
            float wheel_radius;

            // Actuation
            int8_t serial_bus1;
            int8_t serial_bus2;



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



            void FollowLocalPath(std::vector<VectorXi> waypoints, Eigen::Tensor<float, 2> map, PointCloud cloud, VectorXf pos);


            void RunSLAM(int algorithm);


            void RunLocalizer(Eigen::Tensor<float, 2> map);


            void RunMapper();


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
             * @brief 
             */
            void RobotStart();

            /**
             * @brief 
             * 
             */
            void RobotStop();

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
