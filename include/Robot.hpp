#pragma once
#include <iostream>
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
            VectorXf currentPos;
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
             */
            void CreatePath(int algorithm, std::string map_filename);

            /**
             * @brief Broadcast the point cloud points to stdout
             * 
             */
            void BroadcastPointCloud();
    };
}
