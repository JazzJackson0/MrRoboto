#pragma once
#include <iostream>
#include <queue>
#include <vector>
#include <limits>
#include </usr/include/eigen3/Eigen/Dense>
#include "/usr/include/eigen3/unsupported/Eigen/CXX11/Tensor"
using namespace Eigen;

typedef enum {M_OPEN, M_CLOSED, M_NONE} MapStatus; 
typedef enum {F_OPEN, F_CLOSED, F_NONE} FrontierStatus; 
#define xOPEN 0
#define xCLOSED 1
#define xNONE 2

#define MAP_STATUS 33
#define FRONTIER_STATUS 44


struct PointStatus {
    MapStatus map_status;
    FrontierStatus frontier_status;
    PointStatus() {}
    PointStatus(MapStatus m_status, FrontierStatus f_status) : map_status(m_status), frontier_status(f_status) {}
};

struct RecursionPoint {
    bool added;
    RecursionPoint() {}
    RecursionPoint(bool _added) : added(_added) {}
};


class FrontierExplorer {

    private:
        Eigen::Tensor<float, 2> Map;
        int M;
        int N;
        PointStatus **CellStatusMap;
        std::queue<VectorXf> MapQueue;
        std::queue<VectorXf> FrontierQueue;
        std::vector<VectorXf> MapOpenList;
        std::vector<VectorXf> MapClosedList;
        std::vector<VectorXf> FrontierOpenList;
        std::vector<VectorXf> FrontierClosedList;

        /**
         * @brief 
         * 
         */
        void Build_CellStatusMap();

        /**
         * @brief 
         * 
         * @param RecursionMap 
         */
        void Build_RecursionMap(RecursionPoint **&RecursionMap);

        /**
         * @brief 
         * 
         * @param row 
         * @param col 
         * @return true 
         * @return false 
         */
        bool isValid(int row, int col);

        /**
         * @brief 
         * 
         * @param point 
         * @return true 
         * @return false 
         */
        bool isFrontier(VectorXf point);

        /**
         * @brief 
         * 
         * @param frontier 
         * @return VectorXf 
         */
        VectorXf Get_Centroid(std::vector<VectorXf> frontier);

        /**
         * @brief 
         * 
         * @param point 
         * @param adjacents 
         * @param RecursionMap 
         */
        void Get_AdjacentCells(VectorXf point, std::vector<VectorXf> &adjacents, RecursionPoint **RecursionMap);

        /**
         * @brief 
         * 
         * @param point 
         * @param map_frontier 
         * @return int 
         */
        int Get_CellStatus(VectorXf point, int map_frontier);

        /**
         * @brief 
         * 
         * @param point 
         * @param map_frontier 
         * @param status 
         */
        void Update_CellStatus(VectorXf point, int map_frontier, int status);

        /**
         * @brief 
         * 
         * @param point 
         * @return true 
         * @return false 
         */
        bool Has_OpenSpaceNeighbor(VectorXf point);

        /**
         * @brief 
         * 
         * @param robot_pose 
         * @return std::vector<std::vector<VectorXf>> 
         */
        std::vector<std::vector<VectorXf>> Detect_WavefrontFrontier(VectorXf robot_pose);

        /**
         * @brief 
         * 
         * @param frontier_pt 
         * @return std::vector<VectorXf> 
         */
        std::vector<VectorXf> Extract_Frontier2D(VectorXf frontier_pt);

    public:

        /**
         * @brief Construct a new Frontier Explorer object
         * 
         */
        FrontierExplorer();

        /**
         * @brief Construct a new Frontier Explorer object
         * 
         * @param map 
         */
        FrontierExplorer(Eigen::Tensor<float, 2> map);


        /**
         * @brief 
         * 
         * @param map 
         */
        void Load_MAP(Eigen::Tensor<float, 2> map);

        /**
         * @brief 
         * 
         * @param robot_pose 
         * @return VectorXf - Frontier Centroid
         */
        VectorXf FindFrontier(VectorXf robot_pose);
};






