#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <cstdint>
#include <Eigen/Dense>
#include "unsupported/Eigen/CXX11/Tensor"
#include "utils.hpp"

using namespace Eigen;
#define PBM 0
#define PGM 1
#define PPM 2
#define PLY 3


class MapBuilder {

    private:
        int M;
        int N;
        int P;
        std::fstream map_file;
        std::vector<std::string> magic_numbers = {"P4", "P5", "P6", "P1", "P2", "P3", "ply"};
        std::vector<std::string> ply_keywords = {"ply", "format", "comment", "element", "property"};

    public:

        /**
         * @brief Default Contructor
         * 
         */
        MapBuilder();

        /**
         * @brief 
         * 
         * @param m 
         * @param n 
         */
        MapBuilder(int m, int n);

        /**
         * @brief 
         * 
         * @param p 
         * @param m 
         * @param n 
         */
        MapBuilder(int p, int m, int n);


        /**
         * @brief 
         * 
         * @param m 
         * @param n 
         */
        void update2DMapDimensions(int m, int n);


        /**
         * @brief 
         * 
         * @param m 
         * @param n 
         * @param p
         */
        void update3DMapDimensions(int m, int n, int p);

        /**
         * @brief 
         * 
         * @param tensor 
         * @param filename 
         * @param filetype 
         * @param max_value 
         */
        void tensor2DToMapFile(Tensor<float, 2> tensor, std::string filename, int filetype, int max_value);


        /**
         * @brief 
         * 
         * @param filename 
         * @param filetype 
         * @return Tensor<float, 2> 
         */
        Tensor<float, 2> mapFileToTensor2D(std::string filename, int filetype);


        /**
         * @brief 
         * 
         * @param tensor 
         * @param filename 
         * @param filetype 
         */
        void tensor3DToMapFile(Tensor<float, 3> tensor, std::string filename, int filetype);


        /**
         * @brief 
         * 
         * @param filename 
         * @param filetype 
         * @return Tensor<float, 3> 
         */
        Tensor<float, 3> mapFileToTensor3D(std::string filename, int filetype);


        /**
         * @brief 
         * 
         * @param coordinate 
         * @return VectorXf 
         */
        VectorXi mapCoordinateToDataStructureIndex(VectorXf coordinate);


        /**
         * @brief 
         * 
         * @param index 
         * @return VectorXi 
         */
        VectorXf dataStructureIndexToMapCoordinate(VectorXi index);


        /**
		 * @brief Obtains the maximum likelihood map by rounding the probability of each cell to 0 or 1
		 * 
         * @param map 
		 * @return Eigen::Tensor<float, 2> 
		 */
		Eigen::Tensor<float, 2> getMaximumLikelihoodMap(Eigen::Tensor<float, 2> map);


        /**
         * @brief Add inflation layer to map
         * 
         * @param map 
         * @param inflation_radius 
         */
        void applyInflationLayer(Tensor<float, 2> &map, int inflation_radius);

};

