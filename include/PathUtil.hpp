#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "MapBuilder.hpp"
using namespace Eigen;

class PathUtil {

    private:
        MapBuilder *map_builder;


    public:

        PathUtil();

        std::vector<VectorXi> samplePath(std::vector<VectorXi> path);

        std::vector<VectorXf> smoothPath(std::vector<VectorXi> path);
};



