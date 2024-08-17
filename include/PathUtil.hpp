#pragma once
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include "MapBuilder.hpp"
using namespace Eigen;

class PathUtil {

    private:
        MapBuilder *map_builder;


    public:

        PathUtil();

        std::vector<VectorXi> SamplePath(std::vector<VectorXi> path);

        std::vector<VectorXf> SmoothPath(std::vector<VectorXi> path);
};



