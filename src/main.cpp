#include <iostream>
#include <signal.h>
#include <stdlib.h>
// #include <stdio.h>
#include <unistd.h>
#include "../include/Robot.hpp"

#define ROBOT_TRACKWIDTH 0.3
#define ROBOT_WHEEL_RADIUS 0.05

using namespace diffdrive;
Robot *mr_roboto;

void ctrl_c_handler(int s){

    mr_roboto->robotStop("final_map.pgm");
    exit(1); 
}


int main(int argc, char* argv[]) {

    struct sigaction sigIntHandler;
    
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    mr_roboto = Robot::CreateRobot();
    mr_roboto->robotStart(true);
    mr_roboto->setMapDimensions(1000, 1000);
    mr_roboto->setPhysicalParameters(ROBOT_TRACKWIDTH, ROBOT_WHEEL_RADIUS);

    mr_roboto->mapAndLocalize(POSE_GRAPH);
    // mr_roboto->mapEnv();
    // mr_roboto->localize("../tests/map2.pbm");
    
    return 0;

    

    // return 0;
}