#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "../include/Robot.hpp"

using namespace diffdrive;
Robot *mr_roboto;

void ctrl_c_handler(int s){
    
    printf("\nShutting Down Mr. Roboto...\n");
    mr_roboto->RobotStop();
    exit(1); 
}


int main(int argc, char* argv[]) {

    struct sigaction sigIntHandler;
    
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    mr_roboto = Robot::CreateRobot();

    mr_roboto->RobotStart();
    // mr_roboto->MapAndLocalize(EKF);
    // mr_roboto->MapEnv();
    mr_roboto->Localize("../tests/map2.pbm");
    
    
    return 0;
}