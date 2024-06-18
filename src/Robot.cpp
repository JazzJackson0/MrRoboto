#include "../include/Robot.hpp"

namespace diffdrive {

// Private------------------------------------------------------------------------------------------------------------------------
void Robot::StartScanner() {

    lidar->startMotor();
    // Start scan with default scan mode
    LidarScanMode scanMode;
    lidar->startScan(false, true, 0, &scanMode);
}

void Robot::StopScanner() {

    lidar->stop();
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
}

void Robot::RawScan() {

    // Grab Complete 0-360 degree scan previously received
    auto res = lidar->grabScanDataHq(nodes, nodeCount);

    // Scan Failed
    if (IS_FAIL(res))
        std::cout << "Failed to get scan data" << "\n";

    // Scan Success
    else {
        std::cout << "Scan Success" << "\n"; 
    }
}


PointCloud Robot::GetCloud(int broadcast_state) {

    PointCloud cloud;
    int idx = 0;
    for (int i = 0; i < nodeCount; i += 1) {

        VectorXf point(2);
        float dist_cm;
        float dist_m;
        float angle;
        float rad;

        if (nodes[i].dist_mm_q2 == 0) // Measurement is invalid
            continue;
        else {
            float dist_mm = (float) nodes[i].dist_mm_q2 / 4.0f;
            dist_m = dist_mm / 1000;
        }

        angle = ( (float) nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        rad = angle * M_PI / 180.f;

        // Get coordinate of ray.
        int size = currentPos.rows();
        float x = (currentPos[0] + (dist_m * std::cos(rad)));
        float y = (currentPos[1] + (dist_m * std::sin(rad)));

        if (broadcast_state == NO_BROADCAST) {

            point << x, y;
            cloud.points.push_back(point);
            cloud.weights.push_back((float)(nodes[i].quality >> 2)); // Echo Signal Strength 0 - 255
        }
        
        else if (broadcast_state == BROADCAST) {
            std::cout << x * 100 << ", " << y * 100 << "\n"; // Convert point from m to cm
        }
    }

    return cloud;
}


std::vector<VectorXf> Robot::GetScan() {

    std::vector<VectorXf> scan;
    int idx = 0;
    for (int i = 0; i < nodeCount; i += 1) {

        VectorXf point(2);
        point = VectorXf::Zero(2);
        float dist_cm = 0;
        float dist_m = 0;
        float angle;
        float rad;

        if (nodes[i].dist_mm_q2 == 0) // Measurement is invalid
            continue;
        else {
            float dist_mm = (float) nodes[i].dist_mm_q2 / 4.0f;
            dist_m = dist_mm / 1000;
        }

        angle = ( (float) nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        rad = angle * M_PI / 180.f;
        point << (dist_m * 100), angle; // Conver dist to cm
        scan.push_back(point);

        idx++;
    }

    return scan;
}



Robot::Robot() {

    // Create a LIDAR driver instance
    lidar = RPlidarDriver::CreateDriver();
    auto res = lidar->connect("/dev/ttyUSB0", 115200);

    if(SL_IS_OK(res)) { 
    //if(SL_IS_OK(lidar->connect(_channel))) { 
        
        lidar->stopMotor();
        //lidar->setMotorSpeed(0);

        // Display Device Info
        sl_lidar_response_device_info_t deviceInfo;
        res = lidar->getDeviceInfo(deviceInfo);
        
        if(SL_IS_OK(res)) {
            
            // printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            // deviceInfo.model,
            // deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            // deviceInfo.hardware_version);
        }
        
        else 
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);     
    }
    
    else 
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);

    // Set Up NodeCount for Scan Data
    nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);
}



// Public------------------------------------------------------------------------------------------------------------------------
Robot * Robot::CreateRobot() {

    return new Robot();
}

void Robot::RobotStart() {

    StartScanner();

    // Set Current Position
    currentPos = VectorXf::Zero(3);

    // Setup Map Builder
    map_builder = new MapBuilder(600, 600);

    // Setup SLAM 1
    slam1 = new PoseGraphOptSLAM(500, 3, 0.01);
    slam1->FrontEndInit(5, 1.f);

    // Setup SLAM 2
    slam2 = new EKFSlam(3, 2);
    slam2->SetInitialState(currentPos, 0.01, 0.001);

    // Setup Mapping
    og_map = new OccupancyGridMap(500, 500, 0.01, 2 * M_PI, 6);

    // Setup Localization
    pfilter = new ParticleFilter(200, 3, 0.01);
    
    // Setup Path Planner 1
    astar_path = new A_Star();

    // Setup Path Planner 2
    rrt_path = new RRT();  
}

void Robot::RobotStop() {

    StopScanner();
}


void Robot::MapEnv() {

    while (1) {
        RawScan();
        // std::vector<VectorXf> scan = GetScan();
        // std::cout << og_map->UpdateGridMap(currentPos, scan) << std::endl;
        // std::cout << std::endl;

        // Version 2
        PointCloud cloud = GetCloud(NO_BROADCAST);
        std::cout << og_map->UpdateGridMapWithPointCloud(cloud) << std::endl;
        std::cout << std::endl;
    }
}


void Robot::Localize(std::string map_filename) {

    pfilter->AddMap(map_builder->MapFile_to_Tensor2D(map_filename, PBM), 6, 2 * M_PI);
    std::cout << "Map Added!!!" << std::endl;
    while (1) {
        RawScan();
        PointCloud cloud = GetCloud(NO_BROADCAST);
        ControlCommand ctrl;
        ctrl.trans_vel = 1.0;
        ctrl.rot_vel = 1.0;
        pfilter->Run(cloud, ctrl);
    }
}


void Robot::MapAndLocalize(int algorithm) {

    while (1) {
        RawScan();
        PointCloud cloud = GetCloud(NO_BROADCAST);
        ControlCommand ctrl;
        ctrl.trans_vel = 1.0;
        ctrl.rot_vel = 1.0;

        if (algorithm == POSE_GRAPH) {

            slam1->Run(cloud, currentPos); // Change Run() to output the current Map/Graph each iteration.
        }
        
        else if (algorithm == EKF) {

            slam2->Run(cloud, currentPos, ctrl);
        }
    }
}


void Robot::CreatePath(int algorithm, std::string map_filename) {

    VectorXi start(2);
    VectorXi goal(2);
    start << 0, 0;
    goal << 100, 249;

    if (algorithm == A_STAR) {
        astar_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        astar_path->Path(start, goal);
    }   
    
    else if (algorithm == RRT_VANILLA) {
        rrt_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        rrt_path->RRT_Path(start, goal, 1);
    }
    
    else if (algorithm == RRT_STAR) {
        rrt_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        rrt_path->RRTStar_Path(start, goal, 1, 2);
    } 
}


void Robot::BroadcastPointCloud() {

    GetCloud(BROADCAST);
}




}