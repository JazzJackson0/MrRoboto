#include "../include/Robot.hpp"

namespace diffdrive {

// Private------------------------------------------------------------------------------------------------------------------------

bool Robot::map_ready() {
    bool result;
    std::unique_lock<std::mutex> map_state_lock(map_ready_mutex);
    result = map_data_available;
    map_state_lock.unlock();
    return result;
}

bool Robot::map_params_set() {
    return map_set;
}

bool Robot::physical_params_set() {
    return physical_set;
}

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
            dist_cm = dist_m * 100;
        }

        angle = ( (float) nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        rad = angle * M_PI / 180.f;

        // Get coordinate of ray.
        int size = current_pos.rows();
        float x = (current_pos[0] + (dist_cm * std::cos(rad)));
        float y = (current_pos[1] + (dist_cm * std::sin(rad)));

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
            dist_cm = dist_m * 100;
        }

        angle = ( (float) nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        rad = angle * M_PI / 180.f;
        point << dist_cm, angle; // Convert dist to cm
        scan.push_back(point);

        idx++;
    }

    return scan;
}


void Robot::FindFrontier() {

    while (!map_ready());

    while (1) {

        std::unique_lock<std::mutex> map_lock(map_mutex);
        Eigen::Tensor<float, 2> map = current_map;
        map_lock.unlock();
        std::cout << "Current Map Dimensions: " << map.dimension(0) << " x " << map.dimension(1) << std::endl;

        std::unique_lock<std::mutex> cloud_lock(map_mutex);
        PointCloud cloud = current_cloud;
        cloud_lock.unlock();

        std::unique_lock<std::mutex> pos_lock(map_mutex);
        VectorXi pos = map_builder->MapCoordinate_to_DataStructureIndex(current_pos);
        VectorXf flt_pos = current_pos;
        pos_lock.unlock();
        
        map_builder->Apply_InflationLayer(map, 5); // Create Cost Map
        frontier_explorer->Load_MAP(map);
        VectorXi frontier_goal = frontier_explorer->FindFrontier(pos);
        std::vector<VectorXf> waypoints = path_util->SmoothPath(path_util->SamplePath(CreatePath(A_STAR, map, pos, frontier_goal)));
        FollowLocalPath(waypoints, map, cloud, flt_pos);
    }
}


std::vector<VectorXi> Robot::CreatePath(int algorithm, Eigen::Tensor<float, 2> map, VectorXi start, VectorXi goal) {

    std::vector<VectorXi> path;
    path.push_back(start);

    if (algorithm == A_STAR) {
        astar_path->Load_MAP(map);
        return astar_path->Path(start, goal);
    }   
    
    else if (algorithm == RRT_VANILLA) {
        rrt_path->Load_MAP(map);
        return rrt_path->RRT_Path(start, goal, 1);
    }
    
    else if (algorithm == RRT_STAR) {
        rrt_path->Load_MAP(map);
        return rrt_path->RRTStar_Path(start, goal, 1, 2);
    }

    return path; 
}


void Robot::FollowLocalPath(std::vector<VectorXf> smooth_waypoints, Eigen::Tensor<float, 2> map, PointCloud cloud, VectorXf pos) {

    VectorXf odom_vels = odom->Get_NewVelocities();

    for (int i = 0; i < smooth_waypoints.size(); i++) {

        d_window->Set_Goal(smooth_waypoints[i]);
        VectorXf vels = d_window->Run(pos, cloud);

        float setpoint_vel_r = (2 * vels[0] + vels[1] * trackwidth) / (2 * wheel_radius);
        float setpoint_vel_l = (2 * vels[0] - vels[1] * trackwidth) / (2 * wheel_radius);

        float current_vel_r = (2 * odom_vels[0] + odom_vels[1] * trackwidth) / (2 * wheel_radius);
        float current_vel_l = (2 * odom_vels[0] - odom_vels[1] * trackwidth) / (2 * wheel_radius);

        float right_wheel_duty_cycle = pid_right->PID_Update(setpoint_vel_r, current_vel_r);
        float left_wheel_duty_cycle = pid_left->PID_Update(setpoint_vel_l, current_vel_l);

        // Motor Out
        uint32_t right_wheel_duty_cycle_temp = *((uint32_t*) (&right_wheel_duty_cycle));
        uint32_t left_wheel_duty_cycle_temp = *((uint32_t*) (&left_wheel_duty_cycle));
        char right_duty_cycle_buff[4] = {0};
        char left_duty_cycle_buff[4] = {0};
        for (int i = 0; i < 4; i++) {

            right_duty_cycle_buff[i] = *((char*)&right_wheel_duty_cycle_temp + i);
            left_duty_cycle_buff[i] = *((char*)&left_wheel_duty_cycle_temp + i);
        }
        
        // serial->UARTWrite(serial_bus1, right_duty_cycle_buff, 4);
        // serial->UARTWrite(serial_bus2, left_duty_cycle_buff, 4);
    }
}

void Robot::RunSLAM(int algorithm) {

    bool first_map = true;

    while (1) {
        RawScan();
        PointCloud cloud = GetCloud(NO_BROADCAST);

        std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        current_cloud = cloud;
        cloud_lock.unlock();

        // TODO: Put odom behind mutex??
        VectorXf odom_out = odom->Get_NewVelocities();
        // ControlCommand ctrl;
        // ctrl.trans_vel = odom_out[0];
        // ctrl.rot_vel = odom_out[1];

        if (algorithm == POSE_GRAPH) {
            
            std::unique_lock<std::mutex> map_lock(map_mutex);
            current_map = slam1->Run(cloud);
            // std::cout << "SLAM Output Map Dimensions: " << current_map.dimension(0) << " x " << current_map.dimension(1) << std::endl;
            // std::cout << current_map << std::endl;
            // std::cout << std::endl;
            map_lock.unlock();

            std::unique_lock<std::mutex> pos_lock(pos_mutex);
            current_pos = slam1->BroadcastCurrentPose();
            pos_lock.unlock();

            if (first_map) {
                std::unique_lock<std::mutex> map_state_lock(map_ready_mutex);
                map_data_available = true;
                map_state_lock.unlock();
                first_map = false;
            } 
            
        }
        
        else if (algorithm == EKF) {

            // TEMPORARY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            ControlCommand ctrl;
            ctrl.trans_vel = 0.5;
            ctrl.rot_vel = 0.0;
            //-------------------------------------------------

            std::unique_lock<std::mutex> map_lock(map_mutex);
            current_map = slam2->Run(cloud, ctrl);
            // std::cout << current_map << std::endl;
            // std::cout << std::endl;
            map_lock.unlock();

            std::unique_lock<std::mutex> pos_lock(pos_mutex);
            current_pos = slam2->BroadcastCurrentPose();
            pos_lock.unlock();
            
            if (first_map) {
                std::unique_lock<std::mutex> map_state_lock(map_ready_mutex);
                map_data_available = true;
                map_state_lock.unlock();
                first_map = false;
            }
        }
    }
}

// Should probably add some way to recognize when you are localized and stopping pfilter process.
void Robot::RunLocalizer(Eigen::Tensor<float, 2> map) {

    pfilter->AddMap(map, 6, 2 * M_PI);
    std::cout << "Map Added!!!" << std::endl;
    while (1) {
        RawScan();
        PointCloud cloud = GetCloud(NO_BROADCAST);

        std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        current_cloud = cloud;
        cloud_lock.unlock();

        VectorXf odom_out = odom->Get_NewVelocities();
        ControlCommand ctrl;
        ctrl.trans_vel = odom_out[0];
        ctrl.rot_vel = odom_out[1];
        pfilter->Run(cloud, ctrl);
    }
}


// Needs GPS running on separate thread to update the "current_pos" variable.
void Robot::RunMapper() {

    while (1) {
        RawScan();
        // Version 1
        std::vector<VectorXf> scan = GetScan();

        std::unique_lock<std::mutex> scan_lock(scan_mutex);
        current_scan = scan;
        scan_lock.unlock();

        std::unique_lock<std::mutex> pos_lock(pos_mutex);
        VectorXf pos = current_pos;
        pos_lock.unlock();

        std::unique_lock<std::mutex> map_lock(map_mutex);
        current_map = og_map->UpdateGridMap(pos, scan);
        map_lock.unlock();


        // Version 2
        // PointCloud cloud = GetCloud(NO_BROADCAST);

        // std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        // current_cloud = cloud;
        // cloud_lock.unlock();

        // std::unique_lock<std::mutex> map_lock(map_mutex);
        // current_map = og_map->UpdateGridMapWithPointCloud(cloud);
        // map_lock.unlock();
    }
}


void Robot::Save_Map(std::string output_filename) {

    if (!map_params_set()) {
        std::cout << "No Map Available. Cancelling Save..." << std::endl;
        return;
    }

    std::unique_lock<std::mutex> map_lock(map_mutex);
    map_builder->Tensor2D_to_MapFile(current_map, output_filename, PGM, 255);
    map_lock.unlock();
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


void Robot::Set_PhysicalParameters(float robot_trackwidth, float robot_wheel_radius) {

    trackwidth = robot_trackwidth;
    wheel_radius = robot_wheel_radius;
    odom->Set_Trackwidth(trackwidth);
    physical_set = true;
    std::cout << "Phyisical Parameters Set." << std::endl;
    std::cout << "\tTrack Width: " << trackwidth << std::endl;
    std::cout << "\tWheel Radius: " << wheel_radius << std::endl;
}


void Robot::Set_MapDimensions(int height, int width) {
    map_height = height;
    map_width = width;
    map_builder->Update_2DMapDimensions(height, width);
    slam1->Set_MapDimensions(map_height, map_width);
    slam2->Set_MapDimensions(map_height, map_width);
    map_set = true;
    std::cout << "Map Parameters Set: (" << height << " x " << width << ")" << std::endl;
    current_map = Eigen::Tensor<float, 2>(height, width);
    current_map.setConstant(0.3);
}

void Robot::RobotStart() {
    
    std::cout << "Starting Robot..." << std::endl;

    map_set = false;
    physical_set = false;
    map_data_available = false;

    StartScanner();
    // serial = new Serial();
    // serial_bus1 = serial->UARTInit(0); // Motor1 Out
    // serial_bus2 = serial->UARTInit(1); // Motor2 Out

    // Set Current Position
    current_pos = VectorXf::Zero(3);

    // Setup Map Builder
    map_builder = new MapBuilder(800, 800);

    // Setup SLAM 1
    slam1 = new PoseGraphOptSLAM(500, 3, 0.01);
    slam1->FrontEndInit(5, 1.f);
    
    // Setup SLAM 2
    slam2 = new EKFSlam(3, 2);
    slam2->SetInitialState(current_pos, 0.01, 0.001);

    // Setup Mapping
    og_map = new OccupancyGridMap(500, 500, 0.01, 2 * M_PI, 6);

    // Setup Localization
    pfilter = new ParticleFilter(200, 3, 0.01);
    
    // Setup Path Planner 1
    astar_path = new A_Star();

    // Setup Path Planner 2
    rrt_path = new RRT();  

    // Setup Frontier Exploration
    frontier_explorer = new FrontierExplorer();

    //Setup Local Path Planning
    d_window = new DynamicWindowApproach(1, 0.04, 0.2, 0.1); // Temporary garbage values
    d_window->Set_TranslationalVelocityLimits(1, 1, 1); // Temporary garbage values
    d_window->Set_RotationalVelocityLimits(1, 1, 1); // Temporary garbage values

    // Setup PIDs
    pid_left = new PID(1, 1, 1, 1, 1); // Temporary garbage values
    pid_right = new PID(1, 1, 1, 1, 1); // Temporary garbage values
    pid_left->Set_Output_Limits(1, 1); // Temporary garbage values
    pid_right->Set_Output_Limits(1, 1); // Temporary garbage values

    odom = new Odom(1, 0.1); // Temporary garbage values 

    path_util = new PathUtil();
}

void Robot::RobotStop(std::string output_filename) {

    std::cout << "Saving Map..." << std::endl;
    Save_Map(output_filename);
    
    std::cout << "Shutting Down..." << std::endl;
    StopScanner();
    delete map_builder;
    delete slam1;
    delete slam2;
    delete og_map;
    delete pfilter;
    delete astar_path;
    delete rrt_path;
    delete frontier_explorer;
    delete d_window;
    delete pid_left;
    delete pid_right; 
    delete odom;
    delete path_util;
}


void Robot::MapEnv() {

    if (!map_params_set()) {
        std::cout << "Map Size Parameters Not Set. Cancelling Mapping..." << std::endl;
        return;
    }

    if (!physical_params_set()) {
        std::cout << "Physical Robot Parameters Not Set. Cancelling Mapping..." << std::endl;
        return;
    }
    std::thread mapping_thread(&diffdrive::Robot::RunMapper, this);
    std::thread explorer_thread(&diffdrive::Robot::FindFrontier, this);
    mapping_thread.join();
    explorer_thread.join();
}


void Robot::Localize(std::string map_filename) {

    if (!map_params_set()) {
        std::cout << "Map Size Parameters Not Set. Cancelling Localization..." << std::endl;
        return;
    }

    if (!physical_params_set()) {
        std::cout << "Physical Robot Parameters Not Set. Cancelling Localization..." << std::endl;
        return;
    }
    Eigen::Tensor<float, 2> map = map_builder->MapFile_to_Tensor2D(map_filename, PBM);
    std::unique_lock<std::mutex> map_lock(map_mutex);
    current_map = map;
    map_lock.unlock();
    std::thread localizer_thread(&diffdrive::Robot::RunLocalizer, this, map);
    std::thread explorer_thread(&diffdrive::Robot::FindFrontier, this);
    localizer_thread.join();
    explorer_thread.join();
}


void Robot::MapAndLocalize(int algorithm) {

    if (!map_params_set()) {
        std::cout << "Map Size Parameters Not Set. Cancelling SLAM..." << std::endl;
        return;
    }

    if (!physical_params_set()) {
        std::cout << "Physical Robot Parameters Not Set. Cancelling SLAM..." << std::endl;
        return;
    }

    std::thread slam_thread(&diffdrive::Robot::RunSLAM, this, algorithm);
    std::thread explorer_thread(&diffdrive::Robot::FindFrontier, this);
    slam_thread.join();
    explorer_thread.join();
}


std::vector<VectorXi> Robot::CreatePath(int algorithm, std::string map_filename, VectorXi start, VectorXi goal) {

    if (algorithm == A_STAR) {
        astar_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        return astar_path->Path(start, goal);
    }   
    
    else if (algorithm == RRT_VANILLA) {
        rrt_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        return rrt_path->RRT_Path(start, goal, 1);
    }
    
    else if (algorithm == RRT_STAR) {
        rrt_path->Load_MAP(map_builder->MapFile_to_Tensor2D(map_filename, PBM));
        return rrt_path->RRTStar_Path(start, goal, 1, 2);
    } 
}



void Robot::BroadcastPointCloud() {

    GetCloud(BROADCAST);
}







}