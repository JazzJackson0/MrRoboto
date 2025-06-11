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

void Robot::set_physical_parameters(float robot_wheel_radius, float robot_trackwidth) {

    trackwidth = robot_trackwidth;
    wheel_radius = robot_wheel_radius;
    odom->setTrackwidth(trackwidth);
    physical_set = true;
    // std::cout << "Phyisical Parameters Set.\n";
    // std::cout << "Track Width: " << trackwidth << "\n";
    // std::cout << "Wheel Radius: " << wheel_radius << "\n";
}


void Robot::set_map_dimensions(int width, int height) {
    map_height = height;
    map_width = width;
    map_builder->update2DMapDimensions(height, width);
    slam1->setMapDimensions(map_height, map_width);
    slam2->setMapDimensions(map_height, map_width);
    map_set = true;
    // std::cout << "Map Parameters Set: (" << height << " x " << width << ")" << std::endl;
    current_map = Eigen::Tensor<float, 2>(height, width);
    current_map.setConstant(0.3);
}

bool Robot::map_params_set() {
    return map_set;
}

bool Robot::physical_params_set() {
    return physical_set;
}

void Robot::set_parameters() {

    std::ifstream map_f(map_param_path);
    std::ifstream diff_f(robot_param_path);

    json map_data = json::parse(map_f);
    json diffbot_data = json::parse(diff_f);
    set_physical_parameters(diffbot_data["wheel_radius"], diffbot_data["robot_trackwidth"]);
    set_map_dimensions(map_data["width"], map_data["height"]);

}

void Robot::callibrate_camera(const std::string& images_path) {

    calibrator->calibrateCamera(images_path);

    calibrator->refineParameters();

    double diagnosis = calibrator->diagnoseCalibration();
    std::cout << diagnosis << std::endl;
}

void Robot::start_scanner() {

    lidar->startMotor();
    // Start scan with default scan mode
    LidarScanMode scanMode;
    lidar->startScan(false, true, 0, &scanMode);
}

void Robot::stop_scanner() {

    lidar->stop();
    // lidar->disconnect();
    lidar->stopMotor();
    RPlidarDriver::DisposeDriver(lidar);
}

void Robot::raw_scan() {

    // Grab Complete 0-360 degree scan previously received
    auto res = lidar->grabScanDataHq(nodes, nodeCount);

    // Scan Failed
    if (IS_FAIL(res))
        std::cerr << "ERROR: Failed to get scan data" << std::endl;

    // Scan Success
    else
        std::cout << "Scan Success" << std::endl; 
}


PointCloud Robot::get_cloud(int broadcast_state) {

    PointCloud cloud;
    cloud.mean_x = 0.f;
    cloud.mean_y = 0.f;
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

        
        std::unique_lock<std::mutex> pos_lock(pos_mutex);
        VectorXf pos = current_pos;
        pos_lock.unlock();

        // Get coordinate of ray.
        int size = pos.rows();
        float x = (pos[0] + (dist_cm * std::cos(rad)));
        float y = (pos[1] + (dist_cm * std::sin(rad)));

        if (broadcast_state == NO_BROADCAST) {

            point << x, y;
            cloud.points.push_back(point);
            cloud.weights.push_back((float)(nodes[i].quality >> 2)); // Echo Signal Strength 0 - 255
            cloud.mean_x += x;
            cloud.mean_y += y;
            cloud.kd_tree.addData(point, (float)(nodes[i].quality >> 2));
        }
        
        else if (broadcast_state == BROADCAST)
            std::cout << x * 100 << ", " << y * 100 << std::endl; // Convert point from m to cm
    }
    cloud.mean_x /= cloud.points.size();
    cloud.mean_y /= cloud.points.size();
    cloud.kd_tree.buildKDTree();

    return cloud;
}


std::vector<VectorXf> Robot::get_scan() {

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



void Robot::find_frontier() {

    while (!map_ready());

    while (1) {

        std::unique_lock<std::mutex> map_lock(map_mutex);
        Eigen::Tensor<float, 2> map = current_map;
        map_lock.unlock();

        std::unique_lock<std::mutex> cloud_lock(map_mutex);
        PointCloud cloud = current_cloud;
        cloud_lock.unlock();

        std::unique_lock<std::mutex> pos_lock(map_mutex);
        VectorXi pos = map_builder->mapCoordinateToDataStructureIndex(current_pos.head<2>());
        VectorXf flt_pos = current_pos;
        pos_lock.unlock();
        
        map_builder->applyInflationLayer(map, 5); // Create Cost Map
        frontier_explorer->loadMAP(map);

        // std::cout << map << std::endl;
        // std::cout << std::endl;

        VectorXi frontier_goal = frontier_explorer->findFrontier(pos);
        std::vector<VectorXf> waypoints = path_util->smoothPath(path_util->samplePath(create_path(A_STAR, map, pos, frontier_goal)));
        follow_local_path(waypoints, map, cloud, flt_pos);
    }
}


std::vector<VectorXi> Robot::create_path(int algorithm, Eigen::Tensor<float, 2> map, VectorXi start, VectorXi goal) {

    std::vector<VectorXi> path;
    path.push_back(start);

    if (algorithm == A_STAR) {
        astar_path->loadMAP(map);
        return astar_path->path(start, goal);
    }   
    
    else if (algorithm == RRT_VANILLA) {
        rrt_path->loadMAP(map);
        return rrt_path->rrtPath(start, goal, 1);
    }
    
    else if (algorithm == RRT_STAR) {
        rrt_path->loadMAP(map);
        return rrt_path->rrtStarPath(start, goal, 1, 2);
    }

    return path; 
}


void Robot::follow_local_path(std::vector<VectorXf> smooth_waypoints, Eigen::Tensor<float, 2> map, PointCloud cloud, VectorXf pos) {

    float wheel_radius_inv = 1 / wheel_radius;
    for (int i = 0; i < smooth_waypoints.size(); i++) {

        d_window->setGoal(smooth_waypoints[i]);

        std::unique_lock<std::mutex> odom_lock(odom_read_mutex);
        // VectorXf odom_vels = odom->getNewVelocities();
        VectorXf odom_vels = odom->getNewRawVelocities();
        odom_lock.unlock();
        VectorXf vels = d_window->run(pos, cloud, odom_vels);
        
        // Determine Wheel Angular Velocities (w_l & w_r)
        float current_vel_r = (odom_vels[0] + ((odom_vels[1] * trackwidth) * 0.5)) * wheel_radius_inv;
        float current_vel_l = (odom_vels[0] - ((odom_vels[1] * trackwidth) * 0.5)) * wheel_radius_inv;
        float setpoint_vel_r = (vels[0] + ((vels[1] * trackwidth) * 0.5)) * wheel_radius_inv;
        float setpoint_vel_l = (vels[0] - ((vels[1] * trackwidth) * 0.5)) * wheel_radius_inv;
        float right_wheel_duty_cycle = pid_right->pidUpdate(setpoint_vel_r, current_vel_r);
        float left_wheel_duty_cycle = pid_left->pidUpdate(setpoint_vel_l, current_vel_l);

        // std::cout << "Current Velocities: " << odom_vels.transpose() << "\n";
        // std::cout << "Goal Velocities: " << vels.transpose() << "\n";
        // std::cout << "CURENT WHEEL SPEEDS: Right-> " << current_vel_r << " Left-> " << current_vel_l << "\n";
        // std::cout << "GOAL WHEEL SPEEDS: Right-> " << setpoint_vel_r << " Left-> " << setpoint_vel_l << "\n";
        // std::cout << "PID Right UPDATE SPEED: " << right_wheel_duty_cycle << " PID Left UPDATE SPEED: " << left_wheel_duty_cycle << std::endl;

        // Motor Out
        uint32_t left_wheel_duty_cycle_temp = *((uint32_t*) (&left_wheel_duty_cycle));
        uint32_t right_wheel_duty_cycle_temp = *((uint32_t*) (&right_wheel_duty_cycle));
        char duty_cycle_buff[8] = {0};
        for (int i = 0; i < 8; i++) {

            if (i < 4)
                duty_cycle_buff[i] = *((char*)&left_wheel_duty_cycle_temp + i);

            else
                duty_cycle_buff[i] = *((char*)&right_wheel_duty_cycle_temp + (i - 4));
        }   

        serial->uartWrite(UART_NUM, duty_cycle_buff, 8);
    }
}

void Robot::run_slam(int algorithm) {

    bool first_map = true;

    while (1) {
        raw_scan();
        PointCloud cloud = get_cloud(NO_BROADCAST);

        std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        current_cloud = cloud;
        cloud_lock.unlock();

        if (algorithm == POSE_GRAPH) {
            
            Eigen::Tensor<float, 2> new_map = slam1->run(cloud);
            // std::cout << current_map << "\n";
            // std::cout << std::endl;
            
            std::unique_lock<std::mutex> map_lock(map_mutex);
            current_map = new_map;
            map_lock.unlock();


            if (first_map) {
                std::unique_lock<std::mutex> map_state_lock(map_ready_mutex);
                map_data_available = true;
                first_map = false;
            } 

            std::unique_lock<std::mutex> pos_lock(pos_mutex);
            current_pos = slam1->broadcastCurrentPose();  
        }
        
        else if (algorithm == EKF) {

            std::unique_lock<std::mutex> odom_lock(odom_read_mutex);
            // VectorXf odom_out = odom->getNewVelocities();
            VectorXf odom_out = odom->getNewRawVelocities();
            odom_lock.unlock();

            ControlCommand ctrl;
            ctrl.trans_vel = odom_out[0];
            ctrl.rot_vel = odom_out[1];

            // [TEMPORARY] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // ControlCommand ctrl;
            // ctrl.trans_vel = 0.5;
            // ctrl.rot_vel = 0.0;
            //-------------------------------------------------

            Eigen::Tensor<float, 2> new_map = slam2->run(cloud, ctrl);
            // std::cout << current_map << std::endl;
            // std::cout << std::endl;

            if (first_map) {
                std::unique_lock<std::mutex> map_state_lock(map_ready_mutex);
                map_data_available = true;
                first_map = false;
            }

            std::unique_lock<std::mutex> map_lock(map_mutex);
            current_map = new_map;
            map_lock.unlock();

            std::unique_lock<std::mutex> pos_lock(pos_mutex);
            current_pos = slam2->broadcastCurrentPose();       
        }
    }
}



cv::Mat Robot::run_vslam(bool stereo) {

    // Monocular Camera
    if (!stereo) {

        // Open a video capture device (0 for the default webcam)
        if (!cap.open(0))
            std::cerr << "ERROR: Could not open video capture device.\n";

        while (true) {

            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) {
                std::cerr << "ERROR: No frame captured.\n";
                break;  // Exit if no frame is captured
            }

            v_slam->runMono(frame);

            // Exit on 'q' key press
            if (cv::waitKey(1) == 'q')
                break;   
        }
        cap.release();
        cv::destroyAllWindows();
    }
    

    // Stereo Camera
    else {
        if (!cap_left.open(0) || !cap_right.open(1))
            std::cerr << "ERROR: Could not open video capture device(s).\n";

        while (true) {
            // Capture frame-by-frame
            cv::Mat frame_left, frame_right;
            cap_left >> frame_left;
            cap_right >> frame_right;
            if (frame_left.empty() || frame_right.empty()) {
                std::cerr << "ERORR: No frame captured.\n";
                break;  // Exit if no frames are captured
            }

            v_slam->runStereo(frame_left, frame_right);

            // Exit on 'q' key press
            if (cv::waitKey(1) == 'q')
                break;
        }

        cap_left.release();
        cap_right.release();
        cv::destroyAllWindows();
    }
}



// Should probably add some way to recognize when you are localized and stopping pfilter process.
void Robot::run_localizer(Eigen::Tensor<float, 2> map) {

    pfilter->addMap(map, 6, 2 * M_PI);
    std::cout << "Map Added!!!\n";
    while (1) {
        raw_scan();
        PointCloud cloud = get_cloud(NO_BROADCAST);

        std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        current_cloud = cloud;
        cloud_lock.unlock();

        std::unique_lock<std::mutex> odom_lock(odom_read_mutex);
        // VectorXf odom_out = odom->getNewVelocities();
        VectorXf odom_out = odom->getNewRawVelocities();
        odom_lock.unlock();

        ControlCommand ctrl;
        ctrl.trans_vel = odom_out[0];
        ctrl.rot_vel = odom_out[1];
        pfilter->run(cloud, ctrl);
    }
}


// Needs GPS running on separate thread to update the "current_pos" variable.
void Robot::run_mapper() {

    while (1) {
        raw_scan();
        // Version 1
        std::vector<VectorXf> scan = get_scan();

        std::unique_lock<std::mutex> scan_lock(scan_mutex);
        current_scan = scan;
        scan_lock.unlock();

        std::unique_lock<std::mutex> pos_lock(pos_mutex);
        VectorXf pos = current_pos;
        pos_lock.unlock();

        Eigen::Tensor<float, 2> new_map = og_map->updateGridMap(pos, scan);

        std::unique_lock<std::mutex> map_lock(map_mutex);
        current_map = new_map;
        map_lock.unlock();


        // Version 2
        // PointCloud cloud = GetCloud(NO_BROADCAST);

        // std::unique_lock<std::mutex> cloud_lock(cloud_mutex);
        // current_cloud = cloud;
        // cloud_lock.unlock();

        // Eigen::Tensor<float, 2> new_map = og_map->updateGridMapWithPointCloud(cloud);

        // std::unique_lock<std::mutex> map_lock(map_mutex);
        // current_map = new_map;
        // map_lock.unlock();
    }
}


void Robot::save_map(std::string output_filename) {

    if (!map_params_set()) {
        std::cerr << "ERROR: No Map Available. Cancelling Save...\n";
        return;
    }

    std::unique_lock<std::mutex> map_lock(map_mutex);
    Eigen::Tensor<float, 2> map = current_map;
    map_lock.unlock();

    map_builder->tensor2DToMapFile(map, output_filename, PGM, 255);
}


Robot::Robot() {

    // Create a LIDAR driver instance
    lidar = RPlidarDriver::CreateDriver();
    std::string usb0 = "/dev/ttyUSB0";
    std::string usb1 = "/dev/ttyUSB1";
    std::string usb = usb0;
    int attempts = 1;

    while (attempts <= 2) {
        
        if (attempts > 1) fprintf(stderr, "Retrying...\n");   

        auto res = lidar->connect(usb.c_str(), 115200);

        if(SL_IS_OK(res)) { 
        //if(SL_IS_OK(lidar->connect(_channel))) { 
            
            // lidar->stopMotor();
            //lidar->setMotorSpeed(0);

            // Display Device Info
            sl_lidar_response_device_info_t deviceInfo;
            auto res2 = lidar->getDeviceInfo(deviceInfo);
            
            if(SL_IS_OK(res2)) {
                
                // printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                // deviceInfo.model,
                // deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                // deviceInfo.hardware_version);
                attempts = 3;
            }
            
            else {
                fprintf(stderr, "ERROR: Failed to get device information from LIDAR %08x\r\n", res);   
                usb = usb1; 
                attempts++; 
            }
                
        }
        
        else {
            fprintf(stderr, "ERROR: Failed to connect to LIDAR %08x\r\n", res);
            usb = usb1;  
            attempts++; 
        }
    }
    

    // Set Up NodeCount for Scan Data
    nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);
}


// Public------------------------------------------------------------------------------------------------------------------------
Robot * Robot::CreateRobot() {

    return new Robot();
}

void Robot::robotStart(bool autonomous) {
    this->autonomous = autonomous;

    std::cout << "Starting Robot..." << std::endl;

    map_set = false;
    physical_set = false;
    map_data_available = false;

    start_scanner();

    // Setupe for PWM Out Values
    serial = std::make_unique<Serial>();
    serial->uartInit(UART_NUM); 

    // Set Current Position
    current_pos = VectorXf::Zero(3);
    
    // Setup Map Builder
    map_builder = std::make_unique<MapBuilder>(800, 800);


    // Setup SLAM 1
    slam1 = std::make_unique<PoseGraphOptSLAM>(MAX_NODES, POSE_DIMENSION, GUESS_VARIATION);
    slam1->frontEndInit(N_RECENT_POSES, LOOP_CLOSURE_DIST);
    
    // Setup SLAM 2
    slam2 = std::make_unique<EKFSlam>(POSE_DIM, LANDMARK_DIM);
    slam2->setInitialState(current_pos, PROCESS_UNCERTAINTY, MEASUREMENT_UNCERTAINTY);

    // Setup SLAM 3
    v_slam = std::make_unique<vSLAM>();

    // Setup Mapping
    og_map = std::make_unique<OccupancyGridMap>(500, 500, ALPHA, BETA, MAX_SCAN_RANGE);

    // Setup Localization
    pfilter = std::make_unique<ParticleFilter>(MAX_PARTICLES, P_FILTER_POSE_DIM, P_FILTER_TIME_INTERVAL);
    
    // Setup Path Planner 1
    astar_path = std::make_unique<AStar>();

    // Setup Path Planner 2
    rrt_path = std::make_unique<RRT>();  

    // Setup Frontier Exploration
    frontier_explorer = std::make_unique<FrontierExplorer>();

    //Setup Local Path Planning
    d_window = std::make_unique<DynamicWindowApproach>(SMOOTHING_VAL, HEADING_WEIGHT, DISTANCE_WEIGHT, VELOCITY_WEIGHT); 
    d_window->setTranslationalVelocityLimits(MIN_TRANS_VEL, MAX_TRANS_VEL, TRANS_VEL_INTERVAL); 
    d_window->setRotationalVelocityLimits(MIN_ROT_VEL, MAX_ROT_VEL, ROT_VEL_INTERVAL); 
    d_window->setMaxAccelerations(MAX_TRANS_ACCEL, MAX_ROT_ACCEL);

    // Setup PIDs
    pid_left = std::make_unique<PID>(DIRECT, SAMPLE_TIME_MS, KP, KI, KD); 
    pid_right = std::make_unique<PID>(DIRECT, SAMPLE_TIME_MS, KP, KI, KD); 
    pid_left->setOutputLimits(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL); 
    pid_right->setOutputLimits(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL); 

    odom = std::make_unique<Odom>(ODOM_TRACKWIDTH, ODOM_TIMESTEP);

    path_util = std::make_unique<PathUtil>();

    controller = std::make_unique<Controller>(DIFF_BOT);

    // Set Physical Parameters for Vehicle & Environment (Robot, Map, Etc...)
    set_parameters();
}

void Robot::robotStop(std::string output_filename) {

    std::cout << "Saving Map...\n";
    save_map(output_filename);
    
    std::cout << "Shutting Down..." << std::endl;
    stop_scanner();
    // std::cout << "Scanner Stopped" << std::endl;
    Serial::RestoreTerminal();
}


void Robot::mapEnv() {

    if (!map_params_set()) {
        std::cerr << "ERROR: Map Size Parameters Not Set. Cancelling Mapping...\n";
        return;
    }

    if (!physical_params_set()) {
        std::cerr << "ERROR: Physical Robot Parameters Not Set. Cancelling Mapping...\n";
        return;
    }

    if (autonomous) {
        std::thread explorer_thread(&diffdrive::Robot::find_frontier, this);
         explorer_thread.join();
    }

    else {
        std::thread controller_thread(&Controller::controllerRun, controller.get());
        controller_thread.join();
    }
        
    std::thread mapping_thread(&diffdrive::Robot::run_mapper, this);
    mapping_thread.join();
   
}


void Robot::localize(std::string map_filename) {

    if (!map_params_set()) {
        std::cerr << "ERROR: Map Size Parameters Not Set. Cancelling Localization...\n";
        return;
    }

    if (!physical_params_set()) {
        std::cerr << "ERROR: Physical Robot Parameters Not Set. Cancelling Localization...\n";
        return;
    }
    Eigen::Tensor<float, 2> map = map_builder->mapFileToTensor2D(map_filename, PBM);
    std::unique_lock<std::mutex> map_lock(map_mutex);
    current_map = map;
    map_lock.unlock();

    if (autonomous) {
        std::thread explorer_thread(&diffdrive::Robot::find_frontier, this);
         explorer_thread.join();
    }

    else {
        std::thread controller_thread(&Controller::controllerRun, controller.get());
        controller_thread.join();
    }

    std::thread localizer_thread(&diffdrive::Robot::run_localizer, this, map);
    localizer_thread.join();
}


void Robot::mapAndLocalize(int algorithm) {

    if (!map_params_set()) {
        std::cout << "ERROR: Map Size Parameters Not Set. Cancelling SLAM...\n";
        return;
    }

    if (!physical_params_set()) {
        std::cout << "ERROR: Physical Robot Parameters Not Set. Cancelling SLAM...\n";
        return;
    }

    if (autonomous) {
        std::thread explorer_thread(&diffdrive::Robot::find_frontier, this);
         explorer_thread.join();
    }

    else {
        std::thread controller_thread(&Controller::controllerRun, controller.get());
        controller_thread.join();
    }

    std::thread slam_thread(&diffdrive::Robot::run_slam, this, algorithm);
    slam_thread.join();
}


std::vector<VectorXi> Robot::createPath(int algorithm, std::string map_filename, VectorXi start, VectorXi goal) {

    if (algorithm == A_STAR) {
        astar_path->loadMAP(map_builder->mapFileToTensor2D(map_filename, PBM));
        return astar_path->path(start, goal);
    }   
    
    else if (algorithm == RRT_VANILLA) {
        rrt_path->loadMAP(map_builder->mapFileToTensor2D(map_filename, PBM));
        return rrt_path->rrtPath(start, goal, 1);
    }
    
    else if (algorithm == RRT_STAR) {
        rrt_path->loadMAP(map_builder->mapFileToTensor2D(map_filename, PBM));
        return rrt_path->rrtStarPath(start, goal, 1, 2);
    } 
}



void Robot::broadcastPointCloud() {

    get_cloud(BROADCAST);
}


}