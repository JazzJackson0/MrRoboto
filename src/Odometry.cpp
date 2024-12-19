#include "../include/Odometry.hpp"


Odom::Odom() {}

Odom::Odom(float robot_trackwidth, float time_step) : trackwidth(robot_trackwidth), dt(time_step) {

    serial = new Serial();

    // I2C
    serial_bus1 = serial->I2CInit(1, 0x055);
    // serial_bus2 = serial->I2CInit(20, 0x0);

    // UART
    // serial_bus1 = serial->UARTInit(0);
    // serial_bus2 = serial->UARTInit(1);
}


void Odom::Set_Trackwidth(float robot_trackwidth) {

    trackwidth = robot_trackwidth;
}

VectorXf Odom::Get_NewPosition() {
    VectorXf new_position(3);
    int8_t *encoder_buffer = new int8_t[8];
    serial->I2CRead(serial_bus1, encoder_buffer, 8);
    int32_t left = *((int32_t*)encoder_buffer);
    int32_t right = *((int32_t*)(encoder_buffer + 4));
    // left_distance = (float)left;
    // right_distance = (float)right;
    left_distance = *((float *)&left);
    right_distance = *((float *)&right);

    float phi = left_distance - right_distance / trackwidth;
    float r_center = trackwidth / 2;
    VectorXf P(2);
    P << previous_pose[0] - r_center * std::cos(previous_pose[2]), previous_pose[0] - r_center * std::sin(previous_pose[2]);
    new_position << P[0] + r_center * std::cos(phi + previous_pose[2]), P[1] + r_center * std::sin(phi + previous_pose[2]), phi + previous_pose[2];

    delete encoder_buffer;
    return new_position;
}

VectorXf Odom::Get_NewVelocities() {

    VectorXf new_velocity(2);
    int8_t *encoder_buffer = new int8_t[8];
    serial->I2CRead(serial_bus1, encoder_buffer, 8);
    int32_t left = *((int32_t*)encoder_buffer);
    int32_t right = *((int32_t*)(encoder_buffer + 4));
    // left_distance = (float)left;
    // right_distance = (float)right;
    left_distance = *((float *)&left);
    right_distance = *((float *)&right);

    float phi = left_distance - right_distance / trackwidth;
    float r_center = trackwidth / 2;
    new_velocity << r_center/dt, phi/dt;

    delete encoder_buffer;
    return new_velocity;
}

// No true odometry calculation. Just raw imu reads.
VectorXf Odom::Get_NewRawVelocities() {

    VectorXf new_velocity(2);
    int8_t *imu_buffer = new int8_t[16];
    serial->I2CRead(serial_bus1, imu_buffer, 16);
    int32_t rot_x = *((int32_t*)imu_buffer);
    int32_t rot_y = *((int32_t*)imu_buffer + 4);
    int32_t trans_x = *((int32_t*)(imu_buffer + 8));
    int32_t trans_y = *((int32_t*)(imu_buffer + 12));

    float rot = std::sqrt(std::pow(*((float *)&rot_x), 2) + std::pow(*((float *)&rot_y), 2));
    float trans = std::sqrt(std::pow(*((float *)&trans_x), 2) + std::pow(*((float *)&trans_y), 2));
    new_velocity << rot, trans;

    delete imu_buffer;
    return new_velocity;
}














