#include "../include/Odometry.hpp"


Odom::Odom() {}

Odom::Odom(float robot_trackwidth, float time_step) : trackwidth(robot_trackwidth), dt(time_step) {

    serial = new Serial();

    // I2C
    serial->i2cInit(I2C_NUM, I2C_SLAVE_ADDRS);
    // serial->I2CInit(20, 0x0);

    // UART
    // serial->UARTInit(0);
    // serial->UARTInit(1);
}


void Odom::setTrackwidth(float robot_trackwidth) {

    trackwidth = robot_trackwidth;
}

VectorXf Odom::getEncoderDerivedPosition() {
    VectorXf new_position(3);
    uint8_t *encoder_buffer = new uint8_t[12];
    serial->i2cRead(I2C_NUM, encoder_buffer, 12);
    uint32_t x = *((uint32_t*)encoder_buffer);
    uint32_t y = *((uint32_t*)(encoder_buffer + 4));
    uint32_t th = *((uint32_t*)(encoder_buffer + 8));
    float x_pos = *((float *)&x);
    float y_pos = *((float *)&y);
    float theta = *((float *)&th);

    new_position << x_pos, y_pos, theta;
    delete encoder_buffer;
    return new_position;
}

VectorXf Odom::getEncoderDerivedVelocities() {

    VectorXf new_velocity(2);
    uint8_t *encoder_buffer = new uint8_t[8];
    serial->i2cRead(I2C_NUM, encoder_buffer, 8);
    uint32_t rot = *((uint32_t*)encoder_buffer);
    uint32_t trans = *((uint32_t*)(encoder_buffer + 4));
    float rotational = *((float *)&rot);
    float translational = *((float *)&trans);

    new_velocity << rotational, translational;

    delete encoder_buffer;
    return new_velocity;
}

VectorXf Odom::getIMUDerivedVelocities() {

    VectorXf new_velocity(2);
    uint8_t *imu_buffer = new uint8_t[16];
    serial->i2cRead(I2C_NUM, imu_buffer, 16);

    uint32_t rot_x = *((uint32_t*)imu_buffer);
    uint32_t rot_y = *((uint32_t*)(imu_buffer + 4));
    uint32_t trans_x = *((uint32_t*)(imu_buffer + 8));
    uint32_t trans_y = *((uint32_t*)(imu_buffer + 12));

    // std::cout << "Raw IMU VALUES: rot(" << *(float*)&rot_x << ", " << *(float*)&rot_y 
    //     << "), trans(" << *(float*)&trans_x << ", " << *(float*)&trans_y << ")" << std::endl;

    
    float rot = std::hypot(*((float *)&rot_x), *((float *)&rot_y));
    float trans = std::hypot(*((float *)&trans_x), *((float *)&trans_y));
    new_velocity << rot, trans;

    // Test 
    std::cout << "OOOOOOOOOODDDDDDDOOOOOOOOMMMMMMMMM FROM IMU!!!!!!!!!!!!!!!: " << rot << ", " << trans << std::endl;

    delete imu_buffer;
    return new_velocity;
}














