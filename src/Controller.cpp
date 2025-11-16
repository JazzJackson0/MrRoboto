#include "../include/Controller.hpp"

// TODO: Major cleanup

uint8_t* Controller::create_data_packet(uint8_t packet_type, uint8_t motor_directions, float left_speed, float right_speed) {
    uint8_t *data = new uint8_t[packet_type]();
    data[0] = packet_type;
    
    if (packet_type == DIRECTION_PACKET) {
        data[1] = motor_directions;
    }

    else if (packet_type == SPEED_PACKET) {
        uint32_t left_s = (uint32_t) left_speed;
        uint32_t right_s = (uint32_t) right_speed;
        data[1] = (left_s >> 24) & 0xFF;
        data[2] = (left_s >> 16) & 0xFF;
        data[3] = (left_s >> 8) & 0xFF;
        data[4] = left_s & 0xFF;
        data[5] = (right_s >> 24) & 0xFF;
        data[6] = (right_s >> 16) & 0xFF;
        data[7] = (right_s >> 8) & 0xFF;
        data[8] = right_s & 0xFF;
        std::cout << "Left Speed: " << left_speed << std::endl;
        std::cout << "Right Speed: " << right_speed << std::endl;
        std::cout << std::endl;
    }

    else if (packet_type == FULL_PACKET) {
        uint32_t left_s = (uint32_t) left_speed;
        uint32_t right_s = (uint32_t) right_speed;
        data[1] = motor_directions;
        data[2] = (left_s >> 24) & 0xFF;
        data[3] = (left_s >> 16) & 0xFF;
        data[4] = (left_s >> 8) & 0xFF;
        data[5] = left_s & 0xFF;
        data[6] = (right_s >> 24) & 0xFF;
        data[7] = (right_s >> 16) & 0xFF;
        data[8] = (right_s >> 8) & 0xFF;
        data[9] = right_s & 0xFF;

        
        std::cout << "Left Speed: " << left_speed << std::endl;
        std::cout << "Right Speed: " << right_speed << std::endl;
        std::cout << "Full Packet Data: " << std::endl;
        for (int i = 0; i < 10; i++) {
            std::cout << "data[" << i << "]: " << +data[i] << std::endl;
        }
    }

    return data;
}


uint8_t* Controller::create_quad_data_packet(uint8_t packet_type, uint8_t motor_directions, 
    float front_left_speed, float back_left_speed, float front_right_speed, float back_right_speed) {
    uint8_t *data = new uint8_t[packet_type]();
    data[0] = packet_type;
    
    if (packet_type == DIRECTION_PACKET) {
        data[1] = motor_directions;
    }

    else if (packet_type == SPEED_PACKET) {
        uint32_t front_left_s = (uint32_t) front_left_speed;
        uint32_t back_left_s = (uint32_t) back_left_speed;
        uint32_t front_right_s = (uint32_t) front_right_speed;
        uint32_t back_right_s = (uint32_t) back_right_speed;
        data[1] = (front_left_s >> 24) & 0xFF;
        data[2] = (front_left_s >> 16) & 0xFF;
        data[3] = (front_left_s >> 8) & 0xFF;
        data[4] = front_left_s & 0xFF;
        data[5] = (back_left_s >> 24) & 0xFF;
        data[6] = (back_left_s >> 16) & 0xFF;
        data[7] = (back_left_s >> 8) & 0xFF;
        data[8] = back_left_s & 0xFF;
        data[9] = (front_right_s >> 24) & 0xFF;
        data[10] = (front_right_s >> 16) & 0xFF;
        data[11] = (front_right_s >> 8) & 0xFF;
        data[12] = front_right_s & 0xFF;
        data[13] = (back_right_s >> 24) & 0xFF;
        data[14] = (back_right_s >> 16) & 0xFF;
        data[15] = (back_right_s >> 8) & 0xFF;
        data[16] = back_right_s & 0xFF;
    }

    return data;
}

void Controller::remove_waiting(RobotMovements movement) {
    for (int i = 0; i < waiting.size(); i++) {
        if (waiting[i].command == movement) {
            waiting.erase(waiting.begin() + i);
        }
    }
}

void Controller::update_waiting() {
    if (waiting.size() > 0) {
        waiting[0].function(PRESS);
        active[waiting[0].command] = true;
    }
}

void Controller::log_button_press(int btnState, RobotCommands command) {
   
    switch (command) {
        case MOVE_FWD: {
            if (btnState == PRESS) {
                std::cout << "(D-Pad Up) Forward Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(D-Pad Up) Forward Button Released" << std::endl;
            }
        }
        case MOVE_BKWD: {
            if (btnState == PRESS) {
                std::cout << "(D-Pad Down) Backward Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(D-Pad Down) Backward Button Released" << std::endl;
            }
        }
        case TURN_LEFT: {
            if (btnState == PRESS) {
                std::cout << "(Square) Left Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(Square) Left Button Released" << std::endl;
            }
        }
        case TURN_RIGHT: {
            if (btnState == PRESS) {
                std::cout << "(O) Right Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(O) Right Button Released" << std::endl;
            }
        }
        case ACCEL: {
            if (btnState == PRESS) {
                std::cout << "(R1) Accel Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(R1) Accel Button Released" << std::endl;
            }
        }
        case BRAKE: {
            if (btnState == PRESS) {
                std::cout << "(L1) Brake Button Pressed" << std::endl;
            } else if (btnState == RELEASE) {
                std::cout << "(L1) Brake Button Released" << std::endl;
            }
        }
    }
}


void Controller::diff_robot_move(int btnState, RobotCommands command) {
    if (btnState == PRESS) {
        uint8_t * data = create_data_packet(FULL_PACKET, command, 
            (motor.left_speed += DIFF_MOVE_DELTA), (motor.right_speed += DIFF_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, FULL_PACKET);
        delete[] data;
        log_button_press(btnState, command);
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_data_packet(FULL_PACKET, STOP, 
            (motor.left_speed -= DIFF_MOVE_DELTA), (motor.right_speed -= DIFF_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, FULL_PACKET);
        delete[] data;
        log_button_press(btnState, command);
        return;
    }
}


void Controller::diff_robot_accelerate(int btnState) {
    if (btnState == PRESS) {
        motor.left_speed += DIFF_MOVE_DELTA;
        motor.right_speed += DIFF_MOVE_DELTA;
        uint8_t * data = create_data_packet(SPEED_PACKET, 0, motor.left_speed, motor.right_speed);
        serial->uartWrite(this->dev_num, data, SPEED_PACKET);
        delete[] data;
        log_button_press(btnState, ACCEL);
        return;
    }
    else if (btnState == RELEASE) {
        motor.left_speed = (motor.left_speed == 100) ? DIFF_MOVE_DELTA : 0;
        motor.right_speed = (motor.right_speed == 100) ? DIFF_MOVE_DELTA : 0;
        uint8_t * data = create_data_packet(SPEED_PACKET, 0, motor.left_speed, motor.right_speed);
        serial->uartWrite(this->dev_num, data, SPEED_PACKET);
        delete[] data;
        log_button_press(btnState, ACCEL);
        return;
    }
}

void Controller::diff_robot_break(int btnState) {
    // if (btnState == PRESS) {
    //     // // (Hard) Stop-----------------------------
    //     uint8_t * data = create_data_packet(DIRECTION_PACKET, BREAK, 0, 0);
    //     serial->uartWrite(this->dev_num, data, DIRECTION_PACKET);
    //     delete[] data;
    //     log_button_press(btnState, BRAKE);
    //     return;
    // }
    // else if (btnState == RELEASE) {
    //     uint8_t * data = create_data_packet(DIRECTION_PACKET, STOP, 0, 0);
    //     serial->uartWrite(this->dev_num, data, DIRECTION_PACKET);
    //     delete[] data;
    //     log_button_press(btnState, BRAKE);
    //     return;
    // }
}


void Controller::quad_yaw_right(int btnState) {
    // Rotate (Yaw) Right-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, 
            (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed, 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "O Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, 
            (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed, 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "O Button Released" << std::endl;
        return;
    }
}

void Controller::quad_yaw_left(int btnState) {
    // Rotate (Yaw) Left-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "Square Button Pressed" << std::endl;  
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "Square Button Released" << std::endl;  
        return;
    }
}


void Controller::quad_roll_right(int btnState) {
    // Bend (Roll) Right-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "O Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "O Button Released" << std::endl;
        return;
    }
}

void Controller::quad_roll_left(int btnState) {
    // Bend (Roll) Left-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed, 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "Square Button Pressed" << std::endl;  
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed, 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "Square Button Released" << std::endl;  
        return;
    }
}

void Controller::quad_pitch_forward(int btnState) {
    // Pitch Forward-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Up Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Up Button Released" << std::endl;
        return;
    }
}

void Controller::quad_pitch_backward(int btnState) {
    // Pitch Backward-----------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
        (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed,
        (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
        (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed,
        (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}


void Controller::quad_up(int btnState) {
// Move Up--------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed += QUAD_MOVE_DELTA), (motor.back_left_speed += QUAD_MOVE_DELTA),
            (motor.front_right_speed += QUAD_MOVE_DELTA), (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed -= QUAD_MOVE_DELTA), (motor.back_left_speed -= QUAD_MOVE_DELTA),
            (motor.front_right_speed -= QUAD_MOVE_DELTA), (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}

void Controller::quad_down(int btnState) {
// Move Down--------------------------
    if (btnState == PRESS) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed -= QUAD_DOWN_DELTA), (motor.back_left_speed -= QUAD_DOWN_DELTA),
            (motor.front_right_speed -= QUAD_DOWN_DELTA), (motor.back_right_speed -= QUAD_DOWN_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed += QUAD_DOWN_DELTA), (motor.back_left_speed += QUAD_DOWN_DELTA),
            (motor.front_right_speed += QUAD_DOWN_DELTA), (motor.back_right_speed += QUAD_DOWN_DELTA));
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}

void Controller::quad_accelerate(int btnState) {
    // Accelerate-----------------------------
    if (btnState == PRESS) {
        motor.front_left_speed = (motor.front_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed * 1.75);
        motor.front_right_speed = (motor.front_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed * 1.75);
        motor.back_left_speed = (motor.back_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed * 1.75);
        motor.back_right_speed = (motor.back_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed * 1.75);
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "R1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        motor.front_left_speed = (motor.front_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed / 1.75);
        motor.front_right_speed = (motor.front_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed / 1.75);
        motor.back_left_speed = (motor.back_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed / 1.75);
        motor.back_right_speed = (motor.back_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed / 1.75);
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "R1 Button Released" << std::endl;
        return;
    }
}


void Controller::quad_deccelerate(int btnState) {
    // Deccelerate-----------------------------
    if (btnState == PRESS) {
        motor.front_left_speed = (motor.front_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed / 1.75);
        motor.front_right_speed = (motor.front_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed / 1.75);
        motor.back_left_speed = (motor.back_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed / 1.75);
        motor.back_right_speed = (motor.back_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed / 1.75);
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "L1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        motor.front_left_speed = (motor.front_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed * 1.75);
        motor.front_right_speed = (motor.front_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed * 1.75);
        motor.back_left_speed = (motor.back_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed * 1.75);
        motor.back_right_speed = (motor.back_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed * 1.75);
        uint8_t * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(this->dev_num, data, QUAD_PACKET);
        delete[] data;
        // std::cout << "L1 Button Released" << std::endl;
        return;
    }
}



void Controller::quad_motors_on_button_press(SDL_GameControllerButton button) {
    // switch (button) {
    //     case SDL_CONTROLLER_BUTTON_Y: /*return "Triangle";*/
    //     // Pitch Forward
    //         pressed[FORWARD] = true;
    //         if (!active[BACKWARD]) {
    //             active[FORWARD] = true;
    //             quad_pitch_forward(PRESS); 
    //         }
    //         else {
    //             waiting[FORWARD] = [this](int btn) { quad_pitch_forward(btn); };
    //         } 
    //         return;    
    //     case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
    //     // Pitch Backward
    //         pressed[BACKWARD] = true;
    //         if (!active[FORWARD]) {
    //             active[BACKWARD] = true;
    //             quad_pitch_backward(PRESS); 
    //         }
    //         else {
    //             waiting[BACKWARD] = [this](int btn) { quad_pitch_backward(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
    //     // Rotate (Yaw) Right-----------------------------
    //         pressed[RIGHT_YAW] = true;
    //         if (!active[LEFT_YAW]) {
    //             active[RIGHT_YAW] = true;
    //             quad_yaw_right(PRESS); 
    //         }
    //         else {
    //             waiting[RIGHT_YAW] = [this](int btn) { quad_yaw_right(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
    //     // Rotate (Yaw) Left-----------------------------
    //         pressed[LEFT_YAW] = true;
    //         if (!active[RIGHT_YAW]) {
    //             active[LEFT_YAW] = true;
    //             quad_yaw_left(PRESS); 
    //         }
    //         else {
    //             waiting[LEFT_YAW] = [this](int btn) { quad_yaw_left(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
    //     // Move Up--------------------------
    //         pressed[UP] = true;
    //         if (!active[DOWN]) {
    //             active[UP] = true;
    //             quad_up(PRESS); 
    //         }
    //         else {
    //             waiting[UP] = [this](int btn) { quad_up(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
    //     // Move Down--------------------------
    //         pressed[DOWN] = true;
    //         if (!active[UP]) {
    //             active[DOWN] = true;
    //             quad_down(PRESS); 
    //         }
    //         else {
    //             waiting[DOWN] = [this](int btn) { quad_down(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
    //     // Bend (Roll) Left-----------------------------
    //         pressed[LEFT_ROLL] = true;
    //         if (!active[RIGHT_ROLL]) {
    //             active[LEFT_ROLL] = true;
    //             quad_roll_left(PRESS); 
    //         }
    //         else {
    //             waiting[LEFT_ROLL] = [this](int btn) { quad_roll_left(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
    //     // Bend (Roll) Right-----------------------------
    //         pressed[RIGHT_ROLL] = true;
    //         if (!active[LEFT_ROLL]) {
    //             active[RIGHT_ROLL] = true;
    //             quad_roll_right(PRESS); 
    //         }
    //         else {
    //             waiting[RIGHT_ROLL] = [this](int btn) { quad_roll_right(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
    //     // Deccelerate-----------------------------
    //         pressed[DECEL] = true;
    //         if (!active[ACCEL]) {
    //             active[DECEL] = true;
    //             quad_deccelerate(PRESS); 
    //         }
    //         else {
    //             waiting[DECEL] = [this](int btn) { quad_deccelerate(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
    //     // Accelerate-----------------------------
    //         pressed[ACCEL] = true;
    //         if (!active[DECEL]) {
    //             active[ACCEL] = true;
    //             quad_accelerate(PRESS); 
    //         }
    //         else {
    //             waiting[ACCEL] = [this](int btn) { quad_accelerate(btn); };
    //         } 
    //         return;
    //     case SDL_CONTROLLER_BUTTON_BACK: /*return "Share";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_GUIDE: /*return "PS Button";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_START: /*return "Options";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_LEFTSTICK: /*return "Left Stick";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_RIGHTSTICK: /*return "Right Stick";*/
    //         return;
    //     default: return; // "Unknown Button";
    // }
}

void Controller::quad_motors_on_button_release(SDL_GameControllerButton button) {
    // switch (button) {
    //     case SDL_CONTROLLER_BUTTON_Y: /*return "Triangle";*/
    //     // Pitch Forward
    //         pressed[FORWARD] = false;
    //         if (!active[FORWARD]) {
    //             waiting.erase(FORWARD);
    //         }
    //         else {
    //             active[FORWARD] = false;
    //             quad_pitch_forward(RELEASE);
    //             if (pressed[BACKWARD]) { 
    //                 waiting[BACKWARD](PRESS); 
    //                 active[BACKWARD] = true;
    //             } 
    //         }
    //         return;    
    //     case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
    //     // Pitch Backward
    //         pressed[BACKWARD] = false;
    //         if (!active[BACKWARD]) {
    //             waiting.erase(BACKWARD);
    //         }
    //         else {
    //             active[BACKWARD] = false;
    //             quad_pitch_backward(RELEASE);
    //             if (pressed[FORWARD]) { 
    //                 waiting[FORWARD](PRESS); 
    //                 active[FORWARD] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
    //     // Rotate (Yaw) Right-----------------------------
    //         pressed[RIGHT_YAW] = false;
    //         if (!active[RIGHT_YAW]) {
    //             waiting.erase(RIGHT_YAW);
    //         }
    //         else {
    //             active[RIGHT_YAW] = false;
    //             quad_yaw_right(RELEASE);
    //             if (pressed[LEFT_YAW]) { 
    //                 waiting[LEFT_YAW](PRESS); 
    //                 active[LEFT_YAW] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
    //     // Rotate (Yaw) Left-----------------------------
    //         pressed[LEFT_YAW] = false;
    //         if (!active[LEFT_YAW]) {
    //             waiting.erase(LEFT_YAW);
    //         }
    //         else {
    //             active[LEFT_YAW] = false;
    //             quad_yaw_left(RELEASE);
    //             if (RIGHT_YAW) { 
    //                 waiting[RIGHT_YAW](PRESS); 
    //                 active[RIGHT_YAW] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
    //     // Move Up--------------------------
    //         pressed[UP] = false;
    //         if (!active[UP]) {
    //             waiting.erase(UP);
    //         }
    //         else {
    //             active[UP] = false;
    //             quad_up(RELEASE);
    //             if (pressed[DOWN]) { 
    //                 waiting[DOWN](PRESS); 
    //                 active[DOWN] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
    //     // Move Down--------------------------
    //         pressed[DOWN] = false;
    //         if (!active[DOWN]) {
    //             waiting.erase(DOWN);
    //         }
    //         else {
    //             active[DOWN] = false;
    //             quad_down(RELEASE);
    //             if (pressed[UP]) { 
    //                 waiting[UP](PRESS); 
    //                 active[UP] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
    //     // Bend (Roll) Left-----------------------------
    //         pressed[LEFT_ROLL] = false;
    //         if (!active[LEFT_ROLL]) {
    //             waiting.erase(LEFT_ROLL);
    //         }
    //         else {
    //             active[LEFT_ROLL] = false;
    //             quad_roll_left(RELEASE);
    //             if (pressed[RIGHT_ROLL]) { 
    //                 waiting[RIGHT_ROLL](PRESS); 
    //                 active[RIGHT_ROLL] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
    //     // Bend (Roll) Right-----------------------------
    //         pressed[RIGHT_ROLL] = false;
    //         if (!active[RIGHT_ROLL]) {
    //             waiting.erase(RIGHT_ROLL);
    //         }
    //         else {
    //             active[RIGHT_ROLL] = false;
    //             quad_roll_right(RELEASE);
    //             if (pressed[LEFT_ROLL]) { 
    //                 waiting[LEFT_ROLL](PRESS); 
    //                 active[LEFT_ROLL] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
    //     // Deccelerate-----------------------------
    //         pressed[DECEL] = false;
    //         if (!active[DECEL]) {
    //             waiting.erase(DECEL);
    //         }
    //         else {
    //             active[DECEL] = false;
    //             quad_deccelerate(RELEASE);
    //             if (pressed[ACCEL]) { 
    //                 waiting[ACCEL](PRESS); 
    //                 active[ACCEL] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
    //     // Accelerate-----------------------------
    //         pressed[ACCEL] = false;
    //         if (!active[ACCEL]) {
    //             waiting.erase(ACCEL);
    //         }
    //         else {
    //             active[ACCEL] = false;
    //             quad_accelerate(RELEASE);
    //             if (pressed[DECEL]) { 
    //                 waiting[DECEL](PRESS); 
    //                 active[DECEL] = true;
    //             } 
    //         }
    //         return;
    //     case SDL_CONTROLLER_BUTTON_BACK: /*return "Share";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_GUIDE: /*return "PS Button";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_START: /*return "Options";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_LEFTSTICK: /*return "Left Stick";*/
    //         return;
    //     case SDL_CONTROLLER_BUTTON_RIGHTSTICK: /*return "Right Stick";*/
    //         return;
    //     default: return; // "Unknown Button";
    // }
}



void Controller::differential_motors_on_button_press(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Turn Right-----------------------------
            pressed[RIGHT] = true;
            if (!button_active) {
               active[RIGHT] = true;
               button_active = true;
               diff_robot_move(PRESS, TURN_RIGHT); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        RIGHT, 
                        [this](int btn) { 
                            diff_robot_move(PRESS, TURN_RIGHT);
                        }
                    )
                );
            } 
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            pressed[LEFT] = true;
            if (!button_active) {
               active[LEFT] = true;
               button_active = true;
               diff_robot_move(PRESS, TURN_LEFT); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        LEFT, 
                        [this](int btn) { 
                            diff_robot_move(PRESS, TURN_LEFT);
                        }
                    )
                );
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            pressed[FORWARD] = true;
            if (!button_active) {
               active[FORWARD] = true;
               button_active = true;
               diff_robot_move(PRESS, MOVE_FWD); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        FORWARD, 
                        [this](int btn) { 
                            diff_robot_move(PRESS, MOVE_FWD);
                        }
                    )
                );
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Back--------------------------
            pressed[BACKWARD] = true;
            if (!button_active) {
               active[BACKWARD] = true;
               button_active = true;
               diff_robot_move(PRESS, MOVE_BKWD); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        BACKWARD, 
                        [this](int btn) { 
                            diff_robot_move(PRESS, MOVE_BKWD);
                        }
                    )
                );
            } 
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            pressed[BRAKED] = true;
            if (!active[ACCEL]) {
               active[BRAKED] = true;
               button_active = true;
               diff_robot_break(PRESS); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        BRAKED, 
                        [this](int btn) { 
                            diff_robot_break(PRESS);
                        }
                    )
                );
            } 
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            pressed[ACCEL] = true;
           if (!active[BRAKED]) {
                active[ACCEL] = true;
                button_active = true;
                diff_robot_accelerate(PRESS); 
            }
            else {
                waiting.push_back(
                    WaitingCommands(
                        ACCEL, 
                        [this](int btn) { 
                            diff_robot_accelerate(PRESS);
                        }
                    )
                );
            } 
            return;
        default: return; // "Unknown Button";
    }
}



void Controller::differential_motors_on_button_release(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Turn Right-----------------------------
            pressed[RIGHT] = false;
            if (!active[RIGHT]) {
                remove_waiting(RIGHT);
            }
            else {
                active[RIGHT] = false;
                button_active = false;
                diff_robot_move(RELEASE, TURN_RIGHT);
                update_waiting();
            }
            return; 
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            pressed[LEFT] = false;
            if (!active[LEFT]) {
                remove_waiting(LEFT);
            }
            else {
                active[LEFT] = false;
                button_active = false;
                diff_robot_move(RELEASE, TURN_LEFT);
                update_waiting();
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            pressed[FORWARD] = false;
            if (!active[FORWARD]) {
                remove_waiting(FORWARD);
            }
            else {
                active[FORWARD] = false;
                button_active = false;
                diff_robot_move(RELEASE, MOVE_FWD);
                update_waiting();
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Backward--------------------------
            pressed[BACKWARD] = false;
            if (!active[BACKWARD]) {
                remove_waiting(BACKWARD);
            }
            else {
                active[BACKWARD] = false;
                button_active = false;
                diff_robot_move(RELEASE, MOVE_BKWD);
                update_waiting(); 
            }
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            pressed[BRAKED] = false;
            if (!active[BRAKED]) {
                remove_waiting(BRAKED);
            }
            else {
                active[BRAKED] = false;
                button_active = false;
                diff_robot_break(RELEASE);
                update_waiting(); 
            }
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            pressed[ACCEL] = false;
            if (!active[ACCEL]) {
                remove_waiting(ACCELERATE);
            }
            else {
                active[ACCEL] = false;
                button_active = false;
                diff_robot_accelerate(RELEASE);
                update_waiting(); 
            }
            return;
            
        default: return; // "Unknown Button";
    }
}

Controller::Controller() {}


Controller::Controller(int robot_type, std::unique_ptr<Serial> serial, int dev_num) : robot_type(robot_type), 
    serial(std::move(serial)), dev_num(dev_num) {

    for (int i = 0; i < 13; i++) {
        active[i] = false;
        pressed[i] = false;
    }

    button_active = false;

    if (robot_type == DRONE_BOT) {
        motor.front_left_speed = HOVER_SPEED;
        motor.front_right_speed = HOVER_SPEED;
        motor.back_left_speed = HOVER_SPEED;
        motor.back_right_speed = HOVER_SPEED;
        
        // Set to motors hover speed T = F_g
        uint8_t * data = create_quad_data_packet(DIRECTION_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(dev_num, data, DIRECTION_PACKET);

        // Set Motor Directions
        // char * data = create_quad_data_packet(DIRECTION_PACKET, (CLOCKWISE, CLOCKWISE, CLOCKWISE, CLOCKWISE), 0, 0, 0, 0);
        // serial->uartWrite(device_id, data, DIRECTION_PACKET);
    }

    else if (robot_type == DIFF_BOT) {
        motor.left_speed = 0;
        motor.right_speed = 0;
    } 

    // Setup Game Controller
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) { // SDL_INIT_VIDEO | SDL_INIT_GAMEPAD
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
    }

    gamepad = SDL_GameControllerOpen(0);
    if (!gamepad) {
        std::cerr << "Failed to open controller: " << SDL_GetError() << std::endl;
        SDL_Quit();
    }

    std::cout << "Controller connected: " << SDL_GameControllerName(gamepad) << std::endl;
}


Controller::~Controller() {

    if (robot_type == DRONE_BOT) {
        // serial->PWMDeInit(FRONT_LEFT_CHIP, FRONT_LEFT_PWM);
        // serial->PWMDeInit(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM);
        // serial->PWMDeInit(BACK_LEFT_CHIP, BACK_LEFT_PWM);
        // serial->PWMDeInit(BACK_RIGHT_CHIP, BACK_RIGHT_PWM);
    }

    else if (robot_type == DIFF_BOT) {

    }

    SDL_GameControllerClose(gamepad);
    SDL_Quit();
}


void Controller::controllerRun() {

    bool running = true;
    SDL_Event event;
    while (running) {

        while (SDL_PollEvent(&event)) {

            if (event.type == SDL_QUIT) {
                running = false;
            }
            
            else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
                if (robot_type == DRONE_BOT) {
                    this->quad_motors_on_button_press(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }

                else if (robot_type == DIFF_BOT) {
                    this->differential_motors_on_button_press(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }
                
            }
            
            else if (event.type == SDL_CONTROLLERBUTTONUP) {
                if (robot_type == DRONE_BOT) {
                    this->quad_motors_on_button_release(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }

                else if (robot_type == DIFF_BOT) {
                    this->differential_motors_on_button_release(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }
                
            }
        }
    }
}


















