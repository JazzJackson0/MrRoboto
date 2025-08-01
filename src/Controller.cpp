#include "../include/Controller.hpp"

char* Controller::create_data_packet(uint8_t packet_type, uint8_t motor_directions, uint32_t left_speed, uint32_t right_speed) {
    char data[packet_type] = {0};
    data[0] = packet_type;
    
    if (packet_type == DIRECTION_PACKET) {
        data[1] = motor_directions;
    }

    else if (packet_type == SPEED_PACKET) {
        data[1] = left_speed;
        data[5] = right_speed;
    }

    else if (packet_type == FULL_PACKET) {
        data[1] = motor_directions;
        data[2] = left_speed;
        data[6] = right_speed;
    }

    return data;
}


char* Controller::create_quad_data_packet(uint8_t packet_type, uint8_t motor_directions, 
    uint32_t front_left_speed, uint32_t back_left_speed, uint32_t front_right_speed, uint32_t back_right_speed) {
    char data[packet_type] = {0};
    data[0] = packet_type;
    
    if (packet_type == DIRECTION_PACKET) {
        data[1] = motor_directions;
    }

    else if (packet_type == SPEED_PACKET) {
        data[1] = front_left_speed;
        data[5] = back_left_speed;
        data[9] = front_right_speed;
        data[13] = back_right_speed;
    }

    return data;
}


void Controller::diff_right_turn(int btnState) {
    if (btnState == PRESS) {
        char * data = create_data_packet(DIRECTION_PACKET, ((2 << FWD) | BKWD), 0, 0);
        serial->uartWrite(device_fd, data, DIRECTION_PACKET);
        std::cout << "O Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t direction = ((2 << FWD) | FWD);
        if (pressed[BACKWARD]) {
            direction = ((2 << BKWD) | BKWD);
        }
        char * data = create_data_packet(DIRECTION_PACKET, direction, 0, 0);
        serial->uartWrite(device_fd, data, DIRECTION_PACKET);
        std::cout << "O Button Released" << std::endl;
        return;
    }
}

void Controller::diff_left_turn(int btnState) {
    if (btnState == PRESS) {
        char * data = create_data_packet(DIRECTION_PACKET, ((2 << BKWD) | FWD), 0, 0);
        serial->uartWrite(device_fd, data, DIRECTION_PACKET);
        std::cout << "Square Button Pressed" << std::endl;  
        return;
    }
    else if (btnState == RELEASE) {
        uint8_t direction = ((2 << FWD) | FWD);
        if (pressed[BACKWARD]) {
            direction = ((2 << BKWD) | BKWD);
        }
        char * data = create_data_packet(DIRECTION_PACKET, direction, 0, 0);
        serial->uartWrite(device_fd, data, DIRECTION_PACKET);
        std::cout << "Square Button Released" << std::endl;  
        return;
    }
}

void Controller::diff_forward(int btnState) {
    if (btnState == PRESS) {
        char * data = create_data_packet(FULL_PACKET, ((2 << FWD) | FWD), (motor.left_speed += DIFF_MOVE_DELTA), (motor.right_speed += DIFF_MOVE_DELTA));
        serial->uartWrite(device_fd, data, FULL_PACKET);
        std::cout << "D-Pad Up Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_data_packet(SPEED_PACKET, 0, (motor.left_speed -= DIFF_MOVE_DELTA), (motor.right_speed -= DIFF_MOVE_DELTA));
        serial->uartWrite(device_fd, data, SPEED_PACKET);
        std::cout << "D-Pad Up Button Released" << std::endl;
        return;
    }
}

void Controller::diff_backward(int btnState) {
    if (btnState == PRESS) {
        char * data = create_data_packet(FULL_PACKET, ((2 << BKWD) | BKWD), (motor.left_speed += DIFF_MOVE_DELTA), (motor.right_speed += DIFF_MOVE_DELTA));
        serial->uartWrite(device_fd, data, FULL_PACKET);
        std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_data_packet(FULL_PACKET, ((2 << FWD) | FWD), (motor.left_speed -= DIFF_MOVE_DELTA), (motor.right_speed -= DIFF_MOVE_DELTA));
        serial->uartWrite(device_fd, data, FULL_PACKET);
        std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}

void Controller::diff_accelerate(int btnState) {
    if (btnState == PRESS) {
        motor.left_speed = (motor.left_speed * 1.75) >= PERIOD ? PERIOD : (motor.left_speed * 1.75);
        motor.right_speed = (motor.right_speed * 1.75) >= PERIOD ? PERIOD : (motor.right_speed * 1.75);
        char * data = create_data_packet(SPEED_PACKET, 0, motor.left_speed, motor.right_speed);
        serial->uartWrite(device_fd, data, SPEED_PACKET);
        std::cout << "R1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        motor.left_speed = (motor.left_speed / 1.75) >= PERIOD ? PERIOD : (motor.left_speed / 1.75);
        motor.right_speed = (motor.right_speed / 1.75) >= PERIOD ? PERIOD : (motor.right_speed / 1.75);
        char * data = create_data_packet(SPEED_PACKET, 0, motor.left_speed, motor.right_speed);
        serial->uartWrite(device_fd, data, SPEED_PACKET);
        std::cout << "R1 Button Released" << std::endl;
        return;
    }
}


void Controller::diff_deccelerate(int btnState) {
    if (btnState == PRESS) {
        // // (Hard) Stop-----------------------------
        // serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
        // serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);
        // serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
        // serial->pinWrite(MOTOR_SET1_PIN_B, PIN_HIGH);
        // serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed = 0)); 
        // serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed = 0)); 
        std::cout << "L1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        std::cout << "L1 Button Released" << std::endl;
        return;
    }
}


void Controller::quad_yaw_right(int btnState) {
    // Rotate (Yaw) Right-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, 0, 
            (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed, 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "O Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, 0, 
            (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed, 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "O Button Released" << std::endl;
        return;
    }
}

void Controller::quad_yaw_left(int btnState) {
    // Rotate (Yaw) Left-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "Square Button Pressed" << std::endl;  
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "Square Button Released" << std::endl;  
        return;
    }
}


void Controller::quad_roll_right(int btnState) {
    // Bend (Roll) Right-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "O Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "O Button Released" << std::endl;
        return;
    }
}

void Controller::quad_roll_left(int btnState) {
    // Bend (Roll) Left-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed, 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "Square Button Pressed" << std::endl;  
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed, 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "Square Button Released" << std::endl;  
        return;
    }
}

void Controller::quad_pitch_forward(int btnState) {
    // Pitch Forward-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed += QUAD_MOVE_DELTA), 
            motor.front_right_speed, (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Up Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
            motor.front_left_speed, (motor.back_left_speed -= QUAD_MOVE_DELTA), 
            motor.front_right_speed, (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Up Button Released" << std::endl;
        return;
    }
}

void Controller::quad_pitch_backward(int btnState) {
    // Pitch Backward-----------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
        (motor.front_left_speed += QUAD_MOVE_DELTA), motor.back_left_speed,
        (motor.front_right_speed += QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, quad_motor_directions, 
        (motor.front_left_speed -= QUAD_MOVE_DELTA), motor.back_left_speed,
        (motor.front_right_speed -= QUAD_MOVE_DELTA), motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}


void Controller::quad_up(int btnState) {
// Move Up--------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed += QUAD_MOVE_DELTA), (motor.back_left_speed += QUAD_MOVE_DELTA),
            (motor.front_right_speed += QUAD_MOVE_DELTA), (motor.back_right_speed += QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed -= QUAD_MOVE_DELTA), (motor.back_left_speed -= QUAD_MOVE_DELTA),
            (motor.front_right_speed -= QUAD_MOVE_DELTA), (motor.back_right_speed -= QUAD_MOVE_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Released" << std::endl;
        return;
    }
}

void Controller::quad_down(int btnState) {
// Move Down--------------------------
    if (btnState == PRESS) {
        char * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed -= QUAD_DOWN_DELTA), (motor.back_left_speed -= QUAD_DOWN_DELTA),
            (motor.front_right_speed -= QUAD_DOWN_DELTA), (motor.back_right_speed -= QUAD_DOWN_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        char * data = create_quad_data_packet(QUAD_PACKET, 
            0, (motor.front_left_speed += QUAD_DOWN_DELTA), (motor.back_left_speed += QUAD_DOWN_DELTA),
            (motor.front_right_speed += QUAD_DOWN_DELTA), (motor.back_right_speed += QUAD_DOWN_DELTA));
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "D-Pad Down Button Released" << std::endl;
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
        char * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "R1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        motor.front_left_speed = (motor.front_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed / 1.75);
        motor.front_right_speed = (motor.front_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed / 1.75);
        motor.back_left_speed = (motor.back_left_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed / 1.75);
        motor.back_right_speed = (motor.back_right_speed / 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed / 1.75);
        char * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "R1 Button Released" << std::endl;
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
        char * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "L1 Button Pressed" << std::endl;
        return;
    }
    else if (btnState == RELEASE) {
        motor.front_left_speed = (motor.front_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed * 1.75);
        motor.front_right_speed = (motor.front_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed * 1.75);
        motor.back_left_speed = (motor.back_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed * 1.75);
        motor.back_right_speed = (motor.back_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed * 1.75);
        char * data = create_quad_data_packet(QUAD_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(device_fd, data, QUAD_PACKET);
        std::cout << "L1 Button Released" << std::endl;
        return;
    }
}



void Controller::quad_motors_on_button_press(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_Y: /*return "Triangle";*/
        // Pitch Forward
            if (!pressed[BACKWARD]) {
                pressed[FORWARD] = true;
                quad_pitch_forward(PRESS); 
            }
            else {
                waiting[FORWARD] = [this](int btn) { quad_pitch_forward(btn); };
            } 
            return;    
        case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
        // Pitch Backward
            if (!pressed[FORWARD]) {
                pressed[BACKWARD] = true;
                quad_pitch_backward(PRESS); 
            }
            else {
                waiting[BACKWARD] = [this](int btn) { quad_pitch_backward(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
        // Rotate (Yaw) Right-----------------------------
            if (!pressed[LEFT_YAW]) {
                pressed[RIGHT_YAW] = true;
                quad_yaw_right(PRESS); 
            }
            else {
                waiting[RIGHT_YAW] = [this](int btn) { quad_yaw_right(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
        // Rotate (Yaw) Left-----------------------------
            if (!pressed[RIGHT_YAW]) {
                pressed[LEFT_YAW] = true;
                quad_yaw_left(PRESS); 
            }
            else {
                waiting[LEFT_YAW] = [this](int btn) { quad_yaw_left(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
        // Move Up--------------------------
            if (!pressed[DOWN]) {
                pressed[UP] = true;
                quad_up(PRESS); 
            }
            else {
                waiting[UP] = [this](int btn) { quad_up(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
        // Move Down--------------------------
            if (!pressed[UP]) {
                pressed[DOWN] = true;
                quad_down(PRESS); 
            }
            else {
                waiting[DOWN] = [this](int btn) { quad_down(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
        // Bend (Roll) Left-----------------------------
            if (!pressed[RIGHT_ROLL]) {
                pressed[LEFT_ROLL] = true;
                quad_roll_left(PRESS); 
            }
            else {
                waiting[LEFT_ROLL] = [this](int btn) { quad_roll_left(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
        // Bend (Roll) Right-----------------------------
            if (!pressed[LEFT_ROLL]) {
                pressed[RIGHT_ROLL] = true;
                quad_roll_right(PRESS); 
            }
            else {
                waiting[RIGHT_ROLL] = [this](int btn) { quad_roll_right(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
        // Deccelerate-----------------------------
            if (!pressed[ACCEL]) {
                pressed[DECCEL] = true;
                quad_deccelerate(PRESS); 
            }
            else {
                waiting[DECCEL] = [this](int btn) { quad_deccelerate(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
        // Accelerate-----------------------------
            if (!pressed[DECCEL]) {
                pressed[ACCEL] = true;
                quad_accelerate(PRESS); 
            }
            else {
                waiting[ACCEL] = [this](int btn) { quad_accelerate(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_BACK: /*return "Share";*/
            return;
        case SDL_CONTROLLER_BUTTON_GUIDE: /*return "PS Button";*/
            return;
        case SDL_CONTROLLER_BUTTON_START: /*return "Options";*/
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSTICK: /*return "Left Stick";*/
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSTICK: /*return "Right Stick";*/
            return;
        default: return; // "Unknown Button";
    }
}

void Controller::quad_motors_on_button_release(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_Y: /*return "Triangle";*/
        // Pitch Forward
            if (!pressed[FORWARD]) {
                waiting.erase(FORWARD);
            }
            else {
                pressed[FORWARD] = false;
                try { waiting[FORWARD](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;    
        case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
        // Pitch Backward
            if (!pressed[BACKWARD]) {
                waiting.erase(BACKWARD);
            }
            else {
                pressed[BACKWARD] = false;
                try { waiting[BACKWARD](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
        // Rotate (Yaw) Right-----------------------------
            if (!pressed[RIGHT_YAW]) {
                waiting.erase(RIGHT_YAW);
            }
            else {
                pressed[RIGHT_YAW] = false;
                try { waiting[RIGHT_YAW](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
        // Rotate (Yaw) Left-----------------------------
            if (!pressed[LEFT_YAW]) {
                waiting.erase(LEFT_YAW);
            }
            else {
                pressed[LEFT_YAW] = false;
                try { waiting[LEFT_YAW](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
        // Move Up--------------------------
            if (!pressed[UP]) {
                waiting.erase(UP);
            }
            else {
                pressed[UP] = false;
                try { waiting[UP](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
        // Move Down--------------------------
            if (!pressed[DOWN]) {
                waiting.erase(DOWN);
            }
            else {
                pressed[DOWN] = false;
                try { waiting[DOWN](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
        // Bend (Roll) Left-----------------------------
            if (!pressed[LEFT_ROLL]) {
                waiting.erase(LEFT_ROLL);
            }
            else {
                pressed[LEFT_ROLL] = false;
                try { waiting[LEFT_ROLL](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
        // Bend (Roll) Right-----------------------------
            if (!pressed[RIGHT_ROLL]) {
                waiting.erase(RIGHT_ROLL);
            }
            else {
                pressed[RIGHT_ROLL] = false;
                try { waiting[RIGHT_ROLL](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
        // Deccelerate-----------------------------
            if (!pressed[DECCEL]) {
                waiting.erase(DECCEL);
            }
            else {
                pressed[DECCEL] = false;
                try { waiting[DECCEL](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
        // Accelerate-----------------------------
            if (!pressed[ACCEL]) {
                waiting.erase(ACCEL);
            }
            else {
                pressed[ACCEL] = false;
                try { waiting[ACCEL](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_BACK: /*return "Share";*/
            return;
        case SDL_CONTROLLER_BUTTON_GUIDE: /*return "PS Button";*/
            return;
        case SDL_CONTROLLER_BUTTON_START: /*return "Options";*/
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSTICK: /*return "Left Stick";*/
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSTICK: /*return "Right Stick";*/
            return;
        default: return; // "Unknown Button";
    }
}



void Controller::differential_motors_on_button_press(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Turn Right-----------------------------
            if (!pressed[LEFT]) {
               pressed[RIGHT] = true;
               diff_right_turn(PRESS); 
            }
            else {
                waiting[RIGHT] = [this](int btn) { diff_right_turn(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            if (!pressed[RIGHT]) {
               pressed[LEFT] = true;
               diff_left_turn(PRESS); 
            }
            else {
                waiting[LEFT] = [this](int btn) { diff_left_turn(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            if (!pressed[BACKWARD]) {
               pressed[FORWARD] = true;
               diff_forward(PRESS); 
            }
            else {
                waiting[FORWARD] = [this](int btn) { diff_forward(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Back--------------------------
            if (!pressed[FORWARD]) {
               pressed[BACKWARD] = true;
               diff_backward(PRESS); 
            }
            else {
                waiting[BACKWARD] = [this](int btn) { diff_backward(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            if (!pressed[ACCEL]) {
               pressed[DECCEL] = true;
               diff_deccelerate(PRESS); 
            }
            else {
                waiting[DECCEL] = [this](int btn) { diff_deccelerate(btn); };
            } 
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
           if (!pressed[DECCEL]) {
               pressed[ACCEL] = true;
               diff_accelerate(PRESS); 
            }
            else {
                waiting[ACCEL] = [this](int btn) { diff_accelerate(btn); };
            } 
            return;
        default: return; // "Unknown Button";
    }
}

void Controller::differential_motors_on_button_release(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Turn Right-----------------------------
            if (!pressed[RIGHT]) {
                waiting.erase(RIGHT);
            }
            else {
                pressed[RIGHT] = false;
                try { waiting[RIGHT](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return; 
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            if (!pressed[RIGHT]) {
                waiting.erase(RIGHT);
            }
            else {
                pressed[RIGHT] = false;
                try { waiting[RIGHT](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            if (!pressed[RIGHT]) {
                waiting.erase(RIGHT);
            }
            else {
                pressed[RIGHT] = false;
                try { waiting[RIGHT](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Backward--------------------------
            if (!pressed[RIGHT]) {
                waiting.erase(RIGHT);
            }
            else {
                pressed[RIGHT] = false;
                try { waiting[RIGHT](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            if (!pressed[RIGHT]) {
                waiting.erase(RIGHT);
            }
            else {
                pressed[RIGHT] = false;
                try { waiting[RIGHT](RELEASE);
                } catch (const std::out_of_range& e) { return; }
            }
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            
        default: return; // "Unknown Button";
    }
}

Controller::Controller() {}


Controller::Controller(int robot_type, std::unique_ptr<Serial> serial, int dev_num) : robot_type(robot_type), 
    serial(std::move(serial)) {

    this->device_fd = serial->uartInit(dev_num);

    for (int i = 0; i < 12; i++) {
        pressed[i] = false;
    }

    if (robot_type == DRONE_BOT) {
        motor.front_left_speed = HOVER_SPEED;
        motor.front_right_speed = HOVER_SPEED;
        motor.back_left_speed = HOVER_SPEED;
        motor.back_right_speed = HOVER_SPEED;
        
        // Set to motors hover speed T = F_g
        char * data = create_quad_data_packet(DIRECTION_PACKET, 0, motor.front_left_speed, motor.back_left_speed, 
            motor.front_right_speed, motor.back_right_speed);
        serial->uartWrite(device_fd, data, DIRECTION_PACKET);

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


















