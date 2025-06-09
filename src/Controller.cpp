#include "../include/Controller.hpp"

void Controller::quad_motors_on_button_press(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_Y: /*return "Triangle";*/
            // Pitch Forward-----------------------------
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_left_speed += QUAD_MOVE_DELTA)); // Back Right  
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed += QUAD_MOVE_DELTA)); // Back Left
            std::cout << "Triangle Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
            // Pitch Backward-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed += QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed += QUAD_MOVE_DELTA)); // Front Right
            std::cout << "X Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Rotate (Yaw) Right-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed += QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed += QUAD_MOVE_DELTA)); // Back Right
            std::cout << "O Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Rotate (Yaw) Left-----------------------------
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed += QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed += QUAD_MOVE_DELTA)); // Back Left
            std::cout << "Square Button Pressed" << std::endl;  
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Up--------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed += QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed += QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed += QUAD_MOVE_DELTA)); // Back Left  
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed += QUAD_MOVE_DELTA)); // Back Right
            std::cout << "D-Pad Up Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Down--------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed -= QUAD_DOWN_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed -= QUAD_DOWN_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed -= QUAD_DOWN_DELTA)); // Back Left  
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed -= QUAD_DOWN_DELTA)); // Back Right
            std::cout << "D-Pad Down Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
            // Bend (Roll) Left-----------------------------
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed += QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed += QUAD_MOVE_DELTA)); // Back Right
            std::cout << "D-Pad Left Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
            // Bend (Roll) Right-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed += QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed += QUAD_MOVE_DELTA)); // Back Left  
            std::cout << "D-Pad Right Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // Deccelerate-----------------------------
            // serial->pwmUpdate(0, 0, move_speed); // Front Left
            // serial->pwmUpdate(1, 0, move_speed); // Front Right
            // serial->pwmUpdate(0, 1, move_speed); // Back Left  
            // serial->pwmUpdate(1, 1, move_speed); // Back Right
            // std::cout << "L1 Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            // Front Left
            if (motor.front_left_speed > HOVER_SPEED) {
                motor.front_left_speed = (motor.front_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_left_speed * 1.75);
                serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, motor.front_left_speed); 
            }
            // Front Right
            if (motor.front_right_speed > HOVER_SPEED) {
                motor.front_right_speed = (motor.front_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.front_right_speed * 1.75);
                serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, motor.front_right_speed); 
            }
            // Back Left
            if (motor.back_left_speed > HOVER_SPEED) {
                motor.back_left_speed = (motor.back_left_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_left_speed * 1.75);
                serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, motor.back_left_speed); 
            }
            // Back Right
            if (motor.back_right_speed > HOVER_SPEED) {
                motor.back_right_speed = (motor.back_right_speed * 1.75) >= PERIOD ? PERIOD : (motor.back_right_speed * 1.75);
                serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, motor.back_right_speed); 
            }
            std::cout << "R1 Button Pressed" << std::endl;
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
            // Pitch Forward-----------------------------
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_left_speed -= QUAD_MOVE_DELTA)); // Back Right  
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed -= QUAD_MOVE_DELTA)); // Back Left
            std::cout << "Triangle Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_A: /*return "X (Cross)";*/
            // Pitch Backward-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed -= QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed -= QUAD_MOVE_DELTA)); // Front Right
            std::cout << "X Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Rotate (Yaw) Right-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed -= QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed -= QUAD_MOVE_DELTA)); // Back Right
            std::cout << "O Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Rotate (Yaw) Left-----------------------------
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed -= QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed -= QUAD_MOVE_DELTA)); // Back Left
            std::cout << "Square Button Released" << std::endl;  
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Up--------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed -= QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed -= QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed -= QUAD_MOVE_DELTA)); // Back Left  
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed -= QUAD_MOVE_DELTA)); // Back Right
            std::cout << "D-Pad Up Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Down--------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed += QUAD_DOWN_DELTA)); // Front Left
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed += QUAD_DOWN_DELTA)); // Front Right
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed += QUAD_DOWN_DELTA)); // Back Left  
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed += QUAD_DOWN_DELTA)); // Back Right
            std::cout << "D-Pad Down Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT: /*return "D-Pad Left";*/
            // Bend (Roll) Left-----------------------------
            serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed -= QUAD_MOVE_DELTA)); // Front Right
            serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed -= QUAD_MOVE_DELTA)); // Back Right
            std::cout << "D-Pad Left Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: /*return "D-Pad Right";*/
            // Bend (Roll) Right-----------------------------
            serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed -= QUAD_MOVE_DELTA)); // Front Left
            serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed -= QUAD_MOVE_DELTA)); // Back Left  
            std::cout << "D-Pad Right Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // Deccelerate-----------------------------
            // serial->pwmUpdate(0, 0, hover_speed); // Front Left
            // serial->pwmUpdate(1, 0, hover_speed); // Front Right
            // serial->pwmUpdate(0, 1, hover_speed); // Back Left  
            // serial->pwmUpdate(1, 1, hover_speed); // Back Right
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            // Front Left
            if (motor.front_left_speed > HOVER_SPEED) {
                serial->pwmUpdate(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, (motor.front_left_speed /= 1.75)); 
                // TODO: Handle case when motor speed == PERIOD
            }
            // Front Right
            if (motor.front_right_speed > HOVER_SPEED) {
                serial->pwmUpdate(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, (motor.front_right_speed /= 1.75)); 
                // TODO: Handle case when motor speed == PERIOD
            }
            // Back Left
            if (motor.back_left_speed > HOVER_SPEED) {
                serial->pwmUpdate(BACK_LEFT_CHIP, BACK_LEFT_PWM, (motor.back_left_speed /= 1.75)); 
                // TODO: Handle case when motor speed == PERIOD
            }
            // Back Right
            if (motor.back_right_speed > HOVER_SPEED) {
                serial->pwmUpdate(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, (motor.back_right_speed /= 1.75)); 
                // TODO: Handle case when motor speed == PERIOD
            }
            std::cout << "R1 Button Released" << std::endl;
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
            serial->pinWrite(MOTOR_SET2_PIN_A, PIN_LOW);
            serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);
            std::cout << "O Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            serial->pinWrite(MOTOR_SET1_PIN_A, PIN_LOW);
            serial->pinWrite(MOTOR_SET1_PIN_B, PIN_HIGH);
            std::cout << "Square Button Pressed" << std::endl;  
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed += DIFF_MOVE_DELTA)); 
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed += DIFF_MOVE_DELTA)); 
            std::cout << "D-Pad Up Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Back--------------------------
            serial->pinWrite(MOTOR_SET2_PIN_A, PIN_LOW);
            serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);
            serial->pinWrite(MOTOR_SET1_PIN_A, PIN_LOW);
            serial->pinWrite(MOTOR_SET1_PIN_B, PIN_HIGH);
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed += DIFF_MOVE_DELTA)); 
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed += DIFF_MOVE_DELTA)); 
            std::cout << "D-Pad Down Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            // serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET1_PIN_B, PIN_HIGH);
            // serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed = 0)); 
            // serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed = 0)); 
            // std::cout << "L1 Button Pressed" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            // Left
            motor.left_speed = (motor.left_speed * 1.75) >= PERIOD ? PERIOD : (motor.left_speed * 1.75);
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, motor.left_speed); 
            // Right
            motor.right_speed = (motor.right_speed * 1.75) >= PERIOD ? PERIOD : (motor.right_speed * 1.75);
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, motor.right_speed); 
            std::cout << "R1 Button Pressed" << std::endl;
            return;
        default: return; // "Unknown Button";
    }
}

void Controller::differential_motors_on_button_release(SDL_GameControllerButton button) {
    switch (button) {
        case SDL_CONTROLLER_BUTTON_B: /*return "O (Circle)";*/
            // Turn Right-----------------------------
            serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
            serial->pinWrite(MOTOR_SET2_PIN_B, PIN_LOW);
            std::cout << "O Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_X: /*return "Square";*/
            // Turn Left-----------------------------
            serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
            serial->pinWrite(MOTOR_SET1_PIN_B, PIN_LOW);
            std::cout << "Square Button Released" << std::endl;  
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_UP: /*return "D-Pad Up";*/
            // Move Forward--------------------------
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed -= DIFF_MOVE_DELTA)); 
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed -= DIFF_MOVE_DELTA)); 
            std::cout << "D-Pad Up Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN: /*return "D-Pad Down";*/
            // Move Backward--------------------------
            serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
            serial->pinWrite(MOTOR_SET2_PIN_B, PIN_LOW);
            serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
            serial->pinWrite(MOTOR_SET1_PIN_B, PIN_LOW);
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed -= DIFF_MOVE_DELTA)); 
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed -= DIFF_MOVE_DELTA)); 
            std::cout << "D-Pad Down Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: /*return "L1";*/
            // // (Hard) Stop-----------------------------
            // // TODO: Different if moving Back or Forward
            // serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
            // serial->pinWrite(MOTOR_SET1_PIN_B, PIN_HIGH);
            // serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed = 0)); 
            // serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed = 0)); 
            // std::cout << "L1 Button Released" << std::endl;
            return;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: /*return "R1";*/
            // Accelerate-----------------------------
            // Left
            serial->pwmUpdate(LEFT_CHIP, LEFT_PWM, (motor.left_speed /= 1.75)); 
            // TODO: Handle case when motor speed == PERIOD
            // Right
            serial->pwmUpdate(RIGHT_CHIP, RIGHT_PWM, (motor.right_speed /= 1.75)); 
            // TODO: Handle case when motor speed == PERIOD
            std::cout << "R1 Button Released" << std::endl;
            return;
        default: return; // "Unknown Button";
    }
}


Controller::Controller(int robot_type) {

    robotType = robot_type;
    serial = new Serial();
    // Initialize Motor Direction Pins
    serial->pinInit(MOTOR_SET1_PIN_A, PIN_OUT);
    serial->pinInit(MOTOR_SET1_PIN_B, PIN_OUT);
    serial->pinInit(MOTOR_SET2_PIN_A, PIN_OUT);
    serial->pinInit(MOTOR_SET2_PIN_B, PIN_OUT);

    if (robotType == DRONE_BOT) {
        motor.front_left_speed = HOVER_SPEED;
        motor.front_right_speed = HOVER_SPEED;
        motor.back_left_speed = HOVER_SPEED;
        motor.back_right_speed = HOVER_SPEED;
        
        // Set to motors hover speed T = F_g
        serial->pwmInit(FRONT_LEFT_CHIP, FRONT_LEFT_PWM, PERIOD, motor.front_left_speed); 
        serial->pwmInit(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM, PERIOD, motor.front_right_speed);
        serial->pwmInit(BACK_LEFT_CHIP, BACK_LEFT_PWM, PERIOD, motor.back_left_speed);
        serial->pwmInit(BACK_RIGHT_CHIP, BACK_RIGHT_PWM, PERIOD, motor.back_right_speed);

        // Set Motor Directions
        serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
        serial->pinWrite(MOTOR_SET1_PIN_B, PIN_LOW);
        serial->pinWrite(MOTOR_SET2_PIN_A, PIN_LOW);
        serial->pinWrite(MOTOR_SET2_PIN_B, PIN_HIGH);

    }

    else if (robotType == DIFF_BOT) {
        motor.left_speed = 0;
        motor.right_speed = 0;
        
        serial->pwmInit(LEFT_CHIP, LEFT_PWM, PERIOD, motor.left_speed); 
        serial->pwmInit(RIGHT_CHIP, RIGHT_PWM, PERIOD, motor.right_speed);

        // Set Motor Directions
        serial->pinWrite(MOTOR_SET1_PIN_A, PIN_HIGH);
        serial->pinWrite(MOTOR_SET1_PIN_B, PIN_LOW);
        serial->pinWrite(MOTOR_SET2_PIN_A, PIN_HIGH);
        serial->pinWrite(MOTOR_SET2_PIN_B, PIN_LOW);
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

    if (robotType == DRONE_BOT) {
        // serial->PWMDeInit(FRONT_LEFT_CHIP, FRONT_LEFT_PWM);
        // serial->PWMDeInit(FRONT_RIGHT_CHIP, FRONT_RIGHT_PWM);
        // serial->PWMDeInit(BACK_LEFT_CHIP, BACK_LEFT_PWM);
        // serial->PWMDeInit(BACK_RIGHT_CHIP, BACK_RIGHT_PWM);
    }

    else if (robotType == DIFF_BOT) {

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
                if (robotType == DRONE_BOT) {
                    this->quad_motors_on_button_press(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }

                else if (robotType == DIFF_BOT) {
                    this->differential_motors_on_button_press(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }
                
            }
            
            else if (event.type == SDL_CONTROLLERBUTTONUP) {
                if (robotType == DRONE_BOT) {
                    this->quad_motors_on_button_release(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }

                else if (robotType == DIFF_BOT) {
                    this->differential_motors_on_button_release(static_cast<SDL_GameControllerButton>(event.cbutton.button));
                }
                
            }
        }
    }
}


















