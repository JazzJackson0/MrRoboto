#pragma once
#include <iostream>
#include <cstdint>
#include <memory>
#include <functional>
#include <map>
#include <SDL2/SDL.h> // in /usr/include
//#include <SDL3/SDL.h> // in /usr/local/include
#include "Serial.hpp"


// Quadcopter Parameters-------------------------------------------------------------------
#define HOVER_SPEED 2000 // T = F_g (Thrust = Gravitational Force)
#define QUAD_MOVE_DELTA 3000 // T > F_g
#define QUAD_DOWN_DELTA 1000 // T < F_g

// Front Left Motor PWM1A (P9_14 - [PWM1_A] Chip 5 - PWM0)
#define FRONT_LEFT_CHIP 5
#define FRONT_LEFT_PWM 0
// Front Right Motor PWM2B (P8_13 - [PWM2_B] Chip 7 - PWM1)
#define FRONT_RIGHT_CHIP 7
#define FRONT_RIGHT_PWM 1
// Back Left Motor PWM1B (P9_16 - [PWM1_B] Chip 5 - PWM1)
#define BACK_LEFT_CHIP 5
#define BACK_LEFT_PWM 1
// Back Right Motor PWM2A (P8_19 - [PWM2_A] Chip 7 - PWM0)
#define BACK_RIGHT_CHIP 7
#define BACK_RIGHT_PWM 0


// Differential Drive Parameters-------------------------------------------------------------------
#define DIFF_MOVE_DELTA 50.0f // Percentage (0% - 100%)

// Left Motors PWM1A (P9_14 - [PWM1_A] Chip 5 - PWM0)
#define LEFT_CHIP 5
#define LEFT_PWM 0
// Right Motors PWM1B (P9_16 - [PWM1_B] Chip 5 - PWM1)
#define RIGHT_CHIP 5
#define RIGHT_PWM 1

#define UART_DATA_LEN 9


// General Parameters-------------------------------------------------------------------
// PWM Timer Period
#define PERIOD 10000

// Motor Direction Pins (Q: Quadcopter | D: Diff Drive)
#define MOTOR_SET1_PIN_A 67 // Q: Front Left & Back Right Motor Direction (P8_8 - GPIO_67) [Clockwise motors] | D: Left Motors
#define MOTOR_SET1_PIN_B 68 // Q: Front Left & Back Right Motor Direction (P8_10 - GPIO_68) [Clockwise motors] | D: Left Motors
#define MOTOR_SET2_PIN_A 46 // Q: Front Right & Back Left Motor Direction (P8_16 - GPIO_46) [Counter-Clockwise motors] | D: Right Motors
#define MOTOR_SET2_PIN_B 65 // Q: Front Right & Back Left Motor Direction (P8_18 - GPIO_65) [Counter-Clockwise motors] | D: Right Motors

// Robot Type
#define DRONE_BOT 0
#define DIFF_BOT 1


// Packet Types
// Full Packet Format: [Type (1 Byte), Direction (1 Byte), Speed (4 Bytes), Speed (4 Bytes)]
// Direction: [LEFT | RIGHT]
#define DIRECTION_PACKET 2
#define SPEED_PACKET 9
#define FULL_PACKET 10
#define QUAD_PACKET 17

#define PRESS 0
#define RELEASE 1

struct MotorSpeeds {

    // Quad
    float front_right_speed;
    float front_left_speed;
    float back_right_speed;
    float back_left_speed;

    // Drone
    float right_speed;
    float left_speed;
};

struct WaitingCommands {
    int command;
    std::function<void(int)> function;
    WaitingCommands(int command, std::function<void(int)> function) :
        command(command), function(function) {

    }
};

class Controller {

    private:
        
        enum MotorCommands { STP, BKWD, FWD, BRK, CLOCKWISE, COUNTER_CLOCKWISE }; // STP = 0 0, BKWD 0 1, FWD = 1 0, BRK 1 1
        enum RobotCommands { MOVE_FWD = ((2 << FWD) | FWD), MOVE_BKWD = ((2 << BKWD) | BKWD), TURN_RIGHT = ((2 << FWD) | BKWD), 
                              TURN_LEFT = ((2 << BKWD) | FWD), STOP = ((2 << STP) | STP), BRAKE = ((2 << BRK) | BRK), ACCEL };
        enum RobotMovements { FORWARD, BACKWARD, RIGHT, LEFT, UP, DOWN, LEFT_ROLL, RIGHT_ROLL, LEFT_YAW, RIGHT_YAW, BRAKED, ACCELERATE, DECELERATE };
        uint8_t quad_motor_directions = 0;
        std::unique_ptr<Serial> serial;
        int dev_num;
        MotorSpeeds motor;
        int robot_type;

        std::map<int, bool> active;
        std::map<int, bool> pressed;
        std::vector<WaitingCommands> waiting;

        bool button_active;

        
        // Controller
        SDL_GameController *gamepad;


        /**
         * @brief 
         * @param packet_type
         * @param motor_directions
         * @param left_speed
         * @param right_speed
         * @return 
         */
        uint8_t* create_data_packet(uint8_t packet_type, uint8_t motor_directions, float left_speed, float right_speed);
        
        /**
         * @brief 
         * @param packet_type
         * @param motor_directions
         * @param front_left_speed
         * @param back_left_speed
         * @param front_right_speed
         * @param back_right_speed
         * @return 
         */
        uint8_t* create_quad_data_packet(uint8_t packet_type, uint8_t motor_directions, 
            float front_left_speed, float back_left_speed, float front_right_speed, float back_right_speed);

        /**
         * @brief
         * @param movement
         */
        void remove_waiting(RobotMovements movement);

        /**
         * @brief
         */
        void update_waiting();
        
        /**
         * @brief
         * @param btnState
         * @param command 
         */
        void log_button_press(int btnState, RobotCommands command);

        /**
         * @brief 
         * @param btnState 
         * @param command
         */
        void diff_robot_move(int btnState, RobotCommands command);


        /**
         * @brief 
         * @param btnState 
         */
        void diff_robot_accelerate(int btnState);


        /**
         * @brief 
         * @param btnState 
         */
        void diff_robot_break(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_yaw_right(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_yaw_left(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_roll_right(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_roll_left(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_pitch_forward(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_pitch_backward(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_up(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_down(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_accelerate(int btnState);

        /**
         * @brief 
         * @param btnState 
         */
        void quad_deccelerate(int btnState);


        /**
         * @brief 
         * 
         * @param button 
         */
        void quad_motors_on_button_press(SDL_GameControllerButton button);

        /**
         * @brief 
         * 
         * @param button 
         */
        void quad_motors_on_button_release(SDL_GameControllerButton button);


        /**
         * @brief 
         * 
         * @param button 
         */
        void differential_motors_on_button_press(SDL_GameControllerButton button);

        /**
         * @brief 
         * 
         * @param button 
         */
        void differential_motors_on_button_release(SDL_GameControllerButton button);

    public:

        /**
         * @brief Construct a new Controller object
         */
        Controller();

        /**
         * @brief Construct a new Controller object
         * 
         * @param robot_type 
         * @param serial 
         * @param dev_num
         */
        Controller(int robot_type, std::unique_ptr<Serial> serial, int dev_num);

        /**
         * @brief Destroy the Controller object
         * 
         */
        ~Controller();

        /**
         * @brief 
         * 
         */
        void controllerRun();

};


