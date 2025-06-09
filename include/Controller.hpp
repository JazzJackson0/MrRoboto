#pragma once
#include <iostream>
#include <SDL2/SDL.h> // in /usr/include
//#include <SDL3/SDL.h> // in /usr/local/include
#include "../include/Serial.hpp"


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
#define DIFF_MOVE_DELTA 3000

// Left Motors PWM1A (P9_14 - [PWM1_A] Chip 5 - PWM0)
#define LEFT_CHIP 5
#define LEFT_PWM 0
// Right Motors PWM1B (P9_16 - [PWM1_B] Chip 5 - PWM1)
#define RIGHT_CHIP 5
#define RIGHT_PWM 1


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


struct MotorSpeeds {

    // Quad
    int front_right_speed;
    int front_left_speed;
    int back_right_speed;
    int back_left_speed;

    // Drone
    int right_speed;
    int left_speed;
};

class Controller {

    private:
        Serial *serial;
        MotorSpeeds motor;
        int robotType;

        // Controller
        SDL_GameController *gamepad;

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
         * 
         */
        Controller(int robot_type);

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


