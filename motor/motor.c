/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"   
#include "hardware/gpio.h"
#include <stdio.h>
#include "magnometer/mag.h"
#include <math.h>

// Define GPIO pins for motor control and wheel encoders
#define MOTOR_LEFT_FORWARD  2 
#define MOTOR_LEFT_BACKWARD 3
#define MOTOR_RIGHT_FORWARD 5
#define MOTOR_RIGHT_BACKWARD 4
#define RIGHT_WHEEL_ENCODER 17
#define LEFT_WHEEL_ENCODER 16
#define LEFT_IR_SENSOR 13
#define RIGHT_IR_SENSOR 12
#define HOLES_ON_DISC 20    // Hole on encoder disc
#define WHEEL_CIRCUMFERENCE_CM 20.42 // Replace with your wheel's circumference in cm

// For speed control
void set_motor_speed(float left_duty_cycle,float right_duty_cycle)
{
    uint slice_num_A = pwm_gpio_to_slice_num(0);
    uint slice_num_B = pwm_gpio_to_slice_num(1);
    
    pwm_set_chan_level(slice_num_A, PWM_CHAN_A, left_duty_cycle * 32150); 
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, right_duty_cycle * 32150);


}

// Functions to control the movement of the robot
void move_forward()
{
    printf("Forward!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 1);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_FORWARD, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD, 0);
    
}

void stop_movement()
{
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
}

void move_backward()
{
    printf("Backwards!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);
}

void right_turn(){

    printf("Right turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 1);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);

}
void left_turn(){
    
    printf("left turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );

}
void right_turn_with_angle(float degree) {
    float startDeg = measurement();
    float target = 0;
    target = startDeg + degree; 
    printf("start Before while %f\n", startDeg);

    while (target > measurement()) { 
        startDeg = measurement();  
        target = (target < 0) ? -target : target;
        printf("start %f\n", startDeg);
        printf("target %f\n", target);
        printf("measurement %f\n", measurement());
        right_turn(); 
    }  
    stop_movement();
    startDeg = 0;
}

void left_turn_with_angle(float degree) {
    float startDeg = measurement();
    float target = 0;
    target = startDeg - degree;
    printf("start Before while %f\n", startDeg);

    while (target < measurement()) {
        startDeg = measurement();  // Update startDeg within the loop
        target = (target < 0) ? -target : target;
        printf("start %f\n", startDeg);
        printf("target %f\n", target);
        printf("measurement %f\n", measurement());
        left_turn();
    }  
    stop_movement();
    startDeg = 0;
}

void pid_controller(float left_speed, float right_speed, float left_integral, float right_integral, float left_last_error, float right_last_error)
{
    // PID constants for left wheel
    float Kp_left = 1.6;  // Proportional gain
    float Ki_left = 0.40; // Integral gain
    float Kd_left = 0.41; // Derivative gain

    // PID constants for right wheel
    float Kp_right = 1.6;  // Proportional gain
    float Ki_right = 0.40; // Integral gain
    float Kd_right = 0.41; // Derivative gain

    float desired_speed = 60.0; // Set the desired speed in cm/s (adjust as needed)

    // Calculate errors for each wheel
    float left_error = desired_speed - left_speed;
    float right_error = desired_speed - right_speed;

    // Update integral for each wheel
    left_integral += left_error;
    right_integral += right_error;

    // Calculate derivative for each wheel
    float left_derivative = left_error - left_last_error;
    float right_derivative = right_error - right_last_error;

    // PID output for each wheel
    float left_output = Kp_left * left_error + Ki_left * left_integral + Kd_left * left_derivative;
    float right_output = Kp_right * right_error + Ki_right * right_integral + Kd_right * right_derivative;

    // Set motor speed for each wheel
    set_motor_speed(left_output/1000, right_output/1000);

    // Update last error for next iteration
    left_last_error = left_error;
    right_last_error = right_error;
}
