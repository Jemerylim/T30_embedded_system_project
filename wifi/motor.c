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

// Define GPIO pins for motor control and wheel encoders
#define MOTOR_LEFT_FORWARD  2 
#define MOTOR_LEFT_BACKWARD 3
#define MOTOR_RIGHT_FORWARD 5
#define MOTOR_RIGHT_BACKWARD 4
#define RIGHT_WHEEL_ENCODER 17
#define LEFT_WHEEL_ENCODER 16
#define HOLES_ON_DISC 20    // Hole on encoder disc
#define WHEEL_DIAMETER 0.065 // Wheel diameter in meters

// Variables to track the number of steps and last revolution time for each wheel
int right_steps = 0;
int left_steps = 0;
uint32_t left_last_rev_time = 0;
uint32_t right_last_rev_time = 0;
float left_total_distance = 0;
float right_total_distance = 0;

// For speed control
void set_motor_speed(uint slice_num, float duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle * 62500); 
    pwm_set_chan_level(slice_num, PWM_CHAN_B, duty_cycle * 62500 );
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

void move_backward()
{
    printf("Backwards!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);
}

void right_turn(){
    printf("left turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 1);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);

}
void left_turn(){
    printf("right turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );

}
void stop_movement()
{
    printf("Stop!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
}

// Interrupt handler for wheel encoders
void sensor_handler(uint gpio, uint32_t events)
{

    if(gpio ==LEFT_WHEEL_ENCODER)
    {
        left_steps++;
        printf("Left steps: %d\n", left_steps);

        // Check if a full revolution has occurred
        if (left_steps % HOLES_ON_DISC == 0) {
            uint32_t current_time = time_us_32();
            float time_diff = (current_time - left_last_rev_time) / 1000000.0; // Convert to seconds
            float rps = 1.0 / time_diff;
            float speed = (2 * 3.14159265359 * WHEEL_DIAMETER) * rps;
            left_total_distance += speed * time_diff;
            printf("Left wheel RPS: %.2f\n", rps);
            printf("Left wheel speed (m/s): %.2f\n", speed);
            printf("Left wheel total distance (m): %.2f\n", left_total_distance);
            left_last_rev_time = current_time;
        }
    }

    if(gpio == RIGHT_WHEEL_ENCODER)
    {
        right_steps++;
        printf("Right steps: %d\n", right_steps);
        
        // Check if a full revolution has occurred
        if (right_steps % HOLES_ON_DISC == 0) 
        {
            uint32_t current_time = time_us_32();
            float time_diff = (current_time - right_last_rev_time) / 1000000.0; // Convert to seconds
            float rps = 1.0 / time_diff;
            float speed = (2 * 3.14159265359 * WHEEL_DIAMETER) * rps;
            right_total_distance += speed * time_diff;
            printf("Right wheel RPS: %.2f\n", rps);
            printf("Right wheel speed (m/s): %.2f\n", speed);
            printf("Right wheel total distance (m): %.2f\n", right_total_distance);
            right_last_rev_time = current_time;
        }
    }
}