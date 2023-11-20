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
#include "mag.h"
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

volatile uint32_t lastLeftEdgeTime = 0;   // Time of the last edge for the left wheel
volatile uint32_t lastRightEdgeTime = 0;  // Time of the last edge for the right wheel

volatile float speedLeft = 0.0;    // Speed of the left wheel in cm/s
volatile float speedRight = 0.0;   // Speed of the right wheel in cm/s

volatile float distanceLeft = 0.0; // Distance traveled by the left wheel in cm
volatile float distanceRight = 0.0;// Distance traveled by the right wheel in cm

uint32_t left_notch, right_notch = 0;

float left_error = 0.0;
float right_error = 0.0;
float left_integral = 0.0;
float right_integral = 0.0;
float left_derivative = 0.0;
float right_derivative = 0.0;
float left_output = 0.0;
float right_output = 0.0;
float left_last_error = 0.0;  // Initialize the last error for left wheel
float right_last_error = 0.0; // Initialize the last error for right wheel

// Variables to track the number of steps and last revolution time for each wheel
bool left_black = false;
bool right_black = false;
float start_degree = 0;
float degree = 0 ;
float start = 0;


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
void right_turn_with_angle( float degree){
    start = measurement();
    while(start + degree > measurement())
    {
        right_turn();  
    }       
    stop_movement();
        
}
void left_turn_with_angle(float degree){
    start = measurement();
    while(start - degree < measurement())
    {
        left_turn();
    }  
    stop_movement();
        
}

void pid_controller(float left_speed, float right_speed)
{
    float Kp = 0.05;  // Proportional gain
    float Ki = 0.01; // Integral gain
    float Kd = 0.05; // Derivative gain
    float desired_speed = 30.0; // Set the desired speed in cm/s (adjust as needed)

    float left_error = desired_speed - left_speed;   // Calculate left wheel speed error
    float right_error = desired_speed - right_speed; // Calculate right wheel speed error

    left_integral += left_error;
    right_integral += right_error;

    float left_derivative = left_error - left_last_error;
    float right_derivative = right_error - right_last_error;

    float left_output = Kp * left_error + Ki * left_integral + Kd * left_derivative;
    float right_output = Kp * right_error + Ki * right_integral + Kd * right_derivative;

    // printf("left output %f\n", left_output);
    // printf("right output %f\n", right_output);

    set_motor_speed(left_output/100, right_output/100);

    left_last_error = left_error;
    right_last_error = right_error;
}


// Interrupt handler for wheel encoders
void speed_sensor_handler(uint gpio, uint32_t events) {
    uint32_t currentTime = time_us_32();

    if (gpio == LEFT_WHEEL_ENCODER) 
    {
        left_notch ++;
        // Calculate time between consecutive pulses for the left wheel
        uint32_t timeDifference = currentTime - lastLeftEdgeTime;
        lastLeftEdgeTime = currentTime;

        // Calculate speed for the left wheel (cm/s)
        speedLeft = WHEEL_CIRCUMFERENCE_CM / (timeDifference / 100000.0); // Convert timeDifference to seconds

        // Update the distance traveled for the left wheel
        distanceLeft += WHEEL_CIRCUMFERENCE_CM/HOLES_ON_DISC;
    
    } 
    else if (gpio == RIGHT_WHEEL_ENCODER)
    {
        right_notch ++;
        // Calculate time between consecutive pulses for the right wheel
        uint32_t timeDifference = currentTime - lastRightEdgeTime;
        lastRightEdgeTime = currentTime;

        // Calculate speed for the right wheel (cm/s)
        speedRight = WHEEL_CIRCUMFERENCE_CM / (timeDifference / 100000.0); // Convert timeDifference to seconds

        // Update the distance traveled for the right wheel
        distanceRight += WHEEL_CIRCUMFERENCE_CM/HOLES_ON_DISC;        
    }
    printf("left speed: %f\n",speedLeft);
    printf("right speed: %f\n",speedRight);
}

void ir_sensor_handler()
{    
    int left_ir_value = gpio_get(LEFT_IR_SENSOR);
    int right_ir_value = gpio_get(RIGHT_IR_SENSOR);
    printf("left ir value %d\n",left_ir_value);
    printf("right ir value %d\n",right_ir_value);
    if(left_ir_value == 1)
    {
        printf("left black true\n");
        left_black = true;
    }else
    {
        printf("left black false\n");
        left_black = false;
    }
    
    if(right_ir_value == 1)
    {
        printf("right black true\n");
        right_black=true;
    }
    else
    {
        printf("right black false\n");
        right_black = false;
    }       
}

int main()
{

    // Initialize the standard I/O for printf
    stdio_init_all();
    
    // Initialize GPIO pins for motor control and wheel encoders
    gpio_init(MOTOR_LEFT_FORWARD);
    gpio_init(MOTOR_LEFT_BACKWARD);
    gpio_init(MOTOR_RIGHT_BACKWARD);
    gpio_init(MOTOR_RIGHT_FORWARD);
    gpio_init(LEFT_WHEEL_ENCODER);
    gpio_init(RIGHT_WHEEL_ENCODER);
    gpio_init(LEFT_IR_SENSOR);
    gpio_init(RIGHT_IR_SENSOR);
    
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);

    // Set GPIO directions
    gpio_set_dir(MOTOR_LEFT_FORWARD, GPIO_OUT);
    gpio_set_dir(MOTOR_LEFT_BACKWARD, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_BACKWARD, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_FORWARD, GPIO_OUT);
    gpio_set_dir(LEFT_WHEEL_ENCODER,GPIO_IN);
    gpio_set_dir(RIGHT_WHEEL_ENCODER,GPIO_IN);
    gpio_set_dir(LEFT_IR_SENSOR,GPIO_IN);
    gpio_set_dir(RIGHT_IR_SENSOR,GPIO_IN);

    uint slice_num_A = pwm_gpio_to_slice_num(0);
    uint slice_num_B = pwm_gpio_to_slice_num(1);
    pwm_set_clkdiv(slice_num_A, 100); 
    pwm_set_wrap(slice_num_A, 62500); 

    pwm_set_clkdiv(slice_num_B, 100);
    pwm_set_wrap(slice_num_B, 16075); 
    pwm_set_chan_level(slice_num_A, PWM_CHAN_A, 16075); 
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, 16075);
    pwm_set_enabled(slice_num_A, true);
    pwm_set_enabled(slice_num_B, true);


    // Enable interrupts on wheel encoder GPIO pins
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);

    
    while (1)
    {  
        
        // ir_sensor_handler();
        // if (left_black == true && right_black == true)
        // {
        //     move_forward();
        // }
        // else if (left_black == true && right_black == false)
        // {
        //     right_turn();
        // }
        // else if (left_black == false && right_black == true)
        // {
        //     left_turn();
        // }
        // else if (left_black == false && right_black == true)
        // {
        //     stop_movement();
        // }
        move_forward();
        pid_controller(speedLeft, speedRight);
        
    }
    return 0;
}