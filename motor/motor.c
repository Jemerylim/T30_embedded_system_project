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

// Define GPIO pins for motor control and wheel encoders
#define MOTOR_LEFT_FORWARD  2 
#define MOTOR_LEFT_BACKWARD 3
#define MOTOR_RIGHT_FORWARD 5
#define MOTOR_RIGHT_BACKWARD 4
#define RIGHT_WHEEL_ENCODER 17
#define LEFT_WHEEL_ENCODER 16
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
void set_motor_speed(float duty_cycle)
{
    uint slice_num = pwm_gpio_to_slice_num(0);
    pwm_set_clkdiv(slice_num, 100); // Set the clock divider for the PWM slice.
    pwm_set_wrap(slice_num, 62500); // Set the wrap value, which determines the period of the PWM signal (in this case, 62500 cycles).
    pwm_set_enabled(slice_num, true);
   
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle * 62500); 
    pwm_set_chan_level(slice_num, PWM_CHAN_B, duty_cycle * 62500 );
    
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
// void pid_controller(float left_speed, float right_speed)
// {
//     float Kp = 0.1;  // Proportional gain
//     float Ki = 0.05; // Integral gain
//     float Kd = 0.35; // Derivative gain
//     float desired_speed = 10.0; // Set the desired speed in cm/s (adjust as needed)

//     left_error = desired_speed - left_speed;   // Calculate left wheel speed error
//     right_error = desired_speed - right_speed; // Calculate right wheel speed error
//     left_integral += left_error;
//     right_integral += right_error;
//     left_derivative = left_error - left_last_error;
//     right_derivative = right_error - right_last_error;

//     left_output = Kp * left_error + Ki * left_integral + Kd * left_derivative;
//     right_output = Kp * right_error + Ki * right_integral + Kd * right_derivative;
//     // printf("left output %f\n",left_output);
//     // printf("right output %f\n",right_output);

//     set_motor_speed(0, left_output);   // Adjust LEFT_MOTOR using PID output
//     set_motor_speed(1, right_output); // Adjust RIGHT_MOTOR using PID output

//     left_last_error = left_error;   // Update error for next iteration
//     right_last_error = right_error; // Update error for next iteration
// }


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
   
    // pid_controller(speedLeft,speedRight);
}

void ir_sensor_handler(uint gpio, uint32_t events)
{      
    // left is 20 right is 21
    //Testing only
    //Edge rise means black
    //Left
    printf("gpio: %d",gpio);
    printf("events: %d",events);
     if(gpio == 20 && events == GPIO_IRQ_LEVEL_HIGH )
    {
        printf("left black true\n");
        left_black = true;
    }else
    {
        printf("left black false\n");
        left_black = false;
    }
    
    if(gpio == 21 && events == GPIO_IRQ_LEVEL_HIGH)
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

    

    // Enable interrupts on wheel encoder GPIO pins
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);

    gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true, &ir_sensor_handler);
    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true, &ir_sensor_handler);
    
    

    while (1)
    {  
        set_motor_speed(0.3);
        if (left_black == true && right_black == true)
        {
            move_forward();
        }
        else if (left_black == true && right_black == false)
        {
            right_turn_with_angle(90);
        }
        else if (left_black == false && right_black == true)
        {
            left_turn_with_angle(90);
        }
        else if (left_black == false && right_black == true)
        {
            stop_movement();
        }
        

        
    }
}

