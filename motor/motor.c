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

void move_backward(uint32_t left_notch, uint32_t right_notch)
{
    printf("%llu\n %llu\n", left_notch, right_notch);
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
    float start = measurement();    
    while(start + degree > measurement())
    {
        right_turn();  
    }       
    stop_movement();
        
}
void left_turn_with_angle(float degree){
    float start = measurement();    
    while(start - degree < measurement())
    {
        left_turn();
    }  
    stop_movement();
        
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


// Interrupt handler for wheel encoders
void speed_sensor_handler(uint gpio, uint32_t events) {
    // vTaskSuspendAll();
    uint32_t currentTime = time_us_32();
    xMessageBufferSend(xMotorEncoderTimerHandler, (void *) &currentTime, sizeof(uint32_t), 0);
    if (gpio == LEFT_WHEEL_ENCODER) 
    {           
        int dataToSend = 1;
        //set the left wheel encoder to true and send it using MessageBufferHandle to main.c
        xMessageBufferSend( /* The message buffer to write to. */
                    xMotorLeftEncoderHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
        printf("Data to send at handler %d\n", dataToSend);
    } 
    else if (gpio == RIGHT_WHEEL_ENCODER)
    {        
        int dataToSend = 1;
        // set the left wheel encoder to true and send it using MessageBufferHandle to main.c
        xMessageBufferSend( /* The message buffer to write to. */
                    xMotorRightEncoderHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
        printf("Data to send at handler %d\n", dataToSend);
    }        
    // vTaskDelay(1000);
    // gpio_acknowledge_irq(gpio, events);
    // xTaskResumeAll();

}

// void ir_sensor_handler()
// {    
//     int left_ir_value = gpio_get(LEFT_IR_SENSOR);
//     int right_ir_value = gpio_get(RIGHT_IR_SENSOR);
//     printf("left ir value %d\n",left_ir_value);
//     printf("right ir value %d\n",right_ir_value);
//     if(left_ir_value == 1)
//     {
//         printf("left black true\n");
//         left_black = true;
//     }else
//     {
//         printf("left black false\n");
//         left_black = false;
//     }
    
//     if(right_ir_value == 1)
//     {
//         printf("right black true\n");
//         right_black=true;
//     }
//     else
//     {
//         printf("right black false\n");
//         right_black = false;
//     }       
// }

// int main()
// {

//     // Initialize the standard I/O for printf
//     stdio_init_all();
    
//     // Initialize GPIO pins for motor control and wheel encoders
//     gpio_init(MOTOR_LEFT_FORWARD);
//     gpio_init(MOTOR_LEFT_BACKWARD);
//     gpio_init(MOTOR_RIGHT_BACKWARD);
//     gpio_init(MOTOR_RIGHT_FORWARD);
//     gpio_init(LEFT_WHEEL_ENCODER);
//     gpio_init(RIGHT_WHEEL_ENCODER);
//     gpio_init(LEFT_IR_SENSOR);
//     gpio_init(RIGHT_IR_SENSOR);
    
//     // Tell GPIO 0 and 1 they are allocated to the PWM
//     gpio_set_function(0, GPIO_FUNC_PWM);
//     gpio_set_function(1, GPIO_FUNC_PWM);

//     // Set GPIO directions
//     gpio_set_dir(MOTOR_LEFT_FORWARD, GPIO_OUT);
//     gpio_set_dir(MOTOR_LEFT_BACKWARD, GPIO_OUT);
//     gpio_set_dir(MOTOR_RIGHT_BACKWARD, GPIO_OUT);
//     gpio_set_dir(MOTOR_RIGHT_FORWARD, GPIO_OUT);
//     gpio_set_dir(LEFT_WHEEL_ENCODER,GPIO_IN);
//     gpio_set_dir(RIGHT_WHEEL_ENCODER,GPIO_IN);
//     gpio_set_dir(LEFT_IR_SENSOR,GPIO_IN);
//     gpio_set_dir(RIGHT_IR_SENSOR,GPIO_IN);

//     uint slice_num_A = pwm_gpio_to_slice_num(0);
//     uint slice_num_B = pwm_gpio_to_slice_num(1);
//     pwm_set_clkdiv(slice_num_A, 100); 
//     pwm_set_wrap(slice_num_A, 62500); 

//     pwm_set_clkdiv(slice_num_B, 100);
//     pwm_set_wrap(slice_num_B, 16075); 
//     pwm_set_chan_level(slice_num_A, PWM_CHAN_A, 16075); 
//     pwm_set_chan_level(slice_num_B, PWM_CHAN_B, 16075);
//     pwm_set_enabled(slice_num_A, true);
//     pwm_set_enabled(slice_num_B, true);


//     // Enable interrupts on wheel encoder GPIO pins
//     gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);
//     gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);

    
//     while (1)
//     {  
        
//         // ir_sensor_handler();
//         // if (left_black == true && right_black == true)
//         // {
//         //     move_forward();
//         // }
//         // else if (left_black == true && right_black == false)
//         // {
//         //     right_turn();
//         // }
//         // else if (left_black == false && right_black == true)
//         // {
//         //     left_turn();
//         // }
//         // else if (left_black == false && right_black == true)
//         // {
//         //     stop_movement();
//         // }
//         move_forward();
//         pid_controller(speedLeft, speedRight);
        
//     }
//     return 0;
// }