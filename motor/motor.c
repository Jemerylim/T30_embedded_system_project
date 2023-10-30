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
            printf("Left wheel RPS: %.2f\n", rps);
            printf("Left wheel speed (m/s): %.2f\n", speed);
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
            printf("Right wheel RPS: %.2f\n", rps);
            printf("Right wheel speed (m/s): %.2f\n", speed);
            right_last_rev_time = current_time;
        }
    }
}
// Functions to control the movement of the robot
void move_forward()
{
    gpio_put(MOTOR_LEFT_FORWARD, 1);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_FORWARD, 1);
}

void move_backward()
{
    printf("Backwards!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
}

void left_turn(){
    printf("left turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_FORWARD, 0);

}
void right_turn(){
    printf("right turn!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 1);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );
    gpio_put(MOTOR_RIGHT_FORWARD, 0);

}
void stop_movement()
{
    printf("Stop!\n");
    gpio_put(MOTOR_LEFT_FORWARD, 0);
    gpio_put(MOTOR_LEFT_BACKWARD, 0);
    gpio_put(MOTOR_RIGHT_BACKWARD,0 );
    gpio_put(MOTOR_RIGHT_FORWARD, 0);
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

    // Configure PWM for motor control
    uint slice_num = pwm_gpio_to_slice_num(0);
    pwm_set_clkdiv(slice_num, 100); // Set the clock divider for the PWM slice.
    pwm_set_wrap(slice_num, 62500); // Set the wrap value, which determines the period of the PWM signal (in this case, 62500 cycles).
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0.5 * 62500); 
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0.5 * 62500 );
    pwm_set_enabled(slice_num, true);

    // Enable interrupts on wheel encoder GPIO pins
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &sensor_handler);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &sensor_handler);
    


    while (1)
    {   
        // Control the robot's movement
        move_forward();

        sleep_ms(5000);

        move_backward();

        sleep_ms(5000);

        left_turn();
        
        sleep_ms(5000);

        right_turn();

        sleep_ms(5000);    
    }
}

