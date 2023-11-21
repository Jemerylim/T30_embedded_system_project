/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip4_addr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ping.h"
#include "message_buffer.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"

#define mbaTASK_MESSAGE_BUFFER_SIZE       ( 60 )

#ifndef PING_ADDR
#define PING_ADDR "142.251.35.196"
#endif
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

#define TRIG_PIN 10
#define ECHO_PIN 11

// static MessageBufferHandle_t xControlMessageBufferUltra;

int timeout = 26100;
volatile uint64_t start_time_us = 0;
volatile uint64_t end_time_us = 0;
volatile bool echo_received = false;

float x_est = 0; // Estimated state
float P = 1;    // Estimated error covariance
float Q = 0.01; // Process noise covariance
float R = 1;    // Measurement noise covariance

void echo_isr(uint gpio, uint32_t events) {
    if (gpio == ECHO_PIN && events == GPIO_IRQ_EDGE_RISE) {  // Rising edge
        start_time_us = to_us_since_boot(get_absolute_time());
    } else {  // Falling edge
        end_time_us = to_us_since_boot(get_absolute_time());
        echo_received = true;
    }
}

uint64_t getPulse(uint trigPin, uint echoPin) {
    // Reset the echo_received flag
    echo_received = false;

    // Send the trigger pulse
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    //need to change this
    //set alarm

    // Wait for the echo_isr to signal echo received
    uint64_t wait_start = to_us_since_boot(get_absolute_time());
    while (!echo_received) {
        if ((to_us_since_boot(get_absolute_time()) - wait_start) >= timeout) {
            // Timeout scenario
            return 0;
        }
    }

    if (end_time_us > start_time_us) {
        return end_time_us - start_time_us;
    }
    return 0;
}

uint64_t getCm(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_isr);
}

void ultra_task(__unused void *params) {
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    while(true) {
        uint64_t distance_cm = getCm(TRIG_PIN, ECHO_PIN);
        // uint64_t distance_inch = getInch(TRIG_PIN, ECHO_PIN);

        printf("\033[2J\033[H");

        printf("Distance in cm: %llu\n", distance_cm);

        if (distance_cm <= 10.0)
        {
            printf("Obstacle within 10cm");
        }
        else
        {
            printf("No obstacle with 10cm");
        }
        // printf("Distance in inches: %llu\n", distance_inch);

        // xMessageBufferSend( 
        //     xControlMessageBufferUltra,    /* The message buffer to write to. */
        //     (void *) &distance_cm,    /* The source of the data to send. */
        //     sizeof( distance_cm ),    /* The length of the data to send. */
        //     0 ); 

        vTaskDelay(1000); // Sleep for 1 second before taking another reading
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// void ultra_check_task(__unused void *params) {
//     size_t xReceivedBytesUltra;
//     uint64_t fReceivedDataUltra;
//     while(true) {
//         xReceivedBytesUltra = xMessageBufferReceive(xControlMessageBufferUltra, (void *)&fReceivedDataUltra, sizeof(fReceivedDataUltra), portMAX_DELAY);
        
//         if (xReceivedBytesUltra == sizeof(fReceivedDataUltra))
//         {
//              // Convert the uint64_t measurement to a float
//             float measurement = (float)fReceivedDataUltra;

//             // Kalman Filter update
//             float y = measurement - x_est;  // Calculate the innovation
//             float S = P + R;                       // Estimate error + measurement error
//             float K = P / S;                       // Kalman gain
//             x_est = x_est + K * y;                 // Update the estimate
//             P = (1 - K) * P + Q;                   // Update the error covariance

//             printf("\033[2J\033[H");
//             printf("Distance in cm: %.2f\n", x_est);
//             printf("Distance in cm(unfiltered): %llu\n", fReceivedDataUltra);

//             if (x_est <= 20.0)
//             {
//                 printf("Obstacle within 20cm");
//             }
//             else
//             {
//                 printf("No obstacle with 20cm");
//             }
//         }
//         else
//         {
//             printf("Failed to receive distance data from the message buffer.\n");
//         }

//         sleep_ms(1000); // Sleep for 1 second before taking another reading
//         // vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }



// int main( void )
// {
//     stdio_init_all();

//     // setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

//     // TaskHandle_t ultratask;
//     // xTaskCreate(ultra_task, "ultrataskThread", configMINIMAL_STACK_SIZE, NULL, 3, &ultratask);
//     // TaskHandle_t ultrachecktask;
//     // xTaskCreate(ultra_check_task, "ultrachecktaskThread", configMINIMAL_STACK_SIZE, NULL, 2, &ultrachecktask);
	
//     // xControlMessageBufferUltra = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

    

//     // while (true)
//     // {
//     //     /* code */
//     //     uint64_t distance_cm = getCm(TRIG_PIN, ECHO_PIN);
//     //     printf("\033[2J\033[H");
//     //     printf("Distance in cm: %llu\n", distance_cm);
//     //     sleep_ms(1000);
//     // }
    
//     vTaskStartScheduler();
// }