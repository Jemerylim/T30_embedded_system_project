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

#include "motor/motor.h"

#include "globalVariables.h"

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

int timeout = 26100; // Timeout for ultrasonic sensor
volatile uint64_t start_time_us; // Start time for echo pulse
volatile uint64_t end_time_us; // End time for echo pulse
volatile bool echo_received; // Flag indicating if echo pulse is received

float x_est = 0; // Estimated state
float P = 1;    // Estimated error covariance
float Q = 0.01; // Process noise covariance
float R = 1;    // Measurement noise covariance

// Interrupt service routine for handling echo pulse
void echo_isr(uint gpio, uint32_t events) {
    printf("hello i am triggered");
}

// Function to get the duration of the pulse from the ultrasonic sensor
uint64_t getPulse(uint trigPin, uint echoPin) {
    // Reset the echo_received flag
    echo_received = false;

    // Send the trigger pulse
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

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

// Function to convert pulse duration to distance in centimeters
uint64_t getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

// Function to set up GPIO pins for the ultrasonic sensor
void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_isr);
}

// Task for continuously measuring distance with the ultrasonic sensor
void ultra_task(__unused void *params) {
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    while(true) {
        uint64_t distance_cm = getCm(TRIG_PIN, ECHO_PIN);

        printf("Distance in cm: %llu\n", distance_cm);

        if (distance_cm <= 10.0) {
            printf("Obstacle within 10cm\n");
            move_backward();
        } else {
            move_forward();
            printf("No obstacle within 10cm\n");
        }

        vTaskDelay(101); // Sleep for 1 second before taking another reading
    }
}
