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

static MessageBufferHandle_t xControlMessageBufferMoving;
static MessageBufferHandle_t xControlMessageBufferSimple;
static MessageBufferHandle_t xControlMessageBufferMovingPrint;
static MessageBufferHandle_t xControlMessageBufferSimplePrint;
static MessageBufferHandle_t xControlMessageBufferUltra;

const uint TRIG_PIN = 0;
const uint ECHO_PIN = 1;
int timeout = 26100;

float x_est = 0; // Estimated state
float P = 1;    // Estimated error covariance
float Q = 0.01; // Process noise covariance
float R = 1;    // Measurement noise covariance


float read_onboard_temperature() {
    
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    return tempC;
}

/* A Task that obtains the data every 1000 ticks from the inbuilt temperature sensor (RP2040), prints it out and sends it to avg_task via message buffer */
void temp_task(__unused void *params) {
    float temperature = 0.0;

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    while(true) {
        vTaskDelay(1000);
        temperature = read_onboard_temperature();
        // printf("Onboard temperature = %.02f C\n", temperature);
        xMessageBufferSend( 
            xControlMessageBufferMoving,    /* The message buffer to write to. */
            (void *) &temperature,    /* The source of the data to send. */
            sizeof( temperature ),    /* The length of the data to send. */
            0 );                      /* Do not block, should the buffer be full. */
        xMessageBufferSend( 
            xControlMessageBufferSimple,    /* The message buffer to write to. */
            (void *) &temperature,    /* The source of the data to send. */
            sizeof( temperature ),    /* The length of the data to send. */
            0 );                      /* Do not block, should the buffer be full. */
    }
}

/* A Task that indefinitely waits for data from temp_task via message buffer. Once received, it will calculate the moving average and prints out the result. */
void avg_task(__unused void *params) {
    float fReceivedData;
    float sum = 0;
    size_t xReceivedBytes;
    float avg_temp = 0.0;
    
    static float data[4] = {0};
    static int index = 0;
    static int count = 0;

    while(true) {
        xReceivedBytes = xMessageBufferReceive( 
            xControlMessageBufferMoving,        /* The message buffer to receive from. */
            (void *) &fReceivedData,      /* Location to store received data. */
            sizeof( fReceivedData ),      /* Maximum number of bytes to receive. */
            portMAX_DELAY );              /* Wait indefinitely */

        sum -= data[index];            // Subtract the oldest element from sum
        data[index] = fReceivedData;   // Assign the new element to the data
        sum += data[index];            // Add the new element to sum
        index = (index + 1) % 4;       // Update the index - make it circular
        
        if (count < 4) count++;        // Increment count till it reaches 4

        avg_temp = sum / count;

        // printf("Average Temperature = %0.2f C\n", avg_temp);
        xMessageBufferSend( 
            xControlMessageBufferMovingPrint,    /* The message buffer to write to. */
            (void *) &avg_temp,    /* The source of the data to send. */
            sizeof( avg_temp ),    /* The length of the data to send. */
            0 );                      /* Do not block, should the buffer be full. */
    }
}

void simple_avg_task(__unused void *params) {
    float fReceivedData;
    float sum = 0;
    size_t xReceivedBytes;
    int count = 0;
    float sim_avg_temp = 0.0;

    while (true) {
        xReceivedBytes = xMessageBufferReceive(xControlMessageBufferSimple, (void *)&fReceivedData, sizeof(fReceivedData), portMAX_DELAY);

        sum += fReceivedData;
        count++;

        sim_avg_temp = sum / count;

        // printf("Simple Average Temperature = %0.2f C\n", sim_avg_temp);
        xMessageBufferSend( 
            xControlMessageBufferSimplePrint,    /* The message buffer to write to. */
            (void *) &sim_avg_temp,    /* The source of the data to send. */
            sizeof( sim_avg_temp ),    /* The length of the data to send. */
            0 );                      /* Do not block, should the buffer be full. */
    }
}

void print_task(__unused void *params) {
    size_t xReceivedBytesMoving;
    size_t xReceivedBytesSimple;
    float fReceivedDataMoving;
    float fReceivedDataSimple;

    while(true) {
        xReceivedBytesMoving = xMessageBufferReceive(xControlMessageBufferMovingPrint, (void *)&fReceivedDataMoving, sizeof(fReceivedDataMoving), portMAX_DELAY);
        printf("Average Temperature = %0.2f C\n", fReceivedDataMoving);
        xReceivedBytesSimple = xMessageBufferReceive(xControlMessageBufferSimplePrint, (void *)&fReceivedDataSimple, sizeof(fReceivedDataSimple), portMAX_DELAY);
        printf("Simple Average Temperature = %0.2f C\n", fReceivedDataSimple);
    }
}

void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

uint64_t getPulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0) tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) 
    {
        width++;
        sleep_us(1);
        if (width > timeout) return 0;
    }
    absolute_time_t endTime = get_absolute_time();
    
    return absolute_time_diff_us(startTime, endTime);
}

uint64_t getCm(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

uint64_t getInch(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return (long)pulseLength / 74.f / 2.f;
}

void ultra_task(__unused void *params) {
    while(true) {
        uint64_t distance_cm = getCm(TRIG_PIN, ECHO_PIN);
        // uint64_t distance_inch = getInch(TRIG_PIN, ECHO_PIN);

        // printf("\033[2J\033[H");

        // printf("Distance in cm: %llu\n", distance_cm);
        // printf("Distance in inches: %llu\n", distance_inch);

        xMessageBufferSend( 
            xControlMessageBufferUltra,    /* The message buffer to write to. */
            (void *) &distance_cm,    /* The source of the data to send. */
            sizeof( distance_cm ),    /* The length of the data to send. */
            0 ); 

        sleep_ms(1000); // Sleep for 1 second before taking another reading
    }
}

void ultra_check_task(__unused void *params) {
    size_t xReceivedBytesUltra;
    uint64_t fReceivedDataUltra;
    while(true) {
        xReceivedBytesUltra = xMessageBufferReceive(xControlMessageBufferUltra, (void *)&fReceivedDataUltra, sizeof(fReceivedDataUltra), portMAX_DELAY);
        
        if (xReceivedBytesUltra == sizeof(fReceivedDataUltra))
        {
             // Convert the uint64_t measurement to a float
            float measurement = (float)fReceivedDataUltra;

            // Kalman Filter update
            float y = measurement - x_est;  // Calculate the innovation
            float S = P + R;                       // Estimate error + measurement error
            float K = P / S;                       // Kalman gain
            x_est = x_est + K * y;                 // Update the estimate
            P = (1 - K) * P + Q;                   // Update the error covariance

            printf("\033[2J\033[H");
            printf("Distance in cm: %.2f\n", x_est);
            printf("Distance in cm(unfiltered): %llu\n", fReceivedDataUltra);

            if (x_est <= 20.0)
            {
                printf("Obstacle within 20cm");
            }
            else
            {
                printf("No obstacle with 20cm");
            }
        }
        else
        {
            printf("Failed to receive distance data from the message buffer.\n");
        }

        sleep_ms(1000); // Sleep for 1 second before taking another reading
    }
}

int main( void )
{
    stdio_init_all();

    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    TaskHandle_t ultratask;
    xTaskCreate(ultra_task, "ultrataskThread", configMINIMAL_STACK_SIZE, NULL, 3, &ultratask);
    TaskHandle_t ultrachecktask;
    xTaskCreate(ultra_check_task, "ultrachecktaskThread", configMINIMAL_STACK_SIZE, NULL, 2, &ultrachecktask);

    // TaskHandle_t temptask;
    // xTaskCreate(temp_task, "TestTempThread", configMINIMAL_STACK_SIZE, NULL, 3, &temptask);
    // TaskHandle_t avgtask;
    // xTaskCreate(avg_task, "TestAvgThread", configMINIMAL_STACK_SIZE, NULL, 2, &avgtask);
    // TaskHandle_t simavgtask;
    // xTaskCreate(simple_avg_task, "TestSimAvgThread", configMINIMAL_STACK_SIZE, NULL, 2, &simavgtask);
    // TaskHandle_t printtask;
    // xTaskCreate(print_task, "TestPrintThread", configMINIMAL_STACK_SIZE, NULL, 1, &printtask);

    // xControlMessageBufferMoving = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    // xControlMessageBufferSimple = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    // xControlMessageBufferMovingPrint = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    // xControlMessageBufferSimplePrint = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    xControlMessageBufferUltra = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    
    vTaskStartScheduler();
}