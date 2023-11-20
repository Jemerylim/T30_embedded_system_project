#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/ip_addr.h"

// Include the semaphore header
#include "semphr.h" 

//Include drivers
#include "motor.c"
#include "globalVariables.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

#define WIFI_SSID "And"

#define WIFI_PASSWORD "testing123"

//Motor movements

//Define sempahore for starting wifi task before initialising HTTPD
SemaphoreHandle_t wifiConnectedSemaphore;

/*****MOTOR VARIABLES*******/
//Define a semaphore for interrupting the motor.
SemaphoreHandle_t motor_state_semaphore;
bool returnToMotorLoop = pdFALSE;

//initialise wifi
void init_wifi(__unused void *params) {
    // Check if the cyw43 has been initialised
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    //keep trying to connect for 30s
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        printf("%s\n",WIFI_SSID);
        printf("%s\n",WIFI_PASSWORD);
        exit(1);
    } else {
        printf("Connected.\n");
    }

    //Signal that Wi-Fi is ready
    xSemaphoreGive(wifiConnectedSemaphore);
    while(true) {
        // printf("TESTING WIFI%s\n",IP_ADDR_ANY);        
        // ip4_addr_t ip4_address;
        // IP4_ADDR(&ip4_address, 192, 168, 1, 100); // Example IPv4 address: 192.168.1.100
        // char ip_str[IP4ADDR_STRLEN_MAX];
        // ip4addr_ntoa(&ip4_address);
        // printf("IP address: %s\n", ip_str);

        // not much to do, and we're using RAW (callback) lwIP API
        vTaskDelay(1000);
    }

    // cyw43_arch_deinit();    
}

// to prevent error
void vApplicationMinimalIdleHook( void )
{
}
void init_http_server() {
    //Wait for Wi-Fi to be ready
    if (xSemaphoreTake(wifiConnectedSemaphore, portMAX_DELAY) == pdTRUE) {
    printf("Core1 entry:\n");    
        // Initialise web server
        httpd_init();
        printf("Http server initialised\n");


        // Configure SSI and CGI handler
        ssi_init(); 
        printf("SSI Handler initialised\n");
        cgi_init();
        printf("CGI Handler initialised\n");
        printf("Test");
        // Perform functions and polling on core 1
        while (1) {
            // printf("Testing i am init_http_server_task\n");        
            // printf("I am running on core:%ld\n", sio_hw->cpuid);
            vTaskDelay(1000);
        }        
    }   
    // vTaskDelete(NULL);
}

void motor_control(){

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
    // gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);
    // gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &speed_sensor_handler);

    // gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true, &ir_sensor_handler);
    // gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, true, &ir_sensor_handler);
    
    int receivedStatus;
    size_t receivedLength;

    //more the wheels forward to test the duty cycle interrupt via webserver
    move_forward();
    while (1)
    {  
        printf("i am running motor \n");             
        // Receive status message from the message buffer
        receivedLength = xMessageBufferReceive(xMotorStateHandler, &receivedStatus, sizeof(int), 0);

        //if received data from the webserver/buffer
        if (receivedLength> 0) {
            if(receivedStatus == 1){
                set_motor_speed(0.5);
            }            
            else {
                set_motor_speed(0);
            }            
        }
        
        vTaskDelay(1000);
    }
}
void vLaunch( void) {        
    //Create the message buffers with their sizes
    xMotorStateHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    xMotorStateHandlerMutex = xSemaphoreCreateMutex();

    TaskHandle_t wifi_task_handle;    
    /* Create a task, storing the handle. */
    xTaskCreate( init_wifi, "wifiThread", configMINIMAL_STACK_SIZE, NULL, 2, &( wifi_task_handle ) );        
        
    TaskHandle_t http_task_handle;    
    /* Create a task, storing the handle. */
    xTaskCreate( init_http_server, "httpThread", configMINIMAL_STACK_SIZE, NULL, 1, &( http_task_handle ) );

    TaskHandle_t motor_task_handle;    
    /* Create a task, storing the handle. */
    xTaskCreate( motor_control, "motorThread", configMINIMAL_STACK_SIZE, NULL, 3, &( motor_task_handle ) );

    // Assign task `init_wifi` to core 0    
    vTaskCoreAffinitySet(wifi_task_handle, 1 << 0);

    // Assign task `core1_entry` to core 0    
    vTaskCoreAffinitySet(http_task_handle, 1 << 0);

    // Run the motor control on core 2
    vTaskCoreAffinitySet(motor_task_handle, 1);
    
    // vTaskGetInfo(xHandle, &xTaskDetails,pdTRUE, eInvalid);    
    
#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    
}



int main() {
    stdio_init_all();            
    // Create the semaphore for wifi to start first
    wifiConnectedSemaphore = xSemaphoreCreateBinary();        

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
    #if ( portSUPPORT_SMP == 1 )
        rtos_name = "FreeRTOS SMP";
    #else
        rtos_name = "FreeRTOS";
    #endif

    #if ( portSUPPORT_SMP == 1 ) && ( configNUM_CORES == 2 )
        printf("Starting %s on both cores:\n", rtos_name);
        vLaunch();
    #elif ( RUN_FREERTOS_ON_CORE == 1 )
        printf("Starting %s on core 1:\n", rtos_name);
        multicore_launch_core1(vLaunch);
        while (true);
    #else
        printf("Starting %s on core 0:\n", rtos_name);
        vLaunch();     
        
    #endif  
        return 0;
}