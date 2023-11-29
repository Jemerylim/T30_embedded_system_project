#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "wifi/ssi.h"
#include "wifi/cgi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/ip_addr.h"

// Include the semaphore header
#include "semphr.h" 

//Include drivers
#include "motor/motor.c"
#include "globalVariables.h"

//Include barcode driver
#include "irline/barcode_scanner.c"
//Include ultrasonic driver
#include "ultrasonic/ultrasonic.c"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

#define WIFI_SSID "And"

#define WIFI_PASSWORD "testing123"

#define BUTTON_GPIO 20
#define TOGGLE_GPIO 16

void button_handler(uint gpio, uint32_t events) {
    // Toggle the state of GPIO 16
    gpio_xor_mask(1ul << TOGGLE_GPIO);
}

//Motor movements

//Define sempahore for starting wifi task before initialising HTTPD
SemaphoreHandle_t wifiConnectedSemaphore;

/*****MOTOR VARIABLES*******/
//Define a semaphore for interrupting the motor.
SemaphoreHandle_t motor_state_semaphore;

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
            vTaskDelay(1000);
        }        
    }   
    // vTaskDelete(NULL);
}

void motor_control(){
    // Structure to hold multiple parameters
    typedef struct {
        uint32_t left_notch;
        uint32_t right_notch; 
        uint32_t lastLeftEdgeTime;
        uint32_t lastRightEdgeTime;
        float speedLeft;
        float speedRight;
        float distanceLeft;
        float distanceRight;
    } motorParams;

    typedef struct {
        int leftWheelEncoder;
        int rightWheelEncoder;
    } encoderParams;

    motorParams mParam;
    mParam.lastLeftEdgeTime = 0;   // Time of the last edge for the left wheel
    mParam.lastRightEdgeTime = 0;  // Time of the last edge for the right wheel

    mParam.speedLeft = 0.0;    // Speed of the left wheel in cm/s
    mParam.speedRight = 0.0;   // Speed of the right wheel in cm/s

    mParam.distanceLeft = 0.0; // Distance traveled by the left wheel in cm
    mParam.distanceRight = 0.0;// Distance traveled by the right wheel in cm

    mParam.left_notch = 0;
    mParam.right_notch = 0;

    encoderParams eParam;
    eParam.leftWheelEncoder = 0;
    eParam.rightWheelEncoder = 0;

    // float left_error = 0.0;
    // float right_error = 0.0;
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
    
    //init all
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
    pwm_set_wrap(slice_num_B, 62500); 
    pwm_set_chan_level(slice_num_A, PWM_CHAN_A,62500/4); 
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, 62500/4); 
    pwm_set_enabled(slice_num_A, true);
    pwm_set_enabled(slice_num_B, true);    
    
    // Enable interrupts on wheel encoder GPIO pins
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &interrupt_callback_barcode);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER, GPIO_IRQ_EDGE_RISE, true, &interrupt_callback_barcode);        

    int receivedStatus;
    size_t receivedLength;
    int receivedLWE;
    size_t receivedLengthLWE;
    int receivedRWE;
    size_t receivedLengthRWE;
    uint32_t receivedEncoderTimer;
    size_t receivedLengthEncoderT;

    while (1)
    {                     
        // Receive status message from the message buffer
        receivedLength = xMessageBufferReceive(xMotorStateHandler, &receivedStatus, sizeof(int), 0);        

        //if received data from the webserver/buffer
        if (receivedLength> 0) {
            // if receivedStatus is 1, move forward. pressed "motor on" button on website
            if(receivedStatus == 1){
                 move_forward();
                int left_ir_value = gpio_get(LEFT_IR_SENSOR);
                int right_ir_value = gpio_get(RIGHT_IR_SENSOR);

                if (left_ir_value == 0 && right_ir_value == 1){
                    float startDeg = measurement();
                    left_turn_with_angle(90);
                    pid_controller(mParam.speedLeft, mParam.speedRight,left_integral, right_integral, left_last_error, right_last_error); 
                }
                else if (left_ir_value == 1 && right_ir_value == 0){
                    float startDeg = measurement();
                    right_turn_with_angle(90);
                    pid_controller(mParam.speedLeft, mParam.speedRight,left_integral, right_integral, left_last_error, right_last_error); 
                }
                else if (left_ir_value == 1 && right_ir_value == 1){
                    move_forward();
                    pid_controller(mParam.speedLeft, mParam.speedRight,left_integral, right_integral, left_last_error, right_last_error); 

                }
            }            
            else if(receivedStatus==0){
                stop_movement();                
            }            
        }
        


        /********Previous controls********/
        // receivedLengthEncoderT = xMessageBufferReceive(xMotorEncoderTimerHandler, &receivedEncoderTimer, sizeof(uint32_t), 0);
        // receivedLengthLWE = xMessageBufferReceive(xMotorLeftEncoderHandler, &receivedLWE, sizeof(int), 0);        
        // if(receivedLengthLWE > 0){            
        //     if(receivedLWE == 1){                           
        //         mParam.left_notch++;
        //         // Calculate time between consecutive pulses for the left wheel
        //         uint32_t timeDifference = receivedEncoderTimer - mParam.lastLeftEdgeTime;
        //         mParam.lastLeftEdgeTime = receivedEncoderTimer;

        //         // Calculate speed for the left wheel (cm/s)
        //         mParam.speedLeft = WHEEL_CIRCUMFERENCE_CM / (timeDifference / 100000.0); // Convert timeDifference to seconds

        //         // Update the distance traveled for the left wheel
        //         mParam.distanceLeft += WHEEL_CIRCUMFERENCE_CM/HOLES_ON_DISC;
        //         eParam.leftWheelEncoder = false;
        //         xMessageBufferSend(xMotorLWEdataToWebHandler, (void *) &mParam.speedLeft, sizeof(float), 0);
        //         xMessageBufferSend(xMotorLDistToWebHandler, (void *) &mParam.distanceLeft, sizeof(float), 0);
        //         printf("eParam.leftWheelEncoder:%.2f",mParam.distanceLeft);
        //     }
        // }
        // receivedLengthRWE = xMessageBufferReceive(xMotorRightEncoderHandler, &receivedRWE, sizeof(int), 0);
        // if(receivedLengthRWE > 0){
            
        //     if(receivedRWE == 1){                                                
        //         mParam.right_notch++;
        //         // Calculate time between consecutive pulses for the right wheel
        //         uint32_t timeDifference = receivedEncoderTimer - mParam.lastRightEdgeTime;
        //         mParam.lastRightEdgeTime = receivedEncoderTimer;

        //         // Calculate speed for the right wheel (cm/s)
        //         mParam.speedRight = WHEEL_CIRCUMFERENCE_CM / (timeDifference / 100000.0); // Convert timeDifference to seconds

        //         // Update the distance traveled for the right wheel
        //         mParam.distanceRight += WHEEL_CIRCUMFERENCE_CM/HOLES_ON_DISC;
        //         eParam.rightWheelEncoder = false;
        //         xMessageBufferSend(xMotorRWEdataToWebHandler, (void *) &mParam.speedRight, sizeof(float), 0);
        //         xMessageBufferSend(xMotorRDistToWebHandler, (void *) &mParam.distanceRight, sizeof(float), 0);
        //         printf("mParam.rightWheelEncoder:%.2f",mParam.distanceRight);
        //     }   
        // }

        // if(receivedLengthRWE || receivedLengthLWE){
        //     pid_controller(mParam.speedLeft, mParam.speedRight, left_integral, right_integral, left_last_error, right_last_error);
        // }                
        
        vTaskDelay(100);
    }
}

void barcodeTask(){
    //init the barcode sensor pins
    init_barcode_pins();
    adc_init();
    gpio_set_irq_enabled_with_callback(BARCODE_SENSOR, GPIO_IRQ_EDGE_RISE, true, &interrupt_callback_barcode);
    while(true){
        vTaskDelay(103);
    }
}
void vLaunch( void) {        
    //Create the message buffers with their sizes
    xMotorStateHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //Get the buffer data from motor.c to main.c
    xMotorLeftEncoderHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    xMotorRightEncoderHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //send the LWE and RWE data to webserver
    xMotorLWEdataToWebHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    xMotorRWEdataToWebHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //create buffer for wheel encoder timer
    xMotorEncoderTimerHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //Buffer to send Left and Right distance travelled by the wheel to Webserver
    xMotorLDistToWebHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    xMotorRDistToWebHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //Create buffer to send for ultrasonic
    xControlMessageBufferUltra = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
    //Send data from CGI to SSI to check states
    xMotorSSIHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

    //Create a buffer to send decoded barcode text to webserver
    xBarcodeCharHandler = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
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

    /* Create task for ultrasonic*/
    TaskHandle_t ultratask;
    xTaskCreate(ultra_task, "ultrataskThread", configMINIMAL_STACK_SIZE, NULL, 4, &ultratask);
    
    /* Create task for barcode*/
    TaskHandle_t barcodeTaskHandle;
    xTaskCreate(barcodeTask, "barcodeTaskThread", configMINIMAL_STACK_SIZE, NULL, 5, &barcodeTaskHandle);

    // Assign task `init_wifi` to core 0    
    vTaskCoreAffinitySet(wifi_task_handle, 1 << 0);

    // Assign task `core1_entry` to core 0    
    vTaskCoreAffinitySet(http_task_handle, 1 << 0);

    // Run the motor control on core 1
    vTaskCoreAffinitySet(motor_task_handle, 1);

    //Run the ultrasonic task on core 0
    vTaskCoreAffinitySet(ultratask, 1<<0);

    //Run the barcode task on core 1
    vTaskCoreAffinitySet(barcodeTaskHandle, 1);
        
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