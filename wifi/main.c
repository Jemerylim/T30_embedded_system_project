#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/ip_addr.h"

#include "semphr.h" // Include the semaphore header


#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

#define WIFI_SSID "And"

#define WIFI_PASSWORD "testing123"

//Define sempahore
SemaphoreHandle_t wifiConnectedSemaphore;

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
        printf("TESTING WIFI%d\n",IP_ADDR_ANY);        
        ip4_addr_t ip4_address;
        IP4_ADDR(&ip4_address, 192, 168, 1, 100); // Example IPv4 address: 192.168.1.100
        char ip_str[IP4ADDR_STRLEN_MAX];
        ip4addr_ntoa(&ip4_address);
        printf("IP address: %s\n", ip_str);




        // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
        vTaskDelay(1000);
    }

    // cyw43_arch_deinit();    
}

void handle_http_request(__unused void *params){
    // Initialise web server
    httpd_init();
    printf("Http server initialised\n");


    // Configure SSI and CGI handler
    ssi_init(); 
    printf("SSI Handler initialised\n");
    cgi_init();
    printf("CGI Handler initialised\n");
    printf("Test");
    while(true){
        printf("I am running http");
        vTaskDelay(100);

    }    
}
void vApplicationMinimalIdleHook( void )
{
}
void core1_entry() {
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
            printf("Testing am i on second core\n");        
            vTaskDelay(1000);
        }        
    }   
    vTaskDelete(NULL);
}
void vLaunch( void) {    

    //Task to initialise wifi module
    TaskHandle_t wifi_task;
    // xTaskCreatePinnedToCore(init_wifi, "MainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &wifi_task,0);            
    // xTaskCreatePinnedToCore(init_wifi, "Task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &wifi_task, 0); // Assign to core 0    
                                    
    // xTaskCreateAffinitySet(init_wifi, "wifiTask", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, 0,&wifi_task);
    // TaskHandle_t core_entry;
    // xTaskCreateAffinitySet(core1_entry, "wifiTask", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, 1,&core_entry);

    TaskHandle_t xHandle;    

        /* Create a task, storing the handle. */
        xTaskCreate( init_wifi, "NAME", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &( xHandle ) );        
        
    TaskHandle_t xHandle1;    

        /* Create a task, storing the handle. */
        xTaskCreate( core1_entry, "NAME1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &( xHandle1 ) );

       // Assign task `init_wifi` to core 0
       TaskHandle_t init_wifi;
        vTaskCoreAffinitySet(init_wifi, 1 << 1);

        // Assign task `core1_entry` to core 1
        TaskHandle_t core1_entry;
        vTaskCoreAffinitySet(core1_entry, 1 << 0);
    // //Task to handle http requests
    // TaskHandle_t http_task;
    // xTaskCreate(handle_http_request, "httpThread", 2048, NULL, 2, &http_task);
    
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

    // Create the semaphore
    wifiConnectedSemaphore = xSemaphoreCreateBinary();

    
    // cyw43_arch_init();

    // cyw43_arch_enable_sta_mode();

    // // Connect to the WiFI network - loop until connected
    // while(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0){
    //     printf("Attempting to connect...\n");
    // }
    // // Print a success message once connected
    // printf("Connected! \n");
    
    // // Initialise web server
    // httpd_init();
    // printf("Http server initialised\n");


    // // Configure SSI and CGI handler
    // ssi_init(); 
    // printf("SSI Handler initialised\n");
    // cgi_init();
    // printf("CGI Handler initialised\n");
    // printf("Test");    

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