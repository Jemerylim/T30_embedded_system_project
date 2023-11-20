#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "globalVariables.h"
#include "motor/motor.h"

// CGI handler which is run when a request for /led.cgi is detected
const char * cgi_led_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for LED has been made (/led.cgi?led=x)
    if (strcmp(pcParam[0] , "led") == 0){
        // Look at the argument to check if LED is to be turned on (x=1) or off (x=0)
        if(strcmp(pcValue[0], "0") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        else if(strcmp(pcValue[0], "1") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    printf("LED TESTING\n %d\t %d\t %s\t %s\t", iIndex, iNumParams, pcParam[0], *pcValue[0]);
    // Send the index page back to the user
    return "/index.shtml";
}

const char* cgi_motor_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]){
    //Check if a request for motor has been made(/motor.cgi/motor=x)
    if(strcmp(pcParam[0], "motor") == 0){
        //Look at the the argument to check if LED is to be turned on (x=1) or off (x=0)
        if(strcmp(pcValue[0], "0") == 0){
            int dataToSend = 0;
            // Wait for the semaphore before sending
            // if (xSemaphoreTake(xMotorStateHandlerMutex, portMAX_DELAY) == pdTRUE) {                
                // Send the temperature to the buffer for moving average
                xMessageBufferSend( /* The message buffer to write to. */
                    xMotorStateHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
                    printf("Data to send at 0: %d\n", dataToSend);
                // xSemaphoreGive(xMotorStateHandlerMutex);
            // }            
            // Release the semaphore to turn on the RC                        
            stop_movement();
            printf("Stopped\n");
        }                      
        else if(strcmp(pcValue[0], "1") == 0){    
            int dataToSend = 1;
            // if (xSemaphoreTake(xMotorStateHandlerMutex, portMAX_DELAY) == pdTRUE) { 
                // Send the temperature to the buffer for moving average
                xMessageBufferSend( /* The message buffer to write to. */
                    xMotorStateHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
                    printf("Data to send at 1: %d\n", dataToSend);
                // xSemaphoreGive(xMotorStateHandlerMutex);
            // }                        
            // xSemaphoreGive(motor_state_semaphore);                              
            move_forward();
            printf("Forward\n");

        }            
    }
    // printf("Motor TESTING\n %d\t %d\t %s\t %s\t\n", iIndex, iNumParams, *pcParam, *pcValue);

    //Send the index page back to the user
    return "/index.shtml";
}

// tCGI Struct
// Fill this with all of the CGI requests and their respective handlers
static const tCGI cgi_handlers[] = {
    {
        // Html request for "/led.cgi" triggers cgi_handler
        "/led.cgi", cgi_led_handler,        
    },
    {
        "/motor.cgi", cgi_motor_handler,
    }
};

void cgi_init(void)
{
    http_set_cgi_handlers(cgi_handlers, 2);
    
}