#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "globalVariables.h"

//motor control variaables
bool motor_activated = false;

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
            // Release the semaphore to turn on the RC            
            motor_activated = false;
            cyw43_arch_gpio_put(2, 0);// left forward
            cyw43_arch_gpio_put(3, 0);//left backward
            cyw43_arch_gpio_put(4, 0); // right backward
            cyw43_arch_gpio_put(5, 0); // right forward  
            printf("Stopped\n");
        }                      
        else if(strcmp(pcValue[0], "1") == 0){    
            //Go back to motor loop
            returnToMotorLoop = pdTRUE;
            xSemaphoreGive(motor_state_semaphore);                  
            motor_activated = true;
            // cyw43_arch_gpio_put(2, 1);// left forward
            // cyw43_arch_gpio_put(3, 0);//left backward
            // cyw43_arch_gpio_put(4, 0); // right backward
            // cyw43_arch_gpio_put(5, 1); // right forward  
            printf("Forward\n");

        }            
    }
    printf("Motor TESTING\n %d\t %d\t %s\t %s\t\n", iIndex, iNumParams, *pcParam, *pcValue);

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