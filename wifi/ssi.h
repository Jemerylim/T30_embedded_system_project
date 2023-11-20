#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "globalVariables.h"

// SSI tags - tag length limited to 8 bytes by default
const char * ssi_tags[] = {"volt","temp","led", "motor", "lspeed", "rspeed", "ldist", "rdist", "bcode", "ultras", "magno"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {  
  size_t printed;  
  switch (iIndex) {
    case 0: // volt
      {
        const float voltage = adc_read() * 3.3f / (1 << 12);
        printed = snprintf(pcInsert, iInsertLen, "%f", voltage);
      }
      break;
    case 1: // temp
      {
      const float voltage = adc_read() * 3.3f / (1 << 12);
      const float tempC = 27.0f - (voltage - 0.706f) / 0.001721f;
      printed = snprintf(pcInsert, iInsertLen, "%f", tempC);
      }
      break;
    case 2: // led
      {
        bool led_status = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
        if(led_status == true){
          printed = snprintf(pcInsert, iInsertLen, "ON");
        }
        else if(led_status == false){
          printed = snprintf(pcInsert, iInsertLen, "OFF");
        }
      }
      break;
    case 3: //motor
      {
        bool motor_left_forward = gpio_get(2); // motor left forward
        bool motor_left_backward = gpio_get(3); // motor left backward
        bool motor_right_backward = gpio_get(4);// motor right backward 
        bool motor_right_forward = gpio_get(5); // motot right forward
        if((motor_left_forward == 0) && (motor_left_backward == 0) && 
        (motor_right_backward == 0) && (motor_right_forward == 0)) {          
          printed = snprintf(pcInsert, iInsertLen, "Stopped");
        }
        else if((motor_left_forward == 1) || (motor_left_backward == 1) || 
        (motor_right_backward == 1) || (motor_right_forward == 1)){
          printed = snprintf(pcInsert, iInsertLen, "Running");
        }

        printf("GPIO 2: %d\n GPIO 3: %d\n GPIO 4: %d\n GPIO 5: %d\n", gpio_get(2), gpio_get(3),gpio_get(4),gpio_get(5));        
      }    
      break;
      case 4: //lspeed
      {
        printed = snprintf(pcInsert, iInsertLen, "lspeed");
      }
      break;
      case 5: //rspeed
      {
        printed = snprintf(pcInsert, iInsertLen, "rspeed");
      }
      break;
      case 6: //ldist
      {
        printed = snprintf(pcInsert, iInsertLen, "ldist");
      }
      break;
      case 7: //rdist
      {
        printed = snprintf(pcInsert, iInsertLen, "rdist");
      }
      break;
      case 8: //barcode
      {
        printed = snprintf(pcInsert, iInsertLen, "bcode");
      }
      break;
      case 9: //ultra
      {
        printed = snprintf(pcInsert, iInsertLen, "ultras");
      }
      break;
      case 10: //magno
      {
        printed = snprintf(pcInsert, iInsertLen, "magno");
      }
    default:
      printed = 0;
      break;
    }
  return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init() {
  // Initialise ADC (internal pin)
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
