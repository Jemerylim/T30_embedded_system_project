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
        int receivedStatus;
        size_t receivedLength;
        receivedLength = xMessageBufferReceive(xMotorSSIHandler, &receivedStatus, sizeof(int), 0);

        // if(receivedLength > 0){
          // if(receivedStatus == 1){
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
          // }          
          // else{
          //   printed = snprintf(pcInsert, iInsertLen, "Stopped");
          // }
        // }
        
                
      }    
      break;
      case 4: //lspeed
      {
        static float previousLWEdata = 0;  // Static variable to store the previous value
        float receivedLWEdata;
        size_t receivedLength;
        
        receivedLength = xMessageBufferReceive(xMotorLWEdataToWebHandler, &receivedLWEdata, sizeof(float), 0);
        
        if (receivedLength > 0) {
            // New data received, update both the display and the previous value
            printed = snprintf(pcInsert, iInsertLen, "%.2f", receivedLWEdata);
            previousLWEdata = receivedLWEdata;
        } else {
            // No new data, use the previous value
            printed = snprintf(pcInsert, iInsertLen, "%.2f", previousLWEdata);
        }
      }
      break;
      case 5: //rspeed
      {
        static float previousRWEdata = 0;  // Static variable to store the previous value
        float receivedRWEdata;
        size_t receivedLength;

        receivedLength = xMessageBufferReceive(xMotorRWEdataToWebHandler, &receivedRWEdata, sizeof(float), 0); 
        if(receivedLength > 0){
          // New data received, update both the display and the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", receivedRWEdata);
          previousRWEdata = receivedRWEdata;
        } else {
          //no new data, use the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", previousRWEdata);
        }             
      }
      break;
      case 6: //ldist
      {
        static float previousLDdata = 0;  // Static variable to store the previous value
        float receivedLDdata;
        size_t receivedLength;

        receivedLength = xMessageBufferReceive(xMotorLDistToWebHandler, &receivedLDdata, sizeof(float), 0); 
        if(receivedLength > 0){
          // New data received, update both the display and the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", receivedLDdata);
          printf("receive LDATA: %f\n", receivedLDdata);
          previousLDdata = receivedLDdata;
        } else {
          //no new data, use the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", previousLDdata);
        }
      }
      break;
      case 7: //rdist
      {
        static float previousRDdata = 0;  // Static variable to store the previous value
        float receivedRDdata;
        size_t receivedLength;

        receivedLength = xMessageBufferReceive(xMotorRDistToWebHandler, &receivedRDdata, sizeof(float), 0); 
        if(receivedLength > 0){
          // New data received, update both the display and the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", receivedRDdata);
          previousRDdata = receivedRDdata;
        } else {
          //no new data, use the previous value
          printed = snprintf(pcInsert, iInsertLen, "%.2f", previousRDdata);
        }
      }
      break;
      case 8: //barcode
      {
        static char previousDecoded[11];  // Static variable to store the previous value
        char receivedChar[2];
        size_t receivedLength;

        // receivedLength = xMessageBufferReceive(xBarcodeCharHandler, &receivedChar, sizeof(receivedChar)*11, 0); 
        // if(receivedLength > 0){
        //   // printf("receivedChar: %s I AM INSIDE\n", receivedChar);
        //   // New data received, update both the display and the previous value
        //   printed = snprintf(pcInsert, iInsertLen, "%c", receivedChar[0]);
        //   strncpy(previousDecoded, receivedChar, sizeof(previousDecoded) - 1);
        //   previousDecoded[sizeof(previousDecoded) - 1] = '\0';  // Ensure null-termination
        // } else {
        //   //no new data, use the previous value
        //   printed = snprintf(pcInsert, iInsertLen, "%s", previousDecoded);
        // }
        
        receivedLength = xMessageBufferReceive(xBarcodeCharHandler, &receivedChar, sizeof(receivedChar)*2, 0); 
        if (receivedLength > 0) {
            // New data received, update both the display and the previous value
            int validCharCount = 0;
            for (int i = 0; i < receivedLength; ++i) {
                if (isprint(receivedChar[i])) {
                    // Character is printable, consider it valid
                    validCharCount++;
                }
            }

            printf("Number of valid characters: %d\n", validCharCount);

            // Update the display with the first valid character
            if (validCharCount > 0) {
                printed = snprintf(pcInsert, iInsertLen, "%c", receivedChar[0]);
                strncpy(previousDecoded, receivedChar, sizeof(previousDecoded) - 1);
                previousDecoded[sizeof(previousDecoded) - 1] = '\0';  // Ensure null-termination
            } else {
                // No valid characters, use the previous value
                printed = snprintf(pcInsert, iInsertLen, "%s", previousDecoded);
            }
        } else {
            // No new data, use the previous value
            printed = snprintf(pcInsert, iInsertLen, "%s", previousDecoded);
        }
      }
      break;
      case 9: //ultra
      {
        printed = snprintf(pcInsert, iInsertLen, "%d",globalindex);
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
