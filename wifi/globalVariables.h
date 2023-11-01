#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t motor_state_semaphore;
extern bool returnToMotorLoop;