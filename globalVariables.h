#include "FreeRTOS.h"
#include "semphr.h"
#include "message_buffer.h"

// Declare the buffer size for buffer streams
#define mbaTASK_MESSAGE_BUFFER_SIZE       ( 60 )
#define MESSAGE_BUFFER_SIZE  sizeof(int) // Adjust the size based on your data type


extern SemaphoreHandle_t motor_state_semaphore;

/*******Motor Buffers*****/
static MessageBufferHandle_t xMotorStateHandler;
static SemaphoreHandle_t xMotorStateHandlerMutex;

/*******IR Line Buffers*****/
static MessageBufferHandle_t xIRStateHandler;

/*******Magnometer Line Buffers*****/

/*******Ultrasonic Line Buffers*****/


