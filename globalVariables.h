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
static MessageBufferHandle_t xMotorLeftEncoderHandler;
static MessageBufferHandle_t xMotorRightEncoderHandler;
static MessageBufferHandle_t xMotorLWEdataToWebHandler;
static MessageBufferHandle_t xMotorRWEdataToWebHandler;
static MessageBufferHandle_t xMotorEncoderTimerHandler;
static MessageBufferHandle_t xMotorLDistToWebHandler;
static MessageBufferHandle_t xMotorRDistToWebHandler;

/*******IR Line Buffers*****/
static MessageBufferHandle_t xBarcodeCharHandler;

/*******Magnometer Line Buffers*****/

/*******Ultrasonic Line Buffers*****/
static MessageBufferHandle_t xControlMessageBufferUltra;
extern volatile uint64_t start_time_us;
extern volatile uint64_t end_time_us;
extern volatile bool echo_received;

/*******Webserver Buffers*****/
static MessageBufferHandle_t xMotorSSIHandler;

extern int globalindex;

