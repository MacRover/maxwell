#include "rtos_objects.h"
#include "main.h"

QueueHandle_t xCanRxQueue = NULL;
SemaphoreHandle_t xI2C2Mutex = NULL;
EventGroupHandle_t xWatchdogEventGroup = NULL;

// reserve memory on info on how to manage the queue (read/write pointers)
static StaticQueue_t canRxQueueBuffer;

// Reserve memory for queued messages 
static uint8_t CANRxQueueStorage[CAN_RX_QUEUE_LENGTH * sizeof(CanRxMessage)];

static StaticSemaphore_t i2c2MutexBuffer;
static StaticEventGroup_t watchdogEventGroupBuffer;


void RTOS_Objects_Init(void){
    // Using QueueStorage and QueueBuffer create a queue of 8 CAN Messages
    xCanRxQueue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CanRxMessage), CANRxQueueStorage, &canRxQueueBuffer); 
    // Initialize mutex
    xI2C2Mutex = xSemaphoreCreateMutexStatic(&i2c2MutexBuffer); 
    //Initialize event group. Bits are started as cleared, once a task checks in, the corresponding bit will set. 
    xWatchdogEventGroup = xEventGroupCreateStatic(&watchdogEventGroupBuffer); 



    if ((xCanRxQueue == NULL) || (xI2C2Mutex == NULL) || (xWatchdogEventGroup == NULL)) Error_Handler();
}
