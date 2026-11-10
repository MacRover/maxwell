#include "rtos_objects.h"
#include "main.h"
#include "viper_app.h"



QueueHandle_t xCanRxQueue = NULL;
QueueHandle_t xCanTxQueue = NULL;
QueueHandle_t xEepromWriteQueue = NULL;
SemaphoreHandle_t xI2C2Mutex = NULL;
SemaphoreHandle_t xI2C1Mutex = NULL;
EventGroupHandle_t xWatchdogEventGroup = NULL;


// reserve memory on info on how to manage the queue (read/write pointers)
static StaticQueue_t canRxQueueBuffer;
static StaticQueue_t canTxQueueBuffer;
static StaticQueue_t eepromWriteQueueBuffer;

// Reserve memory for queued messages 
static uint8_t CANRxQueueStorage[CAN_RX_QUEUE_LENGTH * sizeof(CanRxMessage)];
static uint8_t CANTxQueueStorage[CAN_TX_QUEUE_LENGTH * sizeof(CanTxMessage)];
static uint8_t eepromWriteQueueStorage[EEPROM_WRITE_QUEUE_LENGTH * sizeof(VIPER_PARAMS_TypeDef)];
static StaticSemaphore_t i2c2MutexBuffer;
static StaticSemaphore_t i2c1MutexBuffer;
static StaticEventGroup_t watchdogEventGroupBuffer;


void RTOS_Objects_Init(void){
    // Using QueueStorage and QueueBuffer create a queue of 8 CAN Messages
    xCanRxQueue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CanRxMessage), CANRxQueueStorage, &canRxQueueBuffer); 

    xCanTxQueue = xQueueCreateStatic(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMessage), CANTxQueueStorage, &canTxQueueBuffer); 

    xEepromWriteQueue = xQueueCreateStatic( EEPROM_WRITE_QUEUE_LENGTH, sizeof(VIPER_PARAMS_TypeDef), eepromWriteQueueStorage,&eepromWriteQueueBuffer);
    // Initialize I2C2 mutex
    xI2C2Mutex = xSemaphoreCreateMutexStatic(&i2c2MutexBuffer); 

    // Initialize I2C1 mutex
    xI2C1Mutex = xSemaphoreCreateMutexStatic(&i2c1MutexBuffer); 
    //Initialize event group. Bits are started as cleared, once a task checks in, the corresponding bit will set. 
    xWatchdogEventGroup = xEventGroupCreateStatic(&watchdogEventGroupBuffer); 



    if ((xCanRxQueue == NULL) || (xCanTxQueue == NULL) || (xEepromWriteQueue==NULL) ||(xI2C2Mutex == NULL) || (xI2C1Mutex == NULL) || (xWatchdogEventGroup == NULL)) Error_Handler();
}




