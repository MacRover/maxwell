#ifndef VIPER_RTOS_RTOS_OBJECTS_H
#define VIPER_RTOS_RTOS_OBJECTS_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

extern QueueHandle_t xCanRxQueue;
extern SemaphoreHandle_t xI2C2Mutex;
extern EventGroupHandle_t xWatchdogEventGroup;

 typedef struct {
    uint32_t canid;
    bool is_extended;
    bool is_remote;
    uint8_t dlc;
    uint8_t data[8]; 
 }  CanRxMessage; 

 #define CAN_RX_QUEUE_LENGTH 8

void RTOS_Objects_Init(void);

#endif /* VIPER_RTOS_RTOS_OBJECTS_H */
