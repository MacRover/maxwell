#ifndef VIPER_RTOS_RTOS_OBJECTS_H
#define VIPER_RTOS_RTOS_OBJECTS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

extern QueueHandle_t xCanRxQueue;
extern SemaphoreHandle_t xI2C2Mutex;
extern EventGroupHandle_t xWatchdogEventGroup;

void RTOS_Objects_Init(void);

#endif /* VIPER_RTOS_RTOS_OBJECTS_H */
