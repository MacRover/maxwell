#ifndef VIPER_RTOS_TASK_FAULT_HANDLER_H
#define VIPER_RTOS_TASK_FAULT_HANDLER_H

#include "main.h"
#include "rtos_objects.h"
#include "viper_app.h"
#include "at24c04c.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

#define FAULT_NOTIFY_ESTOP (1UL << 0)
extern TaskHandle_t FaultHandlerTaskHandle;

/* Handle ESTOP, card enable/disable commands, and hardware faults. */
/* Declaration only: implement before creating this task. */
void vTaskFaultHandler(void *argument);

#endif /* VIPER_RTOS_TASK_FAULT_HANDLER_H */
