#ifndef VIPER_RTOS_TASK_CAN_COMMS_H
#define VIPER_RTOS_TASK_CAN_COMMS_H


#include "main.h"
#include "rtos_objects.h"
#include "viper_app.h"
#include "at24c04c.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef VIPER_CAN_TASK_TEST
#define VIPER_CAN_TASK_TEST 0
#endif

/* Receive and route CAN commands; own CAN transmission. */
/* Declaration only: implement before creating this task. */
void vCanComms(void *argument);

#endif /* VIPER_RTOS_TASK_CAN_COMMS_H */
