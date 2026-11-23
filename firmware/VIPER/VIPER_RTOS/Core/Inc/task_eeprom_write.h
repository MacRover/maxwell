#ifndef VIPER_RTOS_TASK_EEPROM_WRITE_H
#define VIPER_RTOS_TASK_EEPROM_WRITE_H


#include "main.h"
#include "rtos_objects.h"
#include "viper_app.h"
#include "at24c04c.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

#define WDG_EEPROM (1U<<3)


/* Process queued EEPROM write requests and report completion. */
/* Declaration only: implement before creating this task. */
void vEepromWrite(void *pvParameters);

#endif /* VIPER_RTOS_TASK_EEPROM_WRITE_H */
