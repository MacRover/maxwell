#include "task_fault_handler.h"

/*
 * TODO: Handle ESTOP, card enable/disable commands, and hardware faults.
 * Scaffold only; task implementation has not been added yet.
 */

void vTaskFaultHandler(void *argument)
{
    (void)argument;
    uint32_t events;

    for (;;) {
        if (xTaskNotifyWait(0U, FAULT_NOTIFY_ESTOP, &events, portMAX_DELAY) == pdTRUE &&
            (events & FAULT_NOTIFY_ESTOP) != 0U) {
            ESTOP = 1U;
        }
    }
}