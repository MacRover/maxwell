#include "task_watchdog.h"
#include "rtos_objects.h"
#include "task.h"
#include "iwdg.h"
#include "viper_app.h"

void TaskWatchdog(void *argument)
{
  (void)argument;

  const EventBits_t requiredMask = WDG_SUPERLOOP;
  TickType_t lastWakeTime = xTaskGetTickCount();


  for (;;)
  {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(250));

    /* ESTOP is latched: normal tasks may intentionally stop checking in.
     * Keep IWDG alive while stopped; a CPU/scheduler hang still stops refreshes. */
    if (ESTOP)
    {
      HAL_IWDG_Refresh(&hiwdg);
      continue;
    }

    /* Check progress from the previous 250 ms window without waiting again.
     * Clear required bits on success so the next window needs fresh reports. */
    EventBits_t received = xEventGroupWaitBits(xWatchdogEventGroup,
        requiredMask, pdTRUE, pdTRUE, 0);

    if ((received & requiredMask) != requiredMask)
    {

      for (;;)
      {
        vTaskDelay(pdMS_TO_TICKS(2000)); // delay this task so other tasks can run while waiting for reset
      }
    }

    HAL_IWDG_Refresh(&hiwdg); // if all tasks repsond refresh timer


  }
}
