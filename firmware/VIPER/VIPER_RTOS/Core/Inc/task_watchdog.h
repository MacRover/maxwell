#ifndef VIPER_RTOS_TASK_WATCHDOG_H
#define VIPER_RTOS_TASK_WATCHDOG_H

/* Check fresh task health reports and service the hardware watchdog. */
/* Declaration only: implement before creating this task. */
void TaskWatchdog(void *argument);

#endif /* VIPER_RTOS_TASK_WATCHDOG_H */
