/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtos_objects.h"
#include "viper_app.h"
#include "task_watchdog.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// reserve RAM for the watchdog task
static StaticTask_t watchdogTaskBuffer;
static StackType_t watchdogTaskStack[128]; // 128 entires * 4 bytes
static const osThreadAttr_t watchdogTask_attributes = {
  .name = "watchdog",
  .cb_mem = &watchdogTaskBuffer, // where metadata lives
  .cb_size = sizeof(watchdogTaskBuffer), // size of metadata
  .stack_mem = watchdogTaskStack, // where stack lives
  .stack_size = sizeof(watchdogTaskStack), // size of stack
  .priority = osPriorityAboveNormal, // priority is level 4
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t viperSuperLoopHandle;
const osThreadAttr_t viperSuperLoopTask_attributes = {
  .name = "viperSuperLoop",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void superLoop(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  RTOS_Objects_Init();
  
  
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  viperSuperLoopHandle = osThreadNew(superLoop, NULL, &viperSuperLoopTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (osThreadNew(TaskWatchdog, NULL, &watchdogTask_attributes) == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void superLoop(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  // ESTOP --> DISABLE --> FREEZE -> ACTIVE
  viper_superLoop();
  for(;;){}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

