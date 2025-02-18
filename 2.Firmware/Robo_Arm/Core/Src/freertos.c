/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SafeMonitorTaskHandle;
uint32_t SafeMonitorBuffer[ 128 ];
osStaticThreadDef_t SafeMonitorControlBlock;
osThreadId LcdTaskHandle;
uint32_t LcdTaskBuffer[ 256 ];
osStaticThreadDef_t LcdTaskControlBlock;
osThreadId ImuTaskHandle;
uint32_t ImuTaskBuffer[ 2048 ];
osStaticThreadDef_t ImuTaskControlBlock;
osThreadId FunTestHandle;
uint32_t FunTestBuffer[ 128 ];
osStaticThreadDef_t FunTestControlBlock;
osThreadId MotorCommTaskHandle;
uint32_t MotorCommTaskBuffer[ 4096 ];
osStaticThreadDef_t MotorCommTaskControlBlock;
osThreadId RgbTaskHandle;
uint32_t RgbTaskBuffer[ 128 ];
osStaticThreadDef_t RgbTaskControlBlock;
osThreadId KinematicsTaskHandle;
uint32_t KinematicsTaskBuffer[ 4096 ];
osStaticThreadDef_t KinematicsTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void SafeMonitorTask_Entry(void const * argument);
void LcdTask_Entry(void const * argument);
void ImuTask_Entry(void const * argument);
void FunTest_Entry(void const * argument);
void MotorCommTask_Entry(void const * argument);
void RgbTask_Entry(void const * argument);
void KinematicsTask_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 创建二进制信号量
  uart_tx_semaphore = xSemaphoreCreateBinary();
  uart_rx_semaphore = xSemaphoreCreateBinary();

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SafeMonitorTask */
  osThreadStaticDef(SafeMonitorTask, SafeMonitorTask_Entry, osPriorityHigh, 0, 128, SafeMonitorBuffer, &SafeMonitorControlBlock);
  SafeMonitorTaskHandle = osThreadCreate(osThread(SafeMonitorTask), NULL);

  /* definition and creation of LcdTask */
  osThreadStaticDef(LcdTask, LcdTask_Entry, osPriorityNormal, 0, 256, LcdTaskBuffer, &LcdTaskControlBlock);
  LcdTaskHandle = osThreadCreate(osThread(LcdTask), NULL);

  /* definition and creation of ImuTask */
  osThreadStaticDef(ImuTask, ImuTask_Entry, osPriorityNormal, 0, 2048, ImuTaskBuffer, &ImuTaskControlBlock);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of FunTest */
  osThreadStaticDef(FunTest, FunTest_Entry, osPriorityBelowNormal, 0, 128, FunTestBuffer, &FunTestControlBlock);
  FunTestHandle = osThreadCreate(osThread(FunTest), NULL);

  /* definition and creation of MotorCommTask */
  osThreadStaticDef(MotorCommTask, MotorCommTask_Entry, osPriorityHigh, 0, 4096, MotorCommTaskBuffer, &MotorCommTaskControlBlock);
  MotorCommTaskHandle = osThreadCreate(osThread(MotorCommTask), NULL);

  /* definition and creation of RgbTask */
  osThreadStaticDef(RgbTask, RgbTask_Entry, osPriorityNormal, 0, 128, RgbTaskBuffer, &RgbTaskControlBlock);
  RgbTaskHandle = osThreadCreate(osThread(RgbTask), NULL);

  /* definition and creation of KinematicsTask */
  osThreadStaticDef(KinematicsTask, KinematicsTask_Entry, osPriorityHigh, 0, 4096, KinematicsTaskBuffer, &KinematicsTaskControlBlock);
  KinematicsTaskHandle = osThreadCreate(osThread(KinematicsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SafeMonitorTask_Entry */
/**
* @brief Function implementing the SafeMonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SafeMonitorTask_Entry */
__weak void SafeMonitorTask_Entry(void const * argument)
{
  /* USER CODE BEGIN SafeMonitorTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SafeMonitorTask_Entry */
}

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void const * argument)
{
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LcdTask_Entry */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void const * argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
__weak void FunTest_Entry(void const * argument)
{
  /* USER CODE BEGIN FunTest_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FunTest_Entry */
}

/* USER CODE BEGIN Header_MotorCommTask_Entry */
/**
* @brief Function implementing the MotorCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCommTask_Entry */
__weak void MotorCommTask_Entry(void const * argument)
{
  /* USER CODE BEGIN MotorCommTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorCommTask_Entry */
}

/* USER CODE BEGIN Header_RgbTask_Entry */
/**
* @brief Function implementing the RgbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RgbTask_Entry */
__weak void RgbTask_Entry(void const * argument)
{
  /* USER CODE BEGIN RgbTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RgbTask_Entry */
}

/* USER CODE BEGIN Header_KinematicsTask_Entry */
/**
* @brief Function implementing the KinematicsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KinematicsTask_Entry */
__weak void KinematicsTask_Entry(void const * argument)
{
  /* USER CODE BEGIN KinematicsTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KinematicsTask_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
