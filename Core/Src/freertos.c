#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask();

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{

  osDelay(1);

  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  configASSERT(xMagDataReadySemaphore);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

// Default task function
void StartDefaultTask()
{
  for (;;)
  {
    osDelay(1);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}
