#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Drivers/ms5611.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(10);
  }
}
