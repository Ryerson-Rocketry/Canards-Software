#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "wwdg.h"
#include "ms5611.h"
#include "stdio.h"
#include "usart.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;

// osThreadId_t refreshWatchdogHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// const osThreadAttr_t refereshWatchdog_Attributes = {
//     .name = "refershWatchdog",
//     .stack_size = 128 * 4,
//     .priority = (osPriority_t)osPriorityAboveNormal7,
// };

void StartDefaultTask(void *argument);
// void RefreshWatchdog(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  // refreshWatchdogHandle = osThreadNew(RefreshWatchdog, NULL, &refereshWatchdog_Attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{
  for (;;)
  {

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(pdMS_TO_TICKS(250));
  }
}