#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "main.h"
#include "Drivers/ms5611.h"
#include "cmsis_os2.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = osPriorityNormal,
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

  uint16_t prom[8] = {0};
  int32_t pressure = 0;
  int32_t temperature = 0;
  int counter = 0;
  HAL_StatusTypeDef status = HAL_ERROR;

  ms5611Reset();

  do
  {
    status = ms5611ReadPROM(prom);
    counter++;
    if (counter == 10)
    {
      break;
    }
    osDelay(100);
  } while (status != HAL_OK);

  if (status != HAL_OK)
  {
    for (;;)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      osDelay(100); // Fast error blink
    }
  }

  for (;;)
  {
    ms5611GetPressureAndTemp(prom, &pressure, &temperature);
    osDelay(pdMS_TO_TICKS(250));

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(pdMS_TO_TICKS(250));
  }
}
