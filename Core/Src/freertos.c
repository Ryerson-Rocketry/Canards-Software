#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Drivers/ms5611.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t calcAltitudeHandle_attributes = {
    .name = "calcAltitudeTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal5,
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

  uint16_t prom[8];
  int32_t pressure;
  int32_t temperature;
  int counter = 0;
  ms5611Reset();

  do {
    ms5611ReadPROM(prom);
    counter++;
    if (counter == 10) {
      break;
    }
  }
  while (prom[0] == 0); 

  for (;;)
  {
    ms5611GetPressureAndTemp(prom, &pressure, &temperature);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(pdMS_TO_TICKS(250));
  }
}
