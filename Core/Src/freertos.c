#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ms5611.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;
uint16_t prom[8];
int32_t pressure, temperature;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
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
    ms5611Reset();
    ms5611ReadPROM(prom);
    ms5611GetPressureAndTemp(prom, &pressure, &temperature);
    
  }
}
