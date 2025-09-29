#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
<<<<<<< HEAD
#include "ms5611.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;
uint16_t prom[8];
int32_t pressure, temperature;
=======
#include "wwdg.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;
osThreadId_t refreshWatchdogHandle;
>>>>>>> main

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t refereshWatchdog_Attributes = {
    .name = "refershWatchdog",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal7,
};

void StartDefaultTask(void *argument);
void RefreshWatchdog(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  refreshWatchdogHandle = osThreadNew(RefreshWatchdog, NULL, &refereshWatchdog_Attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{
  for (;;)
  {
<<<<<<< HEAD
    ms5611Reset();
    ms5611ReadPROM(prom);
    ms5611GetPressureAndTemp(prom, &pressure, &temperature);
    
=======
    osDelay(1);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}

void RefreshWatchdog(void *argument)
{

  for (;;)
  {

    //  formula for watchdog tiemr: (1/fhclk) * 4096 * Nwwdg_prescaler * (Nrefresh -Nwindow)
    osDelay(10);

    // Toggle LED to see if on
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
    {
      Error_Handler();
    }
>>>>>>> main
  }
}
