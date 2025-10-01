#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ms5611.h"
#include "stdio.h"
#include "usart.h"

// Task handle and attributes
osThreadId_t defaultTaskHandle;
osThreadId_t calcAltitudeHandle;

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
void CalcAltitudeTask(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  calcAltitudeHandle = osThreadNew(CalcAltitudeTask, NULL, &calcAltitudeHandle_attributes);
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

void CalcAltitudeTask(void *argument)
{
  uint16_t prom[8];
  int32_t pressure, temperature;

  char msg[32];

  ms5611Reset();
  ms5611ReadPROM(prom);

  for (;;)
  {
    ms5611GetPressureAndTemp(prom, &pressure, &temperature);

    snprintf(msg, sizeof(msg),
             "Pressure: %ld\r\nTemperature: %ld\r\n",
             (long)pressure, (long)temperature);
    HAL_UART_Transmit(&huart2, msg, sizeof(msg), 20);

    osDelay(10);

    sniprintf(msg, sizeof(msg),
              "Prom1: %ld\r\n Prom2: %ld\r\n Prom3: %ld\r\n Prom4: %ld\r\n Prom5: %ld\r\n Prom6: %ld\r\n Prom7: %ld\r\n Prom8: %ld\r\n",
              prom[0], prom[1], prom[2], prom[3], prom[4], prom[5], prom[6], prom[7]);

    osDelay(10);

    // Pace the loop so you yield CPU (depends on your MS5611 OSR)
    osDelay(pdMS_TO_TICKS(20));
  }
}