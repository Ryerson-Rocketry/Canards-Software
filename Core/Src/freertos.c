#include "FreeRTOS.h"
#include "Drivers/mmc5983ma.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "stdint.h"
#include <stdio.h>
#include <math.h>

#define MAG_PIN_INT MAG_PIN_INT_VAL
const float X_OFFSET = 0.077f;
const float Y_OFFSET = 0.147f;

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 8,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
  gI2c1Mutex = xSemaphoreCreateMutex();
  configASSERT(gI2c1Mutex);

  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  configASSERT(xMagDataReadySemaphore);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

void StartDefaultTask(void *argument)
{
  uint32_t freq = HAL_RCC_GetSysClockFreq();
  printf("CPU Freq: %lu Hz\r\n", freq);
  float magData[4] = {0.0f};
  HAL_StatusTypeDef status = HAL_ERROR;

  HAL_StatusTypeDef initStatus = magInit();
  osDelay(1);

  if (initStatus != HAL_OK)
  {
    printf("magInit FAILED! Status: %d\r\n", initStatus);

    // Halt the program
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      osDelay(100);
    }
  }
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  for (;;)
  {
    status = magGetData(xMagDataReadySemaphore, magData);

    if (status == HAL_OK)
    {
      // Apply Calibration: Subtract the Hard Iron offsets
      float x_cal = magData[0] - X_OFFSET;
      float y_cal = magData[1] - Y_OFFSET;

      // Calculate heading
      float heading_rad = atan2f(y_cal, x_cal);
      float heading_deg = heading_rad * (180.0f / 3.1415926535f);

      // Normalize to 0-360
      if (heading_deg < 0)
      {
        heading_deg += 360.0f;
      }

      printf("Heading: %.2f | X: %.3f, Y: %.3f\r\n",
             heading_deg, x_cal, y_cal);
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    osDelay(100);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == MAG_PIN_INT)
  {
    xSemaphoreGiveFromISR(xMagDataReadySemaphore, &xHigherPriorityTaskWoken);

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}