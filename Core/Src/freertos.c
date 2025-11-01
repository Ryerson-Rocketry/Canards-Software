#include "FreeRTOS.h"
#include "i2c.h"
#include "mmc5983ma.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "stdint.h"
#include <stdio.h>

#define MAG_PIN_INT MAG_PIN_INT_VAL

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

  float magData[4] = {0.0f};
  uint8_t uartTxBuffer[100];
  HAL_StatusTypeDef status = HAL_ERROR;
  int len;

  HAL_StatusTypeDef initStatus = magInit();
  osDelay(1);

  if (initStatus != HAL_OK)
  {
    uint8_t errorMsg[50];
    int len = snprintf((char *)errorMsg, sizeof(errorMsg),
                       "magInit FAILED! Status: %d\r\n", initStatus);

    HAL_UART_Transmit(&huart2, errorMsg, len, 500);

    // Halt the program
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      osDelay(100);
    }
  }
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  for (;;)
  {
    status = magGetData(xMagDataReadySemaphore, magData);

    if (status == HAL_OK)
    {
      len = snprintf((char *)uartTxBuffer, sizeof(uartTxBuffer),
                     "Mag X: %.3f, Y: %.3f, Z: %.3f\r\n",
                     magData[0], magData[1], magData[2]);
    }
    else
    {
      len = snprintf((char *)uartTxBuffer, sizeof(uartTxBuffer),
                     "magGetData failed! Status: %d\r\n", status);
    }

    HAL_UART_Transmit(&huart2, uartTxBuffer, len, 100);

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(10);
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