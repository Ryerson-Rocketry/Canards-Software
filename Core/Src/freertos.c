#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "mmc5983ma.h"

#define GPIO_PIN_MAG_INT GPIO_PIN_MAG_INT_VAL

// Semaphores
SemaphoreHandle_t xMagDataReadySemaphore;

// Task handle and attributes
osThreadId_t defaultTaskHandle;
osThreadId_t retrieveOrientationTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t retrieveOrientation_attributes = {
    .name = "getOrientation",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

void StartDefaultTask();
void calculateOrientation();

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{

  mmc5983maInit();
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


void calculateOrientation()
{

  // needs to be changed to input it into Madgwick filter

  float magData[4];

  char uartBuf[100];

  for (;;)
  {
    mmc5983maReadMagnetometer(xMagDataReadySemaphore, magData);

    snprintf(uartBuf, sizeof(uartBuf),
             "Mag: %.2f, %.2f, %.2f\r\n",
             magData[1], magData[2], magData[3]);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

    osDelay(1);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // flag to see if there is a higher priority task
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == GPIO_PIN_MAG_INT)
  {
    // if there is a higher priority task, xHigherPriorityTaskWoken becomes true
    xSemaphoreGiveFromISR(xMagDataReadySemaphore, &xHigherPriorityTaskWoken);
    // if higher priority task, switch to it after interrupt is done
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}