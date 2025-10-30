#include "FreeRTOS.h"
#include "mmc5983ma.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"

#define MAG_PIN_INT MAG_PIN_INT_VAL

SemaphoreHandle_t xMagDataReadySemaphore;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{

  magInit();
  osDelay(1);

  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  configASSERT(xMagDataReadySemaphore);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

void StartDefaultTask(void *argument)
{
  for (;;)
  {
    float magData[4];
    magGetData(xMagDataReadySemaphore, magData);
    osDelay(1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // flag to see if there is a higher priority task
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == MAG_PIN_INT)
  {
    // if there is a higher priority task, xHigherPriorityTaskWoken becomes true
    xSemaphoreGiveFromISR(xMagDataReadySemaphore, &xHigherPriorityTaskWoken);
    // if higher priority task, switch to it after interrupt is done
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
