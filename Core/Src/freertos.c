#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "BMI088.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

// Semaphores
SemaphoreHandle_t xImuDataReadySemaphore;

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
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

void StartDefaultTask();
void calculateOrientation();

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{

  osDelay(1);

  xImuDataReadySemaphore = xSemaphoreCreateBinary();
  configASSERT(xImuDataReadySemaphore);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  retrieveOrientationTaskHandle = osThreadNew(calculateOrientation, NULL, &retrieveOrientation_attributes);
}

// Default task function
void StartDefaultTask()
{
  for (;;)
  {
    osDelay(10);
  }
}

void calculateOrientation()
{

  float accelData[4];
  float gyroData[4];

  char uartBuf[100];

  HAL_StatusTypeDef init_status = bmi088Init();

  if (init_status != HAL_OK)
  {
    // Initialization FAILED.
    // Blink the LED very fast to tell us something is wrong.
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      osDelay(50); // Fast error blink
    }
  }

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  for (;;)
  {

    // retrieve sensor data here
    if (xSemaphoreTake(xImuDataReadySemaphore, portMAX_DELAY) != pdTRUE)
    {
      continue;
    }

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    bmi088ReadAccelerometer(accelData);
    bmi088ReadGyroscope(gyroData);

    snprintf(uartBuf, sizeof(uartBuf),
             "Accel: %.2f, %.2f, %.2f\r\nGyro: %.2f, %.2f, %.2f\r\n",
             accelData[1], accelData[2], accelData[3], gyroData[1], gyroData[2], gyroData[3]);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

    osDelay(10);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == Accelerometer_INT_Pin || GPIO_Pin == Gyroscope_INT_Pin)
  {
    xSemaphoreGiveFromISR(xImuDataReadySemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}