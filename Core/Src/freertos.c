#include "FreeRTOS.h"
#include "Drivers/mmc5983ma.h"
#include "Drivers/ms5611.h"
#include "Drivers/lsm6dso32_app.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "spi.h"
#include "stdint.h"
#include <stdio.h>
#include <math.h>

#define MAG_PIN_INT MAG_PIN_INT_VAL
const float X_OFFSET = 0.077f;
const float Y_OFFSET = 0.147f;
const float SEA_LEVEL_PA = 101325.0f; // Standard atmospheric pressure

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;
SemaphoreHandle_t xImuAccelReadySemaphore;
SemaphoreHandle_t xImuGyroReadySemaphore;

osThreadId_t readSensorTaskHandle;
const osThreadAttr_t readSensorTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 8,
    .priority = (osPriority_t)osPriorityNormal,
};

void ReadSensorTask(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
  gI2c1Mutex = xSemaphoreCreateMutex();
  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  xImuAccelReadySemaphore = xSemaphoreCreateBinary();
  xImuGyroReadySemaphore = xSemaphoreCreateBinary();

  configASSERT(gI2c1Mutex);
  configASSERT(xMagDataReadySemaphore);
  configASSERT(xImuAccelReadySemaphore);
  configASSERT(xImuGyroReadySemaphore);
  readSensorTaskHandle = osThreadNew(ReadSensorTask, NULL, &readSensorTask_attributes);
}

void ReadSensorTask(void *argument)
{
  uint32_t freq = HAL_RCC_GetSysClockFreq();
  printf("CPU Freq: %lu Hz\r\n", freq);

  float magData[4] = {0.0f};
  uint16_t prom[8] = {0};
  int32_t pressure = 0;
  int32_t temperature = 0;
  float temp_c = 0;
  int counter = 0;
  float accel_data[3];
  float gyro_data[3];

  HAL_StatusTypeDef status = HAL_ERROR;
  HAL_StatusTypeDef magStatus = HAL_ERROR;
  HAL_StatusTypeDef baroStatus = HAL_ERROR;

  HAL_StatusTypeDef initStatus = magInit();
  HAL_StatusTypeDef baroReset = ms5611Reset();

  if (LSM6DSO32_Rocket_Init(&hspi1) != HAL_OK)
  {
    printf("LSM6DSO32 Init Failed!\r\n");
  }

  if (baroReset != HAL_OK)
  {
    printf("Barometer reset FAILED: %d\r\n", baroReset);
  }

  do
  {
    status = ms5611ReadPROM(prom);
    counter++;
    if (counter == 10)
    {
      break;
    }
    osDelay(pdMS_TO_TICKS(100));
  } while (status != HAL_OK);

  if (status != HAL_OK)
  {
    printf("Barometer Failed");
  }

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
    magStatus = magGetData(xMagDataReadySemaphore, magData);
    baroStatus = ms5611GetPressureAndTemp(prom, &pressure, &temperature);

    if (magStatus == HAL_OK)
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

    if (baroStatus == HAL_OK)
    {
      temp_c = (float)temperature / 100.0f;
      printf("Pressure: %ld, Temp: %0.2f", pressure, temp_c);
    }

    if (xSemaphoreTake(xImuAccelReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Accel(accel_data);
      // Now accel_data[0,1,2] contains values in mg
      printf("A[mg]: %.1f, %.1f, %.1f\r\n", accel_data[0], accel_data[1], accel_data[2]);
    }

    // Run Gyro processing when INT2 (Pin 2) triggers
    if (xSemaphoreTake(xImuGyroReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Gyro(gyro_data);
      // Now gyro_data[0,1,2] contains values in dps
      printf("G[dps]: %.1f, %.1f, %.1f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    osDelay(100);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == GPIO_PIN_3)
  { // Magnetometer
    xSemaphoreGiveFromISR(xMagDataReadySemaphore, &xHigherPriorityTaskWoken);
  }
  else if (GPIO_Pin == GPIO_PIN_0)
  { // LSM Accel (INT1)
    xSemaphoreGiveFromISR(xImuAccelReadySemaphore, &xHigherPriorityTaskWoken);
  }
  else if (GPIO_Pin == GPIO_PIN_2)
  { // LSM Gyro (INT2)
    xSemaphoreGiveFromISR(xImuGyroReadySemaphore, &xHigherPriorityTaskWoken);
  }

  /* Force a context switch if a higher priority task was woken */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}