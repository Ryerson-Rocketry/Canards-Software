#include "FreeRTOS.h"
#include "Drivers/mmc5983ma.h"
#include "Drivers/ms5611.h"
#include "Drivers/lsm6dso32_app.h"
#include "StateEstimation/altitude_estimation.h"
#include "Utils/math_utils.h"
#include "Defs/states.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "spi.h"
#include "iwdg.h"
#include "stdint.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "iwdg.h"
#include <string.h>
#include "ff.h"

#define MAG_PIN_INT MAG_PIN_INT_VAL

const float MAG_X_OFFSET = 0.077f;
const float MAG_Y_OFFSET = 0.147f;
const float SEA_LEVEL_PA = 101325.0f; // Standard atmospheric pressure

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;
SemaphoreHandle_t xImuAccelReadySemaphore;
SemaphoreHandle_t xImuGyroReadySemaphore;

osThreadId_t readSensorTaskHandle;
osThreadId_t altitudeEstimationTaskHandle;
osThreadId_t launchDetectionTaskHandle;
osThreadId_t heartBeatMonitorTaskHandle;
osThreadId_t storeDataTaskHandle;

FlightState_t currentFlightState = STATE_PAD;
RawSensorData_t currentRawData;
RocketState_t currentRocketState;
SDCardDataFormat_t snapshot;

volatile bool readSensorTask = false;
volatile bool altitudeEstimationTask = false;
volatile bool launchDetectionTask = false;
volatile bool storeDataTask = false;

const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask",
    .stack_size = 1024 * 8,
    .priority = (osPriority_t)osPriorityAboveNormal5,
};

const osThreadAttr_t altitudeEstimationTask_attributes = {
    .name = "altitudeEstimationTask",
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityAboveNormal3,
};

const osThreadAttr_t launchDetectionTask_attributes = {
    .name = "launchDetectionTask",
    .stack_size = 128 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t heartBeatMonitorTask_attributes = {
    .name = "watchdawgTask",
    .stack_size = 256 * 2,
    .priority = (osPriority_t)osPriorityBelowNormal1,
};

const osThreadAttr_t storeDataTask_attributes = {
    .name = "storeDataToSDCard",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal2,
};

void ReadSensorTask(void *argument);
void AltitudeEstimationTask(void *argument);
void LaunchDetectionTask(void *argument);
void HeartBeatMonitorTask(void *argument);
void StoreDataTask(void *argument);

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
  altitudeEstimationTaskHandle = osThreadNew(AltitudeEstimationTask, NULL, &altitudeEstimationTask_attributes);
  launchDetectionTaskHandle = osThreadNew(LaunchDetectionTask, NULL, &launchDetectionTask_attributes);
  heartBeatMonitorTaskHandle = osThreadNew(HeartBeatMonitorTask, NULL, &heartBeatMonitorTask_attributes);
  storeDataTaskHandle = osThreadNew(StoreDataTask, NULL, &storeDataTask_attributes);
}

void ReadSensorTask(void *argument)
{
  uint32_t freq = HAL_RCC_GetSysClockFreq();
  printf("CPU Freq: %lu Hz\r\n", freq);

  uint16_t prom[8] = {0};
  float temp_c = 0;
  int counter = 0;

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
  HAL_NVIC_EnableIRQ(EXTI0_IRQn); // For LSM Accel (Pin 0)
  HAL_NVIC_EnableIRQ(EXTI2_IRQn); // For LSM Gyro (Pin 2)
  HAL_NVIC_EnableIRQ(EXTI3_IRQn); // For Mag (Pin 3)

  for (;;)
  {
    int32_t pressure_raw = 0;
    int32_t temperature_raw = 0;

    magStatus = magGetData(xMagDataReadySemaphore, currentRawData.mag);
    baroStatus = ms5611GetPressureAndTemp(prom, &pressure_raw, &temperature_raw);

    if (baroStatus == HAL_OK)
    {
      currentRawData.pressure = pressure_raw;
      currentRawData.temperature = temperature_raw;
    }

    if (magStatus == HAL_OK)
    {
      float x_cal = currentRawData.mag[0] - MAG_X_OFFSET;
      float y_cal = currentRawData.mag[1] - MAG_Y_OFFSET;

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
      temp_c = (float)currentRawData.temperature / 100.0f;
      printf("Pressure: %.2f, Temp: %.2f\r\n", currentRawData.pressure, temp_c);
    }

    if (xSemaphoreTake(xImuAccelReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Accel(currentRawData.accel);
      // Now accel_data[0,1,2] contains values in mg
      printf("A[mg]: %.1f, %.1f, %.1f\r\n", currentRawData.accel[0], currentRawData.accel[1], currentRawData.accel[2]);
    }

    // Run Gyro processing when INT2 (Pin 2) triggers
    if (xSemaphoreTake(xImuGyroReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Gyro(currentRawData.gyro);
      // Now gyro_data[0,1,2] contains values in dps
      printf("G[dps]: %.1f, %.1f, %.1f\r\n", currentRawData.gyro[0], currentRawData.gyro[1], currentRawData.gyro[2]);
    }

    currentRawData.timestamp = osKernelGetTickCount();

    xTaskNotifyGive(launchDetectionTaskHandle);
    xTaskNotifyGive(altitudeEstimationTaskHandle);

    readSensorTask = true;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    osDelay(pdMS_TO_TICKS(10));
  }
}

void AltitudeEstimationTask(void *argument)
{
  float dt = 0.0f;
  float processNoise[2][2] = {{0.01f, 0.0f}, {0.0f, 0.01f}};
  float systemCov[2][2] = {{10.0f, 0.0f}, {0.0f, 10.0f}};
  float stateTransition[2][2] = {
      {1.0f, dt},
      {0.0f, 1.0f}};
  float baroAltVar = 0.5f;
  uint32_t last_tick = osKernelGetTickCount();
  float accel_in_m_s2;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (currentFlightState == STATE_PAD)
    {
      // Optional: Keep velocity/position at exactly 0.0
      currentRocketState.position = 0.0f;
      currentRocketState.velocity = 0.0f;
      altitudeEstimationTask = true;
      continue;
    }

    uint32_t current_tick = osKernelGetTickCount();
    currentRocketState.launch_tick = current_tick;
    dt = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    // safety for first run
    if (dt <= 0.0f)
    {
      dt = 0.01f;
    }

    accel_in_m_s2 = (currentRawData.accel[2] - 1000.0f) * 0.00980665f;
    altPredict(dt, &currentRocketState.position, &currentRocketState.velocity, accel_in_m_s2, processNoise, systemCov, stateTransition);
    altUpdate(&currentRocketState.position, &currentRocketState.velocity, pressureToAltitude(currentRawData.pressure, SEA_LEVEL_PA), systemCov, baroAltVar);

    // update struct
    currentRocketState.timestamp = osKernelGetTickCount();
    currentRocketState.acceleration = accel_in_m_s2;
    currentRocketState.pitch += currentRawData.gyro[0] * dt;
    currentRocketState.yaw += currentRawData.gyro[1] * dt;
    currentRocketState.roll += currentRawData.gyro[2] * dt;
    currentRocketState.tilt_angle = sqrtf(powf(currentRocketState.pitch, 2) + powf(currentRocketState.yaw, 2));

    xTaskNotifyGive(storeDataTaskHandle);
    altitudeEstimationTask = true;
  }
}

void LaunchDetectionTask(void *argument)
{

  float accel_z;
  uint32_t accel_start_time = 0;
  bool threshold_active = false;
  uint32_t burnoutStartTime = 0;
  static bool launch_recorded = false;

  for (;;)
  {

    uint32_t notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (notification == 0)
    {
      continue;
    }

    uint32_t now = osKernelGetTickCount();
    accel_z = currentRawData.accel[2] - 1000.0f;

    if (currentFlightState != STATE_PAD && !launch_recorded)
    {
      currentRocketState.launch_tick = osKernelGetTickCount();
      launch_recorded = true;
    }

    switch (currentFlightState)
    {
    case STATE_PAD:
      currentRocketState.pitch = 0.0f;
      currentRocketState.yaw = 0.0f;
      currentRocketState.roll = 0.0f;
      currentRocketState.tilt_angle = 0.0f;

      if (accel_z >= 4000.0f)
      {
        if (!threshold_active)
        {
          accel_start_time = now;
          threshold_active = true;
        }
        if ((now - accel_start_time) >= pdMS_TO_TICKS(100))
        {
          currentFlightState = STATE_BOOST;
          printf("LAUNCH DETECTED at %lu ms\r\n", now);
        }
      }
      else
      {
        threshold_active = false;
      }
      break;

    case STATE_BOOST:
      if ((now - accel_start_time) >= pdMS_TO_TICKS(7800) && accel_z <= 1000.0f)
      {
        burnoutStartTime = now;
        currentFlightState = STATE_BURNOUT;
        printf("BURNOUT at %lu ms\r\n", now);
      }
      break;

    case STATE_BURNOUT:
      if ((now - burnoutStartTime) >= pdMS_TO_TICKS(1000))
      {
        currentFlightState = STATE_CANARDS_ACTIVATE;
        printf("CANARDS ACTIVE\r\n");
      }
      break;
    case STATE_CANARDS_ACTIVATE:
      // turn the gpio pin to high for raspberry pi video recording
      break;
    default:
      printf("Invalid state somehow...");
      break;
    }
    launchDetectionTask = true;
  }
}

void StoreDataTask(void *argument)
{
  FATFS SDFatFS;
  FIL SDFile;
  char SDPath[4];
  bool sdInitialized = false;

  for (;;)
  {

    uint32_t notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (notification == 0)
    {
      continue;
    }

    // can't use = op to copy an array to another, so we must memory copy everything to snapshot
    memcpy(snapshot.accel, currentRawData.accel, sizeof(snapshot.accel));
    memcpy(snapshot.gyro, currentRawData.gyro, sizeof(snapshot.gyro));
    memcpy(snapshot.mag, currentRawData.mag, sizeof(snapshot.mag));

    snapshot.pressure = currentRawData.pressure;
    snapshot.position = currentRocketState.position;
    snapshot.velocity = currentRocketState.velocity;
    snapshot.tiltAngle = currentRocketState.tilt_angle;
    snapshot.pitch = currentRocketState.pitch;
    snapshot.yaw = currentRocketState.yaw;
    snapshot.roll = currentRocketState.roll;
    snapshot.timestamp = osKernelGetTickCount();

    // write binary of snapshot to SD card

    storeDataTask = true;
  }
}

void HeartBeatMonitorTask(void *argument)
{
  for (;;)
  {
    if (readSensorTask && altitudeEstimationTask && launchDetectionTask && storeDataTask)
    {
      HAL_IWDG_Refresh(&hiwdg); // refresh the watchdog
      // reset flags for the next check
      readSensorTask = false;
      altitudeEstimationTask = false;
      launchDetectionTask = false;
      storeDataTask = false;
    }
    osDelay(20);
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

int32_t lsm_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  // Use the correct label for the IMU CS pin
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(handle, &reg, 1, 100);
  HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 100);

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  return 0;
}

int32_t lsm_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  uint8_t addr = reg | 0x80;
  HAL_StatusTypeDef status;
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  // Send address
  status = HAL_SPI_Transmit(hspi, &addr, 1, 100);

  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(hspi, bufp, len, 100);
  }

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  return (status == HAL_OK) ? 0 : -1;
}