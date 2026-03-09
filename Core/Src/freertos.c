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
#include <stdbool.h>
#include "iwdg.h"
#include <string.h>
#include "fatfs.h"
#include <math.h>

#define MAG_PIN_INT MAG_PIN_INT_VAL
#define PI 3.14159265358979323846f
#define GPS_BUF_SIZE 128

const float MAG_X_OFFSET = 0.077f;
const float MAG_Y_OFFSET = 0.147f;
const float SEA_LEVEL_PA = 101325.0f; // Standard atmospheric pressure

extern FATFS SDFatFS;
extern FIL SDFile;
extern char SDPath[4];
extern SD_HandleTypeDef hsd;
bool sdInitialized = false;

const char FLIGHT_DATA_FILE_NAME[] = "LOG.DAT";
char gps_buffer[GPS_BUF_SIZE];

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t xImuAccelReadySemaphore;
SemaphoreHandle_t xImuGyroReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;
SemaphoreHandle_t gSpi2Mutex;

osThreadId_t readSensorTaskHandle;
osThreadId_t altEstTaskHandle;
osThreadId_t launchDetTaskHandle;
osThreadId_t dataStoreTaskHandle;
osThreadId_t gpsRetrieveTaskHandle;
osThreadId_t heartbeatTaskHandle;
osThreadId_t controlTaskHandle;

FlightState_t currentFlightState = STATE_PAD;
RawSensorData_t currentRawData;
RocketState_t currentRocketState;
SDCardDataFormat_t snapshot;

volatile bool readSensorTask = false;
volatile bool altEstTask = false;
volatile bool launchDetTask = false;
volatile bool dataStoreTask = false;
volatile bool gpsRetrieveTask = false;
volatile bool controlTask = false;

const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask", .stack_size = 1024 * 2, .priority = osPriorityAboveNormal6};
const osThreadAttr_t altEstTask_attributes = {
    .name = "altTask", .stack_size = 512 * 2, .priority = osPriorityAboveNormal4};
const osThreadAttr_t launchDetTask_attributes = {
    .name = "launchTask", .stack_size = 256 * 2, .priority = osPriorityAboveNormal2};
const osThreadAttr_t dataStoreTask_attributes = {
    .name = "storeTask", .stack_size = 1024 * 4, .priority = osPriorityNormal};
const osThreadAttr_t gpsRetrieveTask_attributes = {
    .name = "retrieveGpsCoords", .stack_size = 256 * 2, .priority = osPriorityAboveNormal1};
const osThreadAttr_t controlTask_attributes = {
    .name = "controlCanards", .stack_size = 512 * 2, .priority = osPriorityAboveNormal3};
const osThreadAttr_t heartbeat_attributes = {
    .name = "wdgTask", .stack_size = 256 * 2, .priority = osPriorityBelowNormal3};

void vReadSensorTask(void *argument);
void vAltEstTask(void *argument);
void vLaunchDetTask(void *argument);
void vDataStoreTask(void *argument);
void vGpsRetrieveTask(void *argument);
void vControlTask(void *argument);
void vHeartbeatTask(void *argument);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
  gI2c1Mutex = xSemaphoreCreateMutex();
  gSpi2Mutex = xSemaphoreCreateMutex();
  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  xImuAccelReadySemaphore = xSemaphoreCreateBinary();
  xImuGyroReadySemaphore = xSemaphoreCreateBinary();

  configASSERT(gI2c1Mutex);
  configASSERT(gSpi2Mutex);
  configASSERT(xMagDataReadySemaphore);
  configASSERT(xImuAccelReadySemaphore);
  configASSERT(xImuGyroReadySemaphore);

  readSensorTaskHandle = osThreadNew(vReadSensorTask, NULL, &readSensorTask_attributes);
  altEstTaskHandle = osThreadNew(vAltEstTask, NULL, &altEstTask_attributes);
  launchDetTaskHandle = osThreadNew(vLaunchDetTask, NULL, &launchDetTask_attributes);
  dataStoreTaskHandle = osThreadNew(vDataStoreTask, NULL, &dataStoreTask_attributes);
  gpsRetrieveTaskHandle = osThreadNew(vGpsRetrieveTask, NULL, &gpsRetrieveTask_attributes);
  controlTaskHandle = osThreadNew(vControlTask, NULL, &controlTask_attributes);
  heartbeatTaskHandle = osThreadNew(vHeartbeatTask, NULL, &heartbeat_attributes);
}

void vReadSensorTask(void *argument)
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
    magStatus = magGetData(xMagDataReadySemaphore, currentRawData.mag);
    ms5611Run(prom, &currentRawData.pressure, &currentRawData.temperature);

    if (magStatus == HAL_OK)
    {
      float x_cal = currentRawData.mag[0] - MAG_X_OFFSET;
      float y_cal = currentRawData.mag[1] - MAG_Y_OFFSET;

      float heading_rad = atan2f(y_cal, x_cal);
      float heading_deg = heading_rad * (180.0f / PI);

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

    xTaskNotifyGive(launchDetTaskHandle);
    xTaskNotifyGive(altEstTaskHandle);

    readSensorTask = true;
    osDelay(10);
  }
}

void vAltEstTask(void *argument)
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
  uint32_t notification;

  for (;;)
  {
    notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (notification == 0)
    {
      altEstTask = true;
      continue;
    }

    if (currentFlightState == STATE_PAD)
    {
      currentRocketState.position = 0.0f;
      currentRocketState.velocity = 0.0f;
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

    currentRocketState.timestamp = osKernelGetTickCount();
    currentRocketState.acceleration = accel_in_m_s2;
    currentRocketState.rpy[0] += currentRawData.gyro[2] * dt;
    currentRocketState.rpy[1] += currentRawData.gyro[0] * dt;
    currentRocketState.rpy[2] += currentRawData.gyro[1] * dt;
    currentRocketState.tilt_angle = sqrtf(powf(currentRocketState.rpy[1], 2) + powf(currentRocketState.rpy[2], 2));

    xTaskNotifyGive(controlTaskHandle);
    xTaskNotifyGive((TaskHandle_t)dataStoreTaskHandle);
    altEstTask = true;
  }
}

void vLaunchDetTask(void *argument)
{

  float accel_z;
  static uint32_t accel_start_time = 0;
  bool threshold_active = false;
  static uint32_t burnoutStartTime = 0;
  static bool launch_recorded = false;
  static uint32_t canardActivationTime = 0;

  for (;;)
  {

    uint32_t notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (notification == 0)
    {
      launchDetTask = true;
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
      // rpy
      currentRocketState.rpy[0] = 0.0f;
      currentRocketState.rpy[1] = 0.0f;
      currentRocketState.rpy[2] = 0.0f;
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
        canardActivationTime = now;
      }
      break;
    case STATE_CANARDS_ACTIVATE:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      if (currentRocketState.velocity <= 50.0f && (now - canardActivationTime) >= pdMS_TO_TICKS(15000))
      {
        currentFlightState = STATE_DESCENT;
      }
      break;

    case STATE_DESCENT:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
      break;

    default:
      printf("Invalid state somehow...");
      break;
    }
    launchDetTask = true;
  }
}

void vDataStoreTask(void *argument)
{
  UINT bytesWritten;
  uint32_t syncCounter = 0;

  if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 1) != FR_OK)
  {
    printf("Unable to mount disk\n");
    sdInitialized = false;
  }

  FRESULT res = f_open(&SDFile, FLIGHT_DATA_FILE_NAME, FA_OPEN_APPEND | FA_WRITE);
  if (res == FR_OK)
  {
    sdInitialized = true;
    printf("SD File Ready: Writing Only\n");
    printf("SD Card Information:\n");
    printf("Block size  : %lu\n", hsd.SdCard.BlockSize);
    printf("Block nmbr  : %lu\n", hsd.SdCard.BlockNbr);
    printf("Card size   : %lu MB\n", (hsd.SdCard.BlockSize * hsd.SdCard.BlockNbr) / 1024 / 1024);
  }
  else
  {
    printf("Critical: Could not open file for writing\n");
    sdInitialized = false;
  }

  for (;;)
  {
    uint32_t notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (sdInitialized && notification > 0)
    {
      // Copy current data to snapshot for thread-safe writing
      taskENTER_CRITICAL();
      memcpy(snapshot.accel, currentRawData.accel, sizeof(snapshot.accel));
      memcpy(snapshot.gyro, currentRawData.gyro, sizeof(snapshot.gyro));
      memcpy(snapshot.mag, currentRawData.mag, sizeof(snapshot.mag));
      memcpy(snapshot.gps, gps_buffer, sizeof(gps_buffer));
      memcpy(snapshot.rpy, currentRocketState.rpy, sizeof(currentRocketState.rpy));
      taskEXIT_CRITICAL();

      snapshot.pressure = currentRawData.pressure;
      snapshot.position = currentRocketState.position;
      snapshot.velocity = currentRocketState.velocity;
      snapshot.tiltAngle = currentRocketState.tilt_angle;
      snapshot.timestamp = osKernelGetTickCount();

      if (f_write(&SDFile, &snapshot, sizeof(snapshot), &bytesWritten) == FR_OK)
      {
        // This ensures data is saved even if the battery disconnects on landing.
        if (++syncCounter >= 50)
        {
          f_sync(&SDFile);
          syncCounter = 0;
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
      }
    }
    dataStoreTask = true;
  }
}

void vGpsRetrieveTask(void *argument)
{

  for (;;)
  {
    if (currentFlightState == STATE_PAD)
    {
      // pass null terminator
      snapshot.gps[0] = '\0';
    }

    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      // read gps

      xSemaphoreGive(gSpi2Mutex);
    }

    gpsRetrieveTask = true;
    osDelay(100);
  }
}

void vControlTask(void *argument)
{
  float setPoint = 0.0f;
  const float Kp = 1.5f;
  const float Ki = 0.05f;
  const float Kd = 0.2f;

  float rollError = 0;
  static float integral = 0;
  float derivative, output = 0;
  uint32_t activationStartTime = 0;
  bool startTimeCaptured = false;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (currentFlightState == STATE_CANARDS_ACTIVATE)
    {
      setPoint = 0.0f;
      integral = 0;
      output = 0;

      if (!startTimeCaptured)
      {
        activationStartTime = osKernelGetTickCount();
        startTimeCaptured = true;
        integral = 0;
      }

      uint32_t elapsedMillis = osKernelGetTickCount() - activationStartTime;

      if ((elapsedMillis % 10000) < 5000)
      {
        setPoint = 0.0f;
      }
      else
      {
        setPoint = 90.0f;
      }

      rollError = setPoint - currentRocketState.rpy[0];

      if (fabsf(output) < 20.0f)
      {
        integral += rollError * 0.01f;
      }

      derivative = currentRawData.gyro[2];

      output = (Kp * rollError) + (Ki * integral) - (Kd * derivative);

      if (output > 20.0f)
        output = 20.0f;
      if (output < -20.0f)
        output = -20.0f;

      servoMove(output);
    }
    else
    {
      startTimeCaptured = false;
      setPoint = 0.0f;
      integral = 0;
      output = 0;
    }

    controlTask = true;
  }
}

void vHeartbeatTask(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if (readSensorTask && altEstTask && launchDetTask && dataStoreTask && gpsRetrieveTask)
    {
      HAL_IWDG_Refresh(&hiwdg); // refresh the watchdog
      // reset flags for the next check
      readSensorTask = false;
      altEstTask = false;
      launchDetTask = false;
      dataStoreTask = false;
      gpsRetrieveTask = false;
      controlTask = false;
    }
    osDelay(pdMS_TO_TICKS(100));
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

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int _write(int file, char *ptr, int len)
{
  int i = 0;
  for (i = 0; i < len; i++)
  {
    ITM_SendChar((*ptr++));
  }
  return len;
}