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
#include "Configs/flight_configs.h"
#include "Utils/servo.h"
#include "Tasks/tasks.h"

extern FATFS SDFatFS;
extern FIL SDFile;
extern char SDPath[4];
extern SD_HandleTypeDef hsd;
bool sdInitialized = false;

char gps_buffer[GPS_BUF_SIZE];

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t xImuAccelReadySemaphore;
SemaphoreHandle_t xImuGyroReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;
SemaphoreHandle_t gSpi2Mutex;

Rocket_States_t Rocket;

volatile bool readSensorTask = false;
volatile bool altEstTask = false;
volatile bool launchDetTask = false;
volatile bool dataStoreTask = false;
volatile bool gpsRetrieveTask = false;
volatile bool controlTask = false;

const float Kp = 1.5f;
const float Ki = 0.05f;
const float Kd = 0.2f;

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
    magStatus = magGetData(xMagDataReadySemaphore, Rocket.rawData.mag);
    ms5611Run(prom, &Rocket.rawData.pressure, &Rocket.rawData.temperature);

    if (magStatus == HAL_OK)
    {
      float x_cal = Rocket.rawData.mag[0] - MAG_X_OFFSET;
      float y_cal = Rocket.rawData.mag[1] - MAG_Y_OFFSET;

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
      temp_c = (float)Rocket.rawData.temperature / 100.0f;
      printf("Pressure: %.2f, Temp: %.2f\r\n", Rocket.rawData.pressure, temp_c);
    }

    if (xSemaphoreTake(xImuAccelReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Accel(Rocket.rawData.accel);
      // Now accel_data[0,1,2] contains values in mg
      printf("A[mg]: %.1f, %.1f, %.1f\r\n", Rocket.rawData.accel[0], Rocket.rawData.accel[1], Rocket.rawData.accel[2]);
    }

    // Run Gyro processing when INT2 (Pin 2) triggers
    if (xSemaphoreTake(xImuGyroReadySemaphore, 0) == pdTRUE)
    {
      LSM6DSO32_Read_Gyro(Rocket.rawData.gyro);
      // Now gyro_data[0,1,2] contains values in dps
      printf("G[dps]: %.1f, %.1f, %.1f\r\n", Rocket.rawData.gyro[0], Rocket.rawData.gyro[1], Rocket.rawData.gyro[2]);
    }

    Rocket.rawData.timestamp = osKernelGetTickCount();

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

    if (Rocket.flightState == STATE_PAD)
    {
      Rocket.estimate.position = 0.0f;
      Rocket.estimate.velocity = 0.0f;
    }

    uint32_t current_tick = osKernelGetTickCount();
    Rocket.estimate.launch_tick = current_tick;
    dt = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    // safety for first run
    if (dt <= 0.0f)
    {
      dt = 0.01f;
    }

    accel_in_m_s2 = (Rocket.rawData.accel[2] - 1000.0f) * 0.00980665f;
    altPredict(dt, &Rocket.estimate.position, &Rocket.estimate.velocity, accel_in_m_s2, processNoise, systemCov, stateTransition);
    altUpdate(&Rocket.estimate.position, &Rocket.estimate.velocity, pressureToAltitude(Rocket.rawData.pressure, SEA_LEVEL_PA), systemCov, baroAltVar);

    Rocket.estimate.timestamp = osKernelGetTickCount();
    Rocket.estimate.acceleration = accel_in_m_s2;
    Rocket.estimate.rpy[0] += Rocket.rawData.gyro[2] * dt;
    Rocket.estimate.rpy[1] += Rocket.rawData.gyro[0] * dt;
    Rocket.estimate.rpy[2] += Rocket.rawData.gyro[1] * dt;
    Rocket.estimate.tilt_angle = sqrtf(powf(Rocket.estimate.rpy[1], 2) + powf(Rocket.estimate.rpy[2], 2));

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
    accel_z = Rocket.rawData.accel[2] - 1000.0f;

    if (Rocket.flightState != STATE_PAD && !launch_recorded)
    {
      Rocket.estimate.launch_tick = osKernelGetTickCount();
      launch_recorded = true;
    }

    switch (Rocket.flightState)
    {
    case STATE_PAD:
      // rpy
      Rocket.estimate.rpy[0] = 0.0f;
      Rocket.estimate.rpy[1] = 0.0f;
      Rocket.estimate.rpy[2] = 0.0f;
      Rocket.estimate.tilt_angle = 0.0f;

      if (accel_z >= 4000.0f)
      {
        if (!threshold_active)
        {
          accel_start_time = now;
          threshold_active = true;
        }
        if ((now - accel_start_time) >= pdMS_TO_TICKS(100))
        {
          Rocket.flightState = STATE_BOOST;
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
        Rocket.flightState = STATE_BURNOUT;
        printf("BURNOUT at %lu ms\r\n", now);
      }
      break;

    case STATE_BURNOUT:
      if ((now - burnoutStartTime) >= pdMS_TO_TICKS(1000))
      {
        Rocket.flightState = STATE_CANARDS_ACTIVATE;
        printf("CANARDS ACTIVE\r\n");
        canardActivationTime = now;
      }
      break;
    case STATE_CANARDS_ACTIVATE:
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      if (Rocket.estimate.velocity <= 50.0f && (now - canardActivationTime) >= pdMS_TO_TICKS(15000))
      {
        Rocket.flightState = STATE_DESCENT;
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
      // Copy current data to Rocket.snapshot for thread-safe writing
      taskENTER_CRITICAL();
      memcpy(Rocket.snapshot.accel, Rocket.rawData.accel, sizeof(Rocket.snapshot.accel));
      memcpy(Rocket.snapshot.gyro, Rocket.rawData.gyro, sizeof(Rocket.snapshot.gyro));
      memcpy(Rocket.snapshot.mag, Rocket.rawData.mag, sizeof(Rocket.snapshot.mag));
      memcpy(Rocket.snapshot.gps, gps_buffer, sizeof(gps_buffer));
      memcpy(Rocket.snapshot.rpy, Rocket.estimate.rpy, sizeof(Rocket.estimate.rpy));
      taskEXIT_CRITICAL();

      Rocket.snapshot.pressure = Rocket.rawData.pressure;
      Rocket.snapshot.position = Rocket.estimate.position;
      Rocket.snapshot.velocity = Rocket.estimate.velocity;
      Rocket.snapshot.tiltAngle = Rocket.estimate.tilt_angle;
      Rocket.snapshot.timestamp = osKernelGetTickCount();

      if (f_write(&SDFile, &Rocket.snapshot, sizeof(Rocket.snapshot), &bytesWritten) == FR_OK)
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
    if (Rocket.flightState == STATE_PAD)
    {
      // pass null terminator
      Rocket.snapshot.gps[0] = '\0';
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

  float rollError = 0;
  static float integral = 0;
  float derivative, output = 0;
  uint32_t activationStartTime = 0;
  bool startTimeCaptured = false;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (Rocket.flightState == STATE_CANARDS_ACTIVATE)
    {

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

      rollError = setPoint - Rocket.estimate.rpy[0];

      if (fabsf(output) < 20.0f)
      {
        integral += rollError * 0.01f;
      }

      derivative = Rocket.rawData.gyro[2];

      output = (Kp * rollError) + (Ki * integral) - (Kd * derivative);

      if (output > 20.0f)
        output = 20.0f;
      if (output < -20.0f)
        output = -20.0f;

      // TODO: Ensure servo.c is compiled and linked in your build system
      // moveServo(output);
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