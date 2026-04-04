#include "FreeRTOS.h"
#include "Drivers/mmc5983ma.h"
#include "Drivers/lsm6dso32_app.h"
#include "StateEstimation/altitude_estimation.h"
#include "Utils/math_utils.h"
#include "Defs/states.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
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
#include "Drivers/rfm69.h"
#include "i2c.h"
#include "Drivers/ms5611.h"

extern FATFS SDFatFS;
extern FIL SDFile;
extern char SDPath[4];
extern SD_HandleTypeDef hsd;
bool sdInitialized = false;

SemaphoreHandle_t xMagDataReadySemaphore;
SemaphoreHandle_t xImuAccelReadySemaphore;
SemaphoreHandle_t xImuGyroReadySemaphore;
SemaphoreHandle_t gI2c1Mutex;
SemaphoreHandle_t gSpi1Mutex;
SemaphoreHandle_t gSpi2Mutex;

osThreadId_t readSensorTaskHandle;
osThreadId_t altEstTaskHandle;
osThreadId_t oriEstTaskHandle;
osThreadId_t launchDetTaskHandle;
osThreadId_t dataStoreTaskHandle;
osThreadId_t gpsRetrieveTaskHandle;
osThreadId_t radioTaskHandle;
osThreadId_t controlTaskHandle;
osThreadId_t heartbeatTaskHandle;

Rocket_States_t Rocket;

volatile bool readSensorTask = false;
volatile bool altEstTask = false;
volatile bool launchDetTask = false;
volatile bool dataStoreTask = false;
volatile bool radioTask = false;
volatile bool controlTask = false;
volatile bool oriEstTask = false;

const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask", .stack_size = 1024 * 2, .priority = osPriorityAboveNormal6};
const osThreadAttr_t altEstTask_attributes = {
    .name = "altTask", .stack_size = 512 * 2, .priority = osPriorityAboveNormal4};
const osThreadAttr_t oriEstTask_attributes = {
    .name = "altTask", .stack_size = 512 * 2, .priority = osPriorityAboveNormal3};
const osThreadAttr_t launchDetTask_attributes = {
    .name = "launchTask", .stack_size = 256 * 2, .priority = osPriorityAboveNormal2};
const osThreadAttr_t dataStoreTask_attributes = {
    .name = "storeTask", .stack_size = 1024 * 4, .priority = osPriorityNormal};
const osThreadAttr_t controlTask_attributes = {
    .name = "controlCanards", .stack_size = 512 * 2, .priority = osPriorityAboveNormal4};
const osThreadAttr_t heartbeat_attributes = {
    .name = "wdgTask", .stack_size = 256 * 2, .priority = osPriorityBelowNormal3};
const osThreadAttr_t radiotask_attributes = {
    .name = "radioTask", .stack_size = 1024 * 2, .priority = osPriorityNormal};

void vReadSensorTask(void *argument);
void vAltEstTask(void *argument);
void vOriEstTask(void *argument);
void vLaunchDetTask(void *argument);
void vDataStoreTask(void *argument);
void vControlTask(void *argument);
void vRadioTask(void *argument);
void vHeartbeatTask(void *argument);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
  gI2c1Mutex = xSemaphoreCreateMutex();
  gSpi1Mutex = xSemaphoreCreateMutex();
  gSpi2Mutex = xSemaphoreCreateMutex();
  xMagDataReadySemaphore = xSemaphoreCreateBinary();
  xImuAccelReadySemaphore = xSemaphoreCreateBinary();
  xImuGyroReadySemaphore = xSemaphoreCreateBinary();

  configASSERT(gI2c1Mutex);
  configASSERT(gSpi1Mutex);
  configASSERT(gSpi2Mutex);
  configASSERT(xMagDataReadySemaphore);
  configASSERT(xImuAccelReadySemaphore);
  configASSERT(xImuGyroReadySemaphore);

  readSensorTaskHandle = osThreadNew(vReadSensorTask, NULL, &readSensorTask_attributes);
  altEstTaskHandle = osThreadNew(vAltEstTask, NULL, &altEstTask_attributes);
  launchDetTaskHandle = osThreadNew(vLaunchDetTask, NULL, &launchDetTask_attributes);
  dataStoreTaskHandle = osThreadNew(vDataStoreTask, NULL, &dataStoreTask_attributes);
  controlTaskHandle = osThreadNew(vControlTask, NULL, &controlTask_attributes);
  radioTaskHandle = osThreadNew(vRadioTask, NULL, &radiotask_attributes);
  heartbeatTaskHandle = osThreadNew(vHeartbeatTask, NULL, &heartbeat_attributes);
}

void vReadSensorTask(void *argument)
{

  static float accel_filt[3] = {0.0f, 0.0f, 1000.0f};
  const float alpha_accel = 0.1f;
  static float gyro_filt[3] = {0.0f, 0.0f, 0.0f};
  const float alpha_gyro = 0.2f;
  Rocket_States_t Rocket = {0};

  magInit();
  LSM6DSO32_Rocket_Init(&hspi1);
  Barometer_init();
  osDelay(1000);

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  for (;;)
  {
    // I2C1: Magnetometer
    magGetData(xMagDataReadySemaphore, Rocket.rawData.mag);

    // SPI2: Barometer
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      int32_t p_mbar_x100 = Barometer_getPressure(true);
      int32_t t_centiC = Barometer_getTemp(false);

      /* Convert units */
      Rocket.rawData.pressure = (float)p_mbar_x100; // because mbar*100 = Pa
      Rocket.rawData.temperature = t_centiC / 100.0f;
      Rocket.estimate.position = 44330.0f * (1.0f - powf(Rocket.rawData.pressure / SEA_LEVEL_PA, 0.1903f));
      xSemaphoreGive(gSpi2Mutex);
    }

    // SPI1: IMU
    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if (xSemaphoreTake(xImuAccelReadySemaphore, 0) == pdTRUE)
      {
        LSM6DSO32_Read_Accel(Rocket.rawData.accel);
        for (int i = 0; i < 3; i++)
        {
          accel_filt[i] = (alpha_accel * Rocket.rawData.accel[i]) + ((1.0f - alpha_accel) * accel_filt[i]);
          Rocket.rawData.accel[i] = accel_filt[i];
        }
      }

      if (xSemaphoreTake(xImuGyroReadySemaphore, 0) == pdTRUE)
      {
        LSM6DSO32_Read_Gyro(Rocket.rawData.gyro);
        for (int i = 0; i < 3; i++)
        {
          // Formula: Smooth Value = (New * Alpha) + (Old * (1 - Alpha))
          gyro_filt[i] = (alpha_gyro * Rocket.rawData.gyro[i]) + ((1.0f - alpha_gyro) * gyro_filt[i]);

          if (fabsf(gyro_filt[i]) < 0.05f)
          {
            Rocket.rawData.gyro[i] = 0.0f;
          }
          else
          {
            Rocket.rawData.gyro[i] = gyro_filt[i];
          }
        }
      }
      xSemaphoreGive(gSpi1Mutex);
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
  // Process noise - accounts for accelerometer noise AND unmodeled dynamics
  // These values work well for rocket flights where there are aerodynamic effects
  // the simple kinematic model doesn't capture
  float processNoise[2][2] = {
      {0.01f, 0.0f}, // Position process noise (m²)
      {0.0f, 0.1f}   // Velocity process noise (m²/s²) - increased because velocity compounds errors faster
  };

  // Initial uncertainty - start with high uncertainty, filter will converge quickly
  static float systemCov[2][2] = {
      {10.0f, 0.0f}, // Initial altitude uncertainty (m²) - about ±3m standard deviation
      {0.0f, 25.0f}  // Initial velocity uncertainty (m²/s²) - about ±5 m/s standard deviation
  };

  // State transition matrix - pure kinematics, will be updated each cycle
  float stateTransition[2][2] = {
      {1.0f, 0.0f}, // Altitude carries forward
      {0.0f, 1.0f}  // Velocity carries forward
  };
  // Note: stateTransition[0][1] gets set to dt each iteration in your code

  // Barometer measurement noise - MS5611 in flight conditions
  // Using 1.0 accounts for vibration and dynamic pressure effects during flight
  const float baroAltVar = 1.0f; // Variance in m² (standard deviation ≈ 1.0m)

  static uint32_t last_tick = 0;
  float dt = 0.0f;

  float accel_in_m_s2;
  float baro_altitude;

  Rocket.estimate.tilt_angle = 90.0f;

  for (;;)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0)
    {
      altEstTask = true;
      continue;
    }

    uint32_t current_tick = osKernelGetTickCount();

    dt = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    if (dt < 0.001f)
      dt = 0.01f;
    if (dt > 0.1f)
      dt = 0.1f;

    stateTransition[0][1] = dt;

    if (Rocket.flightState == STATE_PAD)
    {
      static bool groundCaptured = false;
      if (!groundCaptured)
      {
        setGroundPressure(Rocket.rawData.pressure);
        groundCaptured = true;
      }
      Rocket.estimate.position = 0.0f;
      Rocket.estimate.velocity = 0.0f;
      systemCov[0][0] = 10.0f;
      systemCov[0][1] = 0.0f;
      systemCov[1][0] = 0.0f;
      systemCov[1][1] = 10.0f;
    }

    accel_in_m_s2 = (Rocket.rawData.accel[2] - 1000.0f) * 0.00980665f * cosf(Rocket.estimate.tilt_angle);
    if (fabsf(accel_in_m_s2) > 500.0f)
      accel_in_m_s2 = 0.0f;

    altPredict(dt, &Rocket.estimate.position, &Rocket.estimate.velocity,
               accel_in_m_s2, processNoise, systemCov, stateTransition);

    baro_altitude = getRelativeAltitude(Rocket.rawData.pressure);
    altUpdate(&Rocket.estimate.position, &Rocket.estimate.velocity,
              baro_altitude, systemCov, baroAltVar);

    // update rocket state estimate and gyro integration
    Rocket.estimate.timestamp = current_tick;
    Rocket.estimate.acceleration = accel_in_m_s2;
    Rocket.estimate.rpy[0] += Rocket.rawData.gyro[2] * dt;
    Rocket.estimate.rpy[1] += Rocket.rawData.gyro[0] * dt;
    Rocket.estimate.rpy[2] += Rocket.rawData.gyro[1] * dt;
    Rocket.estimate.tilt_angle = sqrtf(
        Rocket.estimate.rpy[1] * Rocket.estimate.rpy[1] +
        Rocket.estimate.rpy[2] * Rocket.estimate.rpy[2]);

    xTaskNotifyGive(controlTaskHandle);
    xTaskNotifyGive((TaskHandle_t)dataStoreTaskHandle);
    altEstTask = true;
  }
}

void vOriEstTask(void *argument)
{
}

void vLaunchDetTask(void *argument)
{
  float accel_z;
  static uint32_t accel_start_time = 0;
  static bool threshold_active = false;
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
  char csvBuffer[256];

  if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 1) != FR_OK)
  {
    sdInitialized = false;
    vTaskDelete(NULL);
    return;
  }

  if (f_open(&SDFile, FLIGHT_DATA_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
  {
    sdInitialized = false;
    vTaskDelete(NULL);
    return;
  }

  sdInitialized = true;
  if (f_size(&SDFile) == 0)
  {
    char *fileHeaders =
        "timestamp (ms), flight_state (enum), "
        "accel_x (centi-mg), accel_y (centi-mg), accel_z (centi-mg), "
        "gyro_x (centi-dps), gyro_y (centi-dps), gyro_z (centi-dps), "
        "mag_x (scaled x100), mag_y (scaled x100), mag_z (scaled x100), "
        "roll (centi-deg), pitch (centi-deg), yaw (centi-deg), "
        "tilt_angle (centi-deg), altitude (cm), velocity (cm/s), "
        "roll_error (centi-deg), pitch_error (centi-deg), pwm_angle (centi-deg)"
        "\r\n";
    f_puts(fileHeaders, &SDFile);
  }

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    taskENTER_CRITICAL();
    memcpy(Rocket.snapshot.accel, Rocket.rawData.accel, sizeof(Rocket.snapshot.accel));
    memcpy(Rocket.snapshot.gyro, Rocket.rawData.gyro, sizeof(Rocket.snapshot.gyro));
    memcpy(Rocket.snapshot.mag, Rocket.rawData.mag, sizeof(Rocket.snapshot.mag));
    memcpy(Rocket.snapshot.rpy, Rocket.estimate.rpy, sizeof(Rocket.snapshot.rpy));
    taskEXIT_CRITICAL();

    Rocket.snapshot.pressure = Rocket.rawData.pressure;
    Rocket.snapshot.position = Rocket.estimate.position;
    Rocket.snapshot.velocity = Rocket.estimate.velocity;
    Rocket.snapshot.tiltAngle = Rocket.estimate.tilt_angle;
    Rocket.snapshot.rollError = Rocket.control.rollError;
    Rocket.snapshot.pwmAngle = Rocket.control.pwmAngle;
    Rocket.snapshot.pitchError = Rocket.control.pitchError;
    Rocket.snapshot.flightState = Rocket.flightState;
    Rocket.snapshot.timestamp = osKernelGetTickCount();

    // Now format the snapshot into CSV
    int len = snprintf(csvBuffer, sizeof(csvBuffer), "%lu", Rocket.snapshot.timestamp);
    len += snprintf(csvBuffer, sizeof(csvBuffer), "%d", Rocket.snapshot.flightState);

#define APPEND_SCALED(value) \
  len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len, ",%ld", (int32_t)((value) * 100))
    APPEND_SCALED(Rocket.snapshot.accel[0]);
    APPEND_SCALED(Rocket.snapshot.accel[1]);
    APPEND_SCALED(Rocket.snapshot.accel[2]);
    APPEND_SCALED(Rocket.snapshot.gyro[0]);
    APPEND_SCALED(Rocket.snapshot.gyro[1]);
    APPEND_SCALED(Rocket.snapshot.gyro[2]);
    APPEND_SCALED(Rocket.snapshot.mag[0]);
    APPEND_SCALED(Rocket.snapshot.mag[1]);
    APPEND_SCALED(Rocket.snapshot.mag[2]);
    APPEND_SCALED(Rocket.snapshot.rpy[0]);
    APPEND_SCALED(Rocket.snapshot.rpy[1]);
    APPEND_SCALED(Rocket.snapshot.rpy[2]);
    APPEND_SCALED(Rocket.snapshot.tiltAngle);
    APPEND_SCALED(Rocket.snapshot.position);
    APPEND_SCALED(Rocket.snapshot.velocity);
    APPEND_SCALED(Rocket.snapshot.rollError);
    APPEND_SCALED(Rocket.snapshot.pitchError);
    APPEND_SCALED(Rocket.snapshot.pwmAngle);

    len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len, "\r\n");

#undef APPEND_SCALED

    if (len <= 0 || len >= (int)sizeof(csvBuffer))
    {
      dataStoreTask = true;
      continue;
    }

    if (f_write(&SDFile, csvBuffer, (UINT)len, &bytesWritten) != FR_OK || bytesWritten != (UINT)len)
    {
      dataStoreTask = true;
      continue;
    }

    if (++syncCounter >= 50)
    {
      f_sync(&SDFile);
      syncCounter = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    }

    // write queue to send data to vRadioTask

    dataStoreTask = true;
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
  float pitchSetPoint = 0.0f;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // NOTE: MUST CHANGE TO == STATE_CANARDS_ACTIVATE WHILE FLYING ROCKET
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
      Rocket.control.rollError = rollError;
      Rocket.control.setPoint = setPoint;
      Rocket.control.pitchError = pitchSetPoint - Rocket.estimate.rpy[1];

      if (fabsf(Rocket.control.pitchError) > 30)
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        controlTask = true;
        continue;
      }

      if (fabsf(output) < 20.0f)
      {
        integral += rollError * 0.01f;
      }

      derivative = Rocket.rawData.gyro[2];
      output = (Kp * rollError) + (Ki * integral) - (Kd * derivative);
      Rocket.control.pwmAngle = moveServo(output, Rocket.estimate.velocity, Rocket.rawData.pressure);
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

    if (readSensorTask && altEstTask && launchDetTask && dataStoreTask && controlTask && radioTask)
    {
      HAL_IWDG_Refresh(&hiwdg);
      readSensorTask = false;
      altEstTask = false;
      launchDetTask = false;
      dataStoreTask = false;
      controlTask = false;
      radioTask = false;
    }

    osDelay(pdMS_TO_TICKS(100));
  }
}

void vRadioTask(void *argument)
{

  // i2c scanner code here

  // tasks:
  // 1. write the I2C scanenr to get the ESP32 address, save it
  // 2. Initialize and use queue to send variable "len" from vDataStoreTask to vRadioTask
  // 3. Send the variable "len" to ESP32 using I2C
  // 4. retrieve data using esp32, then transmit it via radio

  if (address != 0x00)
  {
    foudnAddress = address;
  }

  printf(foundAddress);

  for (;;)
  {

    // queue here to retrieve data from vDataStoreTask we want the len string

    // i2c master transmit, string = len

    radioTask = true;
    osDelay(pdMS_TO_TICKS(100));
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == GPIO_PIN_3)
  {
    xSemaphoreGiveFromISR(xMagDataReadySemaphore, &xHigherPriorityTaskWoken);
  }
  else if (GPIO_Pin == GPIO_PIN_0)
  {
    xSemaphoreGiveFromISR(xImuAccelReadySemaphore, &xHigherPriorityTaskWoken);
  }
  else if (GPIO_Pin == GPIO_PIN_2)
  {
    xSemaphoreGiveFromISR(xImuGyroReadySemaphore, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}