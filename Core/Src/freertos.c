#include "FreeRTOS.h"
#include "Drivers/mmc5983ma.h"
#include "Drivers/lsm6dso32_app.h"
#include "StateEstimation/altitude_estimation.h"
#include "Utils/math_utils.h"
#include "Defs/states.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "usart.h"
#include "spi.h"
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
#include "attitude_estimation_wrapper.h"
#include "queue.h"
#include <string.h>
// #include "rfm69.h"
#include "Tasks/sensor.h"
#include "Tasks/sdcard.h"
#include "Tasks/launchDet.h"
#include "Tasks/radio.h"
#include "Drivers/rfm9x.h"
#include "Tasks/gps.h"
#include "tim.h"

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
// osThreadId_t gpsRetrieveTaskHandle;
osThreadId_t radioTaskHandle;
osThreadId_t controlTaskHandle;
osThreadId_t heartbeatTaskHandle;
osThreadId_t gpsTaskHandle;

// QueueHandle_t radioQueueHandle;

// struct data handle init
Rocket_States_t Rocket = {.flightState = STATE_PAD};

// watchdog task flags
volatile bool readSensorTask = false;
volatile bool altEstTask = false;
volatile bool launchDetTask = false;
volatile bool dataStoreTask = false;
// volatile bool radioTask = false;
volatile bool controlTask = false;
volatile bool oriEstTask = false;
volatile bool gpsTask = false; // not in the watchdog check, like the radio

// Priorities follow the data pipeline (high -> low):
//   readSensor -> launchDet -> oriEst -> altEst -> control -> dataStore -> heartbeat
// so each stage preempts and runs as soon as its input is ready. Heartbeat is lowest
// on purpose: the IWDG only gets refreshed when every real task is idle/healthy.
const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal7};
const osThreadAttr_t altEstTask_attributes = {
    .name = "altTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal4};
const osThreadAttr_t oriEstTask_attributes = {
    .name = "attitudeTask", .stack_size = 1024 * 16, .priority = osPriorityAboveNormal5};
const osThreadAttr_t launchDetTask_attributes = {
    .name = "launchTask", .stack_size = 1024 * 2, .priority = osPriorityAboveNormal6};
const osThreadAttr_t dataStoreTask_attributes = {
    .name = "storeTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal1};
const osThreadAttr_t controlTask_attributes = {
    .name = "controlCanards", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal3};
const osThreadAttr_t heartbeat_attributes = {
    .name = "wdgTask", .stack_size = 256 * 4, .priority = osPriorityBelowNormal3};
const osThreadAttr_t radiotask_attributes = {
    .name = "radioTask", .stack_size = 1024 * 4, .priority = osPriorityNormal};
const osThreadAttr_t gps_attributes = {
    .name = "gpsTask", .stack_size = 1024 * 4, .priority = osPriorityNormal};

void vGpsTask(void *argument);
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
  oriEstTaskHandle = osThreadNew(vOriEstTask, NULL, &oriEstTask_attributes);
  launchDetTaskHandle = osThreadNew(vLaunchDetTask, NULL, &launchDetTask_attributes);
  dataStoreTaskHandle = osThreadNew(vDataStoreTask, NULL, &dataStoreTask_attributes);
  controlTaskHandle = osThreadNew(vControlTask, NULL, &controlTask_attributes);
  radioTaskHandle = osThreadNew(vRadioTask, NULL, &radiotask_attributes);
  gpsTaskHandle = osThreadNew(vGpsTask, NULL, &gps_attributes);
  heartbeatTaskHandle = osThreadNew(vHeartbeatTask, NULL, &heartbeat_attributes);
}

void vGpsTask(void *argument)
{
  static uint8_t dummy_tx[GPS_BUF_SIZE];
  memset(dummy_tx, 0xFF, sizeof(dummy_tx));
  uint8_t gpsData[GPS_BUF_SIZE];
  GNSS_Data vehicle_gps;
  memset(&vehicle_gps, 0, sizeof(GNSS_Data));
  CS_HIGH();
  vTaskDelay(pdMS_TO_TICKS(1500));
  // only used for setting up the gps
  // gpsSendCfg(cfg_revert, sizeof(cfg_revert));
  // vTaskDelay(pdMS_TO_TICKS(500));
  // gpsSendCfg(cfg_spiprot, sizeof(cfg_spiprot));
  // vTaskDelay(pdMS_TO_TICKS(500));
  for (;;)
  {
    gpsRead(gSpi2Mutex, gpsData, dummy_tx);
    process_gps_data((char *)gpsData, &vehicle_gps);
    taskENTER_CRITICAL();
    Rocket.gps.has_fix = vehicle_gps.has_fix;
    Rocket.gps.latitude = vehicle_gps.latitude;
    Rocket.gps.longitude = vehicle_gps.longitude;
    Rocket.gps.altitude_m = vehicle_gps.altitude_m;
    Rocket.gps.speed_kmh = vehicle_gps.speed_kmh;
    taskEXIT_CRITICAL();
    osDelay(1000);
    gpsTask = true;
  }
}

void vReadSensorTask(void *argument)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  sensor_HardwareInit();

  for (;;)
  {
    // I2C1: Magnetometer
    sensor_ReadMagnetometer();

    // SPI2: Barometer
    sensor_ReadBarometer();

    // SPI1: IMU - accelerometer
    sensor_ReadIMUAccelerometer();

    // SPI1: IMU - gyro
    sensor_ReadIMUGyro();

    Rocket.rawData.timestamp = osKernelGetTickCount();

    sensor_GroundReference();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    readSensorTask = true;
    osDelay(10);
  }
}

void vAltEstTask(void *argument)
{
  // Q set each cycle from dt and KF_SIGMA_ACCEL
  float processNoise[2][2] = {
      {0.0f, 0.0f},
      {0.0f, 0.0f}
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

  // R = KF_BARO_VAR

  static uint32_t last_tick = 0;
  static uint32_t gateStart = 0; // tick the baro gate closed, 0 = open
  float dt = 0.0f;

  float accel_in_m_s2;
  float baro_altitude;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t current_tick = osKernelGetTickCount();

    dt = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    if (dt < 0.001f)
      dt = 0.01f;
    if (dt > 0.1f)
      dt = 0.1f;

    stateTransition[0][1] = dt;

    const float sa2 = KF_SIGMA_ACCEL * KF_SIGMA_ACCEL;
    const float dt2 = dt * dt;
    processNoise[0][0] = sa2 * dt2 * dt2 / 4.0f;
    processNoise[0][1] = sa2 * dt2 * dt / 2.0f;
    processNoise[1][0] = processNoise[0][1];
    processNoise[1][1] = sa2 * dt2;

    // raw accel, the EMA lags ~0.3 s, no tilt so EKF errors can't leak in
    accel_in_m_s2 = (Rocket.rawData.accelRaw[2] - 1000.0f) * 0.00980665f;
    if (fabsf(accel_in_m_s2) > 500.0f)
      accel_in_m_s2 = 0.0f;

    altPredict(dt, &Rocket.estimate.position, &Rocket.estimate.velocity,
               accel_in_m_s2, processNoise, systemCov, stateTransition);

    // skip baro above KF_BARO_GATE_MACH for at most KF_BARO_GATE_MAX_MS, NaN Mach still updates
    baro_altitude = getRelativeAltitude(Rocket.rawData.pressure);
    const float machEst = servoGetMach(Rocket.rawData.pressure, Rocket.estimate.velocity);
    const bool fast = (machEst > KF_BARO_GATE_MACH);
    if (!fast)
      gateStart = 0;
    else if (gateStart == 0)
      gateStart = current_tick;
    const bool gateTimedOut = fast && (current_tick - gateStart) > KF_BARO_GATE_MAX_MS;
    if (!fast || gateTimedOut)
      altUpdate(&Rocket.estimate.position, &Rocket.estimate.velocity,
                baro_altitude, systemCov, KF_BARO_VAR);

    // track peak Mach from liftoff onward, every cycle, regardless of canard state
    if (Rocket.flightState != STATE_PAD)
      servoUpdatePeakMach(Rocket.rawData.pressure, Rocket.estimate.velocity, accel_in_m_s2, current_tick);
    
    // update rocket state estimate and gyro integration
    Rocket.estimate.timestamp = current_tick;
    Rocket.estimate.acceleration = accel_in_m_s2;

    altEstTask = true;
  }
}

void vOriEstTask(void *argument)
{
  AttitudeEstimationHandle attitudeEstimationHandle = AttitudeEstimation_create();
  float attitude[4];
  float attitudeRPY[3];
  static uint32_t last_tick = 0;
  float dt;
  static float gyroBias[3] = {0.0f, 0.0f, 0.0f}; // gyro offset averaged on the pad, frozen at liftoff

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t current_tick = osKernelGetTickCount();
    dt = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;
    if (dt < 0.001f)
      dt = 0.01f;
    if (dt > 0.1f)
      dt = 0.1f;
    AttitudeEstimation_setDt(attitudeEstimationHandle, dt);

    // Accel frame fix: roll & pitch came out anti-correlated with gravity because the
    // accelerometer X/Y axes are 180 deg about Z relative to the estimator body frame.
    // Negate X and Y (NOT Z, so gravity magnitude and the altitude KF's accel[2] use stay intact).
    float accelCorr[3] = {
        -Rocket.rawData.accel[0],
        -Rocket.rawData.accel[1],
        Rocket.rawData.accel[2]};

    // still = on the pad, about 1 g and not turning
    const float accelNorm = sqrtf(accelCorr[0] * accelCorr[0] + accelCorr[1] * accelCorr[1] +
                                  accelCorr[2] * accelCorr[2]);
    float rate2 = 0.0f; // turning rate squared
    for (int i = 0; i < 3; i++)
      rate2 += (Rocket.rawData.gyroRaw[i] - gyroBias[i]) * (Rocket.rawData.gyroRaw[i] - gyroBias[i]);
    const bool stillOnPad = (Rocket.flightState == STATE_PAD) &&
                            (fabsf(accelNorm - 1000.0f) < EKF_ACCEL_GRAVITY_TOL_MG) &&
                            (rate2 < EKF_STILL_GYRO_DPS * EKF_STILL_GYRO_DPS);

    // raw gyro minus pad offset, the EMA lags ~0.15 s
    float gyroCorr[3];
    for (int i = 0; i < 3; i++)
    {
      if (stillOnPad)
        gyroBias[i] += EKF_GYRO_BIAS_ALPHA * (Rocket.rawData.gyroRaw[i] - gyroBias[i]);
      gyroCorr[i] = Rocket.rawData.gyroRaw[i] - gyroBias[i];
    }

    AttitudeEstimation_predict(attitudeEstimationHandle, gyroCorr, attitude);
    // accel + mag only while still, gyro only otherwise, mag is disturbed in flight
    if (stillOnPad)
      AttitudeEstimation_correct(attitudeEstimationHandle, accelCorr, Rocket.rawData.mag, attitude);
    AttitudeEstimation_getRPY(attitudeEstimationHandle, attitudeRPY);

    Rocket.estimate.rpy[0] = attitudeRPY[0];
    Rocket.estimate.rpy[1] = attitudeRPY[1];
    Rocket.estimate.rpy[2] = attitudeRPY[2];
    Rocket.estimate.tilt_angle = sqrtf(powf(Rocket.estimate.rpy[0], 2.0f) + powf(Rocket.estimate.rpy[1], 2.0f));

    xTaskNotifyGive(controlTaskHandle);
    oriEstTask = true;
  }
}

void vLaunchDetTask(void *argument)
{
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    checkRecorded();

    checkFlightState();

    notifyTasks();

    satisfyWDG();
  }
}

void vDataStoreTask(void *argument)
{
  int len; // must hold the full row length (~294 B); a uint8_t would wrap at 256 and truncate the SD write
  bool sdInitialized = DataStore_SDCardInit();
  // radioQueueHandle = xQueueCreate(1, CSV_BUFFER_SIZE); // item size must match csvBuffer so the full row isn't truncated

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    dataStoreTask = true; // always set flag first — SD ops below must not block WDG

    if (Rocket.flightState == STATE_PAD)
    {
      dataStoreTask = true;
      continue;
    }

    if (!sdInitialized)
    {
      continue;
    }

    DataStore_TelemetrySnapshot();
    len = DataStore_WriteToCSV();

    DataStore_WriteToSDCard(len);

    // Landed: the row above is the last one. Flush everything not yet synced
    // (up to 49 rows) and close the file, then never write again.
    if (Rocket.flightState == STATE_LANDED)
    {
      DataStore_Close();
      sdInitialized = false; // every later cycle exits at the !sdInitialized check
    }
  }
}

void vControlTask(void *argument)
{
  float setPoint = 0.0f;
  float rollError = 0;
  static float integral = 0;
  float derivative, output = 0;
  float pitchSetPoint = 0.0f;

  // On startup: power the servo and drive it to center, holding long enough
  // for it to physically reach the middle before the control loop begins.
  HAL_GPIO_WritePin(Servo_HS_Sw_GPIO_Port, Servo_HS_Sw_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_CENTER_US);
  for (uint32_t t = 0; t < 500; t += 20)
  {
    controlTask = true; // keep the watchdog fed while the servo travels
    osDelay(20);
  }

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (canardsActive()) // STATE_CANARDS_ACTIVATE (two-stage) or STATE_CANARDS_ACTIVATION (single-stage)
    {
      uint32_t elapsedMillis = osKernelGetTickCount();

      if ((elapsedMillis % 10000) < 5000)
      {
        setPoint = 0.0f;
      }
      else
      {
        setPoint = 90.0f;
      }

      // Roll = spin about the longitudinal (Z/nose) axis = estimator yaw (rpy[2]).
      rollError = setPoint - Rocket.estimate.rpy[2];
      Rocket.control.rollError = rollError;
      Rocket.control.setPoint = setPoint;
      Rocket.control.pitchError = pitchSetPoint - Rocket.estimate.rpy[1];

      if (fabsf(Rocket.control.pitchError) > 30)
      {
        // Off-nominal pitch: cut servo power and skip actuation.
        HAL_GPIO_WritePin(Servo_HS_Sw_GPIO_Port, Servo_HS_Sw_Pin, GPIO_PIN_RESET);
        controlTask = true;
        continue;
      }

      // Canards active and pitch in range: power the servo.
      HAL_GPIO_WritePin(Servo_HS_Sw_GPIO_Port, Servo_HS_Sw_Pin, GPIO_PIN_SET);

      if (fabsf(output) < 20.0f)
      {
        integral += rollError * 0.01f;
      }

      derivative = Rocket.rawData.gyro[2]; // spin rate about the Z/nose axis
      output = (Kp * rollError) + (Ki * integral) - (Kd * derivative);

      Rocket.control.pwmAngle = moveServo(output, Rocket.estimate.velocity, Rocket.rawData.pressure);
    }
    else
    {
      // Not in the canards phase: keep the servo unpowered.
      HAL_GPIO_WritePin(Servo_HS_Sw_GPIO_Port, Servo_HS_Sw_Pin, GPIO_PIN_RESET);
      setPoint = 0.0f;
      integral = 0;
      output = 0;
    }

    controlTask = true;
  }
}

// Not part of the watchdog check, a radio fault must never reset the flight computer
void vRadioTask(void *argument)
{
  static char packet[RFM9X_MAX_PAYLOAD + 1];
  bool radioReady = false;
  uint32_t nextWake = osKernelGetTickCount();

  for (;;)
  {
    nextWake += RADIO_PERIOD_MS;

    if (!radioReady)
    {
      radioReady = (rfm9xInit() == HAL_OK);
    }

    if (radioReady)
    {
      int len = Radio_BuildPacket(packet, sizeof(packet));

      if (len > 0 && rfm9xTransmit((const uint8_t *)packet, (uint8_t)len) != HAL_OK)
      {
        radioReady = false;
      }
    }

    osDelayUntil(nextWake);
  }
}

void vHeartbeatTask(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (readSensorTask && altEstTask && launchDetTask && dataStoreTask && controlTask && oriEstTask)
    {
      HAL_IWDG_Refresh(&hiwdg);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
      readSensorTask = false;
      altEstTask = false;
      launchDetTask = false;
      dataStoreTask = false;
      controlTask = false;
      oriEstTask = false;
    }
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