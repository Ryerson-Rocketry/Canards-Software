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
#include "attitude_estimation_wrapper.h"
// #include "Utils/i2c_scanner.h"
#include "queue.h"
#include <string.h>
#include "rfm69.h"
#include "Tasks/sensor.h" 
#include "Tasks/sdcard.h"

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

QueueHandle_t radioQueueHandle;

// struct data handle init
Rocket_States_t Rocket = {.flightState = STATE_CANARDS_ACTIVATE};

// watchdog task flags
volatile bool readSensorTask = false;
volatile bool altEstTask = false;
volatile bool launchDetTask = false;
volatile bool dataStoreTask = false;
volatile bool radioTask = false;
volatile bool controlTask = false;
volatile bool oriEstTask = false;

const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal6};
const osThreadAttr_t altEstTask_attributes = {
    .name = "altTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal4};
const osThreadAttr_t oriEstTask_attributes = {
    .name = "attitudeTask", .stack_size = 1024 * 16, .priority = osPriorityAboveNormal3};
const osThreadAttr_t launchDetTask_attributes = {
    .name = "launchTask", .stack_size = 1024 * 2, .priority = osPriorityAboveNormal2};
const osThreadAttr_t dataStoreTask_attributes = {
    .name = "storeTask", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal1};
const osThreadAttr_t controlTask_attributes = {
    .name = "controlCanards", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal4};
const osThreadAttr_t heartbeat_attributes = {
    .name = "wdgTask", .stack_size = 256 * 4, .priority = osPriorityBelowNormal3};
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
  altEstTaskHandle     = osThreadNew(vAltEstTask,     NULL, &altEstTask_attributes);
  oriEstTaskHandle     = osThreadNew(vOriEstTask,     NULL, &oriEstTask_attributes);
  launchDetTaskHandle  = osThreadNew(vLaunchDetTask,  NULL, &launchDetTask_attributes);
  dataStoreTaskHandle  = osThreadNew(vDataStoreTask,  NULL, &dataStoreTask_attributes);
  controlTaskHandle    = osThreadNew(vControlTask,    NULL, &controlTask_attributes);
  radioTaskHandle = osThreadNew(vRadioTask, NULL, &radiotask_attributes);
  heartbeatTaskHandle  = osThreadNew(vHeartbeatTask,  NULL, &heartbeat_attributes);

  printf("[INIT] tasks: r=%p a=%p o=%p l=%p d=%p c=%p h=%p\r\n",
         (void*)readSensorTaskHandle, (void*)altEstTaskHandle, (void*)oriEstTaskHandle,
         (void*)launchDetTaskHandle,  (void*)dataStoreTaskHandle, (void*)controlTaskHandle,
         (void*)heartbeatTaskHandle);
  if (!dataStoreTaskHandle)  printf("[INIT] ERROR: dataStoreTask not created (heap?)\r\n");
}

void vReadSensorTask(void *argument)
{
  sensor_HardwareInit();

  for (;;)
  {
    // I2C1: Magnetometer
    // sensor_ReadMagnetometer();
    
    // SPI2: Barometer
    sensor_ReadBarometer();

    // SPI1: IMU - accelerometer
    sensor_ReadIMUAccelerometer();

    // SPI1: IMU - gyro
    sensor_ReadIMUGyro();

    Rocket.rawData.timestamp = osKernelGetTickCount();

    sensor_GroundReference();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

    readSensorTask = true;
    osDelay(500);
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

    accel_in_m_s2 = (Rocket.rawData.accel[2] - 1000.0f) * 0.00980665f * cosf(Rocket.estimate.tilt_angle * M_PI / 180.0f);
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

    AttitudeEstimation_predict(attitudeEstimationHandle, Rocket.rawData.gyro, attitude);
    AttitudeEstimation_correct(attitudeEstimationHandle, Rocket.rawData.accel, Rocket.rawData.mag, attitude);
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
  float accel_z;
  static uint32_t accel_start_time = 0;
  static bool threshold_active = false;
  static uint32_t burnoutStartTime = 0;
  static bool launch_recorded = false;
  static uint32_t canardActivationTime = 0;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

    if (Rocket.flightState != STATE_PAD)
    {
      xTaskNotifyGive(altEstTaskHandle);
      xTaskNotifyGive(oriEstTaskHandle);
    }

    if (Rocket.flightState == STATE_BOOST ||
        Rocket.flightState == STATE_BURNOUT ||
        Rocket.flightState == STATE_CANARDS_ACTIVATE ||
        Rocket.flightState == STATE_DESCENT)
    {
      xTaskNotifyGive(dataStoreTaskHandle);
    }

    // Satisfy watchdog for tasks intentionally not running in current state
    if (Rocket.flightState == STATE_PAD)
    {
      altEstTask = true;
      oriEstTask = true;
      dataStoreTask = true;
      radioTask = true;
      controlTask = true;
    }

    launchDetTask = true;
  }
}

void vDataStoreTask(void *argument)
{
  uint8_t len;
  bool sdInitialized = DataStore_SDCardInit();
  radioQueueHandle = xQueueCreate(1, 128); // 128 is the size of csvBuffer

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    dataStoreTask = true;  // always set flag first — SD ops below must not block WDG

    if (!sdInitialized) {
      continue;
    }

    DataStore_TelemetrySnapshot();
    len = DataStore_WriteToCSV();

    DataStore_WriteToSDCard(len);
    send_to_radio();
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

      derivative = Rocket.rawData.gyro[0];
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

    if (readSensorTask && altEstTask && launchDetTask && dataStoreTask && controlTask && oriEstTask)
    {
      HAL_IWDG_Refresh(&hiwdg);
      readSensorTask = false;
      altEstTask = false;
      launchDetTask = false;
      dataStoreTask = false;
      controlTask = false;
      radioTask = false;
      oriEstTask = false;
    }
    else
    {
      printf("WDG: r=%d l=%d a=%d o=%d c=%d d=%d\r\n",
             readSensorTask, launchDetTask, altEstTask, oriEstTask, controlTask, dataStoreTask);
    }
    osDelay(pdMS_TO_TICKS(100));
  }
}

void vRadioTask(void *argument)
{ 
  HAL_StatusTypeDef radioInitialized = rfm69Init();     
  char csvBufferReceived[128];
  char sendingBuffer[128];
  memset(sendingBuffer, 0, sizeof(sendingBuffer));
  uint8_t payloadSize = 50;
  uint8_t remainingDataSize;

  for (;;)
  {
    if (radioInitialized != HAL_OK)
    {
      radioInitialized = rfm69Init();
      continue;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xQueueReceive(radioQueueHandle, csvBufferReceived, 0) == pdPASS)
    {
      strncpy(sendingBuffer, csvBufferReceived, sizeof(sendingBuffer) - 1);
      sendingBuffer[sizeof(sendingBuffer) - 1] = '\0';

      if (rfm69Transmit((uint8_t *)sendingBuffer, payloadSize) != HAL_OK)
      {
        while(1); // FOR DEBUGGING
        printf("RFM69 transmission failed\r\n");
      }

      osDelay(1);
      remainingDataSize = strlen(sendingBuffer) - payloadSize;
      if (remainingDataSize > 0 &&rfm69Transmit((uint8_t *)sendingBuffer + payloadSize, remainingDataSize + 1) != HAL_OK)
      {
        while(1); // FOR DEBUGGING
        printf("RFM69 transmission failed\r\n");
      }
      memset(sendingBuffer, 0, sizeof(sendingBuffer));
    }

    radioTask = true;
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  uint8_t count = 8; // unknown
  if      (pcTaskName[0] == 'r')                            count = 1; // readSensorTask
  else if (pcTaskName[0] == 'a' && pcTaskName[1] == 'l')   count = 2; // altTask
  else if (pcTaskName[0] == 'a' && pcTaskName[1] == 't')   count = 3; // attitudeTask (oriEst)
  else if (pcTaskName[0] == 'l')                            count = 4; // launchTask
  else if (pcTaskName[0] == 's')                            count = 5; // storeTask
  else if (pcTaskName[0] == 'c')                            count = 6; // controlCanards
  else if (pcTaskName[0] == 'w')                            count = 7; // wdgTask

  while (1)
  {
    for (uint8_t i = 0; i < count; i++)
    {
      GPIOC->BSRR = GPIO_PIN_13 | GPIO_PIN_7;
      for (volatile uint32_t j = 0; j < 20000000; j++);  // ~400ms on
      GPIOC->BSRR = ((uint32_t)(GPIO_PIN_13 | GPIO_PIN_7)) << 16u;
      for (volatile uint32_t j = 0; j < 20000000; j++);  // ~400ms off
    }
    for (volatile uint32_t j = 0; j < 100000000; j++); // ~2.5s pause between sequences
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