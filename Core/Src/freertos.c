#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "Kalman/kalman.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

// Task handle and attributes
osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{

  // Tune and change these values
  float altitude_cov = 0.05f;
  float velocity_cov = 0.05f;
  float accel_var = 0.01f;
  float dt = 0.5; // Time elapsed

  // variable initialization
  float accelerometer[3] = {0, 0, 9.81};
  float barometric_alt = 0;
  float gravity = 9.81;
  char logBuffer[50];

  // initial state values, [0] = position, [1] = velocity
  float pos = 0.0f;
  float vel = 0.0f;
  // P = system covariance
  float system_cov[2][2] = {
      {altitude_cov, 0.0f},
      {0.0f, velocity_cov}};

  // Q = process noise covariance matrix
  float process_noise[2][2] = {
      {(0.25f * powf(dt, 4)) * accel_var, (0.5f * powf(dt, 3)) * accel_var},
      {(0.5f * powf(dt, 3)) * accel_var, (powf(dt, 2)) * accel_var}};

  for (;;)
  {
    // poll timer here to get dt

    // queue to get accelerometer data

    // Predict
    kalmanPredict(dt, &pos, &vel, accelerometer[2] - gravity, process_noise, system_cov);

    // queue to get barometer data

    // formula to get barometric altitude

    // Update
    kalmanUpdate(&pos, &vel, barometric_alt, system_cov);

    // poll timer here to get dt
    snprintf(logBuffer, sizeof(logBuffer), "Alt: %.2f m, Vel: %.2f m/s", pos, vel);
    HAL_UART_Transmit(&huart2, (uint8_t *)logBuffer, strlen(logBuffer), 30);
    osDelay(500);
  }
}
