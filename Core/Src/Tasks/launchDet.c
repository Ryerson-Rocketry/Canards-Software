#include "Tasks/launchDet.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "states.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "math.h"
#include <stdio.h>

extern TaskHandle_t altEstTaskHandle;
extern TaskHandle_t oriEstTaskHandle;
extern TaskHandle_t dataStoreTaskHandle;
extern Rocket_States_t Rocket;

extern bool readSensorTask;
extern bool altEstTask;
extern bool launchDetTask;
extern bool dataStoreTask;
extern bool controlTask;
extern bool oriEstTask;

float accel_z;
static uint32_t accel_start_time = 0;
static bool threshold_active = false;
static uint32_t burnoutStartTime = 0;
static bool launch_recorded = false;
static uint32_t canardActivationTime = 0;

void checkRecorded(){
  if (Rocket.flightState != STATE_PAD && !launch_recorded)
  {
    Rocket.estimate.launch_tick = osKernelGetTickCount();
    launch_recorded = true;
  }
}

void checkFlightState(){
  uint32_t now = osKernelGetTickCount();
  accel_z = Rocket.rawData.accel[2] - 1000.0f;
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
}


void notifyTasks(){
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
}

void satisfyWDG(){
  // Satisfy watchdog for tasks intentionally not running in current state
  if (Rocket.flightState == STATE_PAD)
  {
    altEstTask = true;
    oriEstTask = true;
    dataStoreTask = true;
    controlTask = true;
  }

  launchDetTask = true;
}

