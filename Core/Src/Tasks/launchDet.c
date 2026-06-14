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

float accel_z_mg;
float accel_z_m_s2;

static bool threshold_active = false;
static bool launch_recorded = false;
static uint32_t accel_start_time = 0;
static uint32_t booster_burnout_start = 0;
static uint32_t separation_time_start = 0;
static uint32_t sustainer_ignition_start = 0;

uint32_t elapsed_time = 0;

const float LIFTOFF_G = 11.15f;
const float LIFTOFF_ACCEL = 109.462824f;
const float BOOSTER_BURNOUT_TIME = 5000.0f;
const float STAGE_SEPARATION_TIME = 1000.0f;
const float SUSTAINER_IGNITION = 1000.0f;
const float SUSTAINER_BURNOUT_G = 9.75608563f;
const float SUSTAINER_BURNOUT_ACCEL = 95.7072f;
const float SUSTAINER_BURNOUT_TIME = 4900.0f;

void checkRecorded()
{
  if (Rocket.flightState != STATE_PAD && !launch_recorded)
  {
    Rocket.estimate.launch_tick = osKernelGetTickCount();
    launch_recorded = true;
  }
}

void state_pad(uint32_t now)
{
  if (accel_z_mg >= LIFTOFF_G * 1000 || accel_z_m_s2 >= LIFTOFF_ACCEL)
  {
    // Capture the rising-edge time ONCE. Resetting it every cycle would keep
    // (now - accel_start_time) at 0, so the 100ms debounce could never elapse.
    if (!threshold_active)
    {
      accel_start_time = now;
      threshold_active = true;
    }

    if ((now - accel_start_time) >= pdMS_TO_TICKS(100))
    {
      booster_burnout_start = now;
      Rocket.flightState = STATE_BOOSTER_BURNOUT;
    }
  }
  else
  {
    threshold_active = false;
  }
}

void state_booster_burnout(uint32_t now)
{
  elapsed_time = now - booster_burnout_start;

  // booster 4 seconds then 1 sec after separation
  if (elapsed_time >= pdMS_TO_TICKS(BOOSTER_BURNOUT_TIME))
  {
    separation_time_start = now;
    Rocket.flightState = STATE_SEPARATION;
  }
}

void state_separation(uint32_t now)
{
  elapsed_time = now - separation_time_start;

  if (elapsed_time >= pdMS_TO_TICKS(STAGE_SEPARATION_TIME))
  {
    sustainer_ignition_start = now;
    Rocket.flightState = STATE_SUSTAINER_IGNITION;
  }
}

void state_sustainer_ignition(uint32_t now)
{
  elapsed_time = now - sustainer_ignition_start;

  if (elapsed_time >= pdMS_TO_TICKS(SUSTAINER_IGNITION) ||
      accel_z_mg >= SUSTAINER_BURNOUT_G * 1000 ||
      accel_z_m_s2 >= SUSTAINER_BURNOUT_ACCEL)
  {
    Rocket.flightState = STATE_CANARDS_ACTIVATE;
  }
}

void state_canards_activate()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  if (Rocket.estimate.velocity <= 50.0f)
  {
    Rocket.flightState = STATE_DESCENT;
  }
}

void checkFlightState()
{
  uint32_t now = osKernelGetTickCount();
  accel_z_mg = Rocket.rawData.accel[2];
  accel_z_m_s2 = Rocket.rawData.accel[2] * 9.81 / 1000;

  switch (Rocket.flightState)
  {
  case STATE_PAD:
    state_pad(now);
    break;

  case STATE_BOOSTER_BURNOUT:
    state_booster_burnout(now);
    break;

  case STATE_SEPARATION:
    state_separation(now);
    break;

  case STATE_SUSTAINER_IGNITION:
    state_sustainer_ignition(now);
    break;

  case STATE_CANARDS_ACTIVATE:
    state_canards_activate();
    break;

  case STATE_DESCENT:
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    break;

  default:
    printf("Invalid state somehow...");
    break;
  }
}

void notifyTasks()
{
  if (Rocket.flightState != STATE_PAD)
  {
    xTaskNotifyGive(altEstTaskHandle);
    xTaskNotifyGive(oriEstTaskHandle);
  }

  if (Rocket.flightState == STATE_BOOSTER_BURNOUT ||
      Rocket.flightState == STATE_SEPARATION ||
      Rocket.flightState == STATE_SUSTAINER_IGNITION ||
      Rocket.flightState == STATE_CANARDS_ACTIVATE ||
      Rocket.flightState == STATE_DESCENT)
  {
    xTaskNotifyGive(dataStoreTaskHandle);
  }
}

void satisfyWDG()
{
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