#include "Tasks/launchDet.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"
#include "states.h"
#include "flight_configs.h"
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

static bool launch_recorded = false;

uint32_t elapsed_time = 0;

void checkRecorded()
{
  if (Rocket.flightState != STATE_PAD && !launch_recorded)
  {
    Rocket.estimate.launch_tick = osKernelGetTickCount();
    launch_recorded = true;
  }
}

#if ROCKET_CONFIG == ROCKET_TWO_STAGE
// ═══ Two-stage rocket: PAD -> BOOSTER_BURNOUT -> SEPARATION -> SUSTAINER_IGNITION -> CANARDS_ACTIVATE -> DESCENT ═══

static bool threshold_active = false;
static uint32_t accel_start_time = 0;
static uint32_t booster_burnout_start = 0;
static uint32_t separation_time_start = 0;
static uint32_t sustainer_ignition_start = 0;

const float LIFTOFF_ACCEL = 109.462824f;
const float LIFTOFF_G = 11.15f;
// const float LIFTOFF_G = 1.0f;
// const float LIFTOFF_ACCEL = 1.0f * 9.81f;
const float BOOSTER_BURNOUT_TIME = 4500.0f;
const float STAGE_SEPARATION_TIME = 1000.0f;
const float SUSTAINER_IGNITION = 1000.0f;
const float SUSTAINER_BURNOUT_G = 9.75608563f;
const float SUSTAINER_BURNOUT_ACCEL = 95.7072f;
const float SUSTAINER_BURNOUT_TIME = 4900.0f;

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

    if ((now - accel_start_time) >= pdMS_TO_TICKS(10))
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

  if (((elapsed_time >= pdMS_TO_TICKS(SUSTAINER_IGNITION)) &&
       (accel_z_mg >= SUSTAINER_BURNOUT_G * 1000) &&
       (accel_z_m_s2 >= SUSTAINER_BURNOUT_ACCEL)) ||
      (elapsed_time >= pdMS_TO_TICKS(6500)))
  {
    Rocket.flightState = STATE_CANARDS_ACTIVATE;
  }
}

void state_canards_activate()
{
  static bool reached_speed = false;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  // Latch on a high speed first, so a low/garbage velocity reading at activation
  // can't immediately disarm. Only a genuine decay back through 50 m/s descends.
  if (Rocket.estimate.velocity > 50.0f)
  {
    reached_speed = true;
  }

  if (reached_speed && Rocket.estimate.velocity <= 50.0f)
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
    // Servo power is cut by vControlTask's non-active branch.
    break;

  default:
    // printf("Invalid state somehow...");
    break;
  }
}

bool canardsActive(void)
{
  return Rocket.flightState == STATE_CANARDS_ACTIVATE;
}

#elif ROCKET_CONFIG == ROCKET_SINGLE_STAGE
// ═══ Single-stage rocket: PAD -> BOOSTER_BURN -> CANARDS_ACTIVATION -> APOGEE -> LANDED ═══

static uint32_t booster_burn_start = 0;

// One debounce timer per condition (each is only used in its own state).
static bool liftoff_active = false;
static uint32_t liftoff_start = 0;
static bool apogee_active = false;
static uint32_t apogee_start = 0;
static bool landed_active = false;
static uint32_t landed_start = 0;

// Returns true once `condition` has been continuously true for hold_ms.
// Any cycle where it is false resets the timer.
static bool heldFor(bool condition, bool *active, uint32_t *start, uint32_t now, uint32_t hold_ms)
{
  if (!condition)
  {
    *active = false;
    return false;
  }

  if (!*active) // rising edge: start timing once
  {
    *start = now;
    *active = true;
  }

  return (now - *start) >= pdMS_TO_TICKS(hold_ms);
}

// PAD -> BOOSTER_BURN: raw Z accel (gravity included) above 4 g for 100 ms
void state_pad(uint32_t now)
{
  if (heldFor(accel_z_mg >= SS_LIFTOFF_ACCEL_MG, &liftoff_active, &liftoff_start, now, SS_LIFTOFF_HOLD_MS))
  {
    booster_burn_start = now;
    Rocket.flightState = STATE_BOOSTER_BURN;
  }
}

// BOOSTER_BURN -> CANARDS_ACTIVATION: fixed delay after BOOSTER_BURN started
void state_booster_burn(uint32_t now)
{
  elapsed_time = now - booster_burn_start;

  if (elapsed_time >= pdMS_TO_TICKS(SS_CANARDS_DELAY_MS))
  {
    Rocket.flightState = STATE_CANARDS_ACTIVATION;
  }
}

// CANARDS_ACTIVATION -> APOGEE: KF vertical velocity below 25 m/s for 100 ms
void state_canards_activation(uint32_t now)
{
  // Same as two-stage state_canards_activate(): drives PC0 (servo power) high every
  // cycle. KNOWN ISSUE (kept on purpose for now): this overrides vControlTask cutting
  // servo power when |pitch error| > 30 deg.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  if (heldFor(Rocket.estimate.velocity < SS_APOGEE_VELOCITY, &apogee_active, &apogee_start, now, SS_APOGEE_HOLD_MS))
  {
    Rocket.flightState = STATE_APOGEE;
  }
}

// APOGEE -> LANDED: no motion for 2 s, i.e. total accel ~1 g (gravity only, any
// orientation), all gyro rates small, and KF vertical velocity near 0. The velocity
// check stops a calm parachute descent (which also reads ~1 g) from counting as
// landed. Logging is stopped by vDataStoreTask.
void state_apogee(uint32_t now)
{
  float ax = Rocket.rawData.accel[0];
  float ay = Rocket.rawData.accel[1];
  float az = Rocket.rawData.accel[2];
  float accel_total_mg = sqrtf(ax * ax + ay * ay + az * az);

  bool still = (fabsf(accel_total_mg - 1000.0f) < SS_LANDED_ACCEL_TOL_MG) &&
               (fabsf(Rocket.rawData.gyro[0]) < SS_LANDED_GYRO_DPS) &&
               (fabsf(Rocket.rawData.gyro[1]) < SS_LANDED_GYRO_DPS) &&
               (fabsf(Rocket.rawData.gyro[2]) < SS_LANDED_GYRO_DPS) &&
               (fabsf(Rocket.estimate.velocity) < SS_LANDED_VELOCITY);

  if (heldFor(still, &landed_active, &landed_start, now, SS_LANDED_HOLD_MS))
  {
    Rocket.flightState = STATE_LANDED;
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

  case STATE_BOOSTER_BURN:
    state_booster_burn(now);
    break;

  case STATE_CANARDS_ACTIVATION:
    state_canards_activation(now);
    break;

  case STATE_APOGEE:
    // Servo power is cut by vControlTask's non-active branch.
    state_apogee(now);
    break;

  case STATE_LANDED:
    // Final state. SD file is closed by vDataStoreTask.
    break;

  default:
    break;
  }
}

bool canardsActive(void)
{
  return Rocket.flightState == STATE_CANARDS_ACTIVATION;
}

#else
#error "Unknown ROCKET_CONFIG (see flight_configs.h)"
#endif

void notifyTasks()
{
  if (Rocket.flightState != STATE_PAD)
  {
    xTaskNotifyGive(altEstTaskHandle);
  }

  if (Rocket.flightState == STATE_BOOSTER_BURNOUT ||
      Rocket.flightState == STATE_SEPARATION ||
      Rocket.flightState == STATE_SUSTAINER_IGNITION ||
      Rocket.flightState == STATE_CANARDS_ACTIVATE ||
      Rocket.flightState == STATE_DESCENT)
  {
  }
  xTaskNotifyGive(dataStoreTaskHandle);
  xTaskNotifyGive(oriEstTaskHandle);
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