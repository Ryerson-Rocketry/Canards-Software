#include "tim.h"
#include "Utils/servo.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Configs/flight_configs.h"
#include "math.h"
#include <stdbool.h>
#define PEAK_LATCH_HOLD_MS 200U  

static bool negAccelActive = false;
static uint32_t negAccelStart = 0;

static float tempPadK = 288.15f;
static float padPressure = SEA_LEVEL_PA;
static float previousMaxDeflectionAngle = 0;
static volatile float peakMach = 0.0f;
static volatile bool pastPeak = false;




// run before launch on pad
void servoPadPressureTemperature (float pressure, float tempC)
{
  tempPadK = tempC + 273.15f;
  padPressure = pressure;
  peakMach = 0.0f;
  pastPeak = false;
  previousMaxDeflectionAngle = BASE_DEFLECTION_ANGLE;
  negAccelActive = false;
  negAccelStart = 0;

}


static float calculateMach(float pressure, float velocity) 
{
    float pressureRatio = (pressure > 0.0f && padPressure > 0.0f) ? pressure / padPressure : 1.0f; 
    float tempK = tempPadK * powf(pressureRatio, ISA_TEMP_EXP); 
    float c = sqrtf(AIR_GAMMA * DRY_AIR_GAS_CONSTANT * tempK);  
    return velocity / c;
}

void servoUpdatePeakMach(float pressure, float velocity, float accel, uint32_t nowTick)
{
  if (pastPeak)
    return;

  float mach = calculateMach(pressure, velocity);
  if (!isfinite(mach)) {
    return;
  }

  if (mach > peakMach)
    peakMach = mach;

   if (accel < 0.0f) // NaN fails this check, so it counts as "not negative"
  {
    if (!negAccelActive)
    {
      negAccelStart = nowTick;
      negAccelActive = true;
    }
    else if ((nowTick - negAccelStart) >= PEAK_LATCH_HOLD_MS && peakMach > 0.3f)
    {
      pastPeak = true;
    }
  }
  else
  {
    negAccelActive = false;
  }
}

static float getMaxDeflectionAngle(float mach)
{
  if (!pastPeak || !isfinite(mach))
    return (pastPeak) ? previousMaxDeflectionAngle : BASE_DEFLECTION_ANGLE;

  float drop = peakMach - mach;
  if (drop < 0.0f)
    drop = 0.0f;

  float maxAngle = BASE_DEFLECTION_ANGLE + (DEFLECTION_ANGLE_STEP / DEFLECTION_MACH_STEP) * drop;

  if (maxAngle < previousMaxDeflectionAngle)
    maxAngle = previousMaxDeflectionAngle;
  if (maxAngle > HARD_ANGLE_LIMIT)
    maxAngle = HARD_ANGLE_LIMIT;

  previousMaxDeflectionAngle = maxAngle;
  return maxAngle;
}


/* Forward declaration — fixes implicit declaration error */
uint16_t servoAngleToPWM(float angle);

// Pressure scaling factor: local pressure / sea-level pressure (e.g. 0.94 at
// ~500 m). Air thins with altitude, so the same deflection makes less force up high.
float getPressureScaling(float pressure)
{
    if (pressure <= 0.0f)
        return 1.0f; // no/invalid reading -> assume sea level, apply no correction
    return pressure / SEA_LEVEL_PA;
}

uint16_t calcForceToPWM(float torque, float velocity, float pressure)
{
    // angle = coeff(v) * (desired_force / pressure_ratio)
    float force = torque / MOMENT_ARM;                      // desired control force (N)
    float sideForce = force / getPressureScaling(pressure); // pressure-normalized force

    float controlForceCoeff = 0.0f;

    // x = velocity
    if (velocity < 330.0f)
    {
        // subsonic
        controlForceCoeff = 337 * expf(-0.02777f * velocity);
    }
    else
    {
        // supersonic
        controlForceCoeff = 1.74f - (2.7710e-3f * velocity) + (1.4210e-6f) * velocity * velocity;
    }

    float deflectionAngle = (isfinite(controlForceCoeff * sideForce)) ? controlForceCoeff * sideForce : 0;
    float max_deflection_angle = getMaxDeflectionAngle(calculateMach(pressure, velocity));
    
    if (fabsf(deflectionAngle) > max_deflection_angle) {
      if (deflectionAngle < 0) {
        return servoAngleToPWM(-1 * max_deflection_angle);;
      } else {
        return servoAngleToPWM(max_deflection_angle);
      }
    }

    

    return servoAngleToPWM(deflectionAngle);
}

uint16_t servoAngleToPWM(float angle)
{
    float pulse = SERVO_CENTER_US + angle * SERVO_US_PER_DEG;

    if (pulse < SERVO_MIN_US)
        pulse = SERVO_MIN_US;
    if (pulse > SERVO_MAX_US)
        pulse = SERVO_MAX_US;

    return (uint16_t)pulse;
}

/* Commands the servo to the angle required for the given torque + velocity */
uint16_t moveServo(float torque, float velocity, float pressure)
{

    uint16_t pwm = calcForceToPWM(torque, velocity, pressure);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
    return pwm;
}




