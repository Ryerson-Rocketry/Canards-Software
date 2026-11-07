#include "tim.h"
#include "Utils/servo.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Configs/flight_configs.h"
#include "math.h"
#include <stdbool.h>

static float tempPadK = 288.15f;
static float padPressure = SEA_LEVEL_PA;


// run before launch on pad
void servoPadPressureTemperature (float pressure, float tempCentiC)
{
  tempPadK = tempCentiC / 100.0f + 273.15f;
  padPressure = pressure;
}


static float calculateMach(float pressure, float velocity) 
{
    // todo: 1. calcuate temperature, 2. calculate speed of sound 3. divide ground speed by speed of sound 
    float pressureRatio = (pressure > 0.0f && padPressure > 0.0f) ? pressure / padPressure : 1.0f; 
    float tempK = tempPadK * powf(pressureRatio, ISA_TEMP_EXP); 
    float c = sqrtf(AIR_GAMMA * DRY_AIR_GAS_CONSTANT * tempK);  
    return velocity / c;
}

static float getMaxDeflectionAngle(float mach) 
{
   static float peakMach = 0.0f;
   static bool pastPeak = false;


   if (!isfinite(mach))
    return BASE_DEFLECTION_ANGLE;

   if (!pastPeak) 
   {
     if (peakMach < mach)
       peakMach = mach;
     
     // 0/3 is for ensuring that we don't trigger before reaching atleast above 0.3 mach speed
     // -0.02 is for uncertainity, kinda to reduce the influence of noise 
     if (mach < peakMach - 0.02 && peakMach > 0.3)
       pastPeak = true;
   }

   if (!pastPeak)
     return BASE_DEFLECTION_ANGLE;


   float drop = peakMach - mach;
   if (drop < 0)
     drop = 0.0f;
   float max_deflection_angle = BASE_DEFLECTION_ANGLE + (DEFLECTION_ANGLE_STEP / DEFLECTION_MACH_STEP) * (drop); // change in the mach speed 

   if (max_deflection_angle > HARD_ANGLE_LIMIT)
     max_deflection_angle = HARD_ANGLE_LIMIT;

   return max_deflection_angle;



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
        controlForceCoeff = 337 * expf(-0.02777 * velocity);
    }
    else
    {
        // supersonic
        controlForceCoeff = 1.74 - (2.7710 * powf(10, -3) * velocity) + (1.4210 * pow(10, -6)) * pow(velocity, 2);
    }

    float deflectionAngle = controlForceCoeff * sideForce;
    float max_deflection_angle = getMaxDeflectionAngle(calculateMach(pressure, velocity));
    
    if (fabsf(deflectionAngle) > max_deflection_angle) {
      deflectionAngle = copysignf(max_deflection_angle, deflectionAngle);
    }



    // float angle = getControlForceCoeff(velocity) * forceNormalized;
// 
//     if (deflectionAngle > DEFLECTION_ANGLE_LIMIT)
//     {
//         deflectionAngle = DEFLECTION_ANGLE_LIMIT;
//     }
// 
//     if (deflectionAngle < -DEFLECTION_ANGLE_LIMIT)
//     {
//         deflectionAngle = -DEFLECTION_ANGLE_LIMIT;
//     }
// 
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




