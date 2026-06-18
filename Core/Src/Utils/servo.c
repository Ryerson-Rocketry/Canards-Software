#include "tim.h"
#include "Utils/servo.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Configs/flight_configs.h"
#include "math.h"

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

    // float angle = getControlForceCoeff(velocity) * forceNormalized;

    if (deflectionAngle > DEFLECTION_ANGLE_LIMIT)
    {
        deflectionAngle = DEFLECTION_ANGLE_LIMIT;
    }

    if (deflectionAngle < -DEFLECTION_ANGLE_LIMIT)
    {
        deflectionAngle = -DEFLECTION_ANGLE_LIMIT;
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