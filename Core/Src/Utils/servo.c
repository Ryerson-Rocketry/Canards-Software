#include "tim.h"
#include "Utils/servo.h"
#include "main.h"
#include "cmsis_os2.h"
#include "Configs/flight_configs.h"
#include "math.h"

/* Forward declaration — fixes implicit declaration error */
uint16_t servoAngleToPWM(float angle);

float getSlope(float velocity)
{
    if (fabsf(velocity) < 10.0f)
        return 0.0f;
    return 82963.0f * powf(velocity, -1.62f);
}

float getAltitudeScaling(float pressure)
{

    return 15.6 - 1.27 * logf(pressure);
}

uint16_t calcForceToPWM(float torque, float velocity, float pressure)
{
    float force = torque / MOMENT_ARM;
    float slope = getSlope(velocity);
    float angle = slope * force * getAltitudeScaling(pressure);
    return servoAngleToPWM(angle);
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
void moveServo(float torque, float velocity, float pressure)
{
    uint16_t pwm = calcForceToPWM(torque, velocity, pressure);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}