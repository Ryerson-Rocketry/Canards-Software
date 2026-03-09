#include "tim.h"
#include "Utils/servo.h"

void moveServo(float angle)
{
    // lookup table/equation to convert angle to pwm value
    float pwmVal = angle;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmVal);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmVal);
}