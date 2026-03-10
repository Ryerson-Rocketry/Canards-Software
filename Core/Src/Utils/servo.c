#include "tim.h"
#include "Utils/servo.h"
#include "main.h"
#include "cmsis_os2.h"

void moveServo(float angle)
{
    // lookup table/equation to convert angle to pwm value
    float pwmVal = angle;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmVal);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmVal);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}