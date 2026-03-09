#include <stm32f4xx_hal_tim.h>
extern TIM_HandleTypeDef htim3;

void moveServo(float angle)
{
    for (;;)
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
    }
}