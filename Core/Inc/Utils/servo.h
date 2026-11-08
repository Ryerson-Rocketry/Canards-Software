#ifndef UTILS_SERVO_H_
#define UTILS_SERVO_H_
#include <stdint.h>

uint16_t moveServo(float torque, float velocity, float pressure);
void servoPadPressureTemperature(float pressurePa, float tempCentiC);

#endif
