#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>

void sensor_HardwareInit(void);
void sensor_ReadMagnetometer(void);
void sensor_ReadBarometer(void);
void sensor_ReadIMUAccelerometer(void);
void sensor_ReadIMUGyro(void);
void sensor_GroundReference(void);

#endif // SENSOR_H