#include "Tasks/radio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "Defs/states.h"
#include "Drivers/rfm9x.h"
#include <stdio.h>

extern Rocket_States_t Rocket;

// Builds the telemetry CSV line, returns its length or -1 if it does not fit.
// Reads live data because the SD snapshot is not updated on the pad.
int Radio_BuildPacket(char *buffer, size_t size)
{
    float accel[3];
    float gyro[3];
    float pwmAngle;
    FlightState_t flightState;
    GpsFix_t gps;

    taskENTER_CRITICAL();
    for (int i = 0; i < 3; i++)
    {
        accel[i] = Rocket.rawData.accel[i];
        gyro[i] = Rocket.rawData.gyro[i];
    }
    pwmAngle = Rocket.control.pwmAngle;
    flightState = Rocket.flightState;
    gps = Rocket.gps;
    taskEXIT_CRITICAL();

    if (!gps.has_fix)
    {
        gps = (GpsFix_t){0};
    }

    int len = snprintf(buffer, size,
                       RFM9X_CALLSIGN ",%lu,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%ld,%ld,%ld,%ld",
                       (unsigned long)osKernelGetTickCount(),
                       (int)flightState,
                       (long)(accel[0] * 100),
                       (long)(accel[1] * 100),
                       (long)(accel[2] * 100),
                       (long)(gyro[0] * 100),
                       (long)(gyro[1] * 100),
                       (long)(gyro[2] * 100),
                       (long)(pwmAngle * 100),
                       (int)gps.has_fix,
                       (long)(gps.latitude * 1e7),
                       (long)(gps.longitude * 1e7),
                       (long)(gps.altitude_m * 100.0f),
                       (long)(gps.speed_kmh * 100.0f));

    if (len < 0 || (size_t)len >= size)
    {
        return -1;
    }

    return len;
}
