#include <stdbool.h>
#include "main.h"

void bmi088SoftReset(bool gyro, bool accel)
{

    // don't soft reset anything
    if (!gyro && !accel)
    {
        return;
    }

    // soft reset gyro only
    if (!accel)
    {
        
        return;
    }

    // soft reset accel only
    if (!gyro)
    {
        
    }

    // soft reset both

    // write 0xB6 to 0x14 to soft reset gyro
}