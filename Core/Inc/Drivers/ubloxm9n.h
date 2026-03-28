#pragma once

#include "main.h"
#include "Defs/states.h"

HAL_StatusTypeDef GPSInit();
HAL_StatusTypeDef GPSRead(uint8_t buffer[128], uint8_t dummyTx[128], GPS *nmeaState);
void GPSWaitForBoot(void);