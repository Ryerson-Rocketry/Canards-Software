#pragma once

#include "main.h"
#include "Defs/states.h"

HAL_StatusTypeDef GPSInit();
HAL_StatusTypeDef GPSRead(uint8_t buffer[GPS_BUF_SIZE], uint8_t dummyTx[GPS_BUF_SIZE], GPS *nmeaState);