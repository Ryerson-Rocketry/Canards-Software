#pragma once

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define GPS_CS_PORT GPIOC
#define GPS_CS_PIN GPIO_PIN_14
#define GPS_BUF_SIZE 128

void static inline CS_LOW()
{
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_RESET);
}

void static inline CS_HIGH()
{
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_SET);
}

void gpsRead(SemaphoreHandle_t gSpi2Mutex, uint8_t rx[GPS_BUF_SIZE], const uint8_t tx[GPS_BUF_SIZE]);
int gnss_send_mon_ver(void);
