#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

// CS defs (these need to be changed)
#define MAG_CS_PORT GPIOA
#define MAG_CS_PIN GPIO_PIN_0

#define MAG_SPI_TX 0x00
#define MAG_SPI_RX 0x01

// Register Addresses
#define MAG_CTRL_REG_0 0x09
#define MAG_CTRL_REG_1 0x0A
#define MAG_CTRL_REG_2 0x0B

#define MAG_STATUS_REG 0x08

#define MAG_X_OUT_0_REG 0x00
#define MAG_Y_OUT_0_REG 0x02
#define MAG_Z_OUT_0_REG 0x04

#define MAG_XYZ_OUT_REG 0x06

#define GPIO_PIN_MAG_INT_VAL GPIO_PIN_1

void mmc5983maInit();
void mmc5983maReadMagnetometer(SemaphoreHandle_t xMagDataReadySemaphore, float magData[4]);
