#include <stdio.h> 
#include "Utils/i2c_scanner.h"
#include "i2c.h"

void i2cScanner(void){ 
    uint16_t i2cAddress; 
    for (i2cAddress = 0; i2cAddress < 128; i2cAddress++) 
    { 
        if (HAL_I2C_IsDeviceReady(&hi2c1, (i2cAddress << 1), 1, 10) == HAL_OK) 
        { 
            // printf("I2C device found at address: 0x%02X\r\n", i2cAddress);
        } 
    } 
}