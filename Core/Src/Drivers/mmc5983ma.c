#include "mmc5983ma.h"
#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"

#define MAG_CS_GPIO_PORT MAG_CS_PORT
#define MAG_CS_GPIO_PIN MAG_CS_PIN

static inline void MAG_CS_LOW()
{
    HAL_GPIO_WritePin(MAG_CS_GPIO_PORT, MAG_CS_GPIO_PIN, GPIO_PIN_RESET);
}

static inline void MAG_CS_HIGH()
{
    HAL_GPIO_WritePin(MAG_CS_GPIO_PORT, MAG_CS_GPIO_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef mmc5983maWrite(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg << 2 | MAG_SPI_TX, val};
    HAL_StatusTypeDef status;
    MAG_CS_LOW();
    status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
    MAG_CS_HIGH();

    if (status != HAL_OK)
    {
        // idk what we should do if it failed to read maybe print or uart
    }

    return status;
}

HAL_StatusTypeDef mmc5983maRead(uint8_t reg, uint8_t *dataBuff, int length)
{
    if (length < 0)
    {
        // idk what we should do if it failed to read maybe print or uart
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];

    // tell sensor we are reading
    tx[0] = (reg << 2) | MAG_SPI_RX;
    memset(&tx[1], 0x00, length);

    MAG_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, length + 1, HAL_MAX_DELAY);
    MAG_CS_HIGH();

    if (status != HAL_OK)
    {
        return status;
    }

    memcpy(dataBuff, &rx[1], length);
    return status;
}

void mmc5983maInit()
{
    // reset sensor
    mmc5983maWrite(MAG_CTRL_REG_1, 0x80);
    osDelay(10);

    uint8_t magCtrlReg0;

    // Set config registers
    mmc5983maRead(MAG_CTRL_REG_0, &magCtrlReg0, 1);

    // enable auto SR and data ready interrupts pins
    mmc5983maWrite(MAG_CTRL_REG_0, magCtrlReg0 | 0x24);

    uint8_t magCtrlReg1;
    mmc5983maRead(MAG_CTRL_REG_1, &magCtrlReg1, 1);

    // set output resolution and bandwidth
    mmc5983maWrite(MAG_CTRL_REG_1, magCtrlReg1 | 0x01);

    uint8_t magCtrlReg2;
    mmc5983maRead(MAG_CTRL_REG_2, &magCtrlReg2, 1);

    // set freq, enable continuous mode, set how often chip will do set op
    mmc5983maWrite(MAG_CTRL_REG_2, magCtrlReg2 | 0x9E);
    osDelay(1);
}

void mmc5983maSetOperation()
{
    uint8_t magCtrlReg0;
    mmc5983maRead(MAG_CTRL_REG_0, &magCtrlReg0, 1);
    mmc5983maWrite(MAG_CTRL_REG_0, magCtrlReg0 | 0x08);

    // 500ns delay
    for (volatile int i = 0; i < 50; i++)
    {
        __NOP();
    }
}

void mmc5983maResetOperation()
{
    uint8_t magCtrlReg0;
    mmc5983maRead(MAG_CTRL_REG_0, &magCtrlReg0, 1);
    mmc5983maWrite(MAG_CTRL_REG_0, magCtrlReg0 | 0x10);

    // 500ns delay
    for (volatile int i = 0; i < 50; i++)
    {
        __NOP();
    }
}

void mmc5983maStartMagMeas()
{
    uint8_t magCtrlReg0;
    mmc5983maRead(MAG_CTRL_REG_0, &magCtrlReg0, 1);
    mmc5983maWrite(MAG_CTRL_REG_0, magCtrlReg0 | 0x01);
}

bool mmc5983maCheckMeasIsComplete()
{
    uint8_t magStatusReg;
    mmc5983maRead(MAG_STATUS_REG, &magStatusReg, 1);

    if (magStatusReg & 0x01)
        return true;
    return false;
}

void mmc5983maClearInterrupt()
{
    uint8_t magStatusReg;
    mmc5983maRead(MAG_STATUS_REG, &magStatusReg, 1);
    mmc5983maWrite(MAG_STATUS_REG, magStatusReg | 0x01);
}

uint16_t mmc5983ReadMagXVal()
{
    uint8_t rawMag[2];
    mmc5983maRead(MAG_X_OUT_0_REG, rawMag, 2);
    return ((uint16_t)rawMag[0]) | ((uint16_t)rawMag[1] << 8);
}

uint16_t mmc5983ReadMagYVal()
{
    uint8_t rawMag[2];
    mmc5983maRead(MAG_Y_OUT_0_REG, rawMag, 2);
    return ((uint16_t)rawMag[0]) | ((uint16_t)rawMag[1] << 8);
}

uint16_t mmc5983ReadMagZVal()
{
    uint8_t rawMag[2];
    mmc5983maRead(MAG_Z_OUT_0_REG, rawMag, 2);
    return ((uint16_t)rawMag[0]) | ((uint16_t)rawMag[1] << 8);
}

uint8_t mmc5983ReadMagXYZVal()
{
    uint8_t rawMag;
    mmc5983maRead(MAG_XYZ_OUT_REG, &rawMag, 1);
    return rawMag;
}

void mmc5983GetMagData(uint32_t out[3])
{
    uint8_t xyzLast2Bits = mmc5983ReadMagXYZVal();

    uint8_t xLSB = (xyzLast2Bits >> 6) & 0x03; // Bits 7-6
    uint8_t yLSB = (xyzLast2Bits >> 4) & 0x03; // Bits 5-4
    uint8_t zLSB = (xyzLast2Bits >> 2) & 0x03; // Bits 3-2

    uint32_t magX = ((uint32_t)mmc5983ReadMagXVal() << 2) | xLSB;
    uint32_t magY = ((uint32_t)mmc5983ReadMagYVal() << 2) | yLSB;
    uint32_t magZ = ((uint32_t)mmc5983ReadMagZVal() << 2) | zLSB;

    out[0] = magX;
    out[1] = magY;
    out[2] = magZ;
}

void mmc5983maReadMagnetometer(SemaphoreHandle_t magDataReadySemaphore, float magData[4])
{
    // 1. Perform Set Operation
    mmc5983maSetOperation();

    // 2. Set TM_M to 1
    mmc5983maStartMagMeas();

    // 3. Interrupt will tell me if data is ready
    if (xSemaphoreTake(magDataReadySemaphore, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    // 4. Check Meas_M_Done to see if it is 1
    if (!mmc5983maCheckMeasIsComplete())
    {
        return;
    }

    // 5. Write 1 to Meas_M_Done to clear interrupt
    mmc5983maClearInterrupt();

    // 6. Perform Measurement
    uint32_t out1[3];
    mmc5983GetMagData(out1);

    // 7. Perform Reset Operation
    mmc5983maResetOperation();

    // 8. Set TM_M to 1
    mmc5983maStartMagMeas();

    // 9. Interrupt will tell me if data is ready
    if (xSemaphoreTake(magDataReadySemaphore, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    // 10. Check Meas_M_Done to see if it is 1
    if (!mmc5983maCheckMeasIsComplete())
    {
        return;
    }

    // 11. Write 1 to Meas_M_Done to clear interrupt
    mmc5983maClearInterrupt();

    // 12. Perform measurement
    uint32_t out2[3];
    mmc5983GetMagData(out2);

    // 13. Subtract the two measurements and divide by 2
    magData[0];
    for (int i = 0; i < 3; i++)
    {
        // in G units, if we want microTelsa multiply by 100
        magData[i + 1] = ((((int32_t)out1[i] - (int32_t)out2[i]) / 2.0f) - 131072.0f) / 16384.0f;
    }
}