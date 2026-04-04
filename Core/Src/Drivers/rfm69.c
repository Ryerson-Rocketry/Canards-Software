#include "rfm69.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include "spi.h"
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Defs/radio_states.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

static inline void CS_LOW(void)
{
    HAL_GPIO_WritePin(RADIO_CS_PORT, RADIO_CS_PIN, GPIO_PIN_RESET);
}

static inline void CS_HIGH(void)
{
    HAL_GPIO_WritePin(RADIO_CS_PORT, RADIO_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef rfm69Read(uint8_t reg, uint8_t *buf, int len)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    reg &= 0x7F; // Set MSB to 1 for READ command

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {

        SPI1_Switch_Settings(SPI_BAUDRATEPRESCALER_8,
                             SPI_POLARITY_LOW, // CPOL=0
                             SPI_PHASE_1EDGE);

        CS_LOW();

        status = HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);

        if (status != HAL_OK)
        {
            CS_HIGH();
            xSemaphoreGive(gSpi1Mutex);
            return status;
        }

        status = HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
        CS_HIGH();
        xSemaphoreGive(gSpi1Mutex);
    }
    return status;
}

HAL_StatusTypeDef rfm69Write(uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t writePayload[2];
    writePayload[0] = reg | 0x80; // Set MSB to 1 for WRITE command
    writePayload[1] = val;

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {

        SPI1_Switch_Settings(SPI_BAUDRATEPRESCALER_8,
                             SPI_POLARITY_LOW, // CPOL=0
                             SPI_PHASE_1EDGE);

        CS_LOW();
        status = HAL_SPI_Transmit(&hspi1, writePayload, 2, HAL_MAX_DELAY);
        CS_HIGH();

        xSemaphoreGive(gSpi1Mutex);
    }

    return status;
}

void rfm69ManualReset()
{
    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(10));
    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
    osDelay(pdMS_TO_TICKS(50));
}

HAL_StatusTypeDef rfm69SwitchModes(uint8_t mode)
{
    HAL_StatusTypeDef status;
    uint8_t currentOpMode;

    status = rfm69Read(REG_OPMODE, &currentOpMode, 1);

    if (status != HAL_OK)
    {
        return status;
    }

    // clear current mode
    currentOpMode &= ~0x1C;

    status = rfm69Write(REG_OPMODE, currentOpMode | mode);

    return status;
}

HAL_StatusTypeDef rfm69SetDataMod(uint8_t configVal)
{
    HAL_StatusTypeDef status;
    uint8_t currDataModVal;

    status = rfm69Read(REG_DATAMODUL, &currDataModVal, 1);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(REG_DATAMODUL, configVal);

    return status;
}

HAL_StatusTypeDef rfm69SetBitrate(RFM69_Bitrate_Selector rate)
{
    BitrateRegisters regs;
    HAL_StatusTypeDef status;

    switch (rate)
    {
    case RFM69_BITRATE_250KBPS:
        regs = (BitrateRegisters){0x00, 0x80}; // RadioHead Default
        break;
    case RFM69_BITRATE_9600BPS:
        regs = (BitrateRegisters){0x0D, 0x05}; // 32MHz / 9600
        break;
    case RFM69_BITRATE_4800BPS:
        regs = (BitrateRegisters){0x1A, 0x0B}; // 32MHz / 4800
        break;
    default:
        return HAL_ERROR;
    }

    status = rfm69Write(REG_BITRATEMSB, regs.msb);
    if (status != HAL_OK)
        return status;

    status = rfm69Write(REG_BITRATELSB, regs.lsb);
    return status;
}

HAL_StatusTypeDef rfm69SetBand(RFM69_Band band)
{
    FreqRegisters regs;
    HAL_StatusTypeDef status;

    switch (band)
    {
    case RFM69_BAND_433MHZ:
        regs = (FreqRegisters){0x6C, 0x40, 0x00};
        break;
    case RFM69_BAND_868MHZ:
        regs = (FreqRegisters){0xD9, 0x00, 0x00};
        break;
    case RFM69_BAND_915MHZ:
        regs = (FreqRegisters){0xE4, 0xC0, 0x00};
        break;
    default:
        return HAL_ERROR;
    }

    status = rfm69Write(0x07, regs.msb);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(0x08, regs.mid);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(0x09, regs.lsb);
    return status;
}

HAL_StatusTypeDef rfm69SetTxPower(int8_t power_dbm)
{
    // 1. Clamp the input to the physical limits of the PA1 formula
    if (power_dbm < -18)
        power_dbm = -18;
    if (power_dbm > 13)
        power_dbm = 13;

    // 2. Calculate OutputPower (Bits 4-0)
    // Formula: Pout = -18 + OutputPower => OutputPower = Pout + 18
    uint8_t outputPowerVal = (uint8_t)(power_dbm + 18);

    // 3. Construct the register byte:
    // Bit 7: Pa0On = 0 (Off)
    // Bit 6: Pa1On = 1 (On, connected to PA_BOOST for HCW)
    // Bit 5: Pa2On = 0 (Off)
    // Bits 4-0: OutputPower
    uint8_t regVal = 0x40 | (outputPowerVal & 0x1F);

    return rfm69Write(REG_PALEVEL, regVal);
}

HAL_StatusTypeDef rfm69SyncWords(uint8_t syncWord1, uint8_t syncWord2)
{
    HAL_StatusTypeDef status;
    uint8_t syncConfigVal;

    status = rfm69Read(REG_SYNCCONFIG, &syncConfigVal, 1);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(REG_SYNCCONFIG, syncConfigVal | 0x88);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(REG_SYNCVALUE1, syncWord1);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(REG_SYNCVALUE2, syncWord2);

    return status;
}

static HAL_StatusTypeDef rfm69WaitForModeReady(void)
{
    uint8_t flags = 0;
    uint32_t timeout = HAL_GetTick();

    do
    {
        HAL_StatusTypeDef status = rfm69Read(REG_IRQFLAGS1, &flags, 1);
        if (status != HAL_OK)
            return status;

        if (HAL_GetTick() - timeout > 500)
            return HAL_TIMEOUT;

    } while (!(flags & 0x80)); // Bit 7 = ModeReady

    return HAL_OK;
}

uint8_t assemble_packet_config1(PacketFormat_t format,
                                DcFree_t dcFree,
                                CrcOn_t crcOn,
                                CrcAutoClearOff_t crcClear,
                                AddressFiltering_t addrFilter)
{
    uint8_t regValue = 0;

    regValue = (format << RF_PACKETCONFIG1_FORMAT_SHIFT) |
               (dcFree << RF_PACKETCONFIG1_DCFREE_SHIFT) |
               (crcOn << RF_PACKETCONFIG1_CRCON_SHIFT) |
               (crcClear << RF_PACKETCONFIG1_CRCAUTOCLEAROFF_SHIFT) |
               (addrFilter << RF_PACKETCONFIG1_ADDRESSFILTERING_SHIFT);

    return regValue;
}

HAL_StatusTypeDef rfm69ConfigPacketHandler()
{
    HAL_StatusTypeDef status;

    uint8_t config1 = assemble_packet_config1(
        PACKET_FORMAT_VARIABLE, DC_FREE_WHITENING,
        CRC_ON, CRC_AUTO_CLEAR_ON, ADDRESS_FILTERING_NONE);

    status = rfm69Write(REG_PACKETCONFIG1, config1);

    if (status != HAL_OK)
    {
        return status;
    }

    status = rfm69Write(REG_PAYLOADLENGTH, 66);

    return status;
}

HAL_StatusTypeDef rfm69Init()
{
    HAL_StatusTypeDef status;

    // 1. manual reset
    rfm69ManualReset();

    uint8_t version = 0;
    rfm69Read(0x10, &version, 1);
    if (version != 0x24)
        return HAL_ERROR;

    // 2. switch to standby mode
    status = rfm69SwitchModes(STANDBY);

    if (status != HAL_OK)
    {
        return status;
    }

    // 3. set data modulation
    status = rfm69SetDataMod(0x00);

    if (status != HAL_OK)
    {
        return status;
    }

    // 4. set bitrate
    status = rfm69SetBitrate(RFM69_BITRATE_250KBPS);

    if (status != HAL_OK)
    {
        return status;
    }

    // 5. set frequency
    status = rfm69SetBand(RFM69_BAND_433MHZ);

    if (status != HAL_OK)
    {
        return status;
    }

    // 6. turn on transmitter power
    status = rfm69SetTxPower(13);
    if (status != HAL_OK)
    {

        return status;
    }

    // 7. setup sync word
    status = rfm69SyncWords(SYNC_VALUE_1, SYNC_VALUE_2);

    if (status != HAL_OK)
    {

        return status;
    }

    // 8. Configure Packet Handler
    status = rfm69ConfigPacketHandler();
    if (status != HAL_OK)
        return status;

    return status;
}

HAL_StatusTypeDef rfm69Transmit(uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;

    // 1. Go to standby and wait until settled
    status = rfm69SwitchModes(STANDBY);
    if (status != HAL_OK)
        return status;

    status = rfm69WaitForModeReady();
    if (status != HAL_OK)
        return status;

    // 2. Write length byte to FIFO
    status = rfm69Write(REG_FIFO, len);
    if (status != HAL_OK)
        return status;

    // 3. Write payload bytes into FIFO
    for (uint8_t i = 0; i < len; i++)
    {
        status = rfm69Write(REG_FIFO, data[i]);
        if (status != HAL_OK)
            return status;
    }

    // 4. Switch to TX
    status = rfm69SwitchModes(TX);
    if (status != HAL_OK)
        return status;

    status = rfm69WaitForModeReady();
    if (status != HAL_OK)
        return status;

    // 5. Poll PacketSent flag (bit 3 of IRQFLAGS2)
    uint8_t irqFlags = 0;
    uint32_t timeout = HAL_GetTick();

    do
    {
        status = rfm69Read(REG_IRQFLAGS2, &irqFlags, 1);
        if (status != HAL_OK)
            return status;

        if (HAL_GetTick() - timeout > 500)
            return HAL_TIMEOUT;

    } while (!(irqFlags & 0x08));

    // 6. Return to standby when done
    return rfm69SwitchModes(STANDBY);
}