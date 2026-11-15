#include "Drivers/rfm9x.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "spi.h"

#if RFM9X_TX_POWER_DBM < 2 || RFM9X_TX_POWER_DBM > 17
#error "RFM9X_TX_POWER_DBM must be 2 to 17, +20 dBm is limited to 1% duty cycle"
#endif

#define RFM9X_SPI_TIMEOUT_MS 10
#define RFM9X_MUTEX_TIMEOUT_MS 100
#define RFM9X_TX_POLL_MS 10

#define RFM9X_SPI_WRITE 0x80
#define RFM9X_OPMODE_BASE (RFM9X_OPMODE_LORA | RFM9X_OPMODE_LOW_FREQ)
#define RFM9X_PACONFIG_PA_BOOST 0x80
#define RFM9X_MODEMCONFIG2_CRC 0x04
#define RFM9X_MODEMCONFIG3_AGC_AUTO 0x04
#define RFM9X_MODEMCONFIG3_LDRO 0x08

/* Frf = frequency * 2^19 / 32 MHz crystal */
#define RFM9X_FRF ((uint32_t)(((uint64_t)RFM9X_FREQUENCY_HZ << 19) / 32000000ULL))

/* Low data rate optimize is required once a symbol is longer than 16 ms */
#define RFM9X_NEEDS_LDRO (RFM9X_SPREADING_FACTOR + RFM9X_BW_125KHZ - RFM9X_BANDWIDTH >= 11)

extern SemaphoreHandle_t gSpi1Mutex;

typedef struct
{
    uint8_t reg;
    uint8_t value;
} RFM9X_RegValue;

static const RFM9X_RegValue rfm9xConfig[] = {
    {RFM9X_REG_FRFMSB, (uint8_t)(RFM9X_FRF >> 16)},
    {RFM9X_REG_FRFMID, (uint8_t)(RFM9X_FRF >> 8)},
    {RFM9X_REG_FRFLSB, (uint8_t)RFM9X_FRF},
    {RFM9X_REG_FIFOTXBASEADDR, 0x00},
    {RFM9X_REG_MODEMCONFIG1, (RFM9X_BANDWIDTH << 4) | (RFM9X_CODING_RATE << 1)}, /* bit 0 clear = explicit header */
    {RFM9X_REG_MODEMCONFIG2, (RFM9X_SPREADING_FACTOR << 4) | (RFM9X_CRC_ON ? RFM9X_MODEMCONFIG2_CRC : 0)},
    {RFM9X_REG_MODEMCONFIG3, RFM9X_MODEMCONFIG3_AGC_AUTO | (RFM9X_NEEDS_LDRO ? RFM9X_MODEMCONFIG3_LDRO : 0)},
    {RFM9X_REG_PREAMBLEMSB, (uint8_t)(RFM9X_PREAMBLE_LENGTH >> 8)},
    {RFM9X_REG_PREAMBLELSB, (uint8_t)RFM9X_PREAMBLE_LENGTH},
    {RFM9X_REG_SYNCWORD, RFM9X_SYNC_WORD},
    {RFM9X_REG_PACONFIG, RFM9X_PACONFIG_PA_BOOST | (RFM9X_TX_POWER_DBM - 2)},
};

static inline void CS_LOW(void)
{
    HAL_GPIO_WritePin(RFM9X_CS_PORT, RFM9X_CS_PIN, GPIO_PIN_RESET);
}

static inline void CS_HIGH(void)
{
    HAL_GPIO_WritePin(RFM9X_CS_PORT, RFM9X_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef rfm9xRead(uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    reg &= 0x7F; // MSB clear for READ command

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(RFM9X_MUTEX_TIMEOUT_MS)) == pdTRUE)
    {
        CS_LOW();
        status = HAL_SPI_Transmit(&hspi1, &reg, 1, RFM9X_SPI_TIMEOUT_MS);

        if (status == HAL_OK)
        {
            status = HAL_SPI_Receive(&hspi1, val, 1, RFM9X_SPI_TIMEOUT_MS);
        }

        CS_HIGH();
        xSemaphoreGive(gSpi1Mutex);
    }

    return status;
}

static HAL_StatusTypeDef rfm9xWrite(uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t writePayload[2] = {reg | RFM9X_SPI_WRITE, val};

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(RFM9X_MUTEX_TIMEOUT_MS)) == pdTRUE)
    {
        CS_LOW();
        status = HAL_SPI_Transmit(&hspi1, writePayload, 2, RFM9X_SPI_TIMEOUT_MS);
        CS_HIGH();
        xSemaphoreGive(gSpi1Mutex);
    }

    return status;
}

// Writes the RadioHead header and payload in one SPI transaction
static HAL_StatusTypeDef rfm9xWriteFifo(const uint8_t *data, uint8_t len)
{
    static uint8_t packetId = 0;
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t header[1 + RFM9X_HEADER_LEN] = {
        RFM9X_REG_FIFO | RFM9X_SPI_WRITE,
        RFM9X_HEADER_TO,
        RFM9X_HEADER_FROM,
        packetId++,
        RFM9X_HEADER_FLAGS,
    };

    if (xSemaphoreTake(gSpi1Mutex, pdMS_TO_TICKS(RFM9X_MUTEX_TIMEOUT_MS)) == pdTRUE)
    {
        CS_LOW();
        status = HAL_SPI_Transmit(&hspi1, header, sizeof(header), RFM9X_SPI_TIMEOUT_MS);

        if (status == HAL_OK)
        {
            status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, RFM9X_SPI_TIMEOUT_MS);
        }

        CS_HIGH();
        xSemaphoreGive(gSpi1Mutex);
    }

    return status;
}

static HAL_StatusTypeDef rfm9xSetMode(uint8_t mode)
{
    return rfm9xWrite(RFM9X_REG_OPMODE, RFM9X_OPMODE_BASE | mode);
}

static void rfm9xManualReset(void)
{
    HAL_GPIO_WritePin(RFM9X_RST_PORT, RFM9X_RST_PIN, GPIO_PIN_RESET);
    osDelay(2);
    HAL_GPIO_WritePin(RFM9X_RST_PORT, RFM9X_RST_PIN, GPIO_PIN_SET);
    osDelay(10);
}

HAL_StatusTypeDef rfm9xInit(void)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0;

    // 1. manual reset
    rfm9xManualReset();

    // 2. check the radio is answering
    status = rfm9xRead(RFM9X_REG_VERSION, &value);
    if (status != HAL_OK)
        return status;

    if (value != RFM9X_VERSION)
        return HAL_ERROR;

    // 3. LoRa mode can only be selected from sleep
    status = rfm9xWrite(RFM9X_REG_OPMODE, RFM9X_OPMODE_LOW_FREQ | RFM9X_MODE_SLEEP);
    if (status != HAL_OK)
        return status;

    status = rfm9xSetMode(RFM9X_MODE_SLEEP);
    if (status != HAL_OK)
        return status;

    status = rfm9xRead(RFM9X_REG_OPMODE, &value);
    if (status != HAL_OK)
        return status;

    if (value != (RFM9X_OPMODE_BASE | RFM9X_MODE_SLEEP))
        return HAL_ERROR;

    // 4. apply the RF settings
    for (uint32_t i = 0; i < sizeof(rfm9xConfig) / sizeof(rfm9xConfig[0]); i++)
    {
        status = rfm9xWrite(rfm9xConfig[i].reg, rfm9xConfig[i].value);
        if (status != HAL_OK)
            return status;
    }

    // 5. wait in standby, ready to transmit
    return rfm9xSetMode(RFM9X_MODE_STANDBY);
}

HAL_StatusTypeDef rfm9xTransmit(const uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t irqFlags = 0;

    if (data == NULL || len == 0 || len > RFM9X_MAX_PAYLOAD)
        return HAL_ERROR;

    // 1. the FIFO can only be written in standby
    status = rfm9xSetMode(RFM9X_MODE_STANDBY);
    if (status != HAL_OK)
        return status;

    // 2. load the packet at the start of the FIFO
    status = rfm9xWrite(RFM9X_REG_FIFOADDRPTR, 0x00);
    if (status != HAL_OK)
        return status;

    status = rfm9xWriteFifo(data, len);
    if (status != HAL_OK)
        return status;

    status = rfm9xWrite(RFM9X_REG_PAYLOADLENGTH, len + RFM9X_HEADER_LEN);
    if (status != HAL_OK)
        return status;

    // 3. clear old flags, then start transmitting
    status = rfm9xWrite(RFM9X_REG_IRQFLAGS, 0xFF);
    if (status != HAL_OK)
        return status;

    status = rfm9xSetMode(RFM9X_MODE_TX);
    if (status != HAL_OK)
        return status;

    // 4. poll TxDone, the radio returns to standby by itself when done
    uint32_t startTick = HAL_GetTick();

    do
    {
        osDelay(RFM9X_TX_POLL_MS);

        status = rfm9xRead(RFM9X_REG_IRQFLAGS, &irqFlags);
        if (status != HAL_OK)
            return status;

        if (HAL_GetTick() - startTick > RFM9X_TX_TIMEOUT_MS)
        {
            rfm9xSetMode(RFM9X_MODE_STANDBY);
            return HAL_TIMEOUT;
        }
    } while (!(irqFlags & RFM9X_IRQ_TXDONE));

    // 5. clear TxDone for the next packet
    return rfm9xWrite(RFM9X_REG_IRQFLAGS, RFM9X_IRQ_TXDONE);
}
