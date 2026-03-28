#include "Drivers/ubloxm9n.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cmsis_os.h"
#include <string.h>
#include "Configs/flight_configs.h"
#include "Utils/nmea_parse.h"
#include <stdio.h>

extern SemaphoreHandle_t gSpi2Mutex;
void SPI2_Switch_Settings(uint32_t prescaler, uint32_t cpol, uint32_t cpha);

#define CS_PORT GPIOC
#define CS_PIN GPIO_PIN_14

static char nmea_line[128];
static int line_idx = 0;

static inline void CS_LOW(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 50; i++)
        ;
}

static inline void CS_HIGH(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

static void GPS_SPI_Read(uint8_t *rx, uint8_t *tx, uint16_t len)
{
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        return;

    SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_64, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 200);
    CS_HIGH();
    SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_4, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);

    xSemaphoreGive(gSpi2Mutex);
}

void GPSWaitForBoot(void)
{
    /* Clock out dummy bytes during the M9N boot window.
       The module outputs NMEA automatically — no config needed. */
    uint8_t dummy_rx[16];
    uint8_t dummy_tx[16];
    memset(dummy_tx, 0xFF, sizeof(dummy_tx));
    GPS_SPI_Read(dummy_rx, dummy_tx, sizeof(dummy_tx));
    printf("[GPS] Boot wait done\r\n");
}

HAL_StatusTypeDef GPSRead(uint8_t *buffer, uint8_t *dummyTx, GPS *nmeaState)
{
    if (nmeaState == NULL || buffer == NULL || dummyTx == NULL)
        return HAL_ERROR;

    GPS_SPI_Read(buffer, dummyTx, GPS_BUF_SIZE);

    /* Parse byte by byte */
    for (int i = 0; i < GPS_BUF_SIZE; i++)
    {
        uint8_t c = buffer[i];

        /* Skip idle bytes — M9N outputs 0xFF when it has nothing to send */
        if (c == 0xFF || c == 0x00)
            continue;

        /* Start of a new sentence */
        if (c == '$')
            line_idx = 0;

        if (line_idx < (int)(sizeof(nmea_line) - 1))
        {
            nmea_line[line_idx++] = (char)c;

            if (c == '\n')
            {
                nmea_line[line_idx] = '\0';

                /* Work on a local copy so nmeaState is only touched once */
                char line_copy[128];
                memcpy(line_copy, nmea_line, line_idx + 1);
                line_idx = 0;

                nmea_parse(nmeaState, line_copy);
            }
        }
        else
        {
            /* Line buffer overflow — reset and wait for next $ */
            line_idx = 0;
        }
    }

    return HAL_OK;
}