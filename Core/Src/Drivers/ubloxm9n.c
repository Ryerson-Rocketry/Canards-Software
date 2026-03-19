#include "Drivers/ubloxm9n.h"
#include "main.h"
#include "spi.h"
#include "task.h"
#include <string.h>
#include "Configs/flight_configs.h"

#define CS_PORT GPIOC
#define CS_PIN GPIO_PIN_14

static char nmea_line[128];
static int line_idx = 0;

static inline void CS_LOW(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 50; i++)
        ; // Hardware setup time
}

static inline void CS_HIGH(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef GPSInit(void)
{
    uint8_t dummy;
    uint8_t tx = 0xFF;
    CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, &tx, &dummy, 1, 100);
    CS_HIGH();
    return status;
}

HAL_StatusTypeDef GPSRead(uint8_t buffer[GPS_BUF_SIZE], uint8_t dummyTx[GPS_BUF_SIZE], GPS *nmeaState)
{
    CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, dummyTx, buffer, GPS_BUF_SIZE, 100);
    CS_HIGH();

    if (status != HAL_OK)
        return HAL_ERROR;

    for (int i = 0; i < GPS_BUF_SIZE; i++)
    {
        uint8_t c = buffer[i];

        if (c == 0xFF || c == 0x00)
            continue;

        if (c == '$')
        {
            line_idx = 0;
        }

        if (line_idx < (sizeof(nmea_line) - 1))
        {
            nmea_line[line_idx++] = (char)c;

            if (c == '\n')
            {
                nmea_line[line_idx] = '\0';

                // CRITICAL: Protect the GPS struct from being read by the SD task mid-parse
                taskENTER_CRITICAL();
                nmea_parse(nmeaState, nmea_line);
                taskEXIT_CRITICAL();

                line_idx = 0;
            }
        }
        else
        {
            line_idx = 0; // Overflow safety
        }
    }
    return HAL_OK;
}