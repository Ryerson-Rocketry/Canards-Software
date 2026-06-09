#include "Tasks/gps.h"
#include "mmc5983ma.h"
#include "projdefs.h"
#include "spi.h"
#include "stm32f4xx_hal_gpio.h"
#include <string.h>
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal_spi.h"

extern SemaphoreHandle_t gSpi2Mutex;

void gpsSendCfg(const uint8_t *buf, uint16_t len)
{
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_128, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
        CS_LOW();
        HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 100);
        CS_HIGH();
        xSemaphoreGive(gSpi2Mutex);
    }
}

void gpsRead(SemaphoreHandle_t gSpi2Mutex, uint8_t rx[GPS_BUF_SIZE], const uint8_t tx[GPS_BUF_SIZE])
{

    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_128, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);

        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi2, tx, rx, GPS_BUF_SIZE, 100);
        CS_HIGH();

        xSemaphoreGive(gSpi2Mutex);
    }
}

static uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 10);
    return rx;
}

void gnss_spi_sync(void)
{
    // Force CS low, blast 10 bytes of empty air (0xFF) to clear the buffers, then pull CS high
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_RESET);
    for (int i = 0; i < 10; i++)
    {
        spi_xfer(0xFF);
    }
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_SET);

    HAL_Delay(10); // Let the module settle
}

int gnss_send_mon_ver(void)
{
    /* Exact bytes from the document: B5 62 0A 04 00 00 0E 34 */
    const uint8_t mon_ver_poll[8] = {
        0xB5, 0x62, /* UBX sync chars */
        0x0A, 0x04, /* class MON, ID VER */
        0x00, 0x00, /* payload length = 0 (little-endian) */
        0x0E, 0x34  /* checksum CK_A, CK_B */
    };
    if (xSemaphoreTake(gSpi2Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {

        SPI2_Switch_Settings(SPI_BAUDRATEPRESCALER_128, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);

        uint8_t rxbuf[256] = {0};
        uint8_t ff_count = 0;

        /* 1. Send the Command */
        HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_RESET);

        for (int i = 0; i < 8; i++)
        {
            spi_xfer(mon_ver_poll[i]);
        }

        HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_SET);

        /* 2. Give the u-blox CPU time to process and queue the response */
        /* If using FreeRTOS, replace with osDelay(50) or vTaskDelay(pdMS_TO_TICKS(50)) */
        HAL_Delay(100);

        /* 3. Read the Response - Keep CS LOW for entire transaction */
        HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_RESET);

        for (int i = 0; i < 256; i++)
        {
            rxbuf[i] = spi_xfer(0xFF);

            if (rxbuf[i] == 0xFF)
            {
                ff_count++;
                // /* Give it a bit more leeway, u-blox SPI buffers can have dead bytes */
                // if (ff_count > 200) break;
            }
            else
            {
                ff_count = 0;
            }
        }
        HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_SET);

        /* 4. Parse the Buffer Safely */
        for (int i = 0; i < 250; i++)
        {
            if (rxbuf[i] == 0xB5 &&
                rxbuf[i + 1] == 0x62 &&
                rxbuf[i + 2] == 0x0A &&
                rxbuf[i + 3] == 0x04)
            {

                /* Bounds Check: Header(6) + Payload(40) + Checksum(2) = 48 bytes */
                if (i + 48 > 256)
                {
                    return 0; /* Packet is truncated, buffer too small */
                }

                uint16_t plen = rxbuf[i + 4] | (rxbuf[i + 5] << 8);

                char sw_version[31] = {0};
                char hw_version[11] = {0};
                memcpy(sw_version, &rxbuf[i + 6], 30);
                memcpy(hw_version, &rxbuf[i + 36], 10);

                /* printf("GNSS SW: %s\r\n", sw_version); */
                /* printf("GNSS HW: %s\r\n", hw_version); */
                xSemaphoreGive(gSpi2Mutex);

                return 1; /* success */
            }
        }
    }
    xSemaphoreGive(gSpi2Mutex);
    return 0; /* no valid response found */
}
