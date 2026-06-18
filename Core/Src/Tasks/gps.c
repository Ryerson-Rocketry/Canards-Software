#include "Tasks/gps.h"
#include "mmc5983ma.h"
#include "projdefs.h"
#include "spi.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal_spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

extern SemaphoreHandle_t gSpi2Mutex;

// --- Helper: Validate NMEA Checksum ($...*XX) ---
bool verify_checksum(const char *sentence)
{
    char sentence_copy[128];
    memset(sentence_copy, 0, 128);
    strcpy(sentence_copy, sentence);
    uint8_t checksum = 0;
    uint8_t i = 1;

    if (sentence_copy[0] != '$')
        return false;

    while (sentence_copy[i] != '\0' && sentence_copy[i] != '*')
    {
        checksum ^= sentence_copy[i++];
    }

    if (sentence_copy[i] != '*')
        return false;

    unsigned int expected_checksum;

    if (sscanf(&sentence_copy[i + 1], "%2X", &expected_checksum) != 1)
        return false;

    return checksum == (uint8_t)expected_checksum;
}

// Convert DDMM.MMMM to Decimal Degrees
double parse_degrees(const char *raw_val, char direction)
{
    if (strlen(raw_val) == 0)
        return 0.0;
    double raw_num = atof(raw_val);
    int degrees = (int)(raw_num / 100);
    double minutes = raw_num - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W')
        decimal_degrees = -decimal_degrees;
    return decimal_degrees;
}

// --- Helper: Safe Tokenizer (Handles empty fields like ,, safely) ---
int tokenize_sentence(char *buffer, char **tokens, int max_tokens)
{
    char *asterisk = strchr(buffer, '*');
    if (asterisk)
        *asterisk = ','; // Normalize the checksum boundary

    int token_index = 0;
    char *current = buffer;
    while (current && token_index < max_tokens)
    {
        tokens[token_index++] = current;
        char *next_comma = strchr(current, ',');
        if (next_comma)
        {
            *next_comma = '\0';
            current = next_comma + 1;
        }
        else
        {
            current = NULL;
        }
    }
    return token_index;
}

// --- Parser Submodule: GNRMC (lat/lon/speed/fix) ---
void parse_sub_gnrmc(char **tokens, int token_count, GNSS_Data *gnss)
{
    if (token_count < 10)
        return;
    char *status = tokens[2];
    char *lat_val = tokens[3];
    char *lat_dir = tokens[4];
    char *lon_val = tokens[5];
    char *lon_dir = tokens[6];
    char *speed = tokens[7];

    gnss->has_fix = (status[0] == 'A');
    if (!gnss->has_fix)
        return;

    gnss->latitude = parse_degrees(lat_val, lat_dir[0]);
    gnss->longitude = parse_degrees(lon_val, lon_dir[0]);
    gnss->speed_kmh = (strlen(speed) > 0) ? atof(speed) * 1.852f : 0.0f;
}

// --- Parser Submodule: GNGGA (altitude) ---
void parse_sub_gngga(char **tokens, int token_count, GNSS_Data *gnss)
{
    if (token_count < 10)
        return;

    char *altitude = tokens[9];
    gnss->altitude_m = (strlen(altitude) > 0) ? atof(altitude) : 0.0f;
}

// --- Master Router Function ---
bool parse_nmea_sentence(const char *sentence, GNSS_Data *gnss)
{
    char sentence_copy[128];
    memset(sentence_copy, 0, 128);
    strcpy(sentence_copy, sentence);
    if (!verify_checksum(sentence_copy))
        return false;

    char buffer[128];
    strncpy(buffer, sentence_copy, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *tokens[24] = {0};
    int token_count = tokenize_sentence(buffer, tokens, 24);
    if (token_count == 0)
        return false;

    char *header = tokens[0];

    if (strcmp(header, "$GNRMC") == 0)
    {
        parse_sub_gnrmc(tokens, token_count, gnss);
        return true;
    }
    else if (strcmp(header, "$GNGGA") == 0)
    {
        parse_sub_gngga(tokens, token_count, gnss);
        return true;
    }

    return false;
}

void process_gps_data(char *gpsData, GNSS_Data *gnss)
{
    char copy[GPS_BUF_SIZE];
    memset(copy, 0, GPS_BUF_SIZE);
    strncpy(copy, gpsData, GPS_BUF_SIZE - 1);
    copy[GPS_BUF_SIZE - 1] = '\0';
    char *sentence_start = copy;
    char *sentence_end;

    while ((sentence_end = strchr(sentence_start, '\n')) != NULL)
    {
        *sentence_end = '\0';

        char *r_match = strchr(sentence_start, '\r');
        if (r_match)
            *r_match = '\0';

        if (strlen(sentence_start) > 0 && sentence_start[0] == '$')
        {
            parse_nmea_sentence(sentence_start, gnss);
        }

        sentence_start = sentence_end + 1;
    }
}

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