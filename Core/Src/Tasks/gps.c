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
bool verify_checksum(const char *sentence) {
    char sentence_copy[128];
    memset(sentence_copy, 0, 128);
    strcpy(sentence_copy, sentence);
    uint8_t checksum = 0;
    uint8_t i = 1;

    if (sentence_copy[0] != '$') 
        return false;
    
    while (sentence_copy[i] != '\0' && sentence_copy[i] != '*') {
        checksum ^= sentence_copy[i++];
    }

    if (sentence_copy[i] != '*') 
        return false;

    unsigned int expected_checksum;

    if (sscanf(&sentence_copy[i + 1], "%2X", &expected_checksum) != 1) 
        return false;
    

        return checksum == (uint8_t)expected_checksum; // might need both to be int
}

// Convert DDMM.MMMM to Decimal Degrees --- // might change to float
double parse_degrees(const char *raw_val, char direction) {
    if (strlen(raw_val) == 0) 
        return 0.0;
    double raw_num = atof(raw_val);
    int degrees = (int)(raw_num / 100);
    double minutes = raw_num - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimal_degrees = -decimal_degrees;
    return decimal_degrees;
}

// --- Helper: Safe Tokenizer (Handles empty fields like ,, safely) ---
int tokenize_sentence(char *buffer, char **tokens, int max_tokens) {
    char *asterisk = strchr(buffer, '*');
    if (asterisk) *asterisk = ','; // Normalize the checksum boundary
    
    int token_index = 0;
    char *current = buffer;
    while (current && token_index < max_tokens) {
        tokens[token_index++] = current;
        char *next_comma = strchr(current, ',');
        if (next_comma) {
            *next_comma = '\0';
            current = next_comma + 1;
        } else {
            current = NULL;
        }
    }
    return token_index;
}

// --- Parser Submodule: GNRMC ---
void parse_sub_gnrmc(char **tokens, int token_count, GNSS_Data *gnss) {
    if (token_count < 10) return;
    char *utc      = tokens[1];
    char *status   = tokens[2];
    char *lat_val  = tokens[3];
    char *lat_dir  = tokens[4];
    char *lon_val  = tokens[5];
    char *lon_dir  = tokens[6];
    char *speed    = tokens[7];
    char *date_val = tokens[9];

    gnss->has_fix = (status[0] == 'A');
    if (!gnss->has_fix) 
        return;

    if (strlen(utc) >= 6) {
        snprintf(gnss->utc_time, sizeof(gnss->utc_time), "%.2s:%.2s:%.2s", utc, utc + 2, utc + 4);
    }
    if (strlen(date_val) == 6) {
        snprintf(gnss->date, sizeof(gnss->date), "20%.2s-%.2s-%.2s", date_val + 4, date_val + 2, date_val);
    }
    gnss->latitude = parse_degrees(lat_val, lat_dir[0]);
    gnss->longitude = parse_degrees(lon_val, lon_dir[0]);
    gnss->speed_knots = (strlen(speed) > 0) ? atof(speed) : 0.0f;
    gnss->speed_kmh = gnss->speed_knots * 1.852f;
}

// --- Parser Submodule: GNGGA ---
void parse_sub_gngga(char **tokens, int token_count, GNSS_Data *gnss) {
    if (token_count < 10) 
        return;
     
    char *quality   = tokens[6];
    char *sats      = tokens[7];
    char *altitude  = tokens[9];

    gnss->fix_quality        = (strlen(quality) > 0)  ? atoi(quality)  : 0;
    gnss->satellites_tracked = (strlen(sats) > 0)     ? atoi(sats)     : 0;
    gnss->altitude_m         = (strlen(altitude) > 0) ? atof(altitude) : 0.0f;
}

// --- Parser Submodule: GNVTG ---
void parse_sub_gnvtg(char **tokens, int token_count, GNSS_Data *gnss) {
    if (token_count < 9) 
        return;
    
    char *track_true = tokens[1];
    char *spd_kph    = tokens[7];

    if (strlen(track_true) > 0) gnss->heading_true = atof(track_true);
    if (strlen(spd_kph) > 0)    gnss->speed_kmh    = atof(spd_kph);
}

// --- Parser Submodule: GNGSA ---
void parse_sub_gngsa(char **tokens, int token_count, GNSS_Data *gnss) {
    if (token_count < 18) 
        return;
    
    // Fields 3 to 14 contain the 12 possible channels mapping tracked Sat IDs
    memset(gnss->active_sat_ids, 0, sizeof(gnss->active_sat_ids));
    for (int i = 0; i < 12; i++) {
        char *sat_id = tokens[3 + i];
        if (strlen(sat_id) > 0) {
            gnss->active_sat_ids[i] = atoi(sat_id);
        }
    }
    
    char *pdop = tokens[15];
    char *hdop = tokens[16];
    char *vdop = tokens[17];

    if (strlen(pdop) > 0) gnss->pdop = atof(pdop);
    if (strlen(hdop) > 0) gnss->hdop = atof(hdop);
    if (strlen(vdop) > 0) gnss->vdop = atof(vdop);
}

// --- Master Router Function ---
bool parse_nmea_sentence(const char *sentence, GNSS_Data *gnss) {
    char sentence_copy[128]; // debug
    memset(sentence_copy, 0, 128); // debug
    strcpy(sentence_copy, sentence); //debug
    if (!verify_checksum(sentence_copy)) // change back to sentence for actual use 
        return false;

    // Local stack buffer to isolate tokens safely
    char buffer[128];
    strncpy(buffer, sentence_copy, sizeof(buffer) - 1); // change back to sentence for actual use 
    buffer[sizeof(buffer) - 1] = '\0';

    char *tokens[24] = {0};
    int token_count = tokenize_sentence(buffer, tokens, 24);
    if (token_count == 0) return false;

    char *header = tokens[0];

    // Identify header type and dispatch down to specific logic

    // CHANGE THE HEADERS BASED ON WHAT YOU GET AS ACTUAL DATA
    if (strcmp(header, "$GNRMC") == 0) {
        parse_sub_gnrmc(tokens, token_count, gnss);
        return true;
    } else if (strcmp(header, "$GNVTG") == 0) {
        parse_sub_gnvtg(tokens, token_count, gnss);
        return true;
    } else if (strcmp(header, "$GNGGA") == 0) {
        parse_sub_gngga(tokens, token_count, gnss);
        return true;
    } else if (strcmp(header, "$GNGSA") == 0) {
        parse_sub_gngga(tokens, token_count, gnss); // Maps precision parameters
        return true;
    }

    return false; // Unknown or unhandled NMEA header
}


void process_gps_data(char *gpsData, GNSS_Data *gnss) {
    char copy[512]; //debug
    memset(copy, 0, 512); // debug
    strcpy(copy, gpsData); // debug
    char *sentence_start = (char*)copy; // change back to gpsData for actual use 
    char *sentence_end;
    
    // separate into individual nmea sentences and parse them
    while ((sentence_end = strchr(sentence_start, '\n')) != NULL) {
        *sentence_end = '\0'; // Break sentence string from the flat array block
        
        // Wipe matching carriage returns (\r)
        char *r_match = strchr(sentence_start, '\r');
        if (r_match) *r_match = '\0';
        
        if (strlen(sentence_start) > 0 && sentence_start[0] == '$') {
            parse_nmea_sentence(sentence_start, gnss);
        }
        
        sentence_start = sentence_end + 1; // Slide forward to start of next sentence
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