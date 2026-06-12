#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdbool.h>


#define GPS_CS_PORT GPIOC
#define GPS_CS_PIN GPIO_PIN_14
#define GPS_BUF_SIZE 128

#ifndef GPS_H
#define GPS_H
// Structure to hold everything known about the GNSS state
typedef struct {
    // Quality Tracking
    bool has_fix;
    int fix_quality;        // 0 = Invalid, 1 = GPS Fix, 2 = DGPS Fix (from GGA)
    int satellites_tracked; // Number of satellites in use (from GGA)
    
    // Position & Time
    char utc_time[9];       // HH:MM:SS
    char date[11];          // YYYY-MM-DD
    double latitude;        // Decimal degrees
    double longitude;       // Decimal degrees
    float altitude_m;       // Altitude above sea level (meters)
    
    // Dynamics
    float speed_knots;      // Speed in knots
    float speed_kmh;        // Speed in km/h
    float heading_true;     // True track heading (degrees)
    
    // Precision Dilution (DOPs from GSA)
    float pdop;             // Position Dilution
    float hdop;             // Horizontal Dilution
    float vdop;             // Vertical Dilution
    int active_sat_ids[12]; // Array of active satellite IDs being tracked
} GNSS_Data;



static const uint8_t cfg_spiprot[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x18, 0x00,
    0x00, 0x07, 0x00, 0x00,
    0x01, 0x00, 0x7A, 0x10, 0x01, /* SPIOUTPROT-UBX  */
    0x02, 0x00, 0x7A, 0x10, 0x01, /* SPIOUTPROT-NMEA */
    0x01, 0x00, 0x79, 0x10, 0x01, /* SPIINPROT-UBX   */
    0x02, 0x00, 0x79, 0x10, 0x01, /* SPIINPROT-NMEA  */
    0xDF, 0x08};

static const uint8_t cfg_revert[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0xFF, 0xFF, 0x00, 0x00, /* clearMask  */
    0x00, 0x00, 0x00, 0x00, /* saveMask   */
    0xFF, 0xFF, 0x00, 0x00, /* loadMask   */
    0x17,                   /* deviceMask */
    0x2F, 0xAE};

void static inline CS_LOW()
{
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_RESET);
}

void static inline CS_HIGH()
{
    HAL_GPIO_WritePin(GPS_CS_PORT, GPS_CS_PIN, GPIO_PIN_SET);
}

void gpsRead(SemaphoreHandle_t gSpi2Mutex, uint8_t rx[GPS_BUF_SIZE], const uint8_t tx[GPS_BUF_SIZE]);
int gnss_send_mon_ver(void);
int gpsRepairConfig(SemaphoreHandle_t spiMutex);
void gpsSendCfg(const uint8_t *buf, uint16_t len);
void process_gps_data(char *gpsData, GNSS_Data *gnss);

#endif