#include "Utils/nmea_parse.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double nmea_to_dec(float nmea_coord, char direction)
{
    int degrees = (int)(nmea_coord / 100);
    float minutes = nmea_coord - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W')
        decimal *= -1;
    return decimal;
}

int gps_checksum(char *nmea_data)
{
    char *start = strchr(nmea_data, '$');
    char *end = strchr(nmea_data, '*');
    if (!start || !end)
        return 0;

    int crc = 0;
    for (char *p = start + 1; p < end; p++)
        crc ^= *p;

    int receivedHash = (int)strtol(end + 1, NULL, 16);
    return (crc == receivedHash);
}

void nmea_parse(GPS *gps_data, char *input)
{
    if (!gps_checksum(input))
        return;

    float utc, lat, lon, alt, hdop, speed;
    char latDir, lonDir;
    int quality, sats;

    // GGA: Position and Satellites (Supports $GPGGA, $GNGGA)
    if (strstr(input, "GGA"))
    {
        if (sscanf(input, "$%*2sGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f",
                   &utc, &lat, &latDir, &lon, &lonDir, &quality, &sats, &hdop, &alt) >= 9)
        {
            gps_data->latitude = nmea_to_dec(lat, latDir);
            gps_data->longitude = nmea_to_dec(lon, lonDir);
            gps_data->altitude = alt;
            gps_data->satelliteCount = sats;
            gps_data->fix = (quality > 0);
            gps_data->hdop = hdop;
        }
    }
    // RMC: Speed (Supports $GPRMC, $GNRMC)
    else if (strstr(input, "RMC"))
    {
        char status;
        if (sscanf(input, "$%*2sRMC,%f,%c,%f,%c,%f,%c,%f",
                   &utc, &status, &lat, &latDir, &lon, &lonDir, &speed) >= 7)
        {
            if (status == 'A')
            {
                gps_data->speed = speed * 0.514444f; // Knots to m/s
                gps_data->fix = 1;
            }
        }
    }
}