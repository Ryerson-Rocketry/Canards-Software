//
// Created by sztuka on 22.01.2023.
//

#ifndef STM32_SERIAL_DMA_NEO6M_PARSE_H
#define STM32_SERIAL_DMA_NEO6M_PARSE_H

#include <stm32f4xx.h>
#include <string.h>
#include <stdlib.h>
#include "Defs/states.h"

/*******************************************************************************
 * @brief Parses NMEA data from the GPS module
 * @param gps_data Pointer to GPS struct, writes data to it
 * @param buffer Pointer to buffer string with NMEA data
 ******************************************************************************/
void nmea_parse(GPS *gps_data, char *input);
#endif // STM32_SERIAL_DMA_NEO6M_PARSE_H
