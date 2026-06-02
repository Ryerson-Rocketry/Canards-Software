#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>

bool DataStore_SDCardInit(void);
void DataStore_TelemetrySnapshot(void);
uint8_t DataStore_WriteToCSV(void);
void DataStore_WriteToSDCard(int len);

#endif // SDCARD_H