#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>

bool DataStore_SDCardInit(void);
void DataStore_TelemetrySnapshot(void);
int DataStore_WriteToCSV(void);
void DataStore_WriteToSDCard(int len);
// void send_to_radio(void);

#endif // SDCARD_H