#include "Tasks/sdcard.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "flight_configs.h"
#include "states.h"
#include "stdio.h"
#include "string.h"
#include "fatfs.h"

extern FATFS SDFatFS;
extern FIL SDFile;
extern char SDPath[4];
extern SD_HandleTypeDef hsd;
extern bool dataStoreTask;
extern QueueHandle_t radioQueueHandle;
extern TaskHandle_t radioTaskHandle;
extern Rocket_States_t Rocket;

static bool sdInitialized = false;
static char csvBuffer[128];
static uint32_t syncCounter = 0;
UINT bytesWritten;

// check if writing to sd has been attempted, if not check if sdcard is present
// if present, attempt to mount and open file, if successful set sdInitialized to true
// write the headers if its the first time writing to the file
// 
// save a snapshot of the rocket state
// format the snapshot into csv and write to csvBuffer
// write csvBuffer to the sd card
// then call f_sync every 200 writes to ensure data is written to the card
//
// notify the radio task

bool DataStore_SDCardInit(void){
    bool cardPresent = (HAL_GPIO_ReadPin(SDIO_NCD_GPIO_Port, SDIO_NCD_Pin) == GPIO_PIN_RESET);
    printf("[SD] card detect pin=%d present=%d\r\n",
            HAL_GPIO_ReadPin(SDIO_NCD_GPIO_Port, SDIO_NCD_Pin), cardPresent);

    if (!cardPresent) {
        printf("[SD] No card detected. Skipping SD interface.\r\n");
        sdInitialized = false;
        return false;
    }

    if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 1) != FR_OK) {
        printf("[SD] Mount failed.\r\n");
        sdInitialized = false;
        return false;
    }

    if (f_open(&SDFile, FLIGHT_DATA_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        printf("[SD] Open failed.\r\n");
        sdInitialized = false;
        return false;
    }  

    sdInitialized = true;
    printf("[SD] ready\r\n");

    if (f_size(&SDFile) == 0)
    {
        char *fileHeaders =
            "timestamp (ms), flight_state (enum), "
            "accel_x (centi-mg), accel_y (centi-mg), accel_z (centi-mg), "
            "gyro_x (centi-dps), gyro_y (centi-dps), gyro_z (centi-dps), "
            "mag_x (scaled x100), mag_y (scaled x100), mag_z (scaled x100), "
            "roll (centi-deg), pitch (centi-deg), yaw (centi-deg), "
            "tilt_angle (centi-deg), altitude (cm), velocity (cm/s), "
            "roll_error (centi-deg), pitch_error (centi-deg), pwm_angle (centi-deg)"
            "\r\n";
        f_puts(fileHeaders, &SDFile);
    }
    else
    {
        printf("[SD] no card detected, skipping SD\r\n");
        sdInitialized = false;
    }   
    return sdInitialized;
}

void DataStore_TelemetrySnapshot(void){
    // NOTE: UNCOMMENT WHEN FLYING THE ROCKET

    // if (Rocket.flightState == STATE_PAD)
    // {
    //   dataStoreTask = true;
    //   continue;
    // }
    taskENTER_CRITICAL();
    memcpy(Rocket.snapshot.accel, Rocket.rawData.accel, sizeof(Rocket.snapshot.accel));
    memcpy(Rocket.snapshot.gyro, Rocket.rawData.gyro, sizeof(Rocket.snapshot.gyro));
    memcpy(Rocket.snapshot.mag, Rocket.rawData.mag, sizeof(Rocket.snapshot.mag));
    memcpy(Rocket.snapshot.rpy, Rocket.estimate.rpy, sizeof(Rocket.snapshot.rpy));
    taskEXIT_CRITICAL();

    Rocket.snapshot.pressure    = Rocket.rawData.pressure;
    Rocket.snapshot.position    = Rocket.estimate.position;
    Rocket.snapshot.velocity    = Rocket.estimate.velocity;
    Rocket.snapshot.tiltAngle   = Rocket.estimate.tilt_angle;
    Rocket.snapshot.rollError   = Rocket.control.rollError;
    Rocket.snapshot.pwmAngle    = Rocket.control.pwmAngle;
    Rocket.snapshot.pitchError  = Rocket.control.pitchError;
    Rocket.snapshot.flightState = Rocket.flightState;
    Rocket.snapshot.timestamp   = osKernelGetTickCount();
}

uint8_t DataStore_WriteToCSV(void) 
{
    int len = snprintf(csvBuffer, sizeof(csvBuffer), "%lu", Rocket.snapshot.timestamp);
    len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len, ",%d", Rocket.snapshot.flightState);

    #define APPEND_SCALED(value) \
        len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len, ",%ld", (int32_t)((value) * 100))
            APPEND_SCALED(Rocket.snapshot.accel[0]);
            APPEND_SCALED(Rocket.snapshot.accel[1]);
            APPEND_SCALED(Rocket.snapshot.accel[2]);
            APPEND_SCALED(Rocket.snapshot.gyro[0]);
            APPEND_SCALED(Rocket.snapshot.gyro[1]);
            APPEND_SCALED(Rocket.snapshot.gyro[2]);
            APPEND_SCALED(Rocket.snapshot.mag[0]);
            APPEND_SCALED(Rocket.snapshot.mag[1]);
            APPEND_SCALED(Rocket.snapshot.mag[2]);
            APPEND_SCALED(Rocket.snapshot.rpy[0]);
            APPEND_SCALED(Rocket.snapshot.rpy[1]);
            APPEND_SCALED(Rocket.snapshot.rpy[2]);
            APPEND_SCALED(Rocket.snapshot.tiltAngle);
            APPEND_SCALED(Rocket.snapshot.position);
            APPEND_SCALED(Rocket.snapshot.velocity);
            APPEND_SCALED(Rocket.snapshot.rollError);
            APPEND_SCALED(Rocket.snapshot.pitchError);
            APPEND_SCALED(Rocket.snapshot.pwmAngle);

            len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len, "\r\n");

    #undef APPEND_SCALED
    return (uint8_t)len;
}

void DataStore_WriteToSDCard(int len){
    if (len <= 0 || len >= (int)sizeof(csvBuffer))
    {
        dataStoreTask = true;
    }

    if (f_write(&SDFile, csvBuffer, (UINT)len, &bytesWritten) != FR_OK || bytesWritten != (UINT)len)
    {
        dataStoreTask = true;
    }

    // can be shortened?
    if (++syncCounter >= 50)
    {
        f_sync(&SDFile);
        syncCounter = 0;
    }
}

void send_to_radio()
{
    // write queue to send data to vRadioTask
    xQueueSend(radioQueueHandle, csvBuffer, 0);

    dataStoreTask = true;
    xTaskNotifyGive(radioTaskHandle);
}






