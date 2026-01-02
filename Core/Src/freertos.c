#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Drivers/ms5611.h"
#include "ff.h"
#include <string.h>

// Task handle and attributes
osThreadId_t defaultTaskHandle;

// FatFs variables
FATFS SDFatFS;
FIL SDFile;
UINT byteswritten;
char SDPath[4] = "0:";

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);

// FreeRTOS initialization
void MX_FREERTOS_Init(void)
{
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

// Default task function
void StartDefaultTask(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    /* 1. Mount the card */
    if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 1) == FR_OK)
    {

      /* 2. Open file for writing */
      if (f_open(&SDFile, "DONE.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
      {

        /* 3. Write your victory message */
        char *msg = "Project Finished! HAL + SDIO is working.";
        f_write(&SDFile, msg, strlen(msg), (void *)&byteswritten);

        /* 4. Close the file */
        f_close(&SDFile);

        /* Success! Turn on the built-in LED (usually PA5) */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      }
    }

    osDelay(10);
  }
}
