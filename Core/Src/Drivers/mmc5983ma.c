#include "i2c.h"
#include "main.h"
#include "mmc5983ma.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "projdefs.h"
#include "semphr.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t gI2c1Mutex;

HAL_StatusTypeDef write(uint8_t regAddress, uint8_t data)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS << 1, regAddress, 1, &data, 1, 100);
    xSemaphoreGive(gI2c1Mutex);
  }
  return status;
}

HAL_StatusTypeDef read(uint8_t regAddress, uint8_t* out, int length)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  if (xSemaphoreTake(gI2c1Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS << 1, regAddress, 1, out, length, 100);
    xSemaphoreGive(gI2c1Mutex);
  }
  return status;
}

HAL_StatusTypeDef magInit(void){
  write(MAG_CTRL_REG_0, 0x80); // Software reset
  osDelay(pdMS_TO_TICKS(5));

  write(MAG_CTRL_REG_0, 0x24); // automatic set/reset mode 
  write(MAG_CTRL_REG_1, 0x03); // set bandwidth to 800Hz
  write(MAG_CTRL_REG_2, 0x00); // set mode to continuous

  return HAL_OK;
}

HAL_StatusTypeDef triggerAndWait(SemaphoreHandle_t sem)
{
  // trigger take measurement, interrupt, and automatic set/reset
  if (write(MAG_CTRL_REG_0, 0x01 | 0x04 | 0x20) != HAL_OK)
    return HAL_ERROR;

  // Wait for DRDY Semaphore
  if (xSemaphoreTake(sem, pdMS_TO_TICKS(200)) != pdTRUE)
  {
    printf("[DEBUG]: Semaphore Timeout\r\n");
    return HAL_TIMEOUT;
  }

  // VERIFY Hardware Status
  uint8_t status;
  read(MAG_STATUS_REG, &status, 1);
  if (!(status & 0x01))
  { // Check Meas_M_Done bit
    return HAL_BUSY;
  }

  return HAL_OK;
}

HAL_StatusTypeDef readMagData(SemaphoreHandle_t magDataReadySemaphore, float magData[3]){
  HAL_StatusTypeDef status = HAL_ERROR;
  if (triggerAndWait(magDataReadySemaphore) != HAL_OK)
    return HAL_ERROR;
  
  // read the data from the sensor, supposedly does the differential calculation internally
  if (read(MAG_XOUT_0, (uint8_t*)magData, 7) != HAL_OK)
    return HAL_ERROR;

  
  // Reconstruct 18-bit unsigned integer values
  uint8_t rawBuf[7] = {0};
  uint32_t raw_x = ((uint32_t)rawBuf[0] << 10) | ((uint32_t)rawBuf[1] << 2) | ((uint32_t)rawBuf[6] >> 6);
  uint32_t raw_y = ((uint32_t)rawBuf[2] << 10) | ((uint32_t)rawBuf[3] << 2) | ((uint32_t)(rawBuf[6] >> 4) & 0x03);
  uint32_t raw_z = ((uint32_t)rawBuf[4] << 10) | ((uint32_t)rawBuf[5] << 2) | ((uint32_t)(rawBuf[6] >> 2) & 0x03);

  // Convert raw 18-bit counts to Gauss (Null field output at 131072, Sensitivity = 16384 LSB/Gauss)
  magData[0] = ((float)raw_x - 131072.0f) / 16384.0f;
  magData[1] = ((float)raw_y - 131072.0f) / 16384.0f;
  magData[2] = ((float)raw_z - 131072.0f) / 16384.0f;

  return HAL_OK;
}
