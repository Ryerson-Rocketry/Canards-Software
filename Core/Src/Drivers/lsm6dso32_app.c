#include <string.h>
#include <stdio.h>
#include "main.h"
#include "Drivers/lsm6dso32.h"
#include <cmsis_os.h>

/* Forward declarations for the platform functions we discussed */
extern int32_t lsm_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
extern int32_t lsm_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

static stmdev_ctx_t dev_ctx;

void platform_delay(uint32_t ms)
{
    osDelay(ms);
}

HAL_StatusTypeDef LSM6DSO32_Rocket_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t whoamI, rst;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lsm_platform_write;
    dev_ctx.read_reg = lsm_platform_read;
    dev_ctx.mdelay = platform_delay; // <--- ADD THIS
    dev_ctx.handle = hspi;

    /* Check device ID */
    lsm6dso32_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DSO32_ID)
        return HAL_ERROR;

    /* Restore default configuration */
    lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dso32_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* --- Rocket Specific Config (Integrated from ST Example) --- */

    /* Disable I3C to save power and prevent interference */
    lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);

    /* Enable Block Data Update (BDU) - very important for high-speed flight */
    lsm6dso32_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set full scale: 16g for launch, 2000dps for roll stabilization */
    lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_16g);
    lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_2000dps);

    /* Set ODR to 104Hz (Rocket-grade speed) */
    lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_104Hz_NORMAL_MD);
    lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_104Hz_NORMAL_MD);

    /* Interrupt routing logic (as you discovered previously) */
    lsm6dso32_pin_int1_route_t int1_route = {0};
    lsm6dso32_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    lsm6dso32_pin_int1_route_set(&dev_ctx, &int1_route);

    return HAL_OK;
}

void LSM6DSO32_Read_Accel(float *accel_mg)
{
    int16_t data_raw[3];
    lsm6dso32_acceleration_raw_get(&dev_ctx, data_raw);
    accel_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw[0]);
    accel_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw[1]);
    accel_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw[2]);
}

void LSM6DSO32_Read_Gyro(float *gyro_dps)
{
    int16_t data_raw[3];
    lsm6dso32_angular_rate_raw_get(&dev_ctx, data_raw);
    // ST driver uses mdps, we convert to dps for standard avionics math
    gyro_dps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw[0]) / 1000.0f;
    gyro_dps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw[1]) / 1000.0f;
    gyro_dps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw[2]) / 1000.0f;
}
