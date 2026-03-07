#include "stdio.h"
#include "main.h"
#include "Drivers/lsm6dso32.h"

/* * 1. External Platform Functions
 * Use 'extern' so this file knows these exist in your other .c files.
 */
extern int32_t lsm_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
extern int32_t lsm_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/* Global context handle */
static stmdev_ctx_t lsm_ctx;

/**
 * @brief  Rocket-profile Initialization
 * High-G (16g) and High-Rotation (2000dps) configuration.
 */
HAL_StatusTypeDef LSM6DSO32_Rocket_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t whoamI;

    lsm_ctx.write_reg = lsm_platform_write;
    lsm_ctx.read_reg = lsm_platform_read;
    lsm_ctx.handle = hspi;

    /* 1. Hardware Verification */
    lsm6dso32_device_id_get(&lsm_ctx, &whoamI);
    if (whoamI != LSM6DSO32_ID)
        return HAL_ERROR;

    /* 2. Software Reset */
    lsm6dso32_reset_set(&lsm_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do
    {
        lsm6dso32_reset_get(&lsm_ctx, &rst);
    } while (rst);

    /* 3. Rocket Configuration */
    lsm6dso32_block_data_update_set(&lsm_ctx, PROPERTY_ENABLE);

    // 16g Full Scale
    lsm6dso32_xl_full_scale_set(&lsm_ctx, LSM6DSO32_16g);
    lsm6dso32_xl_data_rate_set(&lsm_ctx, LSM6DSO32_XL_ODR_104Hz_NORMAL_MD);

    // 2000dps Full Scale
    lsm6dso32_gy_full_scale_set(&lsm_ctx, LSM6DSO32_2000dps);
    lsm6dso32_gy_data_rate_set(&lsm_ctx, LSM6DSO32_GY_ODR_104Hz_NORMAL_MD);

    /* * 4. Interrupt Routing FIX
     * In this driver version, the fields are nested inside 'pin_int1_route'
     * or similar bitfield structures.
     */
    lsm6dso32_pin_int1_route_t int1_route = {0};
    lsm6dso32_pin_int2_route_t int2_route = {0};

    lsm6dso32_pin_int1_route_get(&lsm_ctx, &int1_route);
    // Accessing the nested union/struct field
    int1_route.pin_int1_ctrl.drdy_xl = PROPERTY_ENABLE;
    lsm6dso32_pin_int1_route_set(&lsm_ctx, &int1_route);

    lsm6dso32_pin_int2_route_get(&lsm_ctx, &int2_route);
    int2_route.drdy_g = PROPERTY_ENABLE;
    lsm6dso32_pin_int2_route_set(&lsm_ctx, &int2_route);

    return HAL_OK;
}

/**
 * @brief  Reads and processes Accelerometer data into mg.
 */
void LSM6DSO32_Read_Accel(float *accel_mg)
{
    int16_t data_raw[3];
    lsm6dso32_acceleration_raw_get(&lsm_ctx, data_raw);

    accel_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw[0]);
    accel_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw[1]);
    accel_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw[2]);
}

/**
 * @brief  Reads and processes Gyroscope data into dps.
 */
void LSM6DSO32_Read_Gyro(float *gyro_dps)
{
    int16_t data_raw[3];
    lsm6dso32_angular_rate_raw_get(&lsm_ctx, data_raw);

    gyro_dps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw[0]) / 1000.0f;
    gyro_dps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw[1]) / 1000.0f;
    gyro_dps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw[2]) / 1000.0f;
}