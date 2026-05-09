/**
 * @file read_aux_data_mode.h
 * @brief ICM-45686 IMU SPI transport and initialization interface
 */

#ifndef __READ_AUX_DATA_MODE_H__
#define __READ_AUX_DATA_MODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize IMU device and apply configuration
 * @param use_ln   Use low-noise mode if non-zero
 * @param accel_en Enable accelerometer if non-zero
 * @param gyro_en  Enable gyroscope if non-zero
 * @return 0 on success, negative on error
 */
int setup_imu(int use_ln, int accel_en, int gyro_en);

/**
 * @brief Read raw sensor data from IMU
 * @param accel_mg  Output: acceleration in mg (3-axis)
 * @param gyro_dps  Output: angular velocity in dps (3-axis)
 * @param temp_degc Output: temperature in degrees Celsius
 * @return 0 on success, negative on error
 */
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

/* SPI4 byte-level read/write (defined in spi.c) */
u8 SPI4_ReadWriteByte(u8 TxData);

#ifdef __cplusplus
}
#endif

#endif /* __READ_AUX_DATA_MODE_H__ */
