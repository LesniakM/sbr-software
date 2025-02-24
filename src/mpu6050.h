#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_TEMP_OUT_L 0x42
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48

#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C

#define MPU6050_REG_WHO_AM_I 0x75

#define G_CONSTANT 9.80665F
#define M_PI 3.14159265358979F

enum DLPF_CFG
{
    BW_256HZ,
    BW_188HZ,
    BW_98HZ,
    BW_42HZ,
    BW_20HZ,
    BW_10HZ,
    BW_5HZ,
};

/**
 * @brief Initializes the MPU6050 sensor.
 *
 * This function configures the MPU6050 by:
 * - Waking up the device from sleep mode.
 * - Setting the accelerometer and gyroscope to default sensitivity (±2g, ±250°/s).
 * - Configuring the digital low-pass filter (DLPF) bandwidth to 42Hz.
 * - Introducing short delays to ensure proper register configuration.
 *
 * @param dev_i2c Pointer to the I2C device specification.
 *
 * @note The default settings provide basic sensor functionality.
 *       Additional configuration may be required for specific applications.
 */
void mpu6050_init(const struct i2c_dt_spec *dev_i2c);

/**
 * @brief Reads 12 bytes of sensor data from the MPU6050.
 *
 * This function reads 10 bytes of data starting from the register
 * `MPU6050_REG_ACCEL_YOUT_H` in a burst operation. The data includes:
 * - 6 bytes of accelerometer data (X, Y, Z axes)
 * - 2 bytes of temperature data
 * - 4 bytes of gyroscope data (X, Y, Z axes)
 *
 * @param dev_i2c  Pointer to the I2C device specification.
 * @param buffer   Pointer to a buffer where the retrieved data will be stored.
 *
 * @note The data is stored in the provided buffer in the following order:
 * 1. acc-X HB
 * 2. acc-X LB
 * 3. acc-Y HB
 * 4. acc-Y LB
 * 5. acc-Z HB
 * 6. acc-Z LB
 * 7. Temp HB
 * 8. Temp LB
 * 9. gyro-X HB
 * 10.gyro-X LB
 * 11.gyro-Y HB
 * 12.gyro-Y LB
 */
void mpu6050_get_readings(const struct i2c_dt_spec *dev_i2c, uint8_t *buffer);

float accel_angle(float z_acc, float x_acc);
float gyro_angle(float angle_speed, float time_ms);

/**
 * @brief Converts raw gyroscope data to degrees per second (°/s).
 *
 * This function converts a raw 16-bit gyroscope reading from the MPU6500
 * into a float representing the angular velocity in degrees per second (°/s).
 * The conversion factor is based on a full-scale range of ±250°/s
 * (sensitivity: 131.072 LSB/°/s).
 *
 * @param raw_val  Raw gyroscope value (16-bit signed integer).
 * @return Converted angular velocity in degrees per second.
 */
float convert_gyro(int16_t raw_val);

/**
 * @brief Converts raw accelerometer data to acceleration in m/s².
 *
 * This function converts a raw 16-bit accelerometer reading from the MPU6500
 * into a float representing acceleration in meters per second squared (m/s²).
 * The conversion factor is based on a full-scale range of ±2g
 * (sensitivity: 16,384 LSB/g). The result is multiplied by `G_CONSTANT`
 * to express acceleration in standard gravity units.
 *
 * @param raw_val  Raw accelerometer value (16-bit signed integer).
 * @return Converted acceleration in meters per second squared (m/s²).
 */
float convert_accel(int16_t raw_val);

#endif // MPU6050_H