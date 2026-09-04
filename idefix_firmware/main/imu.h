/*
 * imu.h
 *
 * GY-521 breakout over I2C, driver/i2c_master.h new-style API.
 *
 * WHO_AM_I reads 0x70, the MPU6500 signature, not the genuine
 * MPU6050's 0x68 -- this breakout carries an MPU6500-family die.
 * The register map (accel/gyro/WHO_AM_I/PWR_MGMT_1) is close enough
 * that this project treats it as MPU6050-compatible. See the "IMU
 * bring-up" section of hardwareX_notes.md for the full bring-up
 * record, including a driver gotcha: i2c_master_transmit_receive()
 * returns ESP_ERR_INVALID_STATE on this chip/ESP-IDF v5.3.5
 * combination -- this module uses separate i2c_master_transmit() +
 * i2c_master_receive() calls instead, which work reliably.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} imu_sample_t;

/*
 * Bring up the I2C bus and device, wake the sensor from its power-on
 * SLEEP state, verify WHO_AM_I, then run a stationary gyro bias
 * calibration (averages 200 samples over about a second). The board
 * must be still during this call; wheels do not need to be on the
 * ground, motors are untouched by this module either way.
 *
 * Gyro bias is stored internally and subtracted by imu_read().
 * Accel is not zero-averaged -- gravity is a real signal, not bias --
 * imu_init() only logs the averaged magnitude as a sanity check.
 *
 * Must be called once at startup, before any imu_read() calls.
 */
esp_err_t imu_init(void);

/*
 * Read one accel+gyro sample, converted to physical units (g and
 * deg/s at the sensor's power-on-default full-scale ranges), gyro
 * bias already subtracted. Call from exactly one task at a fixed
 * rate, same convention as encoder_read_velocity_rad_s.
 */
esp_err_t imu_read(imu_sample_t *out);
