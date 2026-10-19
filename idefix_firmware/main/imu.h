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

/*
 * Standstill gyro bias tracking. The boot-time calibration in imu_init()
 * goes stale as the die warms up (measured ~1 deg/min of yaw drift about
 * 15 min after reset), and the EKF trusts the gyro ~100x more than wheel
 * odometry for yaw rate, so a stale bias becomes heading drift while
 * parked. Call with a sample from imu_read() ONLY when the robot is known
 * to be still (wheels commanded to coast and measured stopped): any
 * remaining rate is then bias, and is folded in with a ~20 s time
 * constant. Samples above IMU_BIAS_TRACK_MAX_DPS on any axis are skipped
 * so a real rotation with stopped wheels (robot carried or turned by
 * hand) cannot be absorbed as bias. Same task as imu_read(), no locking.
 */
#define IMU_BIAS_TRACK_ALPHA     0.0005f   // per 100 Hz sample: tau = 0.01 s / 0.0005 = 20 s
#define IMU_BIAS_TRACK_MAX_DPS   1.0f      // ~7x the 0.14 dps per-sample noise std at rest

void imu_track_gyro_bias(const imu_sample_t *still_sample);

/*
 * Current z-axis gyro bias in deg/s, for logging.
 */
float imu_gyro_bias_z_dps(void);
