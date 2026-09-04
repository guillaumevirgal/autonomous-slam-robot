/*
 * imu.c
 *
 * MPU6500-family IMU (GY-521 breakout) over the ESP-IDF v5.3
 * driver/i2c_master.h new-style API. I2C_NUM_0, GPIO21=SCL,
 * GPIO47=SDA, 100 kHz, glitch_ignore_cnt=7. Internal ESP32 pull-ups
 * left disabled -- the breakout's own onboard 4.7k pull-ups to its
 * own VCC are used instead.
 */

#include "imu.h"
#include "pins.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "imu";

#define IMU_I2C_ADDR            0x68
#define IMU_REG_WHO_AM_I        0x75
#define IMU_REG_PWR_MGMT_1      0x6B
#define IMU_REG_ACCEL_XOUT_H    0x3B
#define IMU_REG_GYRO_XOUT_H     0x43

// Power-on-default full-scale ranges (ACCEL_CONFIG/GYRO_CONFIG untouched).
#define IMU_ACCEL_LSB_PER_G     16384.0f  // +/-2g
#define IMU_GYRO_LSB_PER_DPS    131.0f    // +/-250 deg/s

#define IMU_CAL_SAMPLES         200       // ~1s at 5ms spacing

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static float s_gyro_bias_x_dps = 0.0f;
static float s_gyro_bias_y_dps = 0.0f;
static float s_gyro_bias_z_dps = 0.0f;

// Write-then-read a register block. i2c_master_transmit_receive() (the
// combined call) returns ESP_ERR_INVALID_STATE on this chip/IDF version
// even on a healthy bus; the split transmit()+receive() form works
// reliably. See hardwareX_notes.md "IMU bring-up" for the full trace.
static esp_err_t imu_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t tr = i2c_master_transmit(s_dev, &reg, 1, 100);
    if (tr != ESP_OK) {
        return tr;
    }
    return i2c_master_receive(s_dev, buf, len, 100);
}

static esp_err_t imu_read_accel_gyro_raw(int16_t *accel, int16_t *gyro)
{
    uint8_t accel_raw[6];
    uint8_t gyro_raw[6];

    esp_err_t err = imu_read_regs(IMU_REG_ACCEL_XOUT_H, accel_raw, sizeof(accel_raw));
    if (err != ESP_OK) {
        return err;
    }
    err = imu_read_regs(IMU_REG_GYRO_XOUT_H, gyro_raw, sizeof(gyro_raw));
    if (err != ESP_OK) {
        return err;
    }

    accel[0] = (int16_t) ((accel_raw[0] << 8) | accel_raw[1]);
    accel[1] = (int16_t) ((accel_raw[2] << 8) | accel_raw[3]);
    accel[2] = (int16_t) ((accel_raw[4] << 8) | accel_raw[5]);
    gyro[0]  = (int16_t) ((gyro_raw[0] << 8) | gyro_raw[1]);
    gyro[1]  = (int16_t) ((gyro_raw[2] << 8) | gyro_raw[3]);
    gyro[2]  = (int16_t) ((gyro_raw[4] << 8) | gyro_raw[5]);
    return ESP_OK;
}

esp_err_t imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {0};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = GPIO_NUM_47;
    bus_cfg.scl_io_num = GPIO_NUM_21;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = false;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {0};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = IMU_I2C_ADDR;
    dev_cfg.scl_speed_hz = 100000;

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t who_am_i = 0;
    err = imu_read_regs(IMU_REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (0x70 expected for this breakout)", who_am_i);

    // Wake from the power-on-default SLEEP state. WHO_AM_I answers even
    // while asleep, but accel/gyro registers don't update until cleared.
    uint8_t pwr_mgmt_1[2];
    pwr_mgmt_1[0] = IMU_REG_PWR_MGMT_1;
    pwr_mgmt_1[1] = 0x00;
    err = i2c_master_transmit(s_dev, pwr_mgmt_1, sizeof(pwr_mgmt_1), 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake write failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stationary gyro bias calibration. Accel is summed too, but only to
    // log a magnitude sanity check -- gravity is a real signal, not bias,
    // so it is not zero-averaged or subtracted.
    int64_t gyro_sum[3];
    gyro_sum[0] = 0;
    gyro_sum[1] = 0;
    gyro_sum[2] = 0;
    int64_t accel_sum[3];
    accel_sum[0] = 0;
    accel_sum[1] = 0;
    accel_sum[2] = 0;
    int cal_ok_count = 0;

    for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        if (imu_read_accel_gyro_raw(accel_raw, gyro_raw) == ESP_OK) {
            accel_sum[0] += accel_raw[0];
            accel_sum[1] += accel_raw[1];
            accel_sum[2] += accel_raw[2];
            gyro_sum[0]  += gyro_raw[0];
            gyro_sum[1]  += gyro_raw[1];
            gyro_sum[2]  += gyro_raw[2];
            cal_ok_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (cal_ok_count == 0) {
        ESP_LOGE(TAG, "calibration: no successful samples");
        return ESP_FAIL;
    }

    s_gyro_bias_x_dps = ((float) gyro_sum[0] / cal_ok_count) / IMU_GYRO_LSB_PER_DPS;
    s_gyro_bias_y_dps = ((float) gyro_sum[1] / cal_ok_count) / IMU_GYRO_LSB_PER_DPS;
    s_gyro_bias_z_dps = ((float) gyro_sum[2] / cal_ok_count) / IMU_GYRO_LSB_PER_DPS;

    float accel_avg_x = (float) accel_sum[0] / cal_ok_count;
    float accel_avg_y = (float) accel_sum[1] / cal_ok_count;
    float accel_avg_z = (float) accel_sum[2] / cal_ok_count;
    float accel_mag_g = sqrtf(accel_avg_x * accel_avg_x +
                               accel_avg_y * accel_avg_y +
                               accel_avg_z * accel_avg_z) / IMU_ACCEL_LSB_PER_G;

    ESP_LOGI(TAG, "gyro bias (deg/s, N=%d): X=%.3f Y=%.3f Z=%.3f",
             cal_ok_count, s_gyro_bias_x_dps, s_gyro_bias_y_dps, s_gyro_bias_z_dps);
    ESP_LOGI(TAG, "accel magnitude sanity check: %.3fg (expect ~1g)", accel_mag_g);

    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    esp_err_t err = imu_read_accel_gyro_raw(accel_raw, gyro_raw);
    if (err != ESP_OK) {
        return err;
    }

    out->accel_x_g = (float) accel_raw[0] / IMU_ACCEL_LSB_PER_G;
    out->accel_y_g = (float) accel_raw[1] / IMU_ACCEL_LSB_PER_G;
    out->accel_z_g = (float) accel_raw[2] / IMU_ACCEL_LSB_PER_G;
    out->gyro_x_dps = (float) gyro_raw[0] / IMU_GYRO_LSB_PER_DPS - s_gyro_bias_x_dps;
    out->gyro_y_dps = (float) gyro_raw[1] / IMU_GYRO_LSB_PER_DPS - s_gyro_bias_y_dps;
    out->gyro_z_dps = (float) gyro_raw[2] / IMU_GYRO_LSB_PER_DPS - s_gyro_bias_z_dps;

    return ESP_OK;
}
