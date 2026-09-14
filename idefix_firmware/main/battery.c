/*
 * battery.c
 *
 * Battery voltage sense over a resistor divider on an ADC1 pin, plus the
 * active buzzer used to alarm on low voltage. See pins.h for the exact
 * GPIO and divider values, battery.h for the public API.
 */

#include "battery.h"
#include "pins.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "battery";

// Divider: Vbat -[100k]- ADC pin -[22k]- GND. The ADC only ever sees the
// bottom-resistor fraction of Vbat, so the reading is scaled back up by
// the inverse of this ratio to recover the pack voltage.
#define BATTERY_DIVIDER_TOP_KOHM      100.0f
#define BATTERY_DIVIDER_BOTTOM_KOHM    22.0f
#define BATTERY_DIVIDER_RATIO         (BATTERY_DIVIDER_BOTTOM_KOHM / \
                                        (BATTERY_DIVIDER_TOP_KOHM + BATTERY_DIVIDER_BOTTOM_KOHM))

// 12 dB attenuation gives the full ~0-3.3V input range, needed since a
// healthy 3S pack (up to ~12.6V) divides down to ~2.27V, and a fresh 4S
// pack would be higher still -- headroom matters more than resolution here.
#define BATTERY_ADC_ATTEN       ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH    ADC_BITWIDTH_DEFAULT
#define BATTERY_ADC_SAMPLES     16   // averaged per read, this is a slow-moving quantity

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_channel_t             s_adc_channel;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_calibrated = false;

esp_err_t battery_init(void)
{
    adc_unit_t unit_id;
    esp_err_t err = adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &unit_id, &s_adc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d is not a valid ADC pin: %s", BATTERY_ADC_GPIO, esp_err_to_name(err));
        return err;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {0};
    unit_cfg.unit_id = unit_id;
    unit_cfg.clk_src = ADC_RTC_CLK_SRC_DEFAULT;   // oneshot mode uses the RTC controller clock on
                                                    // the ESP32-S3, plain 0 is not a valid value here
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

    err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {0};
    chan_cfg.atten = BATTERY_ADC_ATTEN;
    chan_cfg.bitwidth = BATTERY_ADC_BITWIDTH;

    err = adc_oneshot_config_channel(s_adc_handle, s_adc_channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    // Calibration turns raw counts into millivolts using per-chip eFuse
    // data. Not every chip has the eFuse bits burnt for this; fall back to
    // a nominal (less accurate, no per-chip correction) conversion rather
    // than failing battery_init() outright if it's unavailable.
    adc_cali_curve_fitting_config_t cali_cfg = {0};
    cali_cfg.unit_id = unit_id;
    cali_cfg.chan = s_adc_channel;
    cali_cfg.atten = BATTERY_ADC_ATTEN;
    cali_cfg.bitwidth = BATTERY_ADC_BITWIDTH;

    esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    s_calibrated = (cali_err == ESP_OK);
    if (!s_calibrated) {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s), using nominal 3.3V/4095 conversion",
                 esp_err_to_name(cali_err));
    }

    gpio_config_t buzzer_cfg = {0};
    buzzer_cfg.pin_bit_mask = (1ULL << BUZZER_GPIO);
    buzzer_cfg.mode = GPIO_MODE_OUTPUT;
    buzzer_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    buzzer_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    buzzer_cfg.intr_type = GPIO_INTR_DISABLE;

    err = gpio_config(&buzzer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(BUZZER_GPIO, 0);   // off at boot

    ESP_LOGI(TAG, "battery sense on GPIO%d (ADC1 ch%d), buzzer on GPIO%d, calibrated=%d",
             BATTERY_ADC_GPIO, s_adc_channel, BUZZER_GPIO, (int) s_calibrated);

    return ESP_OK;
}

bool battery_read_voltage_v(float *out_voltage_v)
{
    int64_t sum_mv = 0;
    int valid = 0;
    int read_fail = 0;
    int cali_fail = 0;
    esp_err_t last_read_err = ESP_OK;
    esp_err_t last_cali_err = ESP_OK;
    int last_raw = -1;

    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, s_adc_channel, &raw);
        if (err != ESP_OK) {
            read_fail++;
            last_read_err = err;
            continue;
        }
        last_raw = raw;

        int mv;
        if (s_calibrated) {
            esp_err_t cali_err = adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
            if (cali_err != ESP_OK) {
                cali_fail++;
                last_cali_err = cali_err;
                continue;
            }
        } else {
            mv = (raw * 3300) / 4095;   // nominal full-scale fallback
        }

        sum_mv += mv;
        valid++;
    }

    if (valid == 0) {
        // One consolidated line per call rather than one per failed sample:
        // logging inside the loop at up to BATTERY_ADC_SAMPLES times per
        // call risks the ESP_LOG console's small ring buffer evicting
        // earlier lines before a slow reader (e.g. a reattaching monitor)
        // catches up, same issue documented for imu console output in
        // hardwareX_notes.md.
        ESP_LOGW(TAG, "battery: all %d samples failed (read_fail=%d last=%s, "
                      "cali_fail=%d last=%s, calibrated=%d, last_raw=%d)",
                 BATTERY_ADC_SAMPLES, read_fail, esp_err_to_name(last_read_err),
                 cali_fail, esp_err_to_name(last_cali_err), (int) s_calibrated, last_raw);
        return false;   // no measurement this cycle, not "battery reads 0V"
    }

    float adc_v = ((float) sum_mv / (float) valid) / 1000.0f;
    *out_voltage_v = adc_v / BATTERY_DIVIDER_RATIO;
    return true;
}

void battery_set_buzzer(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? 1 : 0);
}
