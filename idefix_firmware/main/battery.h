/*
 * battery.h
 *
 * Battery voltage sense (resistor divider on an ADC1 pin, see pins.h for
 * the exact divider values) plus the active buzzer used to alarm on low
 * voltage. Grouped in one module because the buzzer's only job right now
 * is reporting battery state; split it out if it ever needs a second use.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Buzzer turns on at or below this pack voltage, and off again only once
// the voltage recovers above BATTERY_LOW_VOLTAGE_CLEAR_V. The gap between
// the two (not a single threshold) is hysteresis: without it, a reading
// sitting right at the threshold would chatter the buzzer on and off on
// every sample from ordinary ADC/load noise.
#define BATTERY_LOW_VOLTAGE_THRESHOLD_V   10.0f
#define BATTERY_LOW_VOLTAGE_CLEAR_V       10.3f

/*
 * Bring up the ADC1 unit/channel for battery sense (with calibration if
 * the chip supports it, falling back to an uncalibrated conversion if
 * not) and configure the buzzer GPIO as an output, driven low (off).
 * Call once at startup.
 */
esp_err_t battery_init(void);

/*
 * Read the pack voltage in volts into *out_voltage_v. Averages several raw
 * ADC samples internally to reduce noise. Safe to call from any single
 * task at a modest rate (this is a slow-moving quantity, no need for a
 * fast poll).
 *
 * Returns true if at least one sample succeeded, in which case
 * *out_voltage_v is a real reading -- note that a genuinely disconnected
 * battery reads close to 0V, which is a valid (if degenerate) result, not
 * a failure. Returns false only if every sample failed at the ADC/driver
 * level (out_voltage_v left untouched); callers should treat that as "no
 * measurement this cycle", not as "battery voltage is zero".
 */
bool battery_read_voltage_v(float *out_voltage_v);

/*
 * Drive the buzzer GPIO directly. true = on, false = off. Callers
 * implement the hysteresis themselves (see BATTERY_LOW_VOLTAGE_* above);
 * this function just sets the pin.
 */
void battery_set_buzzer(bool on);
