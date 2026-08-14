/*
 * encoder.h
 *
 * Quadrature encoder abstraction over the ESP32-S3 PCNT peripheral.
 *
 * Why PCNT (not GPIO interrupts): the N20 gearmotor with magnetic
 * encoder produces 7 pulses per motor revolution before the gearbox,
 * so at ~200 RPM output shaft x 50:1 gearbox x 4 quadrature edges you
 * are looking at tens of thousands of edges per second per motor.
 * Interrupt-per-edge would consume real CPU. PCNT counts in hardware
 * with zero CPU cost.
 *
 * PCNT gotcha (handled internally): the S3's PCNT counter is signed
 * 16-bit and wraps. We register watch points at +/- 30000 and
 * accumulate wraps into a signed 64-bit software counter in the ISR.
 * The public API returns the 64-bit accumulated count, so wraparound
 * is invisible to callers.
 *
 * Quadrature decoding: 4x edge counting (both channels, both edges).
 * This is the standard "full quadrature" mode and gives us 4 counts
 * per encoder pulse period, i.e. 28 counts per motor revolution
 * before the gearbox for a 7-pulse encoder.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

// Encoder mechanical constants
#define ENCODER_PULSES_PER_MOTOR_REV    7 // 7 pulses per motor-shaft revolution per channel
#define ENCODER_QUAD_MULTIPLIER         4 // 4x quadrature decoding (both edges, both channels)
#define ENCODER_GEAR_RATIO              150 // 150:1 gearbox reduction
#define ENCODER_COUNTS_PER_OUTPUT_REV   (ENCODER_PULSES_PER_MOTOR_REV * ENCODER_QUAD_MULTIPLIER * ENCODER_GEAR_RATIO) // COUNTS_PER_OUTPUT_REV = 7 * 4 * 150 = 4200

typedef enum {
    ENCODER_A = 0,
    ENCODER_B = 1,
    ENCODER_COUNT
} encoder_id_t;

/*
 * Initialize both encoders. Configures two PCNT units in 4x quadrature
 * mode, registers overflow watch points, and starts counting.
 */
esp_err_t encoder_init_all(void);

/*
 * Read accumulated signed count since boot (or since encoder_reset).
 * Never wraps within the useful lifetime of the robot.
 */
int64_t encoder_read_counts(encoder_id_t enc);

/*
 * Compute instantaneous velocity in radians per second on the OUTPUT
 * shaft (i.e. after the gearbox). Call this from your control task at
 * a fixed rate. Internally uses a rolling difference of counts over
 * the elapsed time between calls.
 *
 * IMPORTANT: this function must be called from exactly one task, at a
 * consistent rate. If you need velocity in two places, read once and
 * share via a shared struct.
 */
float encoder_read_velocity_rad_s(encoder_id_t enc);

/*
 * Zero the accumulated count. Useful for odometry resets. Does NOT
 * stop the hardware counter, just resets the software accumulator
 * and the last-read timestamp used by velocity computation.
 */
void encoder_reset(encoder_id_t enc);
