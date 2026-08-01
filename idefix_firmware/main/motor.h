/*
 * motor.h
 *
 * Motor driver abstraction over the TB6612FNG + ESP32-S3 MCPWM peripheral.
 *
 * The rest of the firmware never touches MCPWM registers or the IN1/IN2
 * direction pins directly. Everything goes through motor_set_duty() with
 * a signed [-1.0, 1.0] duty cycle, where the sign encodes direction.
 *
 * This module owns:
 *   - MCPWM timer + operator + comparator + generator setup
 *   - IN1 / IN2 GPIO configuration and per-tick direction encoding
 *   - The TB6612FNG STBY line (asserted globally by motor_init_all)
 *
 * Design choices:
 *   - 20 kHz PWM frequency: above the human audible range, well below
 *     the TB6612FNG's ~100 kHz absolute max. Reduces motor whine.
 *   - Locked-antiphase drive is NOT used; we use sign-magnitude drive
 *     (IN1/IN2 = 10 or 01 for forward/reverse, PWM on PWMA/PWMB). This
 *     matches the TB6612FNG datasheet and gives lower quiescent current
 *     than slow-decay locked-antiphase.
 *   - Coast (both IN low) is used for zero duty rather than brake
 *     (both IN high). Coast is friendlier to the gearbox and avoids
 *     regenerative current spikes into the buck converter.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Logical motor identifiers. The mapping to physical PWM/IN pins lives
 * inside motor.c and pins.h. */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_COUNT
} motor_id_t;

/*
 * Initialize both motors: configures MCPWM peripheral, IN1/IN2 GPIOs,
 * and the TB6612FNG STBY line. STBY is driven LOW during setup and
 * HIGH at the end, so motors are guaranteed off until this returns.
 *
 * Must be called once at startup, before any motor_set_duty() calls.
 */
esp_err_t motor_init_all(void);

/*
 * Set signed duty cycle for a motor.
 *   duty in [-1.0, +1.0]. Sign encodes direction.
 *   duty = 0.0 puts the motor in coast (both IN pins LOW, PWM = 0).
 *   Values outside the range are clamped, not rejected.
 *
 * Direction convention: positive duty should produce positive encoder
 * counts. If it doesn't, swap either the motor leads OR the encoder
 * A/B channels (not both). Do this by editing the swap flag in
 * motor.c or encoder.c, not by negating in application code.
 */
void motor_set_duty(motor_id_t motor, float duty);

/*
 * Emergency stop: drives STBY LOW, which disables the entire TB6612FNG.
 * Call this from watchdogs, e-stop paths, and any fault handler.
 * Recovery requires motor_init_all() or a dedicated re-enable call
 * (not provided yet; we want the operator to reset intentionally).
 */
void motor_emergency_stop(void);
