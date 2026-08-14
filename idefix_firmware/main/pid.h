/*
 * pid.h
 *
 * Plain PID controller. Zero dependencies on hardware, FreeRTOS, or
 * ROS. This module is deliberately small and self-contained so it can
 * be unit-tested on the desktop by feeding synthetic setpoints and
 * measurements and checking the output.
 *
 * Features:
 *   - Anti-windup via CONDITIONAL INTEGRATION (primary): integrator only
 *     accumulates when the output is not saturating in the same direction
 *     as the current error. Prevents wind-up during output saturation.
 *   - Integral clamp (belt-and-braces): additional bound on the accumulated
 *     integral term to catch pathological cases (stalled plant, huge dt).
 *   - Derivative-on-measurement (not on error): avoids "derivative
 *     kick" when the setpoint changes stepwise, which is exactly what
 *     Nav2 will do when new /cmd_vel commands arrive.
 *   - Output saturation clamp.
 *   - Explicit dt argument: the caller controls timing. The PID does
 *     not assume anything about how often it is called, though for a
 *     mobile robot 100-200 Hz is a good starting point.
 */

#pragma once

#include <stdbool.h>

typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;

    /* Output limits (symmetric or asymmetric, both supported) */
    float out_min;
    float out_max;

    /* Integral clamp: bounds the accumulated integral term
     * independent of output clamp. Prevents wind-up during long
     * saturations. Set to a value comparable to out_max / ki. */
    float integral_min;
    float integral_max;

    /* Internal state */
    float integral;
    float prev_measurement;
    bool  has_prev_measurement;
} pid_t;

/*
 * Initialize a PID with the given gains and limits. Resets internal
 * state. Safe to call at runtime to change gains.
 */
void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_min, float integral_max);

/*
 * Reset internal state (integral, previous measurement). Call this
 * whenever you re-enable the control loop after a disable, or when
 * the setpoint discontinuously changes and you want a clean slate.
 */
void pid_reset(pid_t *pid);

/*
 * Compute one control update.
 *   setpoint    : desired value
 *   measurement : current measured value
 *   dt          : elapsed time since last call, in seconds. Must be > 0.
 * Returns the clamped control output.
 */
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);
