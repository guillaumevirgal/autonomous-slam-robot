#include "pid.h"

static float clamp(float x, float lo, float hi) // Clamp a value between a low and a high bound
{
    if (x < lo) return lo; // Too low -> return the lower bound
    if (x > hi) return hi; // Too high -> return the upper bound
    return x;
}

void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_min, float integral_max){
    // Store the three gains.
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    // Store the output limits ([-1.0; 1.0])
    pid->out_min = out_min;
    pid->out_max = out_max;
    // Store the integral limits
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)// Call this when re-enabling the loop after a stop, so old accumulated error does not suddenly kick the motor.
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev_measurement = false;
}

// setpoint    = what we want (rad/s)
// measurement = what we actually have (rad/s)
// dt          = seconds since the last call
// returns     = the control output
float pid_update(pid_t *pid, float setpoint, float measurement, float dt){
    if (dt <= 0.0f) return 0.0f; // Guard against a zero or negative

    float error = setpoint - measurement; // How far from the target

    // P term
    float p_term = pid->kp * error;

    // I term
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
    float i_term = pid->ki * pid->integral;

    // D term 
    float d_term = 0.0f;
    if (pid->has_prev_measurement) {
        float d_measurement = (measurement - pid->prev_measurement) / dt;
        d_term = -pid->kd * d_measurement;
    }
    pid->prev_measurement = measurement; // Save this measurement for the next call's derivative
    pid->has_prev_measurement = true;

    // Combine the 3 terms
    float output = p_term + i_term + d_term;
    return clamp(output, pid->out_min, pid->out_max);
}
