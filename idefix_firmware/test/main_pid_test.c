// main_stage2_pid.c
// STAGE 2 diagnostic: closed-loop PID velocity control on both motors.
//
// Purpose: drive both motors through a series of step setpoints at low,
// medium, and high magnitude (both signs) and capture setpoint, measured
// velocity (raw and filtered), and duty at 10 Hz for HardwareX step-response
// figures. PID runs at 100 Hz per motor with measured dt from encoder timestamps.
//
// Precondition: run the open-loop test FIRST with the encoder A invert
// applied inside encoder_init_all(). Both motors must read POSITIVE at
// commanded +0.15 duty. If either reads negative, DO NOT run this file:
// fix the sign convention.
//
// Safety: wheels OFF the ground until gains converge. Duty ceiling
// clamped to +/-0.24. Keep a hand on the LiPo connector.
//
// Log columns: t_ms, setpoint, meas_a_raw, meas_a_filt, duty_a,
//              meas_b_raw, meas_b_filt, duty_b

#include "motor.h"
#include "encoder.h"
#include "pid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdio.h>

static const char *TAG = "stage2_pid";

// Loop timing
#define CONTROL_PERIOD_MS   10      // 100 Hz control loop
#define LOG_EVERY_N_TICKS   10      // 10 ticks -> 10 Hz logging

// Filter (moving average) window length in samples
#define FILTER_WINDOW       5

// Coast bypass threshold: below this |setpoint| we command coast and skip the PID
#define COAST_EPS_RAD_S     0.05f

// PID initial gains (starting point for tuning, expected to iterate)
#define PID_KP              0.04f
#define PID_KI              1.0f
#define PID_KD              0.0f

// Output and integral clamps
#define PID_OUT_MIN         (-0.24f)
#define PID_OUT_MAX         ( 0.24f)
#define PID_INT_MIN         (-2.0f)
#define PID_INT_MAX         ( 2.0f)

// Encoder counts per output shaft revolution. Must stay in sync with encoder.c.
// (7 PPR motor shaft * 4x quadrature * 150 gear ratio = 4200)
#define COUNTS_PER_REV      4200

// Setpoint schedule: sequence of {end_time_ms, setpoint_rad_s} phases.
// Each phase runs from the previous end_time (or 0) until its own end_time.
// Total run time: 32 s. Three positive steps at 1, 2, 3 rad/s and three
// negative steps of the same magnitudes, with 2 s of coast between each.
typedef struct {
    int64_t end_ms;    // this phase runs UNTIL this elapsed time
    float   setpoint;  // setpoint (rad/s) during this phase
} phase_t;

static const phase_t schedule[] = {
    { 2000,   0.0f },  // warmup (coast)
    { 5000,  +1.0f },  // low positive step (3 s)
    { 7000,   0.0f },  // return to zero (2 s)
    {10000,  +2.0f },  // medium positive step (3 s)
    {12000,   0.0f },  // return to zero (2 s)
    {15000,  +3.0f },  // high positive step (3 s)
    {17000,   0.0f },  // return to zero (2 s)
    {20000,  -1.0f },  // low negative step (3 s)
    {22000,   0.0f },  // return to zero (2 s)
    {25000,  -2.0f },  // medium negative step (3 s)
    {27000,   0.0f },  // return to zero (2 s)
    {30000,  -3.0f },  // high negative step (3 s)
    {32000,   0.0f },  // final coast (2 s)
};

#define SCHEDULE_LEN (sizeof(schedule) / sizeof(schedule[0]))

// Look up the current setpoint given elapsed time. Returns 0 if past the end.
static float lookup_setpoint(int64_t elapsed_ms){
    for (int i = 0; i < (int)SCHEDULE_LEN; i++) {
        if (elapsed_ms < schedule[i].end_ms) {
            return schedule[i].setpoint;
        }
    }
    return 0.0f;
}

// Moving-average filter state, one buffer per motor
static float filt_buf_a[FILTER_WINDOW] = {0};
static float filt_buf_b[FILTER_WINDOW] = {0};
static int   filt_idx_a = 0;
static int   filt_idx_b = 0;

// Push a new sample into a ring buffer and return the current arithmetic mean
static float filter_push(float *buf, int *idx, float new_sample){
    buf[*idx] = new_sample;              // Overwrite the oldest slot
    *idx = (*idx + 1) % FILTER_WINDOW;   // Advance the ring index
    float sum = 0.0f;                    // Accumulator for the mean
    for (int i = 0; i < FILTER_WINDOW; i++) {
        sum += buf[i];                   // Sum all slots
    }
    return sum / (float) FILTER_WINDOW;  // Return the arithmetic mean
}

// Convert delta-counts / delta-microseconds to rad/s on the output shaft
static float counts_to_rad_s(int64_t d_counts, int64_t d_us){
    if (d_us <= 0) return 0.0f;                                    // Guard against zero or negative dt
    float revs = (float) d_counts / (float) COUNTS_PER_REV;        // Counts -> output shaft revolutions
    float rad = revs * 2.0f * (float) M_PI;                        // Revolutions -> radians
    float seconds = (float) d_us * 1e-6f;                          // Microseconds -> seconds
    return rad / seconds;                                          // rad/s
}

// Per-motor PID instances (independent gains, independent state)
static pid_t pid_a;
static pid_t pid_b;

static void control_task(void *arg){
    // Prime the "last" values so the first control iteration has a real dt.
    // We read counts and timestamp separately: at 100 Hz, the sub-millisecond
    // misalignment between them is negligible.
    int64_t last_counts_a = encoder_read_counts(ENCODER_A);
    int64_t last_counts_b = encoder_read_counts(ENCODER_B);
    int64_t last_time_us  = esp_timer_get_time();

    // Loop timing setup
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    int tick_counter = 0;
    int64_t start_us = last_time_us; // treat priming time as t=0

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // Take timestamp first, then counts. The gap is a few us at most.
        int64_t now_us = esp_timer_get_time();
        int64_t counts_a = encoder_read_counts(ENCODER_A);
        int64_t counts_b = encoder_read_counts(ENCODER_B);

        // Elapsed time since test start
        int64_t elapsed_ms = (now_us - start_us) / 1000;

        // Look up the current setpoint from the schedule
        float sp = lookup_setpoint(elapsed_ms);

        // Compute per-motor deltas
        int64_t d_counts_a = counts_a - last_counts_a;
        int64_t d_counts_b = counts_b - last_counts_b;
        int64_t d_us       = now_us   - last_time_us;

        // Update the "last" values for the next iteration
        last_counts_a = counts_a;
        last_counts_b = counts_b;
        last_time_us  = now_us;

        // Raw velocities (may have single-LSB quantization dither, ~0.15 rad/s)
        float meas_a_raw = counts_to_rad_s(d_counts_a, d_us);
        float meas_b_raw = counts_to_rad_s(d_counts_b, d_us);

        // Filtered velocities: moving average of FILTER_WINDOW samples, applied
        // right before the PID. Reduces LSB dither by ~sqrt(FILTER_WINDOW).
        float meas_a_filt = filter_push(filt_buf_a, &filt_idx_a, meas_a_raw);
        float meas_b_filt = filter_push(filt_buf_b, &filt_idx_b, meas_b_raw);

        // Measured dt in seconds (both motors share the same loop timestamp)
        float dt = (float) d_us * 1e-6f;

        // Coast bypass at very small setpoint: skip PID, command coast, reset integrators
        float duty_a, duty_b;
        if (fabsf(sp) < COAST_EPS_RAD_S) {
            duty_a = 0.0f;
            duty_b = 0.0f;
            pid_reset(&pid_a);
            pid_reset(&pid_b);
        } else {
            duty_a = pid_update(&pid_a, sp, meas_a_filt, dt);
            duty_b = pid_update(&pid_b, sp, meas_b_filt, dt);
        }

        // Command the motors
        motor_set_duty(MOTOR_A, duty_a);
        motor_set_duty(MOTOR_B, duty_b);

        // Log at 10 Hz
        tick_counter = tick_counter + 1;
        if (tick_counter >= LOG_EVERY_N_TICKS) {
            tick_counter = 0;
            printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                   elapsed_ms,
                   sp,
                   meas_a_raw, meas_a_filt, duty_a,
                   meas_b_raw, meas_b_filt, duty_b);
        }

        // Stop cleanly at the end of the schedule
        if (elapsed_ms > schedule[SCHEDULE_LEN - 1].end_ms) {
            motor_set_duty(MOTOR_A, 0.0f);
            motor_set_duty(MOTOR_B, 0.0f);
            ESP_LOGI(TAG, "schedule complete, motors stopped");
            vTaskDelete(NULL);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "STAGE 2 closed-loop PID velocity control starting");
    ESP_LOGI(TAG, "gains: Kp=%.3f Ki=%.3f Kd=%.3f  out=[%.3f,%.3f]  int=[%.3f,%.3f]",
             PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX, PID_INT_MIN, PID_INT_MAX);
    ESP_LOGI(TAG, "schedule: +1, +2, +3, -1, -2, -3 rad/s, 3 s each, coast between");

    // Bring up hardware. Encoder A invert is applied inside encoder_init_all().
    ESP_ERROR_CHECK(motor_init_all());
    ESP_ERROR_CHECK(encoder_init_all());

    // Initialize both PID instances with identical gains initially
    pid_init(&pid_a, PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX, PID_INT_MIN, PID_INT_MAX);
    pid_init(&pid_b, PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX, PID_INT_MIN, PID_INT_MAX);

    // Print the CSV header (matches the printf format in control_task)
    printf("t_ms,setpoint,meas_a_raw,meas_a_filt,duty_a,meas_b_raw,meas_b_filt,duty_b\n");

    // Launch the control task pinned to core 1 (same core as the open-loop test)
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "control task running");
}