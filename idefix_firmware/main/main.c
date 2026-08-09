// main_deadband_ramp.c
// STAGE 1 diagnostic: motor deadband ramp.
// Positive staircase 0 -> +0.24, hold each step, then negative
// staircase 0 -> -0.24, hold each step. That's it.

#include "motor.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "deadband";

#define CONTROL_PERIOD_MS   10
#define LOG_EVERY_N_TICKS   10
#define STEP_MS             2000        // 2 s per level
#define STEP_INCREMENT      0.01f       // 1% duty per step
#define STEPS_PER_LEG       25          // 25 * 0.01 = 0.25, past PID clamp

static void control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    int tick_counter = 0;
    int64_t start_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_ms = (now_us - start_us) / 1000;
        int total_step = elapsed_ms / STEP_MS;

        // First STEPS_PER_LEG steps are positive ramp,
        // next STEPS_PER_LEG steps are negative ramp,
        // then hold at 0 forever.
        float duty;
        if (total_step < STEPS_PER_LEG) {
            duty = (float)total_step * STEP_INCREMENT;          // 0.00, 0.01, ... 0.24
        } else if (total_step < 2 * STEPS_PER_LEG) {
            duty = -(float)(total_step - STEPS_PER_LEG) * STEP_INCREMENT;   // 0.00, -0.01, ... -0.24
        } else {
            duty = 0.0f;
        }

        motor_set_duty(MOTOR_A, duty);
        motor_set_duty(MOTOR_B, duty);

        float measured_a = encoder_read_velocity_rad_s(ENCODER_A);
        float measured_b = encoder_read_velocity_rad_s(ENCODER_B);

        tick_counter++;
        if (tick_counter >= LOG_EVERY_N_TICKS) {
            tick_counter = 0;
            printf("%lld,%.3f,%.3f,%.3f\n", elapsed_ms, duty, measured_a, measured_b);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Deadband ramp: +staircase then -staircase");
    ESP_LOGI(TAG, "%d steps of %.3f, %d ms each, total %d s per direction",
             STEPS_PER_LEG, (double)STEP_INCREMENT, STEP_MS,
             (STEPS_PER_LEG * STEP_MS) / 1000);

    ESP_ERROR_CHECK(motor_init_all());
    ESP_ERROR_CHECK(encoder_init_all());

    printf("t_ms,commanded_duty,measured_a_rad_s,measured_b_rad_s\n");

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "control task running");
}