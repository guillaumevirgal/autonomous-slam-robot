idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "encoder.c"
        "pid.c"
        "imu.c"
        "battery.c"
        "esp32_serial_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        # driver: gives us mcpwm_prelude.h, pulse_cnt.h, gpio.h
        driver
        # esp_timer: high-resolution microsecond clock for velocity dt
        esp_timer
        # esp_adc: adc_oneshot.h / adc_cali.h for battery voltage sense
        esp_adc
        # micro_ros espidf
        micro_ros_espidf_component
        # freertos: task creation and vTaskDelayUntil
        freertos
        # log: ESP_LOGx macros
        log
)
