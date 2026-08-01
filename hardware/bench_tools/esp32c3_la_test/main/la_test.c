// esp32c3_la_test/main/la_test.c
// Independent bench validation for the Binghe 8CH 24 MHz USB logic analyzer.
// Generates known-good square waves, a PWM signal, and a UART pattern on
// seven pins so channels CH1 through CH7 can each be verified. CH8 is left
// unconnected on purpose to check for crosstalk / ghost signals.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

// Pin assignments per LA channel (label = LA channel, GPIO = ESP32-C3 pin)
#define PIN_HEARTBEAT  GPIO_NUM_3   // CH1: 1 Hz "alive" toggle
#define PIN_1KHZ_50    GPIO_NUM_4   // CH2: 1 kHz, 50% duty
#define PIN_10KHZ      GPIO_NUM_5   // CH3: 10 kHz, 50% duty
#define PIN_100KHZ     GPIO_NUM_6   // CH4: 100 kHz, 50% duty
#define PIN_500KHZ     GPIO_NUM_7   // CH5: 500 kHz, 50% duty (7-bit resolution)
#define PIN_1KHZ_25    GPIO_NUM_10  // CH6: 1 kHz, 25% duty (shares timer 0)
#define PIN_UART_TX    GPIO_NUM_8   // CH7: UART1 TX, 115200 8N1

// LEDC uses 4 timers on ESP32-C3, so we choose 4 distinct frequencies
// and stack the 25%-duty channel on the 1 kHz timer.

static void ledc_configure_all(void) {
    // Four timers, one per distinct frequency.
    // 500 kHz timer must run at 7-bit resolution because 500 kHz * 256 = 128 MHz
    // exceeds the 80 MHz APB source clock. 500 kHz * 128 = 64 MHz, fits.
    ledc_timer_config_t t0 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t0);

    ledc_timer_config_t t1 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = 10000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t1);

    ledc_timer_config_t t2 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_2,
        .freq_hz         = 100000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t2);

    ledc_timer_config_t t3 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_7_BIT,   // 500 kHz demands lower res
        .timer_num       = LEDC_TIMER_3,
        .freq_hz         = 500000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t3);

    // CH2: 1 kHz, 50% duty on timer 0
    ledc_channel_config_t c0 = {
        .gpio_num   = PIN_1KHZ_50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 128,   // 50% of 256
        .hpoint     = 0,
    };
    ledc_channel_config(&c0);

    // CH6: 1 kHz, 25% duty on timer 0 (shared timer, different channel + duty)
    ledc_channel_config_t c1 = {
        .gpio_num   = PIN_1KHZ_25,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 64,    // 25% of 256
        .hpoint     = 0,
    };
    ledc_channel_config(&c1);

    // CH3: 10 kHz, 50% duty on timer 1
    ledc_channel_config_t c2 = {
        .gpio_num   = PIN_10KHZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 128,
        .hpoint     = 0,
    };
    ledc_channel_config(&c2);

    // CH4: 100 kHz, 50% duty on timer 2
    ledc_channel_config_t c3 = {
        .gpio_num   = PIN_100KHZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_2,
        .duty       = 128,
        .hpoint     = 0,
    };
    ledc_channel_config(&c3);

    // CH5: 500 kHz, 50% duty on timer 3 (7-bit, so 50% = 64)
    ledc_channel_config_t c4 = {
        .gpio_num   = PIN_500KHZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_4,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_3,
        .duty       = 64,    // 50% of 128
        .hpoint     = 0,
    };
    ledc_channel_config(&c4);
}

static void uart1_init(void) {
    const uart_port_t uart = UART_NUM_1;
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(uart, 256, 0, 0, NULL, 0);
    uart_param_config(uart, &cfg);
    // Only TX matters here, so leave RX/RTS/CTS untouched.
    uart_set_pin(uart, PIN_UART_TX, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void heartbeat_gpio_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_HEARTBEAT,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_HEARTBEAT, 0);
}

void app_main(void) {
    heartbeat_gpio_init();
    ledc_configure_all();
    uart1_init();

    // Log the wiring plan on boot so it lands in any monitor capture
    printf("\n=== LA test firmware running ===\n");
    printf(" CH1 GPIO3  = 1 Hz heartbeat (GPIO toggle)\n");
    printf(" CH2 GPIO4  = 1 kHz,   50%% duty (LEDC)\n");
    printf(" CH3 GPIO5  = 10 kHz,  50%% duty (LEDC)\n");
    printf(" CH4 GPIO6  = 100 kHz, 50%% duty (LEDC)\n");
    printf(" CH5 GPIO7  = 500 kHz, 50%% duty (LEDC, 7-bit)\n");
    printf(" CH6 GPIO10 = 1 kHz,   25%% duty (LEDC, duty test)\n");
    printf(" CH7 GPIO8  = UART1 TX 115200 8N1, byte 0x55\n");
    printf(" CH8        = leave unconnected (crosstalk check)\n");
    printf("================================\n");

    const uint8_t pattern = 0x55;   // 01010101, easy to eyeball in the decoder

    // Heartbeat and UART cadence:
    //   500 ms high, 500 ms low = 1 Hz on CH1.
    //   Five UART bytes per half-second at ~100 ms spacing, so the UART
    //   decoder always has fresh traffic during any capture window.
    while (1) {
        gpio_set_level(PIN_HEARTBEAT, 1);
        for (int i = 0; i < 5; i++) {
            uart_write_bytes(UART_NUM_1, &pattern, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        gpio_set_level(PIN_HEARTBEAT, 0);
        for (int i = 0; i < 5; i++) {
            uart_write_bytes(UART_NUM_1, &pattern, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
