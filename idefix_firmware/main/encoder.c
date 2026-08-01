/*
 * encoder.c
 *
 * PCNT-based quadrature decoder for two N20 encoders. Targets the
 * ESP-IDF v5.3 PCNT new driver API (driver/pulse_cnt.h). Uses the
 * standard trick for extending PCNT's 16-bit hardware counter to a
 * software-tracked 64-bit accumulator via watch-point interrupts.
 */


// -> is used for accessing members (variables, methods) of a structure or class through a pointer

#include "encoder.h"
#include "pins.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"

#include <math.h>

static const char *TAG = "encoder"; // A tag string used by ESP_LOG functions to prefix log messages.

// Encoder mechanical constants
#define ENCODER_PULSES_PER_MOTOR_REV    7 // 7 pulses per motor-shaft revolution per channel
#define ENCODER_QUAD_MULTIPLIER         4 // 4x quadrature decoding (both edges, both channels)
#define ENCODER_GEAR_RATIO              150 // 150:1 gearbox reduction
#define ENCODER_COUNTS_PER_OUTPUT_REV   (ENCODER_PULSES_PER_MOTOR_REV * ENCODER_QUAD_MULTIPLIER * ENCODER_GEAR_RATIO) // COUNTS_PER_OUTPUT_REV = 7 * 4 * 150 = 4200

// PCNT hardware counter on the S3 is signed 16-bit (-32768..+32767). Choosing +/-30000 gives 2768 counts of margin, which at 10 kHz edge rate is 276 milliseconds.
#define PCNT_HIGH_LIMIT   30000 
#define PCNT_LOW_LIMIT   -30000

typedef struct {
    pcnt_unit_handle_t  unit;
    pcnt_channel_handle_t ch_a; /* watches channel A edges */
    pcnt_channel_handle_t ch_b; /* watches channel B edges */
    volatile int64_t    accumulator;      /* extended counter */
    volatile int64_t    last_read_counts; /* for velocity dt */
    volatile int64_t    last_read_time_us;
    bool                invert_direction;
} encoder_ctx_t;

static encoder_ctx_t s_encoders[ENCODER_COUNT]; // One struct per encoder, kept private to this file (`static`).




/* Watch-point ISR: called from PCNT hardware when the counter reaches one of our configured limits. 
   Watch-point event, so we don't need to clear it manually. */
static bool IRAM_ATTR pcnt_watch_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx){

    encoder_ctx_t *enc = (encoder_ctx_t *)user_ctx; // Declares enc as an encoder_ctx_t

    // Add the watch point value (+30000 or -30000) to the accumulator.
    enc->accumulator += edata->watch_point_value; // watch_point_value is defined by ESP-IDF
    return false; /* no higher priority task woken */
}






/* Configure one PCNT unit for one encoder 
Sets up 4x quadrature decoding, glitch filtering, watch points, and starts the counter. Called once per encoder from encoder_init_all().
*/

static esp_err_t configure_pcnt(encoder_ctx_t *enc, gpio_num_t cha, gpio_num_t chb){
    
    pcnt_unit_config_t unit_cfg = {0};
    unit_cfg.high_limit = PCNT_HIGH_LIMIT;         // upper watch point
    unit_cfg.low_limit  = PCNT_LOW_LIMIT;          // lower watch point
    unit_cfg.intr_priority = 0;                    // 0 = let IDF pick priority
    unit_cfg.flags.accum_count = true;             // auto-reset HW counter at watch point
 
    // Ask the driver to create a new PCNT unit with that config.
    esp_err_t err = pcnt_new_unit(&unit_cfg, &enc->unit);
    ESP_ERROR_CHECK(err);
 
    // Glitch filtering    
    pcnt_glitch_filter_config_t glitch_cfg = {0};
    glitch_cfg.max_glitch_ns = 1000; // Any pulse shorter than 1000 ns will be ignored in hardware to reject noise pulses
    err = pcnt_unit_set_glitch_filter(enc->unit, &glitch_cfg);
    ESP_ERROR_CHECK(err);
 
    // Channel A
    // Channel A watches edges on encoder pin A. Channel B pin is used as the "level" input, so we know which direction the shaft turns.
    pcnt_chan_config_t chan_a_cfg = {0};
    chan_a_cfg.edge_gpio_num  = cha;               // count edges on this pin
    chan_a_cfg.level_gpio_num = chb;               // read this pin to decide direction
 
    // Create channel A inside the unit.
    err = pcnt_new_channel(enc->unit, &chan_a_cfg, &enc->ch_a);
    ESP_ERROR_CHECK(err);
 
    // Edge action
    err = pcnt_channel_set_edge_action(enc->ch_a,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE,  // negative edge -> decrease
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE); // positive edge -> increase
    ESP_ERROR_CHECK(err);
 
    // Level action
    err = pcnt_channel_set_level_action(enc->ch_a,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,    // B low  -> keep edge action as configured above
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);// B high -> invert it (increase becomes decrease and vice versa)
    ESP_ERROR_CHECK(err);
 
    // Channel B (mirror image of channel A)
    pcnt_chan_config_t chan_b_cfg = {0};
    chan_b_cfg.edge_gpio_num  = chb;               // count edges on pin B this time
    chan_b_cfg.level_gpio_num = cha;               // use pin A for direction
 
    err = pcnt_new_channel(enc->unit, &chan_b_cfg, &enc->ch_b);
    ESP_ERROR_CHECK(err);
 
    // Opposite polarity to channel A so that both channels sum correctly.
    err = pcnt_channel_set_edge_action(enc->ch_b,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    ESP_ERROR_CHECK(err);
 
    // Same as Channel A
    err = pcnt_channel_set_level_action(enc->ch_b,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    ESP_ERROR_CHECK(err);
 

    // Watch points
    // Ask PCNT to fire an event when the counter reaches these values
    err = pcnt_unit_add_watch_point(enc->unit, PCNT_HIGH_LIMIT);
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_add_watch_point(enc->unit, PCNT_LOW_LIMIT);
    ESP_ERROR_CHECK(err);
 

    // ISR callback
    pcnt_event_callbacks_t cbs = {0};
    cbs.on_reach = pcnt_watch_cb; // `cbs.on_reach` is called when a watch point fires
    err = pcnt_unit_register_event_callbacks(enc->unit, &cbs, enc); 
    ESP_ERROR_CHECK(err);
 

    // Start the unit
    err = pcnt_unit_enable(enc->unit);       // move unit from init -> enabled
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_clear_count(enc->unit);  // zero the hardware counter
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_start(enc->unit);        // begin counting
    ESP_ERROR_CHECK(err);
 
    // Initialize the software fields
    enc->accumulator       = 0;                        // start at zero
    enc->last_read_counts  = 0;                        // no previous read yet
    enc->last_read_time_us = esp_timer_get_time();     // record now as "last time"
    enc->invert_direction  = false;                    // no sign flip by default
 
    return ESP_OK;
}
 
// ---- Public: initialize both encoders ----
esp_err_t encoder_init_all(void)
{
    // Configure encoder A using its two GPIO pins from pins.h
    esp_err_t err_a = configure_pcnt(&s_encoders[ENCODER_A],
                                     ENCODER_A_CHA_GPIO,
                                     ENCODER_A_CHB_GPIO);
    ESP_ERROR_CHECK(err_a);
 
    // Configure encoder B likewise
    esp_err_t err_b = configure_pcnt(&s_encoders[ENCODER_B],
                                     ENCODER_B_CHA_GPIO,
                                     ENCODER_B_CHB_GPIO);
    ESP_ERROR_CHECK(err_b);
 
    // Print a friendly message so we know init succeeded
    ESP_LOGI(TAG, "encoders initialized (4x quadrature, 1 us glitch filter)");
    return ESP_OK;
}
 
// Read total accumulated count, Returns a signed 64-bit count. Sign encodes direction.
int64_t encoder_read_counts(encoder_id_t enc_id)
{
    // Bounds check on the encoder id.
    if (enc_id >= ENCODER_COUNT) {
        return 0;
    }
 
    // Grab a pointer to the right context struct.
    encoder_ctx_t *enc = &s_encoders[enc_id];
 
    // Local variable to hold the current hardware count.
    int hw_count = 0;
 
    
    portDISABLE_INTERRUPTS(); // MUTEX start (disable ISR)
 
    // Ask the PCNT driver for the current hardware counter value.
    pcnt_unit_get_count(enc->unit, &hw_count);
 
    // Combine the 64-bit accumulator with the current 16-bit hardware value.
    int64_t total = enc->accumulator + (int64_t) hw_count;
 
    
    portENABLE_INTERRUPTS(); // MUTEX end 
 
    // If direction is inverted for this encoder, flip the sign.
    if (enc->invert_direction) {
        return -total;
    }
    return total;
}
 
// Read instantaneous velocity in rad/s (output shaft)
float encoder_read_velocity_rad_s(encoder_id_t enc_id)
{
    // Bounds check.
    if (enc_id >= ENCODER_COUNT) {
        return 0.0f;
    }
 
    // Pointer to the target encoder's state.
    encoder_ctx_t *enc = &s_encoders[enc_id];
 
    // Read the current time and the current count.
    int64_t now_us = esp_timer_get_time();
    int64_t counts_now = encoder_read_counts(enc_id);
 
    // Compute how many counts have accumulated since the last call.
    int64_t d_counts = counts_now - enc->last_read_counts;
 
    // Compute how many microseconds have passed since the last call.
    int64_t d_us = now_us - enc->last_read_time_us;
 
    // Update the "last" values so the next call has a fresh baseline.
    enc->last_read_counts  = counts_now;
    enc->last_read_time_us = now_us;
 
    // Guard against a zero or negative time delta.
    if (d_us <= 0) {
        return 0.0f;
    }
 

    // Convert counts to rad/s in three steps
    // 1. Counts -> output shaft revolutions.
    float revs = (float) d_counts / (float) ENCODER_COUNTS_PER_OUTPUT_REV;
 
    // 2. Revolutions -> radians (one revolution = 2*pi radians).
    float rad = revs * 2.0f * (float) M_PI;
 
    // 3. Convert microseconds to seconds.
    float seconds = (float) d_us * 1e-6f;
 
    // Final result: radians per second.
    return rad / seconds;
}
 
// Reset a single encoder's software counter
void encoder_reset(encoder_id_t enc_id)
{
    // Bounds check.
    if (enc_id >= ENCODER_COUNT) {
        return;
    }
 
    encoder_ctx_t *enc = &s_encoders[enc_id];
 
    // Same critical section pattern as encoder_read_counts
    portDISABLE_INTERRUPTS();
 
    // Zero the hardware counter.
    pcnt_unit_clear_count(enc->unit);
 
    // Zero all software state.
    enc->accumulator       = 0;
    enc->last_read_counts  = 0;
    enc->last_read_time_us = esp_timer_get_time();
 
    portENABLE_INTERRUPTS();
}
 