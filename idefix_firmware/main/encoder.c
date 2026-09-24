/*
 * encoder.c
 *
 * PCNT-based quadrature decoder for two N20 encoders. Targets the
 * ESP-IDF v5.3 PCNT new driver API (driver/pulse_cnt.h). PCNT's 16-bit
 * hardware counter is extended by the driver itself (flags.accum_count):
 * on each watch-point hit at +/-PCNT_*_LIMIT its ISR adds the limit to an
 * internal accum_value, and pcnt_unit_get_count() returns hw + accum_value,
 * compensating a still-pending overflow event.
 *
 * Do NOT also accumulate watch_point_value in an on_reach callback: this
 * file used to, which counted every overflow twice. Each time a wheel's net
 * count crossed +/-30000 the total jumped by a further 30000 counts
 * (~0.99 m on that wheel), showing up as single /odom samples of ~200-290
 * rad/s and ~132 deg pose jumps (30000 counts -> 8.585 rad -> 131.9 deg
 * after wrapping).
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


// PCNT hardware counter on the S3 is signed 16-bit (-32768..+32767). Choosing +/-30000 gives 2768 counts of margin, which at 10 kHz edge rate is 276 milliseconds.
#define PCNT_HIGH_LIMIT   30000 
#define PCNT_LOW_LIMIT   -30000

typedef struct {
    pcnt_unit_handle_t  unit;
    pcnt_channel_handle_t ch_a; /* watches channel A edges */
    pcnt_channel_handle_t ch_b; /* watches channel B edges */
    volatile int64_t    last_read_counts; /* for velocity dt */
    volatile int64_t    last_read_time_us;
    bool                invert_direction;
    // Serializes encoder_read_counts() against encoder_reset(). The driver's
    // accum_value vs. its overflow ISR (registered on core 0, while
    // control_task reads from core 1) is guarded by the driver's own
    // spinlock inside pcnt_unit_get_count(), so no cross-core ISR guard is
    // needed here any more.
    portMUX_TYPE         mux;
} encoder_ctx_t;

static encoder_ctx_t s_encoders[ENCODER_COUNT]; // One struct per encoder, kept private to this file (`static`).










/* Configure one PCNT unit for one encoder 
Sets up 4x quadrature decoding, glitch filtering, watch points, and starts the counter. Called once per encoder from encoder_init_all().
*/

static esp_err_t configure_pcnt(encoder_ctx_t *enc, gpio_num_t cha, gpio_num_t chb, bool invert){
    
    pcnt_unit_config_t unit_cfg = {0};
    unit_cfg.high_limit = PCNT_HIGH_LIMIT;         // upper watch point
    unit_cfg.low_limit  = PCNT_LOW_LIMIT;          // lower watch point
    unit_cfg.intr_priority = 0;                    // 0 = let IDF pick priority
    unit_cfg.flags.accum_count = true;             // driver extends the 16-bit counter past the limits; see file header
 
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
    // accum_count relies on these: the HW counter resets to 0 at a limit and
    // the driver's ISR adds that limit to its accum_value.
    err = pcnt_unit_add_watch_point(enc->unit, PCNT_HIGH_LIMIT);
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_add_watch_point(enc->unit, PCNT_LOW_LIMIT);
    ESP_ERROR_CHECK(err);
 


    // Start the unit
    err = pcnt_unit_enable(enc->unit);       // move unit from init -> enabled
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_clear_count(enc->unit);  // zero the hardware counter
    ESP_ERROR_CHECK(err);
    err = pcnt_unit_start(enc->unit);        // begin counting
    ESP_ERROR_CHECK(err);
 
    // Initialize the software fields
    enc->last_read_counts  = 0;                        // no previous read yet
    enc->last_read_time_us = esp_timer_get_time();     // record now as "last time"
    enc->invert_direction  = invert;                   // sign flip
    enc->mux               = (portMUX_TYPE) portMUX_INITIALIZER_UNLOCKED;
 
    return ESP_OK;
}
 
// ---- Public: initialize both encoders ----
esp_err_t encoder_init_all(void)
{
    // Configure encoder A using its two GPIO pins from pins.h
    esp_err_t err_a = configure_pcnt(&s_encoders[ENCODER_A],
                                     ENCODER_A_CHA_GPIO,
                                     ENCODER_A_CHB_GPIO,
                                     true);
    ESP_ERROR_CHECK(err_a);
 
    // Configure encoder B likewise
    esp_err_t err_b = configure_pcnt(&s_encoders[ENCODER_B],
                                     ENCODER_B_CHA_GPIO,
                                     ENCODER_B_CHB_GPIO,
                                     false);
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
 
    // Already overflow-extended by the driver (accum_count). int, so it wraps
    // after 2^31 counts (~511k output revolutions, ~70 km of wheel travel).
    int count = 0;


    portENTER_CRITICAL(&enc->mux); // vs. encoder_reset()

    pcnt_unit_get_count(enc->unit, &count);

    int64_t total = (int64_t) count;


    portEXIT_CRITICAL(&enc->mux);
 
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
    portENTER_CRITICAL(&enc->mux);

    // Zeroes both the hardware counter and the driver's accum_value.
    pcnt_unit_clear_count(enc->unit);

    enc->last_read_counts  = 0;
    enc->last_read_time_us = esp_timer_get_time();

    portEXIT_CRITICAL(&enc->mux);
}
 