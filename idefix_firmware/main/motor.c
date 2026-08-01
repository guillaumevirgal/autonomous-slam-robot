/*
TB6612FNG + MCPWM implementation
 */

#include "motor.h"
#include "pins.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "motor";

// PWM configuration.
#define MCPWM_TIMER_RESOLUTION_HZ   10000000  // MCPWM group clock is set to 10 MHz (100 ns tick)
#define MCPWM_PWM_PERIOD_TICKS      500       // 500 ticks -> 20 kHz PWM frequency;  values run from 0 (0% duty) to 500 (100% duty)


typedef struct {
    mcpwm_timer_handle_t   timer;             // the PWM timer (sets the period)
    mcpwm_oper_handle_t    oper;              // operator (links timer to comparator)
    mcpwm_cmpr_handle_t    comparator;        // comparator (sets the duty cycle)
    mcpwm_gen_handle_t     generator;         // generator (drives the actual GPIO)
    gpio_num_t             in1_gpio;          // TB6612 direction pin 1
    gpio_num_t             in2_gpio;          // TB6612 direction pin 2
    bool                   invert_direction;  // if true, positive duty spins the other way
} motor_ctx_t;

static motor_ctx_t s_motors[MOTOR_COUNT]; // One context per motor

// Sets IN1 and IN2 as outputs and drives them both LOW (coast).
static esp_err_t configure_direction_pins(gpio_num_t in1, gpio_num_t in2){
    gpio_config_t cfg = {0};
    cfg.pin_bit_mask = (1ULL << in1) | (1ULL << in2);  // select both pins
    cfg.mode         = GPIO_MODE_OUTPUT;               // output mode
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;            // no internal pull-up
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;          // no internal pull-down
    cfg.intr_type    = GPIO_INTR_DISABLE;              // no interrupts on these pins
 
    
    esp_err_t err = gpio_config(&cfg); // Apply the configuration
    if (err != ESP_OK) {
        return err;   
    }

    // Both LOW = coast (motor freewheels)
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 0);
    return ESP_OK;
}

// Configure MCPWM timer, operator, comparator, generator for one motor
static esp_err_t configure_mcpwm(motor_ctx_t *m, gpio_num_t pwm_gpio)
{
    // Timer: sets the PWM period
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src    = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = MCPWM_PWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &m->timer));

    // Operator: connects a timer to comparators/generators
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &m->oper)); // Create the operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m->oper, m->timer)); // Attach the operator to the timer

    // Comparator: the duty-cycle threshold
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,   // update at timer-equals-zero 
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &cmp_cfg, &m->comparator)); // Create the comparator inside the operator
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->comparator, 0)); // Start at compare value 0, meaning 0% duty

    // Generator: drives the actual GPIO pin
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = pwm_gpio }; // the GPIO that outputs the PWM wave
    ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gen_cfg, &m->generator));  // Create the generator

    // At the start of each period (counter empty), drive the pin HIGH
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(m->generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
    // When the counter reaches the compare value, drive the pin LOW
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event( m->generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m->comparator, MCPWM_GEN_ACTION_LOW)));

    // Enable and start the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(m->timer)); // move timer from init to enabled
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP)); // Start the timer running freely

    return ESP_OK;
}

esp_err_t motor_init_all(void) //initialize both motors
{
    // Step 1: force STBY LOW before touching anything else, STBY LOW = TB6612 output disabled
    gpio_config_t stby_cfg = {
        .pin_bit_mask = (1ULL << TB6612_STBY_GPIO), // select the STBY pin
        .mode         = GPIO_MODE_OUTPUT,           // output mode
    };
    ESP_ERROR_CHECK(gpio_config(&stby_cfg));
    gpio_set_level(TB6612_STBY_GPIO, 0); // Drive STBY LOW: driver disabled for now

    // Configure motor A
    s_motors[MOTOR_A].in1_gpio         = MOTOR_A_IN1_GPIO; // store its IN1 pin
    s_motors[MOTOR_A].in2_gpio         = MOTOR_A_IN2_GPIO; // store its IN2 pin
    s_motors[MOTOR_A].invert_direction = false;            // no sign flip by default
    ESP_ERROR_CHECK(configure_direction_pins(MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO));  // Set up the direction pins for motor A
    ESP_ERROR_CHECK(configure_mcpwm(&s_motors[MOTOR_A], MOTOR_A_PWM_GPIO));  // Set up the MCPWM pin for motor A

    // Configure motor B
    s_motors[MOTOR_B].in1_gpio         = MOTOR_B_IN1_GPIO;
    s_motors[MOTOR_B].in2_gpio         = MOTOR_B_IN2_GPIO;
    s_motors[MOTOR_B].invert_direction = false;
    ESP_ERROR_CHECK(configure_direction_pins(MOTOR_B_IN1_GPIO, MOTOR_B_IN2_GPIO));
    ESP_ERROR_CHECK(configure_mcpwm(&s_motors[MOTOR_B], MOTOR_B_PWM_GPIO));

    // Step 4: only now enable the driver by driving STBY HIGH
    gpio_set_level(TB6612_STBY_GPIO, 1);

    ESP_LOGI(TAG, "motors initialized, STBY high, duty = 0");
    return ESP_OK;
}

void motor_set_duty(motor_id_t motor, float duty){ // Duty is in the range [-1.0, +1.0], sign selects direction
    if (motor >= MOTOR_COUNT) return;  // Ignore invalid motor ids
    motor_ctx_t *m = &s_motors[motor]; // Pointer to the target motor

    // Clamp the duty into the legal range instead of rejecting it
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;

    // Apply direction inversion if configured
    if (m->invert_direction) duty = -duty;

    // Determine direction pins
    if (duty > 0.0f) { // Forward
        gpio_set_level(m->in1_gpio, 1);
        gpio_set_level(m->in2_gpio, 0);
    } else if (duty < 0.0f) { // Reverse
        gpio_set_level(m->in1_gpio, 0);
        gpio_set_level(m->in2_gpio, 1);
    } else { // Coast
        gpio_set_level(m->in1_gpio, 0);
        gpio_set_level(m->in2_gpio, 0);
    }

    // Convert magnitude to comparator 
    float magnitude = duty < 0.0f ? -duty : duty; // Absolute value of duty
    uint32_t compare = (uint32_t)(magnitude * MCPWM_PWM_PERIOD_TICKS); // Scale [0.0; 1.0] into [0; PERIOD_TICKS].
    if (compare > MCPWM_PWM_PERIOD_TICKS) compare = MCPWM_PWM_PERIOD_TICKS; // Safety clamp

    mcpwm_comparator_set_compare_value(m->comparator, compare); // Push the new duty, takes effect when timer = 0
}

void motor_emergency_stop(void)
{
    // Set both motors to coast (0 duty, both direction pins low)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set_duty((motor_id_t)i, 0.0f);
    }
    // STBY to low
    gpio_set_level(TB6612_STBY_GPIO, 0);
    ESP_LOGW(TAG, "EMERGENCY STOP: STBY driven LOW");
}
