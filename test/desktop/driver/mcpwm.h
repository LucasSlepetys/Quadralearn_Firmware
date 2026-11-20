//Desktop-only stand-in for ESP-IDF. No hardware access.
#ifndef TEST_MCPWM_H
#define TEST_MCPWM_H
#include <stdint.h>
typedef int esp_err_t;
enum { ESP_OK = 0, ESP_FAIL = -1 };
typedef enum { MCPWM_UNIT_0, MCPWM_UNIT_1 } mcpwm_unit_t;
typedef enum { MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_2 } mcpwm_timer_t;
typedef enum { MCPWM_OPR_A, MCPWM_OPR_B } mcpwm_generator_t;
typedef enum { MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B, MCPWM2A, MCPWM2B } mcpwm_io_signals_t;
typedef enum { MCPWM_DUTY_MODE_0, MCPWM_DUTY_MODE_1 } mcpwm_duty_type_t;
typedef enum { MCPWM_UP_COUNTER } mcpwm_counter_type_t;
typedef struct {
    uint32_t frequency;
    float cmpr_a, cmpr_b;
    mcpwm_duty_type_t duty_mode;
    mcpwm_counter_type_t counter_mode;
} mcpwm_config_t;
esp_err_t mcpwm_init(mcpwm_unit_t, mcpwm_timer_t, const mcpwm_config_t *);
esp_err_t mcpwm_gpio_init(mcpwm_unit_t, mcpwm_io_signals_t, int);
esp_err_t mcpwm_set_signal_low(mcpwm_unit_t, mcpwm_timer_t, mcpwm_generator_t);
esp_err_t mcpwm_set_duty_in_us(mcpwm_unit_t, mcpwm_timer_t, mcpwm_generator_t, uint32_t);
esp_err_t mcpwm_set_duty_type(mcpwm_unit_t, mcpwm_timer_t, mcpwm_generator_t, mcpwm_duty_type_t);
#endif
