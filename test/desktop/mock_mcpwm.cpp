/* Hardware-free MCPWM spy. Checks sequencing and neighbour isolation, not real
 * electrical waveforms. Scope verification is still required on the ESP32.
 */
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "Arduino.h"
#include "mock_mcpwm.h"
int mock_pwm_gpio[12], mock_pwm_enabled[12], mock_pwm_timer_inits[6];
unsigned int mock_pwm_pulse[12];
int mock_pwm_writes, mock_pwm_bad_order, mock_fail_operation;
int mock_fail_after_matches;
static unsigned long loaded_ms[12];

static int fail(int operation)
{
    if (mock_fail_operation != operation) return 0;
    if (mock_fail_after_matches > 0) { mock_fail_after_matches--; return 0; }
    mock_fail_operation = 0;
    return 1;
}
esp_err_t mcpwm_init(mcpwm_unit_t unit, mcpwm_timer_t timer, const mcpwm_config_t *config)
{
    if (fail(1)) return ESP_FAIL;
    int pair = 3*unit + timer;
    if (mock_pwm_enabled[2*pair] || mock_pwm_enabled[2*pair+1] || config->frequency != 50
        || config->cmpr_a != 0 || config->cmpr_b != 0) mock_pwm_bad_order++;
    mock_pwm_timer_inits[pair]++;
    return ESP_OK;
}
esp_err_t mcpwm_gpio_init(mcpwm_unit_t unit, mcpwm_io_signals_t signal, int gpio)
{
    if (fail(4)) return ESP_FAIL;
    int n = 6*unit + signal;
    if (mock_pwm_enabled[n]) mock_pwm_bad_order++;
    mock_pwm_gpio[n] = gpio;
    return ESP_OK;
}
esp_err_t mcpwm_set_signal_low(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t output)
{
    if (fail(2)) return ESP_FAIL;
    mock_pwm_enabled[6*unit + 2*timer + output] = 0;
    return ESP_OK;
}
esp_err_t mcpwm_set_duty_in_us(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t output, uint32_t pulse)
{
    if (fail(3)) return ESP_FAIL;
    int n = 6*unit + 2*timer + output;
    mock_pwm_pulse[n] = pulse;
    loaded_ms[n] = millis();
    mock_pwm_writes++;
    return ESP_OK;
}
esp_err_t mcpwm_set_duty_type(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t output, mcpwm_duty_type_t)
{
    if (fail(5)) return ESP_FAIL;
    int n = 6*unit + 2*timer + output;
    if ((unsigned long)(millis() - loaded_ms[n]) < 20 || mock_pwm_pulse[n] == 0) mock_pwm_bad_order++;
    mock_pwm_enabled[n] = 1;
    return ESP_OK;
}
esp_err_t gpio_reset_pin(gpio_num_t gpio)
{
    for (int n = 0; n < 12; n++) if (mock_pwm_gpio[n] == gpio) mock_pwm_enabled[n] = 0;
    return ESP_OK;
}
esp_err_t gpio_set_level(gpio_num_t, uint32_t value) { if (value != 0) mock_pwm_bad_order++; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t, int) { return ESP_OK; }
