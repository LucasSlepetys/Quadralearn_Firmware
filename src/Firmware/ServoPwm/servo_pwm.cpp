/* ServoPwm: checked pulse values -> ESP32-S3 MCPWM signals at 50 Hz.
 * Each pair shares a timer; disabling one output must not stop its neighbour.
 * Prepare holds the selected GPIO LOW; enable attaches PWM. No centre pulse.
 */
#include "servo_pwm.h"
#include "servo_pwm_pins.h"
#include <Arduino.h>
#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>

static int timer_ready[6], connected[12], prepared[12], enabled[12];
static unsigned long prepared_ms[12];

static int channel_index(int leg, int joint)
{
    if (servo_id_for_joint(leg, joint) == SERVO_UNASSIGNED) return -1;
    return leg * SERVO_JOINT_COUNT + joint;
}

static int valid_command(const ServoCommand *command)
{
    if (command == NULL || channel_index(command->leg, command->joint) < 0) return 0;
    if (command->servo_id != servo_id_for_joint(command->leg, command->joint)) return 0;
    return command->pwm_frequency_hz == 50.0 && isfinite(command->pulse_us)
        && command->pulse_us >= 1.0 && command->pulse_us < 20000.0;
}

void servo_pwm_initialize(void)
{
    servo_pwm_disable_all();
    memset(prepared, 0, sizeof(prepared));
}

int servo_pwm_disable(int leg, int joint)
{
    int n = channel_index(leg, joint);
    if (n < 0) return 0;
    int ok = 1;
    if (connected[n]) {
        ok = mcpwm_set_signal_low((mcpwm_unit_t)(n / 6), (mcpwm_timer_t)((n % 6) / 2),
                                 (mcpwm_generator_t)(n % 2)) == ESP_OK;
        //Disconnect this GPIO as well; do not wait for a timer event to go LOW.
        gpio_num_t pin = (gpio_num_t)servo_pwm_gpio[leg][joint];
        if (gpio_reset_pin(pin) != ESP_OK) ok = 0;
        if (gpio_set_level(pin, 0) != ESP_OK) ok = 0;
        if (gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) ok = 0;
        connected[n] = 0;
    }
    enabled[n] = 0;
    prepared[n] = 0;
    return ok;
}

int servo_pwm_disable_all(void)
{
    int ok = 1;
    for (int leg = 0; leg < SERVO_LEG_COUNT; leg++) {
        for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
            if (!servo_pwm_disable(leg, joint)) ok = 0;
        }
    }
    return ok;
}

int servo_pwm_prepare(const ServoCommand *command)
{
    if (!valid_command(command)) return 0;
    int n = channel_index(command->leg, command->joint);
    if (enabled[n]) return 0;
    prepared[n] = 0; //A failed new preparation cannot leave an old pulse armed.
    gpio_num_t pin = (gpio_num_t)servo_pwm_gpio[command->leg][command->joint];
    if (!connected[n]) {
        if (gpio_reset_pin(pin) != ESP_OK || gpio_set_level(pin, 0) != ESP_OK
            || gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) return 0;
    }
    mcpwm_unit_t unit = (mcpwm_unit_t)(n / 6);
    mcpwm_timer_t timer = (mcpwm_timer_t)((n % 6) / 2);
    mcpwm_generator_t output = (mcpwm_generator_t)(n % 2);
    if (!timer_ready[n / 2]) {
        mcpwm_config_t config = {0};
        config.frequency = 50;
        config.duty_mode = MCPWM_DUTY_MODE_0;
        config.counter_mode = MCPWM_UP_COUNTER;
        if (mcpwm_init(unit, timer, &config) != ESP_OK) return 0;
        timer_ready[n / 2] = 1; //Never reinitialize a timer used by another output.
    }
    if (mcpwm_set_signal_low(unit, timer, output) != ESP_OK) return 0;
    if (mcpwm_set_duty_in_us(unit, timer, output, (uint32_t)lround(command->pulse_us)) != ESP_OK) return 0;
    prepared[n] = 1;
    prepared_ms[n] = millis();
    return 1;
}

int servo_pwm_enable(int leg, int joint)
{
    int n = channel_index(leg, joint);
    if (n < 0 || !prepared[n] || enabled[n]) return 0;
    if ((unsigned long)(millis() - prepared_ms[n]) < 25) return 0;
    if (!connected[n]) {
        //Internal output has been LOW for a full period; compare is now loaded.
        if (mcpwm_gpio_init((mcpwm_unit_t)(n / 6), (mcpwm_io_signals_t)(n % 6), servo_pwm_gpio[leg][joint]) != ESP_OK) {
            gpio_reset_pin((gpio_num_t)servo_pwm_gpio[leg][joint]);
            gpio_set_level((gpio_num_t)servo_pwm_gpio[leg][joint], 0);
            gpio_set_direction((gpio_num_t)servo_pwm_gpio[leg][joint], GPIO_MODE_OUTPUT);
            prepared[n] = 0;
            return 0;
        }
        connected[n] = 1;
    }
    if (mcpwm_set_duty_type((mcpwm_unit_t)(n / 6), (mcpwm_timer_t)((n % 6) / 2),
                            (mcpwm_generator_t)(n % 2), MCPWM_DUTY_MODE_0) != ESP_OK) {
        servo_pwm_disable(leg, joint);
        return 0;
    }
    enabled[n] = 1;
    return 1;
}

int servo_pwm_write(const ServoCommand *command)
{
    if (!valid_command(command)) return 0;
    int n = channel_index(command->leg, command->joint);
    if (!enabled[n]) return 0;
    if (mcpwm_set_duty_in_us((mcpwm_unit_t)(n / 6), (mcpwm_timer_t)((n % 6) / 2),
                             (mcpwm_generator_t)(n % 2), (uint32_t)lround(command->pulse_us)) != ESP_OK) {
        servo_pwm_disable(command->leg, command->joint);
        return 0;
    }
    return 1;
}

int servo_pwm_is_enabled(int leg, int joint)
{
    int n = channel_index(leg, joint);
    return n >= 0 && enabled[n];
}
