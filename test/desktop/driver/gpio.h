#ifndef TEST_GPIO_H
#define TEST_GPIO_H
#include "mcpwm.h"
typedef int gpio_num_t;
enum { GPIO_MODE_OUTPUT = 1 };
esp_err_t gpio_reset_pin(gpio_num_t);
esp_err_t gpio_set_level(gpio_num_t, uint32_t);
esp_err_t gpio_set_direction(gpio_num_t, int);
#endif
