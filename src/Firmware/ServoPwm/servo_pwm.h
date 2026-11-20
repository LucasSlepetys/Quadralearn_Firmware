/* Reusable MCPWM output driver. Takes checked ServoCommand values; no bench UI,
 * joint mapping, trajectory generation or feedback sampling belongs here.
 */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include "ServoCommand/servo_command.h"

//Initialize once at boot. No GPIO is attached to MCPWM until explicitly enabled.
void servo_pwm_initialize(void);
//Load the first pulse with the output held LOW. Call only while disabled.
int servo_pwm_prepare(const ServoCommand *command);
//Enable after at least 25 ms, allowing the initial compare value to latch.
int servo_pwm_enable(int leg, int joint);
int servo_pwm_write(const ServoCommand *command);
int servo_pwm_disable(int leg, int joint);
int servo_pwm_disable_all(void);
int servo_pwm_is_enabled(int leg, int joint);
#endif
