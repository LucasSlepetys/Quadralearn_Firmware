#ifndef MOCK_MCPWM_H
#define MOCK_MCPWM_H
extern int mock_pwm_gpio[12], mock_pwm_enabled[12], mock_pwm_timer_inits[6];
extern unsigned int mock_pwm_pulse[12];
extern int mock_pwm_writes, mock_pwm_bad_order, mock_fail_operation;
extern int mock_fail_after_matches; //Test-only: fail the second/third matching hardware call.
//Inject one API failure: 1 init, 2 force-low, 3 pulse, 4 attach, 5 enable.
#endif
