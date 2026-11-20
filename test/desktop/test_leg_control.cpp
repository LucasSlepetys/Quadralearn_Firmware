/* Isolated LegControl checks: fake ADC/PWM/time, no board or serial connection.
 * A pass proves coordination and failure handling, not physical hold accuracy.
 */
#include "Arduino.h"
#include "mock_mcpwm.h"
#include "Firmware/LegControl/leg_control.h"
#include "Firmware/ServoPwm/servo_pwm.h"
#include "leg_control_config.h"
#include "Configuration/installed_joint_configuration.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned long now_ms = 1000;
static unsigned int pin_mV[64];
static int checks, failures, delays, samples;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { delays++; now_ms += ms; }
void pinMode(int, int) {}
void analogReadResolution(int) {}
void analogSetPinAttenuation(int, int) {}
uint32_t analogReadMilliVolts(int pin) { samples++; return pin_mV[pin]; }

static void check(int condition, const char *description)
{
    checks++;
    if (!condition) { failures++; printf("FAIL: %s\n", description); }
}
static int fr_off(void) { return !mock_pwm_enabled[3] && !mock_pwm_enabled[4] && !mock_pwm_enabled[5]; }
static void advance(LegControl *c, unsigned long ms)
{
    for (unsigned long t = 0; t < ms; t += 5) { now_ms += 5; leg_control_update(c); }
}
static void ready(LegControl *c)
{
    leg_control_disable(c, "Test reset", -1);
    for (int p = 0; p < 64; p++) pin_mV[p] = 1450;
    check(leg_control_initialize(c, 1, &leg_hold_config), "initialize FR without PWM");
    for (int j = 0; j < 3; j++) check(leg_control_configure_joint(c, j, 5, 5), "test-only five-degree margins accepted");
    advance(c, 110);
    check(leg_control_feedback_is_fresh(c), "three valid feedback windows converted to model angles");
}

int main(void)
{
    printf("LegControl: all-three hold, no partial commands, sampling, failure shutdown.\n");
    servo_pwm_initialize();
    LegControl c = {0}; c.leg = -1;
    LegControlConfig bad = leg_hold_config; bad.preload_ms = 1;
    check(!leg_control_initialize(&c, 1, &bad) && fr_off(), "invalid preload does not enable outputs");
    ready(&c);
    check(samples == 66 && delays == 0, "22 short rounds read three inputs each, with no blocking waits");
    check(fr_off(), "read/configure does not enable any output");
    check(!leg_control_configure_joint(&c, 1, 0, 5), "invalid margin clears that joint's configuration");
    check(!leg_control_enable_hold(&c) && c.problem_joint == 1 && fr_off(), "missing one margin blocks the whole hold");
    leg_control_configure_joint(&c, 1, 5, 5);
    check(leg_control_enable_hold(&c) && c.mode == LEG_ARMING && fr_off(), "all pulses prepared while all outputs remain off");
    double q0[3] = {c.measured.angles.alpha_rad, c.measured.angles.theta_1_rad, c.measured.angles.theta_2_rad};
    for (int j = 0; j < 3; j++) check(c.command.joint[j].joint_angle_rad == q0[j], "initial command uses the measured angle, not HOME");
    advance(&c, 25); check(fr_off(), "wait until configured preload duration");
    advance(&c, 5);
    check(c.mode == LEG_HOLDING && mock_pwm_enabled[3] && mock_pwm_enabled[4] && mock_pwm_enabled[5], "all three outputs enabled after preload");
    check(mock_pwm_gpio[3] == 42 && mock_pwm_gpio[4] == 41 && mock_pwm_gpio[5] == 40, "FR PWM pins match the installed assignment");
    for (int n = 0; n < 12; n++) if (n < 3 || n > 5) check(!mock_pwm_enabled[n], "other legs remain disabled");
    LegServoCommand held = c.command;
    pin_mV[1] = 1500; advance(&c, 110);
    check(memcmp(&held, &c.command, sizeof(held)) == 0, "changing feedback does not move the saved hold target");
    check(!leg_control_enable_hold(&c) && c.mode == LEG_HOLDING, "re-enabling does not replace active hold");

    //Later callers can supply q; no hardware write occurs unless all q pass first.
    LegJointAngles target = c.measured.angles; target.theta_1_rad += 0.01;
    int writes = mock_pwm_writes;
    check(leg_control_set_target(&c, target) && mock_pwm_writes == writes+3, "complete valid target writes three outputs");
    held = c.command; writes = mock_pwm_writes;
    target.theta_2_rad = NAN;
    check(!leg_control_set_target(&c, target) && c.problem_joint == 2, "invalid knee reports which joint failed");
    check(mock_pwm_writes == writes && memcmp(&held, &c.command, sizeof(held)) == 0, "invalid target performs zero partial writes and keeps previous hold");
    target = c.measured.angles; target.alpha_rad = c.limits[0].joint_max_rad + 0.1;
    check(!leg_control_set_target(&c, target) && mock_pwm_writes == writes, "outside-limit roll rejected before every write");

    //A second-channel failure must also shut down the first channel already handled.
    ready(&c); mock_fail_operation = 3; mock_fail_after_matches = 1;
    check(!leg_control_enable_hold(&c) && fr_off() && c.problem_joint == 1, "second preparation failure clears all three outputs");
    ready(&c); leg_control_enable_hold(&c); mock_fail_operation = 5; mock_fail_after_matches = 1; advance(&c, 30);
    check(c.mode == LEG_DISABLED && fr_off() && c.problem_joint == 1, "second enable failure shuts down the already-enabled first joint");
    ready(&c); leg_control_enable_hold(&c); advance(&c, 30); target = c.measured.angles;
    mock_fail_operation = 3; mock_fail_after_matches = 1;
    check(!leg_control_set_target(&c, target) && fr_off() && c.problem_joint == 1, "second target-write failure shuts down the whole leg");

    ready(&c); leg_control_enable_hold(&c); advance(&c, 30);
    pin_mV[11] = 0; advance(&c, 110);
    check(c.mode == LEG_DISABLED && fr_off() && c.problem_joint == 2, "bad knee feedback disables all three and identifies the knee");
    pin_mV[11] = 1450; advance(&c, 220);
    check(leg_control_feedback_is_fresh(&c) && fr_off(), "feedback recovery never automatically re-enables PWM");
    leg_control_enable_hold(&c); advance(&c, 30);
    now_ms += leg_hold_config.feedback_timeout_ms + 1; leg_control_update(&c);
    check(c.mode == LEG_DISABLED && fr_off(), "stale old window disables before a new sample could hide the gap");

    //LegControl disable is local: a different leg's driver output is not touched.
    JointMapping mapping; installed_joint_mapping_for_joint(0, 0, &mapping);
    double q; joint_angle_from_servo(&mapping, 90, &q);
    ServoStopClearance gap = {5, 5}; ServoCommand other;
    servo_command_prepare(0, 0, q, &gap, &other); servo_pwm_prepare(&other);
    now_ms += 30; servo_pwm_enable(0, 0);
    ready(&c); leg_control_enable_hold(&c); advance(&c, 30);
    leg_control_disable(&c, "PWM disabled", -1);
    check(fr_off() && mock_pwm_enabled[0], "disabling FR leaves an unrelated output unchanged");
    servo_pwm_disable_all();
    check(delays == 0 && mock_pwm_bad_order == 0, "no blocking delay or incorrect PWM preload ordering");
    printf("Leg control host checks: %d passed, %d failed. No physical hardware tested.\n", checks-failures, failures);
    return failures ? 1 : 0;
}
