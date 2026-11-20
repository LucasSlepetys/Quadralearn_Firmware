/* Bench tests use fake time, ADC, serial and MCPWM. They prove request handling
 * and pulse calculations, NOT physical wiring, servo direction or clearance.
 */
#include "Arduino.h"
#include "mock_mcpwm.h"
#include "Firmware/ServoPwm/servo_pwm.h"
#include "Firmware/ServoBench/servo_bench.h"
#include "Firmware/BenchLink/bench_link.h"
#include "Firmware/CaptureLink/capture_link.h"
#include "Configuration/installed_joint_configuration.h"
#include "board_pins.h"
#include "servo_bench_config.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static int checks, failures, input_position, delays, samples;
static unsigned long now_ms = 1000;
static unsigned int pin_mV[64];
static char input[512], output[24000];
TestSerial Serial;
void setup(void);
void loop(void);
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { now_ms += ms; delays++; }
void pinMode(int, int) {}
void analogReadResolution(int) {}
void analogSetPinAttenuation(int, int) {}
uint32_t analogReadMilliVolts(int gpio) { samples++; return pin_mV[gpio]; }
void TestSerial::begin(unsigned long) {}
void TestSerial::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(output + strlen(output), sizeof(output) - strlen(output), format, args);
    va_end(args);
}
void TestSerial::println(const char *s) { this->printf("%s\n", s); }
int TestSerial::available(void) { return input[input_position] != '\0'; }
int TestSerial::read(void) { return available() ? input[input_position++] : -1; }

static void check(int condition, const char *message)
{
    checks++;
    if (!condition) { failures++; ::printf("FAIL: %s\n", message); }
}
static int all_off(void)
{
    for (int n = 0; n < 12; n++) if (mock_pwm_enabled[n]) return 0;
    return 1;
}
static void advance(ServoBench *bench, unsigned long ms, int heartbeat = 1)
{
    for (unsigned long elapsed = 0; elapsed < ms; elapsed += 5) {
        now_ms += 5;
        if (heartbeat) bench->last_request_ms = now_ms;
        servo_bench_update(bench);
    }
}
static void ready(ServoBench *bench)
{
    servo_bench_select(bench, 1, 1);
    pin_mV[19] = 1450; //S05 calibration knot: physical a = 90 degrees.
    advance(bench, 105);
    check(bench->feedback_valid && fabs(bench->measured_a_deg - 90.0) < 1e-8, "GPIO19 feedback uses S05's table");
    check(servo_bench_configure(bench, 5, 5), "TEST-ONLY 5-degree clearances accepted");
}

int main(int argc, char **argv)
{
    for (int gpio = 0; gpio < 64; gpio++) pin_mV[gpio] = 1450;
    //Python integration bridge runs the actual main/serial handlers with fake hardware.
    if (argc == 2 && strcmp(argv[1], "--bridge") == 0) {
        setup(); ::printf("%s", output); fflush(stdout);
        char incoming[512];
        while (fgets(incoming, sizeof(incoming), stdin)) {
            unsigned long ms; int gpio; unsigned int value;
            if (sscanf(incoming, "TICK %lu", &ms) == 1) {
                input[0] = '\0'; input_position = 0;
                for (unsigned long step = 0; step < ms; step += 5) { now_ms += 4; loop(); }
                ::printf("TICK_OK\n"); fflush(stdout); continue;
            }
            if (sscanf(incoming, "TEST %d %u", &gpio, &value) == 2 && gpio >= 0 && gpio < 64) {
                pin_mV[gpio] = value; ::printf("TEST_OK\n"); fflush(stdout); continue;
            }
            output[0] = '\0'; snprintf(input, sizeof(input), "%s", incoming); input_position = 0;
            loop(); ::printf("%s", output); fflush(stdout);
        }
        return 0;
    }

    ::printf("PWM driver: boot disabled, preload, pin identity, shared timers and API failures.\n");
    servo_pwm_initialize();
    check(all_off() && mock_pwm_writes == 0, "boot generates no pulse");
    ServoStopClearance clearance = {5, 5}; //Examples for software tests only.
    JointMapping mapping;
    installed_joint_mapping_for_joint(1, 1, &mapping);
    double q;
    joint_angle_from_servo(&mapping, 90, &q);
    ServoCommand command;
    check(servo_command_prepare(1, 1, q, &clearance, &command), "prepare actual S05 angle/pulse");
    check(!servo_pwm_write(&command), "write cannot implicitly enable PWM");
    check(!servo_pwm_enable(1, 1), "enable cannot use unprepared pulse");
    check(servo_pwm_prepare(&command) && all_off(), "preload keeps signal LOW");
    check(mock_pwm_gpio[4] == 0 && mock_pwm_pulse[4] == (unsigned)lround(command.pulse_us), "calibrated pulse loads before any PWM pin attachment");
    check(!servo_pwm_enable(1, 1), "wait before releasing force-LOW");
    now_ms += 30;
    check(servo_pwm_enable(1, 1), "enable after preload interval");
    check(mock_pwm_enabled[4] && mock_pwm_gpio[4] == 41 && !mock_pwm_enabled[3] && !mock_pwm_enabled[5], "only S05 enabled on GPIO41");
    ServoCommand bad = command; bad.pulse_us = NAN;
    check(!servo_pwm_write(&bad), "NaN never reaches MCPWM");
    bad = command; bad.servo_id = SERVO_S04;
    check(!servo_pwm_write(&bad), "mismatched servo identity rejected");
    bad = command; bad.pwm_frequency_hz = 100;
    check(!servo_pwm_write(&bad), "wrong calibration frequency rejected");
    bad = command; bad.leg = 4;
    check(!servo_pwm_prepare(&bad), "invalid index rejected before GPIO access");

    //Prepare S06 on the other output of the SAME timer: S05 must stay active.
    ServoCommand neighbour;
    installed_joint_mapping_for_joint(1, 2, &mapping);
    joint_angle_from_servo(&mapping, 90, &q);
    servo_command_prepare(1, 2, q, &clearance, &neighbour);
    int inits = mock_pwm_timer_inits[2];
    check(servo_pwm_prepare(&neighbour), "second timer output can be prepared");
    now_ms += 30; check(servo_pwm_enable(1, 2), "second output enables separately");
    check(mock_pwm_timer_inits[2] == inits && mock_pwm_enabled[4], "shared timer is not reset");
    check(servo_pwm_disable(1, 1) && mock_pwm_enabled[5], "disable S05 does not stop S06");
    servo_pwm_disable_all();
    for (int failure = 2; failure <= 3; failure++) {
        mock_fail_operation = failure;
        check(!servo_pwm_prepare(&command) && all_off(), "failed preparation keeps outputs disabled");
    }
    servo_pwm_prepare(&command); now_ms += 30; mock_fail_operation = 5;
    check(!servo_pwm_enable(1, 1) && all_off(), "hardware enable error remains disabled");
    servo_pwm_prepare(&command); now_ms += 30; servo_pwm_enable(1, 1); mock_fail_operation = 3;
    check(!servo_pwm_write(&command) && all_off(), "hardware write error disables selected channel");
    servo_pwm_prepare(&command); now_ms += 30; servo_pwm_enable(1, 1); mock_fail_operation = 2;
    check(!servo_pwm_disable(1, 1) && all_off(), "force-LOW failure uses GPIO fallback and reports failure");
    check(mock_pwm_bad_order == 0, "no output attached active or enabled before preload");

    //Driver-only synthetic commands test routing for all 12 outputs. This does
    //not authorize them in the bench or provide missing RR installed limits.
    const int expected_pwm[12] = {7,15,16,42,41,40,18,3,46,45,48,47};
    bad = command; bad.leg = 2; bad.joint = 0; bad.servo_id = SERVO_S07;
    mock_fail_operation = 1;
    check(!servo_pwm_prepare(&bad) && all_off(), "timer initialization error produces no output");
    servo_pwm_prepare(&bad); now_ms += 30; mock_fail_operation = 4;
    check(!servo_pwm_enable(2, 0) && all_off(), "GPIO attachment error produces no output");
    check(!servo_pwm_enable(2, 0), "failed preparation cannot enable a stale pulse");
    for (int n = 0; n < 12; n++) {
        bad = command; bad.leg = n/3; bad.joint = n%3; bad.servo_id = (ServoId)(n+1);
        check(servo_pwm_prepare(&bad), "every output can be prepared independently");
        now_ms += 30;
        check(servo_pwm_enable(n/3, n%3) && mock_pwm_gpio[n] == expected_pwm[n], "unit/timer/output map selects expected GPIO");
    }
    check(servo_pwm_disable(1, 1), "disable S05 while other outputs run");
    for (int n = 0; n < 12; n++) check(mock_pwm_enabled[n] == (n != 4), "single-output disable leaves all others unchanged");
    servo_pwm_disable_all();
    check(all_off() && mock_pwm_bad_order == 0, "all-output shutdown and shared timer behavior correct");

    ::printf("Bench controller: measured initial hold, quintic moves, limits, stop and timeouts.\n");
    ServoBench bench;
    servo_bench_initialize(&bench);
    check(!servo_bench_arm(&bench), "empty state cannot arm");
    ready(&bench);
    check(!servo_bench_configure(&bench, 0, 5) && !bench.configured, "zero margin invalidates configuration");
    servo_bench_configure(&bench, 5, 5);
    int before_delays = delays;
    check(servo_bench_arm(&bench) && bench.mode == BENCH_ARMING && all_off(), "ARM begins disabled preload");
    double start = bench.measured_q_rad;
    check(fabs(bench.command.joint_angle_rad - start) < 1e-12, "initial hold uses fresh measured q");
    advance(&bench, 35);
    check(bench.mode == BENCH_HOLDING && mock_pwm_enabled[4], "S05 initial hold enabled");
    check(!servo_bench_move(&bench, 0.2, 1), "excessive step rejected");
    check(!servo_bench_move(&bench, 0.01, 0.1), "too-short move rejected");
    check(!servo_bench_move(&bench, NAN, 1), "nonfinite move rejected");
    const double step = 0.017453292519943295;
    unsigned int start_pulse = mock_pwm_pulse[4];
    double duration_s = fmax(bench_min_move_s, 1.875 / bench_max_speed_deg_s);
    check(servo_bench_move(&bench, step, duration_s), "one-degree move uses configured duration and speed");
    advance(&bench, (unsigned long)(500 * duration_s));
    check(fabs(bench.command.joint_angle_rad - (start + step*0.5)) < step*0.04, "midpoint matches quintic F(0.5)");
    advance(&bench, (unsigned long)(500 * duration_s) + 20);
    check(bench.mode == BENCH_HOLDING && fabs(bench.command.joint_angle_rad - (start+step)) < 1e-12, "move reaches target without overshoot");
    check(mock_pwm_pulse[4] > start_pulse, "FR pitch positive q produces larger calibrated pulse");
    check(servo_bench_move(&bench, -step, duration_s), "opposite move accepted");
    advance(&bench, 200);
    double held = bench.command.joint_angle_rad;
    servo_bench_stop(&bench); advance(&bench, 500);
    check(bench.mode == BENCH_HOLDING && bench.command.joint_angle_rad == held, "STOP freezes last command");
    check(delays == before_delays, "bench sampling never calls delay");
    advance(&bench, 1005, 0);
    check(bench.mode == BENCH_DISABLED && all_off(), "lost heartbeat disables PWM");
    ready(&bench); servo_bench_arm(&bench); advance(&bench, 35);
    now_ms += 300; bench.last_request_ms = now_ms; servo_bench_update(&bench);
    check(bench.mode == BENCH_DISABLED && all_off(), "stale feedback detected before a new sample");
    ready(&bench); servo_bench_arm(&bench); advance(&bench, 35);
    pin_mV[19] = 0; advance(&bench, 110);
    check(bench.mode == BENCH_DISABLED && !bench.feedback_valid && all_off(), "invalid feedback disables PWM");
    ready(&bench);
    bench.measured_q_rad = bench.limits.joint_max_rad + step;
    check(!servo_bench_arm(&bench), "outside-limit current angle cannot arm");
    servo_bench_select(&bench, 0, 0); advance(&bench, 105); servo_bench_configure(&bench, 5, 5);
    check(!servo_bench_arm(&bench) && all_off(), "other joints remain excluded from first test");
    servo_bench_select(&bench, 3, 2); advance(&bench, 105);
    check(!servo_bench_configure(&bench, 5, 5) && !servo_bench_arm(&bench), "unconfigured RR cannot arm");

    ::printf("USB routing: calibration remains read/capture-only; reconnect cannot resume PWM.\n");
    ready(&bench);
    BenchLink link; bench_link_initialize(&link, &bench, NULL);
    output[0] = '\0';
    bench_link_dispatch("QLBENCH1 1 HELLO 1 1 0 0", &link);
    bench_link_dispatch("QLBENCH1 2 SELECT 1 1 0 0", &link);
    advance(&bench, 105);
    bench_link_dispatch("QLBENCH1 3 CONFIG 1 1 5 5", &link);
    bench_link_dispatch("QLBENCH1 4 ARM 1 1 0 0", &link); advance(&bench, 35);
    check(mock_pwm_enabled[4], "protocol ARM reaches only S05");
    //An invalid new command must not drop a valid hold or conceal its reason.
    bench_link_dispatch("QLBENCH1 5 MOVE 1 1 1 2", &link);
    check(mock_pwm_enabled[4] && bench.mode == BENCH_HOLDING, "rejected large move preserves holding PWM");
    check(strstr(bench.message, "Step exceeds") != NULL, "move rejection states the specific setting");
    bench_link_dispatch("QLBENCH1 4 ARM 1 1 0 0", &link);
    check(all_off(), "repeated movement ID disables output");
    bench_link_dispatch("QLBENCH1 5 ARM 1 1 0 0", &link); advance(&bench, 35);
    check(!bench_link_dispatch("QLCAP1 1 HELLO 0 0", &link) && all_off() && bench.leg == -1,
          "calibration session disables bench and falls through to old handler");
    bench_link_dispatch("QLBENCH1 6 ARM 1 1 0 0", &link);
    check(all_off(), "old bench session cannot arm after calibration");
    bench_link_dispatch("QLBENCH1 7 HELLO 1 1 0 0", &link);
    now_ms += 1001; bench_link_update(&link);
    bench_link_dispatch("QLBENCH1 8 ARM 1 1 0 0", &link);
    check(all_off() && !link.active, "expired session requires new handshake");
    bench_link_dispatch("QLBENCH1 9 MOVE 1 1 nan 1", &link);
    check(all_off() && strstr(output, "Malformed bench request"), "malformed request never produces output");
    bench_link_dispatch("QLBENCH1 10 OFF -1 -1 0 0", &link);
    check(all_off(), "OFF works without active session or selection");
    check(mock_pwm_bad_order == 0, "all driver sequences respected preload and LOW routing");
    ::printf("Servo bench host checks: %d passed, %d failed. No physical hardware tested.\n", checks-failures, failures);
    return failures ? 1 : 0;
}
