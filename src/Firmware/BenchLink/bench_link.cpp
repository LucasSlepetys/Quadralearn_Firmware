/* BenchLink: decode single-joint / whole-leg hold requests and report measured/commanded values.
 * No calibration writes. A calibration request always exits the PWM bench first.
 */
#include "bench_link.h"
#include "../ServoPwm/servo_pwm.h"
#include "board_pins.h"
#include "servo_pwm_pins.h"
#include "servo_bench_config.h"
#include "leg_control_config.h"
#include "Configuration/installed_joint_configuration.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void number(double value)
{
    if (isfinite(value)) Serial.printf("%.8f", value);
    else Serial.printf("null");
}


//Shared timing/limits come from C++ even in the whole-leg window.
static void settings(BenchLink *link)
{
    unsigned long feedback_timeout = link->whole_leg ? link->leg_control->config.feedback_timeout_ms : bench_feedback_timeout_ms;
    Serial.printf(",\"settings\":{\"max_step_deg\":%.17g,\"max_speed_deg_s\":%.17g,\"min_move_s\":%.17g,\"max_move_s\":%.17g,",
                  bench_max_step_deg, bench_max_speed_deg_s, bench_min_move_s, bench_max_move_s);
    Serial.printf("\"status_interval_ms\":%lu,\"reply_timeout_ms\":%lu,\"link_timeout_ms\":%lu,\"feedback_timeout_ms\":%lu}}\n",
                  bench_ui_status_interval_ms, bench_ui_reply_timeout_ms, bench_link_timeout_ms, feedback_timeout);
}

static void reply_leg(BenchLink *link, unsigned long id, int ok, const char *message)
{
    LegControl *c = link->leg_control;
    const char *modes[] = {"disabled", "arming", "holding"};
    int allowed = 1, configured = 1, enabled = 1;
    for (int j = 0; j < 3; j++) {
        allowed &= bench_joint_allowed[c->leg][j];
        configured &= c->configured[j];
        enabled &= servo_pwm_is_enabled(c->leg, j);
    }
    int fresh = leg_control_feedback_is_fresh(c);
    Serial.printf("{\"protocol\":\"servo-bench/1\",\"scope\":\"leg\",\"id\":%lu,\"ok\":%s,\"message\":\"%s\",",
                  id, ok ? "true" : "false", message);
    Serial.printf("\"session_active\":%s,\"leg\":%d,\"joint\":-1,\"mode\":\"%s\",\"problem_joint\":%d,",
                  link->active ? "true" : "false", c->leg, modes[c->mode],
                  c->problem_joint >= 0 ? c->problem_joint : (fresh ? -1 : c->feedback_problem_joint));
    Serial.printf("\"allowed\":%s,\"configured\":%s,\"enabled\":%s,\"feedback_valid\":%s,\"feedback_age_ms\":%lu,\"joints\":[",
                  allowed ? "true" : "false", configured ? "true" : "false", enabled ? "true" : "false",
                  fresh ? "true" : "false", (unsigned long)(millis() - c->feedback_ms));
    double q[3] = {c->measured.angles.alpha_rad, c->measured.angles.theta_1_rad, c->measured.angles.theta_2_rad};
    for (int j = 0; j < 3; j++) {
        JointMapping mapping;
        int sigma = installed_joint_mapping_for_joint(c->leg, j, &mapping) ? mapping.direction_sign : 0;
        Serial.printf("%s{\"leg\":%d,\"joint\":%d,\"servo\":%d,\"pwm_gpio\":%d,\"feedback_gpio\":%d,",
                      j ? "," : "", c->leg, j, (int)servo_id_for_joint(c->leg, j), servo_pwm_gpio[c->leg][j], servo_feedback_gpio[c->leg][j]);
        Serial.printf("\"allowed\":%s,\"configured\":%s,\"enabled\":%s,\"feedback_valid\":%s,\"V\":",
                      bench_joint_allowed[c->leg][j] ? "true" : "false", c->configured[j] ? "true" : "false",
                      servo_pwm_is_enabled(c->leg, j) ? "true" : "false", fresh ? "true" : "false");
        number(c->has_feedback ? c->feedback[j].mean_V : NAN);
        Serial.printf(",\"spread_mV\":"); number(c->has_feedback ? 1000*(c->feedback[j].max_V-c->feedback[j].min_V) : NAN);
        Serial.printf(",\"a_measured_deg\":"); number(fresh ? c->measured.servo_angle_deg[j] : NAN);
        Serial.printf(",\"q_measured_rad\":"); number(fresh ? q[j] : NAN);
        Serial.printf(",\"a_command_deg\":"); number(c->mode != LEG_DISABLED ? c->command.joint[j].servo_angle_deg : NAN);
        Serial.printf(",\"q_command_rad\":"); number(c->mode != LEG_DISABLED ? c->command.joint[j].joint_angle_rad : NAN);
        Serial.printf(",\"pulse_us\":"); number(c->mode != LEG_DISABLED ? c->command.joint[j].pulse_us : NAN);
        Serial.printf(",\"a_min_deg\":"); number(c->configured[j] ? c->limits[j].servo_min_deg : NAN);
        Serial.printf(",\"a_max_deg\":"); number(c->configured[j] ? c->limits[j].servo_max_deg : NAN);
        Serial.printf(",\"q_min_rad\":"); number(c->configured[j] ? c->limits[j].joint_min_rad : NAN);
        Serial.printf(",\"q_max_rad\":"); number(c->configured[j] ? c->limits[j].joint_max_rad : NAN);
        Serial.printf(",\"frequency_hz\":50,\"sigma\":%d}", sigma);
    }
    Serial.printf("]");
    settings(link);
}

static void disable_active(BenchLink *link, const char *reason)
{
    if (link->whole_leg) leg_control_disable(link->leg_control, reason, -1);
    else servo_bench_disable(link->bench, reason);
}

static const char *active_message(BenchLink *link)
{
    return link->whole_leg ? link->leg_control->message : link->bench->message;
}

static void close_controllers(BenchLink *link)
{
    servo_bench_close(link->bench); //New connection/calibration intentionally stops all outputs.
    if (link->leg_control) leg_control_disable(link->leg_control, "PWM disabled", -1);
    link->whole_leg = 0;
}

static void reply(BenchLink *link, unsigned long id, int ok, const char *message)
{
    if (link->whole_leg) { reply_leg(link, id, ok, message); return; }
    ServoBench *b = link->bench;
    int selected = b->leg >= 0;
    JointMapping mapping;
    int sigma = selected && installed_joint_mapping_for_joint(b->leg, b->joint, &mapping) ? mapping.direction_sign : 0;
    const char *modes[] = {"disabled", "arming", "holding", "moving"};
    Serial.printf("{\"protocol\":\"servo-bench/1\",\"id\":%lu,\"ok\":%s,\"message\":\"%s\",",
                  id, ok ? "true" : "false", message);
    Serial.printf("\"session_active\":%s,", link->active ? "true" : "false");
    Serial.printf("\"leg\":%d,\"joint\":%d,\"servo\":%d,\"pwm_gpio\":%d,\"feedback_gpio\":%d,",
                  b->leg, b->joint, selected ? (int)servo_id_for_joint(b->leg, b->joint) : 0,
                  selected ? servo_pwm_gpio[b->leg][b->joint] : -1,
                  selected ? servo_feedback_gpio[b->leg][b->joint] : -1);
    Serial.printf("\"mode\":\"%s\",\"allowed\":%s,\"configured\":%s,\"enabled\":%s,\"feedback_valid\":%s,",
                  modes[b->mode], selected && bench_joint_allowed[b->leg][b->joint] ? "true" : "false",
                  b->configured ? "true" : "false", selected && servo_pwm_is_enabled(b->leg, b->joint) ? "true" : "false",
                  b->feedback_valid && (unsigned long)(millis() - b->feedback_ms) <= bench_feedback_timeout_ms ? "true" : "false");
    Serial.printf("\"feedback_age_ms\":%lu,\"V\":", (unsigned long)(millis() - b->feedback_ms));
    number(b->feedback_ms ? b->feedback.mean_V : NAN);
    Serial.printf(",\"spread_mV\":"); number(b->feedback_ms ? 1000.0*(b->feedback.max_V - b->feedback.min_V) : NAN);
    Serial.printf(",\"a_measured_deg\":"); number(b->feedback_valid ? b->measured_a_deg : NAN);
    Serial.printf(",\"q_measured_rad\":"); number(b->feedback_valid ? b->measured_q_rad : NAN);
    Serial.printf(",\"q_command_rad\":"); number(b->mode != BENCH_DISABLED ? b->command.joint_angle_rad : NAN);
    Serial.printf(",\"a_command_deg\":"); number(b->mode != BENCH_DISABLED ? b->command.servo_angle_deg : NAN);
    Serial.printf(",\"pulse_us\":"); number(b->mode != BENCH_DISABLED ? b->command.pulse_us : NAN);
    Serial.printf(",\"q_min_rad\":"); number(b->configured ? b->limits.joint_min_rad : NAN);
    Serial.printf(",\"q_max_rad\":"); number(b->configured ? b->limits.joint_max_rad : NAN);
    Serial.printf(",\"a_min_deg\":"); number(b->configured ? b->limits.servo_min_deg : NAN);
    Serial.printf(",\"a_max_deg\":"); number(b->configured ? b->limits.servo_max_deg : NAN);
    Serial.printf(",\"frequency_hz\":50,\"sigma\":%d", sigma);
    settings(link);
}

void bench_link_initialize(BenchLink *link, ServoBench *bench, LegControl *leg_control)
{
    BenchLink empty = {0};
    *link = empty;
    link->bench = bench;
    link->leg_control = leg_control;
}

void bench_link_update(BenchLink *link)
{
    if (link->active && (unsigned long)(millis() - link->last_request_ms) >= bench_link_timeout_ms) {
        disable_active(link, "Link timeout; reconnect before enabling");
        link->active = 0; //A late ARM cannot resume an expired session.
    }
}

int bench_link_dispatch(const char *line, void *context)
{
    BenchLink *link = (BenchLink *)context;
    if (line == NULL || strncmp(line, "QLBENCH1", 8) != 0) {
        if (link->active || link->bench->leg >= 0 || link->whole_leg) close_controllers(link);
        link->active = 0;
        return 0; //The unchanged capture protocol handles this line afterwards.
    }
    //QLBENCH1 id OP leg joint value1 value2; MOVE uses radians and seconds.
    unsigned long id = 0;
    char operation[16], extra;
    int leg = -1, joint = -1;
    double value1 = 0, value2 = 0;
    int count = sscanf(line, "QLBENCH1 %lu %15s %d %d %lf %lf %c", &id, operation, &leg, &joint, &value1, &value2, &extra);
    if (count != 6 || id == 0 || id > 2147483647UL || !isfinite(value1) || !isfinite(value2)) {
        disable_active(link, "Malformed bench request");
        link->active = 0;
        reply(link, id <= 2147483647UL ? id : 0, 0, "Malformed bench request; output disabled");
        return 1;
    }
    if (strcmp(operation, "HELLO") == 0) {
        close_controllers(link);
        link->active = 1;
        link->last_id = id;
        link->last_request_ms = millis();
        reply(link, id, 1, "Connected; outputs disabled");
        return 1;
    }
    //OFF is always accepted, even after timeout or if the UI lost its selection.
    if (strcmp(operation, "OFF") == 0) {
        bench_link_update(link); //OFF must not revive an already expired session.
        disable_active(link, "PWM disabled");
        if (id > link->last_id) {
            link->last_id = id;
            if (link->active) {
                link->last_request_ms = millis();
                link->bench->last_request_ms = millis();
            }
        }
        reply(link, id, 1, active_message(link));
        return 1;
    }
    bench_link_update(link);
    if (!link->active || id <= link->last_id) {
        disable_active(link, "Expired or repeated request; reconnect");
        link->active = 0;
        reply(link, id, 0, active_message(link));
        return 1;
    }
    link->last_id = id;
    //A well-formed new request is a heartbeat even if its requested move is rejected.
    link->last_request_ms = millis();
    link->bench->last_request_ms = millis();
    int ok = 0;
    const char *message = "Unknown bench operation; request ignored";
    if (strcmp(operation, "SELECT") == 0 || strcmp(operation, "LEG_SELECT") == 0) {
        int want_leg = strcmp(operation, "LEG_SELECT") == 0;
        if (link->bench->mode != BENCH_DISABLED || (link->leg_control && link->leg_control->mode != LEG_DISABLED)) {
            message = "Disable PWM before changing joint or control mode";
        } else if (leg < 0 || leg >= SERVO_LEG_COUNT || (!want_leg && (joint < 0 || joint >= 3))) {
            message = "Unknown leg or joint selection";
        } else if (want_leg && !link->leg_control) {
            message = "Whole-leg controller is unavailable";
        } else {
            close_controllers(link);
            if (want_leg) {
                ok = leg_control_initialize(link->leg_control, leg, &leg_hold_config);
                link->whole_leg = ok;
                message = link->leg_control->message;
            } else {
                ok = servo_bench_select(link->bench, leg, joint);
                message = link->bench->message;
            }
        }
    } else if (link->whole_leg && leg == link->leg_control->leg) {
        LegControl *c = link->leg_control;
        if (strcmp(operation, "LEG_STATUS") == 0 && joint == -1) ok = 1;
        else if (strcmp(operation, "LEG_CONFIG") == 0) ok = leg_control_configure_joint(c, joint, value1, value2);
        else if (strcmp(operation, "LEG_ARM") == 0 && joint == -1) {
            int allowed = 1;
            for (int j = 0; j < 3; j++) {
                if (!bench_joint_allowed[leg][j]) {
                    allowed = 0; c->problem_joint = j;
                    c->message = "Read-only joint; bench_joint_allowed is 0 in servo_bench_config.h";
                    break;
                }
            }
            if (allowed) ok = leg_control_enable_hold(c);
        } else if (strcmp(operation, "LEG_STOP") == 0 && joint == -1) {
            if (c->mode == LEG_ARMING) leg_control_disable(c, "Arming cancelled", -1);
            else { c->message = "Holding current leg command"; c->problem_joint = -1; }
            ok = 1;
        } else {
            c->message = "Whole-leg mode supports read, clearances, hold and disable only";
            c->problem_joint = -1;
        }
        message = c->message;
    } else if (!link->whole_leg && leg == link->bench->leg && joint == link->bench->joint && leg >= 0) {
        if (strcmp(operation, "STATUS") == 0) ok = 1;
        else if (strcmp(operation, "CONFIG") == 0) ok = servo_bench_configure(link->bench, value1, value2);
        else if (strcmp(operation, "ARM") == 0) ok = servo_bench_arm(link->bench);
        else if (strcmp(operation, "MOVE") == 0) ok = servo_bench_move(link->bench, value1, value2);
        else if (strcmp(operation, "STOP") == 0) { servo_bench_stop(link->bench); ok = 1; }
        else link->bench->message = "Unknown bench operation; request ignored";
        message = link->bench->message;
    } else message = "Selection mismatch; read the intended joint/leg first";
    //Input rejection keeps the previous target. Feedback/link/PWM faults still disable.
    reply(link, id, ok, message);
    return 1;
}
