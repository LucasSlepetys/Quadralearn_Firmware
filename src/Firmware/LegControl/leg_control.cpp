/* LegControl: keeps one leg's feedback windows, measured pose and output state.
 * All three commands are checked before any writes. A driver failure turns the
 * selected leg off; the caller owns communication timeout and motion planning.
 */
#include "leg_control.h"
#include "../ServoFeedback/servo_feedback.h"
#include "../ServoPwm/servo_pwm.h"
#include "EndpointCapture/endpoint_capture.h"
#include "Configuration/installed_joint_configuration.h"
#include <Arduino.h>
#include <math.h>

static int reject(LegControl *c, const char *message, int joint)
{
    c->message = message;
    c->problem_joint = joint;
    return 0;
}

int leg_control_initialize(LegControl *c, int leg, const LegControlConfig *config)
{
    LegControl empty = {0};
    *c = empty;
    c->leg = -1;
    c->problem_joint = c->feedback_problem_joint = -1;
    c->message = "Disabled; select a leg";
    if (!config || leg < 0 || leg >= SERVO_LEG_COUNT) return 0;
    if (!config->sample_interval_ms || config->sample_count <= 0 || config->preload_ms < 25
        || (double)config->sample_interval_ms * config->sample_count >= config->feedback_timeout_ms)
        return reject(c, "Invalid sampling, feedback timeout or preload in leg_control_config.h", -1);
    c->leg = leg;
    c->config = *config;
    servo_feedback_initialize_leg(leg);
    c->message = "Reading whole-leg feedback; PWM disabled";
    return 1;
}

void leg_control_disable(LegControl *c, const char *reason, int joint)
{
    int ok = 1;
    if (c->leg >= 0) {
        for (int j = 0; j < 3; j++) {
            if (!servo_pwm_disable(c->leg, j)) { ok = 0; joint = j; }
        }
    }
    c->mode = LEG_DISABLED;
    c->problem_joint = joint;
    c->message = ok ? reason : "PWM disable failed; disconnect servo power";
}

int leg_control_configure_joint(LegControl *c, int joint, double home_deg, double other_deg)
{
    if (c->mode != LEG_DISABLED) return reject(c, "Disable PWM before changing clearances", joint);
    if (joint < 0 || joint >= 3) return reject(c, "Unknown joint", -1);
    c->configured[joint] = 0;
    ServoStopClearance clearance = {home_deg, other_deg};
    if (!servo_command_limits(c->leg, joint, &clearance, &c->limits[joint]))
        return reject(c, "Invalid clearances or missing installed calibration", joint);
    c->clearance[joint] = clearance;
    c->configured[joint] = 1;
    c->message = "Clearances applied; PWM still disabled";
    c->problem_joint = -1;
    return 1;
}

int leg_control_feedback_is_fresh(const LegControl *c)
{
    return c->has_feedback && c->feedback_valid
        && (unsigned long)(millis() - c->feedback_ms) <= c->config.feedback_timeout_ms;
}

static void sample_feedback(LegControl *c, unsigned long now)
{
    if ((unsigned long)(now - c->last_sample_ms) < c->config.sample_interval_ms) return;
    c->last_sample_ms = now;
    ServoFeedbackReading one[3];
    servo_feedback_read_leg(c->leg, 1, 0, one); //One round: three reads, no delay.
    for (int j = 0; j < 3; j++) {
        if (c->samples == 0) { c->window[j] = one[j]; c->sum_V[j] = 0; }
        c->sum_V[j] += one[j].mean_V;
        c->window[j].min_V = fmin(c->window[j].min_V, one[j].mean_V);
        c->window[j].max_V = fmax(c->window[j].max_V, one[j].mean_V);
    }
    if (++c->samples < c->config.sample_count) return;
    for (int j = 0; j < 3; j++) {
        c->feedback[j] = c->window[j];
        c->feedback[j].mean_V = c->sum_V[j] / c->samples;
    }
    c->samples = 0;
    c->has_feedback = 1;
    c->feedback_ms = now;
    //Our familiar V -> a -> q -> FK conversion; no calibration records are written.
    c->feedback_valid = measured_leg_pose_from_readings(c->leg, c->feedback, &c->measured);
    c->feedback_problem_joint = -1;
    if (!c->feedback_valid) {
        //Only diagnose individual joints when the complete conversion failed.
        for (int j = 0; j < 3; j++) {
            EndpointCapture value;
            JointMapping mapping;
            const ServoCalibration *table = servo_calibration_for_id(servo_id_for_joint(c->leg, j));
            if (!endpoint_capture_from_reading(table, c->feedback[j], &value)
                || !installed_joint_mapping_for_joint(c->leg, j, &mapping)) {
                c->feedback_problem_joint = j;
                break;
            }
        }
    }
}

static int prepare_targets(LegControl *c, LegJointAngles angles, LegServoCommand *next)
{
    for (int j = 0; j < 3; j++) {
        if (!c->configured[j]) return reject(c, "Apply clearances for all three joints", j);
    }
    //q -> a -> pulse, and usable endpoint checks for the complete leg.
    if (servo_command_prepare_leg(c->leg, angles, c->clearance, next)) return 1;
    const double q[3] = {angles.alpha_rad, angles.theta_1_rad, angles.theta_2_rad};
    for (int j = 0; j < 3; j++) {
        ServoCommand unused;
        if (!servo_command_prepare(c->leg, j, q[j], &c->clearance[j], &unused))
            return reject(c, "Angle outside usable limits or calibration", j);
    }
    return reject(c, "Cannot prepare complete leg command", -1);
}

int leg_control_enable_hold(LegControl *c)
{
    if (c->mode != LEG_DISABLED) return reject(c, "Leg already active; disable before enabling again", -1);
    if (!leg_control_feedback_is_fresh(c))
        return reject(c, "Cannot enable with invalid or stale feedback", c->feedback_problem_joint);
    LegServoCommand next;
    if (!prepare_targets(c, c->measured.angles, &next)) return 0;
    for (int j = 0; j < 3; j++) {
        if (!servo_pwm_prepare(&next.joint[j])) {
            leg_control_disable(c, "PWM preparation failed; whole leg disabled", j);
            return 0;
        }
    }
    c->command = next;
    c->arm_start_ms = millis(); //Wait after the LAST prepared channel.
    c->mode = LEG_ARMING;
    c->problem_joint = -1;
    c->message = "Preparing three measured holds; outputs still LOW";
    return 1;
}

void leg_control_update(LegControl *c)
{
    if (c->leg < 0) return;
    unsigned long now = millis();
    if (c->mode != LEG_DISABLED && !leg_control_feedback_is_fresh(c))
        leg_control_disable(c, "Feedback invalid or stale; whole leg disabled", c->feedback_problem_joint);
    sample_feedback(c, now);
    if (c->mode == LEG_DISABLED) return;
    if (!leg_control_feedback_is_fresh(c)) {
        leg_control_disable(c, "Feedback invalid; whole leg disabled", c->feedback_problem_joint);
        return;
    }
    if (c->mode == LEG_ARMING && (unsigned long)(now - c->arm_start_ms) >= c->config.preload_ms) {
        for (int j = 0; j < 3; j++) {
            if (!servo_pwm_enable(c->leg, j)) {
                leg_control_disable(c, "PWM enable failed; whole leg disabled", j);
                return;
            }
        }
        c->mode = LEG_HOLDING;
        c->message = "Holding all three measured starting angles";
    }
}

int leg_control_set_target(LegControl *c, LegJointAngles angles)
{
    if (!leg_control_feedback_is_fresh(c)) {
        leg_control_disable(c, "Feedback invalid or stale; whole leg disabled", c->feedback_problem_joint);
        return 0;
    }
    if (c->mode != LEG_HOLDING) return reject(c, "Leg target requires an enabled hold", -1);
    LegServoCommand next;
    if (!prepare_targets(c, angles, &next)) return 0; //No partial writes for a bad target.
    for (int j = 0; j < 3; j++) {
        if (!servo_pwm_write(&next.joint[j])) {
            leg_control_disable(c, "PWM write failed; whole leg disabled", j);
            return 0;
        }
    }
    c->command = next; //Writes are sequential; publish only after all three succeeded.
    c->message = "Holding complete leg target";
    c->problem_joint = -1;
    return 1;
}
