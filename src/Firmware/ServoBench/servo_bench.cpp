/* ServoBench: read one joint, hold its measured starting angle, then make slow
 * bounded test moves. PWM generation and angle conversions stay in their modules.
 * This optional coordinator owns test timing and the USB/feedback timeout.
 */
#include "servo_bench.h"
#include "../ServoPwm/servo_pwm.h"
#include "../ServoFeedback/servo_feedback.h"
#include "Configuration/installed_joint_configuration.h"
#include "EndpointCapture/endpoint_capture.h"
#include "board_pins.h"
#include "servo_bench_config.h"
#include <Arduino.h>
#include <math.h>

static const double rad_per_deg = 0.017453292519943295;

static int reject(ServoBench *bench, const char *reason)
{
    bench->message = reason; //Reject this request without changing the current target.
    return 0;
}

void servo_bench_initialize(ServoBench *bench)
{
    ServoBench empty = {0};
    *bench = empty;
    bench->leg = -1;
    bench->joint = -1;
    bench->message = "Disabled; select a joint";
}

void servo_bench_disable(ServoBench *bench, const char *reason)
{
    int ok = servo_pwm_disable_all();
    bench->mode = BENCH_DISABLED;
    bench->message = ok ? reason : "PWM disable failed; disconnect servo power";
}

void servo_bench_close(ServoBench *bench)
{
    servo_bench_disable(bench, "Disabled");
    servo_bench_initialize(bench);
}

int servo_bench_select(ServoBench *bench, int leg, int joint)
{
    if (bench->mode != BENCH_DISABLED) return reject(bench, "Disable PWM before selecting another joint");
    if (servo_id_for_joint(leg, joint) == SERVO_UNASSIGNED) return reject(bench, "Unknown leg or joint selection");
    servo_bench_close(bench);
    bench->leg = leg;
    bench->joint = joint;
    bench->last_request_ms = millis();
    servo_feedback_initialize(servo_feedback_gpio[leg][joint]);
    bench->message = "Reading feedback; PWM disabled";
    return 1;
}

int servo_bench_configure(ServoBench *bench, double home_deg, double other_deg)
{
    if (bench->mode != BENCH_DISABLED) return reject(bench, "Disable PWM before changing clearances");
    bench->configured = 0;
    ServoStopClearance clearance = {home_deg, other_deg};
    if (!servo_command_limits(bench->leg, bench->joint, &clearance, &bench->limits)) {
        bench->message = "Invalid clearances or missing installed calibration";
        return 0;
    }
    bench->clearance = clearance;
    bench->configured = 1;
    bench->message = "Clearances applied; PWM still disabled";
    return 1;
}

static int fresh_feedback(const ServoBench *bench)
{
    return bench->feedback_valid && (unsigned long)(millis() - bench->feedback_ms) <= bench_feedback_timeout_ms;
}

int servo_bench_arm(ServoBench *bench)
{
    if (bench->mode != BENCH_DISABLED) return reject(bench, "PWM already active; use Move or Disable");
    if (bench->leg < 0) return reject(bench, "Select a joint before enabling");
    if (!bench_joint_allowed[bench->leg][bench->joint]) {
        return reject(bench, "Read-only joint; bench_joint_allowed is 0 in servo_bench_config.h");
    }
    if (!bench->configured) return reject(bench, "Apply valid endpoint clearances before enabling");
    if (!fresh_feedback(bench)) return reject(bench, "Cannot enable with invalid or stale feedback");
    //q starts at measured feedback, then our existing q -> a -> pulse conversion.
    ServoCommand command;
    if (!servo_command_prepare(bench->leg, bench->joint, bench->measured_q_rad, &bench->clearance, &command)) {
        bench->message = "Starting angle is outside the usable limits";
        return 0;
    }
    if (!servo_pwm_prepare(&command)) {
        servo_bench_disable(bench, "PWM preparation failed");
        return 0;
    }
    bench->command = command;
    bench->q_start_rad = command.joint_angle_rad;
    bench->q_target_rad = command.joint_angle_rad;
    bench->arm_start_ms = millis();
    bench->mode = BENCH_ARMING;
    bench->message = "Preparing initial hold; output still LOW";
    return 1;
}

int servo_bench_move(ServoBench *bench, double delta_q_rad, double duration_s)
{
    if (!fresh_feedback(bench)) {
        servo_bench_disable(bench, "Feedback invalid or stale; PWM disabled");
        return 0;
    }
    if (bench->mode != BENCH_HOLDING) return reject(bench, "Move requires HOLDING; enable first or wait for the current move");
    if (!isfinite(delta_q_rad) || !isfinite(duration_s) || fabs(delta_q_rad) < 1e-12)
        return reject(bench, "Move needs finite values and a nonzero step");
    if (fabs(delta_q_rad) > bench_max_step_deg * rad_per_deg)
        return reject(bench, "Step exceeds bench_max_step_deg; reduce Step");
    if (duration_s < bench_min_move_s || duration_s > bench_max_move_s)
        return reject(bench, "Duration is outside bench_min_move_s / bench_max_move_s");
    //The quintic's peak slope is 1.875, so peak speed is 1.875*|delta_q|/T.
    if (1.875 * fabs(delta_q_rad) / duration_s > bench_max_speed_deg_s * rad_per_deg)
        return reject(bench, "Peak speed exceeds bench_max_speed_deg_s; increase Duration or reduce Step");
    double target = bench->command.joint_angle_rad + delta_q_rad;
    ServoCommand checked;
    if (!servo_command_prepare(bench->leg, bench->joint, target, &bench->clearance, &checked))
        return reject(bench, "Target outside usable limits or calibration; reduce Step or change direction");
    bench->q_start_rad = bench->command.joint_angle_rad;
    bench->q_target_rad = target;
    bench->move_duration_s = duration_s;
    bench->move_start_ms = millis();
    bench->mode = BENCH_MOVING;
    bench->message = "Moving";
    return 1;
}

void servo_bench_stop(ServoBench *bench)
{
    if (bench->mode == BENCH_ARMING) servo_bench_disable(bench, "Arming cancelled");
    else if (bench->mode != BENCH_DISABLED) {
        bench->mode = BENCH_HOLDING; //Freeze the last sent target, not a delayed feedback reading.
        bench->q_target_rad = bench->command.joint_angle_rad;
        bench->message = "Stopped; holding last command";
    }
}

static void sample_feedback(ServoBench *bench, unsigned long now)
{
    if ((unsigned long)(now - bench->last_sample_ms) < bench_sample_interval_ms) return;
    bench->last_sample_ms = now;
    //One ADC sample per update. Reuse ServoFeedback without its blocking waits.
    ServoFeedbackReading one = servo_feedback_read(servo_feedback_gpio[bench->leg][bench->joint], 1, 0);
    if (bench->samples == 0) { bench->window = one; bench->sum_V = 0; }
    bench->sum_V += one.mean_V;
    bench->window.min_V = fmin(bench->window.min_V, one.mean_V);
    bench->window.max_V = fmax(bench->window.max_V, one.mean_V);
    if (++bench->samples < bench_sample_count) return;
    bench->window.mean_V = bench->sum_V / bench->samples;
    bench->feedback = bench->window;
    bench->samples = 0;
    EndpointCapture value;
    JointMapping mapping;
    const ServoCalibration *table = servo_calibration_for_id(servo_id_for_joint(bench->leg, bench->joint));
    bench->feedback_valid = endpoint_capture_from_reading(table, bench->feedback, &value)
        && installed_joint_mapping_for_joint(bench->leg, bench->joint, &mapping)
        && joint_angle_from_servo(&mapping, value.servo_angle_deg, &bench->measured_q_rad);
    if (bench->feedback_valid) bench->measured_a_deg = value.servo_angle_deg;
    bench->feedback_ms = now;
}

void servo_bench_update(ServoBench *bench)
{
    if (bench->leg < 0) return;
    unsigned long now = millis();
    //Check old timestamps first; a late sample must not hide a stalled loop.
    if (bench->mode != BENCH_DISABLED) {
        if ((unsigned long)(now - bench->last_request_ms) >= bench_link_timeout_ms) {
            servo_bench_disable(bench, "Link timeout; reconnect and explicitly enable again");
        } else if (!fresh_feedback(bench)) {
            servo_bench_disable(bench, "Feedback stale or invalid");
        }
    }
    sample_feedback(bench, now);
    if (bench->mode == BENCH_DISABLED) return;
    if (!fresh_feedback(bench)) { servo_bench_disable(bench, "Feedback invalid"); return; }
    if (bench->mode == BENCH_ARMING && (unsigned long)(now - bench->arm_start_ms) >= 30) {
        if (!servo_pwm_enable(bench->leg, bench->joint)) {
            servo_bench_disable(bench, "PWM enable failed");
            return;
        }
        bench->mode = BENCH_HOLDING;
        bench->message = "Holding initial measured position";
    }
    if (bench->mode != BENCH_MOVING || (unsigned long)(now - bench->last_output_ms) < bench_output_interval_ms) return;
    bench->last_output_ms = now;
    double u = fmin(1.0, (unsigned long)(now - bench->move_start_ms) / (1000.0 * bench->move_duration_s));
    double F = u*u*u * (10.0 + u * (-15.0 + 6.0*u)); //10u^3 - 15u^4 + 6u^5.
    double q = bench->q_start_rad + (bench->q_target_rad - bench->q_start_rad) * F;
    ServoCommand command;
    if (!servo_command_prepare(bench->leg, bench->joint, q, &bench->clearance, &command)
        || !servo_pwm_write(&command)) {
        servo_bench_disable(bench, "Command or PWM update failed");
        return;
    }
    bench->command = command;
    if (u >= 1.0) { bench->mode = BENCH_HOLDING; bench->message = "Move complete; holding"; }
}
