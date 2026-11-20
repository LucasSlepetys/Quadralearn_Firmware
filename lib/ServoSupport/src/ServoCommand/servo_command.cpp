/*
 * ServoCommand: applies endpoint clearances, converts model angles to physical
 * servo angles, then looks up pulse widths. Single-joint and whole-leg functions
 * share the same checks. This module never generates PWM or moves a motor.
 */
#include "servo_command.h"
#include "../Configuration/installed_joint_configuration.h"
#include <math.h>
#include <stddef.h>

int servo_command_limits(
    int leg,
    int joint,
    const ServoStopClearance *clearance,
    ServoCommandLimits *limits
) {
    if (clearance == NULL || limits == NULL) return 0;
    if (!isfinite(clearance->home_deg) || !isfinite(clearance->other_deg)) return 0;
    if (clearance->home_deg <= 0 || clearance->other_deg <= 0) return 0;

    const InstalledJointCalibration *measured = installed_joint_calibration_for_joint(leg, joint);
    JointMapping mapping;
    if (measured == NULL || !installed_joint_mapping_for_joint(leg, joint, &mapping)) return 0;

    double travel_deg = fabs(measured->servo_other_deg - measured->servo_home_deg);
    if (!isfinite(travel_deg) || clearance->home_deg + clearance->other_deg >= travel_deg) return 0;
    int s_a = (measured->servo_other_deg > measured->servo_home_deg) ? +1 : -1;

    //Move inward from both stops, regardless of which endpoint has the larger angle.
    double a_home = measured->servo_home_deg + s_a * clearance->home_deg;
    double a_other = measured->servo_other_deg - s_a * clearance->other_deg;
    ServoCommandLimits next;
    next.servo_min_deg = fmin(a_home, a_other);
    next.servo_max_deg = fmax(a_home, a_other);
    if (next.servo_min_deg <= fmin(measured->servo_home_deg, measured->servo_other_deg)
        || next.servo_max_deg >= fmax(measured->servo_home_deg, measured->servo_other_deg)) return 0;

    //Both usable endpoints must be covered by this servo's measured PWM table.
    const ServoCalibration *table = servo_calibration_for_id(servo_id_for_joint(leg, joint));
    double unused_pulse_us;
    if (!servo_pulse_from_angle(table, next.servo_min_deg, &unused_pulse_us)) return 0;
    if (!servo_pulse_from_angle(table, next.servo_max_deg, &unused_pulse_us)) return 0;

    //Convert both endpoints to q and sort them; sigma may reverse their order.
    if (!joint_limits_from_servo_limits(&mapping, next.servo_min_deg, next.servo_max_deg,
                                       &next.joint_min_rad, &next.joint_max_rad)) return 0;
    *limits = next;
    return 1;
}

int servo_command_prepare(
    int leg,
    int joint,
    double joint_angle_rad,
    const ServoStopClearance *clearance,
    ServoCommand *command
) {
    if (command == NULL || !isfinite(joint_angle_rad)) return 0;
    ServoCommandLimits limits;
    if (!servo_command_limits(leg, joint, clearance, &limits)) return 0;
    if (joint_angle_rad < limits.joint_min_rad || joint_angle_rad > limits.joint_max_rad) return 0;

    JointMapping mapping;
    if (!installed_joint_mapping_for_joint(leg, joint, &mapping)) return 0;
    ServoCommand next = {0};
    next.leg = leg;
    next.joint = joint;
    next.servo_id = servo_id_for_joint(leg, joint);
    next.joint_angle_rad = joint_angle_rad;

    //a = a_0 + sigma*(q - q_0)*180/pi.
    if (!servo_angle_from_joint(&mapping, joint_angle_rad, &next.servo_angle_deg)) return 0;

    //The q check above rejects out-of-range targets. Only remove conversion roundoff here.
    const double rounding_deg = 1e-10;
    if (next.servo_angle_deg < limits.servo_min_deg - rounding_deg
        || next.servo_angle_deg > limits.servo_max_deg + rounding_deg) return 0;
    next.servo_angle_deg = fmin(fmax(next.servo_angle_deg, limits.servo_min_deg), limits.servo_max_deg);

    //Interpolate D(a), then pulse_us = (D/100)*(1000000/f_PWM).
    const ServoCalibration *table = servo_calibration_for_id(next.servo_id);
    if (!servo_pulse_from_angle(table, next.servo_angle_deg, &next.pulse_us)) return 0;
    next.pwm_frequency_hz = table->pwm_frequency_hz;
    *command = next;
    return 1;
}

int servo_command_prepare_leg(
    int leg,
    LegJointAngles angles,
    const ServoStopClearance clearance[SERVO_JOINT_COUNT],
    LegServoCommand *command
) {
    if (clearance == NULL || command == NULL) return 0;
    const double q[SERVO_JOINT_COUNT] = {angles.alpha_rad, angles.theta_1_rad, angles.theta_2_rad};
    LegServoCommand next = {0};
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        if (!servo_command_prepare(leg, joint, q[joint], &clearance[joint], &next.joint[joint])) return 0;
    }
    *command = next; //No partial leg result if any joint failed.
    return 1;
}
