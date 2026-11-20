/*
 * JointMapping: converts physical servo angles a [deg] to model angles q [rad],
 * and back, using q_0, a_0 and sigma. Also converts supplied servo limits.
 * Pure maths with 1:1 angular coupling; no feedback reading or PWM output.
 */
#include "joint_mapping.h"
#include <math.h>
#include <stddef.h>

static const double degrees_to_radians = 3.141592653589793 / 180.0;
static const double radians_to_degrees = 180.0 / 3.141592653589793;

//Check supplied references, not whether they match the physical installation.
static int mapping_is_valid(const JointMapping *mapping)
{
    if (mapping == NULL) return 0;
    if (!isfinite(mapping->model_home_rad) || !isfinite(mapping->servo_home_deg)) return 0;
    if (mapping->direction_sign != 1 && mapping->direction_sign != -1) return 0;
    return 1;
}

int joint_angle_from_servo(
    const JointMapping *mapping,
    double servo_angle_deg,
    double *joint_angle_rad
) {
    if (joint_angle_rad == NULL || !isfinite(servo_angle_deg) || !mapping_is_valid(mapping)) return 0;

    double q_0 = mapping->model_home_rad;
    double a_0 = mapping->servo_home_deg;
    int sigma = mapping->direction_sign;

    //q = q_0 + sigma*(a - a_0)*pi/180; model angles are radians.
    double q = q_0 + sigma * (servo_angle_deg - a_0) * degrees_to_radians;
    if (!isfinite(q)) return 0;
    *joint_angle_rad = q;
    return 1;
}

int servo_angle_from_joint(
    const JointMapping *mapping,
    double joint_angle_rad,
    double *servo_angle_deg
) {
    if (servo_angle_deg == NULL || !isfinite(joint_angle_rad) || !mapping_is_valid(mapping)) return 0;

    double q_0 = mapping->model_home_rad;
    double a_0 = mapping->servo_home_deg;
    int sigma = mapping->direction_sign;

    //a = a_0 + sigma*(q - q_0)*180/pi; servo angles are degrees.
    double a = a_0 + sigma * (joint_angle_rad - q_0) * radians_to_degrees;
    if (!isfinite(a)) return 0;
    *servo_angle_deg = a;
    return 1;
}

int joint_limits_from_servo_limits(
    const JointMapping *mapping,
    double servo_min_deg,
    double servo_max_deg,
    double *joint_min_rad,
    double *joint_max_rad
) {
    if (joint_min_rad == NULL || joint_max_rad == NULL || joint_min_rad == joint_max_rad) return 0;
    if (!isfinite(servo_min_deg) || !isfinite(servo_max_deg) || servo_min_deg >= servo_max_deg) return 0;

    //q_A = q(a_min), q_B = q(a_max); keep both outputs untouched if either fails.
    double q_A, q_B;
    if (!joint_angle_from_servo(mapping, servo_min_deg, &q_A)) return 0;
    if (!joint_angle_from_servo(mapping, servo_max_deg, &q_B)) return 0;

    //A negative sigma reverses the endpoint order, so sort the model angles.
    *joint_min_rad = fmin(q_A, q_B);
    *joint_max_rad = fmax(q_A, q_B);
    return 1;
}
