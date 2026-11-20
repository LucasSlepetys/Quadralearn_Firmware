/*
 * MeasuredLegPose: V -> a -> q -> FK for any configured leg.
 * Reuses calibration validation, installed mappings and the desktop FK equations.
 * Does not save endpoints or change the requested walking pose.
 */
#include "measured_leg_pose.h"
#include "../Configuration/installed_joint_configuration.h"
#include "../Configuration/robot_model.h"
#include "../EndpointCapture/endpoint_capture.h"
#include <math.h>
#include <stddef.h>

int measured_leg_pose_from_readings(
    int leg,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT],
    MeasuredLegPose *pose
) {
    if (pose == NULL) return 0;

    //1. V -> a: same validated voltage windows as the calibration interface.
    //This helper only converts readings; it does not preview or save a capture.
    LegEndpointCapture converted = {0};
    if (!leg_endpoint_capture_from_readings(leg, readings, &converted)) return 0;

    MeasuredLegPose next = {0};
    double q[SERVO_JOINT_COUNT];
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        JointMapping mapping;
        if (!installed_joint_mapping_for_joint(leg, joint, &mapping)) return 0;
        next.servo_angle_deg[joint] = converted.joint[joint].servo_angle_deg;

        //2. q = q_0 + sigma*(a - a_0)*pi/180, using this installed joint's references.
        if (!joint_angle_from_servo(&mapping, next.servo_angle_deg[joint], &q[joint])) return 0;
    }
    next.angles.alpha_rad = q[SERVO_JOINT_HIP_ROLL];
    next.angles.theta_1_rad = q[SERVO_JOINT_HIP_PITCH];
    next.angles.theta_2_rad = q[SERVO_JOINT_KNEE_PITCH];

    //3. FK: P_H = R_x*d; K_H = R_x*(d+l1); F_H = R_x*(d+l1+l2).
    next.points_H = leg_forward_kinematics(
        next.angles,
        ik_config.upper_link_length_m,
        ik_config.lower_link_length_m,
        ik_config.hip_roll_to_pitch_m[leg],
        ik_config.upper_lateral_offset_m[leg],
        ik_config.lower_lateral_offset_m[leg]
    );

    const double *points[] = {next.points_H.H, next.points_H.P, next.points_H.K, next.points_H.F};
    for (int point = 0; point < 4; point++) {
        for (int axis = 0; axis < 3; axis++) {
            if (!isfinite(points[point][axis])) return 0;
        }
    }
    *pose = next; //Publish only a complete result; never substitute zero angles.
    return 1;
}
