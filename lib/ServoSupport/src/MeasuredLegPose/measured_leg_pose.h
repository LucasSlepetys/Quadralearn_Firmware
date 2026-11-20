/*
 * MeasuredLegPose: converts one leg's feedback windows into physical angles,
 * mathematical angles and FK points in its fixed hip-roll frame H.
 * Pure calculation: no sampling, capture storage, body pose estimate or PWM.
 */
#ifndef MEASURED_LEG_POSE_H
#define MEASURED_LEG_POSE_H

#include "../FeedbackTypes/feedback_types.h"
#include "../Configuration/servo_configuration.h"
#include "../ForwardKinematics/forward_kinematics.h"

typedef struct {
    double servo_angle_deg[SERVO_JOINT_COUNT]; //Physical a: roll, hip pitch, knee
    LegJointAngles angles; //Mathematical q = {alpha, theta_1, theta_2}, radians
    LegPointsHip points_H; //H/P/K/F in metres; F is the ball centre
} MeasuredLegPose;

//Returns 1 on success, 0 for invalid feedback or an unconfigured joint.
//Output stays unchanged on failure. No previous pose is needed or stored.
int measured_leg_pose_from_readings(
    int leg,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT],
    MeasuredLegPose *pose
);

#endif // MEASURED_LEG_POSE_H
