/*
 * Forward kinematics: reconstructs H/P/K/F from model joint angles and CAD geometry.
 * Also transforms those points from the hip frame to a supplied world pose.
 */
#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

#include "../KinematicsTypes/kinematics_types.h"

//Each point is {x, y, z} in metres
typedef struct {
    double H[3]; //Hip roll
    double P[3]; //Hip pitch
    double K[3]; //Knee
    double F[3]; //Ball centre, not ball bottom
} LegPointsHip; //All four points expressed in the fixed hip-roll frame H

typedef struct {
    double H[3];
    double P[3];
    double K[3];
    double F[3];
} LegPointsWorld; //The same four points expressed in W

//Reconstruct one leg from its mathematical joint angles [rad] in hip roll frame
LegPointsHip leg_forward_kinematics(
    LegJointAngles angles,
    double upper_link_length_m,
    double lower_link_length_m,
    const double hip_roll_to_pitch_m[3],
    double upper_lateral_offset_m,
    double lower_lateral_offset_m
);

//X_W = b_W + R_WB * (h_B + X_H); our fixed H axes are parallel to B
LegPointsWorld leg_points_hip_to_world(
    LegPointsHip points_H,
    const double hip_position_B_m[3],
    const double body_position_W_m[3],
    const double R_WB[3][3]
);

#endif // FORWARD_KINEMATICS
