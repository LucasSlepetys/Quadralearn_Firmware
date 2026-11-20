/*
 * Shared kinematics types: joint angles and measured geometry, without any gait
 * or IK dependency. Same fields and units as the desktop mathematical model.
 */
#ifndef KINEMATICS_TYPES_H
#define KINEMATICS_TYPES_H

typedef struct {
    double upper_link_length_m; //L1: projected upper-link length
    double lower_link_length_m; //L2: projected lower-link length
    double hip_roll_to_pitch_m[4][3]; //d in M; rows FL, FR, RL, RR
    double upper_lateral_offset_m[4]; //a1, signed along unrolled +y
    double lower_lateral_offset_m[4]; //a2, signed along unrolled +y
} InverseKinematicsConfig; //Keep the familiar desktop name; geometry also serves FK.

typedef struct {
    double alpha_rad;   //Hip roll, right-hand rotation about +x_H
    double theta_1_rad; //Hip pitch, from downward towards +x
    double theta_2_rad; //Knee relative to upper link; feedback may have either sign
} LegJointAngles;

#endif // KINEMATICS_TYPES_H
