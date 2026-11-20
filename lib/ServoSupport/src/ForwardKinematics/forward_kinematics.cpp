/*
 * Forward kinematics: reconstructs H/P/K/F from model joint angles and CAD geometry.
 * Also transforms those points from the hip frame to a supplied world pose.
 */
#include "forward_kinematics.h"
#include <math.h>

LegPointsHip leg_forward_kinematics(
    LegJointAngles angles,
    double upper_link_length_m,
    double lower_link_length_m,
    const double hip_roll_to_pitch_m[3],
    double upper_lateral_offset_m,
    double lower_lateral_offset_m
) {

    //Same symbols as the paper equations
    double alpha = angles.alpha_rad;
    double theta_1 = angles.theta_1_rad;
    double theta_2 = angles.theta_2_rad;
    double L1 = upper_link_length_m;
    double L2 = lower_link_length_m;
    double a1 = upper_lateral_offset_m;
    double a2 = lower_lateral_offset_m;
    double d[3] = {
        hip_roll_to_pitch_m[0],
        hip_roll_to_pitch_m[1],
        hip_roll_to_pitch_m[2]
    };

    //1. Link vectors in the unrolled axes; theta_2 is relative to the upper link
    double l1[3] = {
        L1 * sin(theta_1),
        a1,
        -L1 * cos(theta_1)
    };

    double l2[3] = {
        L2 * sin(theta_1 + theta_2),
        a2,
        -L2 * cos(theta_1 + theta_2)
    };

    //2. Unrolled positions from hip roll: P_M = d; K_M = d+l1; F_M = d+l1+l2
    double K_M[3];
    double F_M[3];
    for (int coordinate = 0; coordinate < 3; coordinate++) {
        K_M[coordinate] = d[coordinate] + l1[coordinate];
        F_M[coordinate] = d[coordinate] + l1[coordinate] + l2[coordinate];
    }

    //3. Apply hip roll: R_x(alpha), from M to H
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    double R_x[3][3] = {
        {1, 0,         0},
        {0, cos_alpha, -sin_alpha},
        {0, sin_alpha,  cos_alpha}
    };

    //H_H = 0; P_H = R_x*d; K_H = R_x*K_M; F_H = R_x*F_M
    LegPointsHip points_H;
    for (int row = 0; row < 3; row++) {
        points_H.H[row] = 0;
        points_H.P[row] = R_x[row][0] * d[0]
                        + R_x[row][1] * d[1]
                        + R_x[row][2] * d[2];
        points_H.K[row] = R_x[row][0] * K_M[0]
                        + R_x[row][1] * K_M[1]
                        + R_x[row][2] * K_M[2];
        points_H.F[row] = R_x[row][0] * F_M[0]
                        + R_x[row][1] * F_M[1]
                        + R_x[row][2] * F_M[2];
    }

    return points_H;
}

LegPointsWorld leg_points_hip_to_world(
    LegPointsHip points_H,
    const double hip_position_B_m[3],
    const double body_position_W_m[3],
    const double R_WB[3][3]
) {

    //4. X_B = h_B + X_H; the fixed H and B axes are parallel
    double H_B[3];
    double P_B[3];
    double K_B[3];
    double F_B[3];
    for (int coordinate = 0; coordinate < 3; coordinate++) {
        H_B[coordinate] = hip_position_B_m[coordinate] + points_H.H[coordinate];
        P_B[coordinate] = hip_position_B_m[coordinate] + points_H.P[coordinate];
        K_B[coordinate] = hip_position_B_m[coordinate] + points_H.K[coordinate];
        F_B[coordinate] = hip_position_B_m[coordinate] + points_H.F[coordinate];
    }

    //5. X_W = b_W + R_WB*X_B; use rows of R_WB, not its transpose
    LegPointsWorld points_W;
    for (int row = 0; row < 3; row++) {
        points_W.H[row] = body_position_W_m[row]
                        + R_WB[row][0] * H_B[0]
                        + R_WB[row][1] * H_B[1]
                        + R_WB[row][2] * H_B[2];
        points_W.P[row] = body_position_W_m[row]
                        + R_WB[row][0] * P_B[0]
                        + R_WB[row][1] * P_B[1]
                        + R_WB[row][2] * P_B[2];
        points_W.K[row] = body_position_W_m[row]
                        + R_WB[row][0] * K_B[0]
                        + R_WB[row][1] * K_B[1]
                        + R_WB[row][2] * K_B[2];
        points_W.F[row] = body_position_W_m[row]
                        + R_WB[row][0] * F_B[0]
                        + R_WB[row][1] * F_B[1]
                        + R_WB[row][2] * F_B[2];
    }

    return points_W;
}
