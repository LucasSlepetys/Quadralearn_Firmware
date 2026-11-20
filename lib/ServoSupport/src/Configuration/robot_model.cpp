/*
 * Robot model: measured CAD lengths and signed offsets for all four legs.
 * Read-only geometry shared by FK and later locomotion; no servo calibration.
 */
#include "robot_model.h"

//CAD geometry from our Fusion measurements
const double robot_ball_radius_m = 0.0195;

//Hip-roll origins relative to the body origin: {x, y, z} in B [m]
const double robot_hip_position_B_m[4][3] = {
    { 0.0745,  0.108904733709, 0.0}, //FL
    { 0.0755, -0.108904733709, 0.0}, //FR
    {-0.0755,  0.108904733709, 0.0}, //RL
    {-0.0745, -0.108904733709, 0.0}  //RR
};

//IK offsets are expressed in the unrolled leg frame M
const InverseKinematicsConfig ik_config = {
    0.07581339370413468, //L1: projected upper-link length [m]
    0.0615914012677,     //L2: projected lower link [m], Fusion inspection 2026-11-19
    { //Hip roll to hip pitch: {d_x, d_y, d_z} [m]
        {-0.02175,  0.0185, -0.0100}, //FL
        {-0.02175, -0.0175, -0.0100}, //FR
        {-0.02175,  0.0185, -0.0100}, //RL
        {-0.02175, -0.0175, -0.0100}  //RR
    },
    {0.006597, -0.006097, 0.006597, -0.006097}, //a1: upper lateral offsets [m]
    {0.0220,   -0.0220,   0.0220,   -0.0220}   //a2: lower lateral offsets [m]
};
