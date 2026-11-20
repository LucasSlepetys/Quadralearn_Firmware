/*
 * Servo configuration: stores each servo's calibration table, its leg/joint
 * assignment, and each joint's CAD home angle and away direction.
 * Read-only getters supply these values; installed HOME/OTHER readings live separately.
 */
#include "servo_configuration.h"
#include <stddef.h>

//Source: Servo_Calibration.xlsx, S01-S12, cells A3:C21; imported 2026-11-19.
//Rows are physical angle [deg], feedback [V], duty [percent].
//50 Hz; this frequency is not stored in the workbook.
//Calibration coverage is not an assembled joint movement limit.

//S01: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s01_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,  0.0495,  1.8},
    { 10,   0.196,  2.3},
    { 20,   0.363,  2.9},
    { 30,  0.5225,  3.5},
    { 40,   0.671,    4},
    { 50,   0.855,  4.6},
    { 60,   0.995,  5.1},
    { 70,   1.155,  5.6},
    { 80,   1.295,  6.2},
    { 90,   1.475,  6.8},
    {100,    1.62,  7.3},
    {110,     1.8,  7.9},
    {120,   1.963,  8.5},
    {130,     2.1,    9},
    {140,    2.24,  9.5},
    {150,   2.415,   10},
    {160,    2.55, 10.6},
    {170,     2.7, 11.1},
    {180,    2.83, 11.6},
};

//S02: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s02_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,    0.19,  2.1},
    { 20,    0.32,  2.8},
    { 30,    0.46,  3.3},
    { 40,    0.61,  3.8},
    { 50,    0.76,  4.3},
    { 60,     0.9,  4.8},
    { 70,   1.055,  5.3},
    { 80,   1.205,  5.8},
    { 90,    1.37,  6.4},
    {100,    1.52,  6.9},
    {110,    1.69,  7.5},
    {120,    1.84,    8},
    {130,       2,  8.6},
    {140,    2.15,  9.1},
    {150,    2.29,  9.7},
    {160,  2.4665, 10.2},
    {170,     2.6, 10.7},
    {180,    2.73, 11.2},
};

//S03: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s03_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.34,  2.7},
    { 30,     0.5,  3.3},
    { 40,    0.65,  3.8},
    { 50,    0.78,  4.3},
    { 60,     0.9,  4.8},
    { 70,    1.05,  5.3},
    { 80,    1.21,  5.9},
    { 90,    1.37,  6.4},
    {100,    1.53,    7},
    {110,    1.69,  7.5},
    {120,    1.85,  8.1},
    {130,    2.01,  8.7},
    {140,    2.17,  9.3},
    {150,    2.33,  9.8},
    {160,    2.47, 10.3},
    {170,    2.61, 10.8},
    {180,    2.77, 11.4},
};

//S04: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s04_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.05,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.34,  2.9},
    { 30,    0.48,  3.4},
    { 40,    0.64,  3.9},
    { 50,     0.8,  4.4},
    { 60,    0.93,    5},
    { 70,     1.1,  5.5},
    { 80,    1.25,    6},
    { 90,    1.41,  6.6},
    {100,    1.57,  7.1},
    {110,    1.71,  7.6},
    {120,    1.86,  8.2},
    {130,    2.01,  8.7},
    {140,    2.17,  9.2},
    {150,     2.3,  9.7},
    {160,    2.47, 10.3},
    {170,    2.61, 10.8},
    {180,    2.77, 11.4},
};

//S05: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s05_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.06,  1.8},
    { 10,    0.19,  2.3},
    { 20,   0.335,  2.8},
    { 30,    0.48,  3.4},
    { 40,    0.65,    4},
    { 50,     0.8,  4.5},
    { 60,    0.96,  5.1},
    { 70,    1.13,  5.6},
    { 80,    1.29,  6.2},
    { 90,    1.45,  6.7},
    {100,    1.59,  7.2},
    {110,    1.74,  7.8},
    {120,    1.89,  8.2},
    {130,    2.03,  8.8},
    {140,    2.17,  9.3},
    {150,    2.33,  9.8},
    {160,    2.49, 10.4},
    {170,    2.65, 10.9},
    {180,    2.81, 11.5},
};

//S06: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s06_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.32,  2.8},
    { 30,    0.48,  3.3},
    { 40,    0.64,  3.9},
    { 50,     0.8,  4.4},
    { 60,    0.92,  4.9},
    { 70,   1.115,  5.5},
    { 80,    1.27,    6},
    { 90,    1.42,  6.6},
    {100,    1.57,  7.1},
    {110,   1.725,  7.6},
    {120,    1.89,  8.1},
    {130,    2.03,  8.6},
    {140,    2.17,  9.2},
    {150,    2.35,  9.8},
    {160,    2.49, 10.3},
    {170,    2.65, 10.8},
    {180,    2.77, 11.3},
};

//S07: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s07_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.4},
    { 20,    0.36,  2.9},
    { 30,    0.52,  3.5},
    { 40,    0.64,    4},
    { 50,     0.8,  4.5},
    { 60,    0.96,  5.1},
    { 70,    1.13,  5.6},
    { 80,    1.29,  6.2},
    { 90,    1.45,  6.7},
    {100,    1.61,  7.3},
    {110,    1.77,  7.9},
    {120,    1.91,  8.4},
    {130,    2.09,  8.9},
    {140,    2.23,  9.5},
    {150,    2.37,   10},
    {160,    2.53, 10.5},
    {170,    2.67, 11.1},
    {180,    2.83, 11.6},
};

//S08: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s08_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.34,  2.9},
    { 30,    0.48,  3.3},
    { 40,    0.61,  3.8},
    { 50,    0.76,  4.4},
    { 60,   0.925,    5},
    { 70,    1.09,  5.4},
    { 80,    1.25,    6},
    { 90,    1.42,  6.6},
    {100,    1.61,  7.3},
    {110,    1.77,  7.8},
    {120,    1.93,  8.4},
    {130,    2.09,    9},
    {140,    2.25,  9.5},
    {150,    2.41, 10.1},
    {160,    2.55, 10.5},
    {170,    2.69, 11.1},
    {180,    2.85, 11.7},
};

//S09: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s09_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.32,  2.8},
    { 30,    0.48,  3.3},
    { 40,     0.6,  3.8},
    { 50,    0.77,  4.4},
    { 60,    0.92,    5},
    { 70,    1.11,  5.5},
    { 80,    1.27,  6.1},
    { 90,    1.41,  6.6},
    {100,    1.59,  7.2},
    {110,    1.75,  7.8},
    {120,    1.89,  8.3},
    {130,    2.09,  8.9},
    {140,    2.22,  9.4},
    {150,    2.37,   10},
    {160,    2.53, 10.5},
    {170,    2.67, 11.1},
    {180,    2.81, 11.6},
};

//S10: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s10_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.32,  2.8},
    { 30,    0.48,  3.3},
    { 40,     0.6,  3.8},
    { 50,    0.76,  4.3},
    { 60,    0.91,  4.8},
    { 70,    1.07,  5.3},
    { 80,    1.21,  5.9},
    { 90,    1.37,  6.4},
    {100,    1.51,  6.9},
    {110,    1.67,  7.4},
    {120,    1.81,  7.9},
    {130,    1.95,  8.4},
    {140,    2.11,    9},
    {150,    2.25,  9.5},
    {160,    2.41, 10.1},
    {170,    2.55, 10.6},
    {180,    2.69, 11.1},
};

//S11: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s11_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.34,  2.9},
    { 30,    0.48,  3.4},
    { 40,    0.64,  3.9},
    { 50,     0.8,  4.4},
    { 60,    0.96,    5},
    { 70,    1.13,  5.5},
    { 80,    1.29,  6.1},
    { 90,    1.44,  6.6},
    {100,    1.57,  7.1},
    {110,    1.73,  7.7},
    {120,    1.85,  8.2},
    {130,    2.01,  8.7},
    {140,    2.17,  9.2},
    {150,    2.31,  9.8},
    {160,    2.47, 10.3},
    {170,    2.61, 10.8},
    {180,    2.77, 11.4},
};

//S12: measurements stay attached to this physical motor.
static const ServoCalibrationPoint s12_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0,    0.04,  1.8},
    { 10,     0.2,  2.3},
    { 20,    0.34,  2.9},
    { 30,    0.48,  3.4},
    { 40,    0.64,  3.9},
    { 50,     0.8,  4.5},
    { 60,    0.94,    5},
    { 70,    1.09,  5.4},
    { 80,    1.24,  5.9},
    { 90,    1.41,  6.5},
    {100,    1.55,    7},
    {110,    1.69,  7.5},
    {120,    1.85,  8.1},
    {130,    2.01,  8.7},
    {140,    2.17,  9.2},
    {150,    2.33,  9.7},
    {160,    2.45, 10.3},
    {170,    2.61, 10.8},
    {180,    2.77, 11.4},
};

//Array index is servo ID minus one. sizeof counts rows without a second hardcoded count.
static const ServoCalibration calibrations[SERVO_COUNT] = {
    {s01_points, (int)(sizeof(s01_points) / sizeof(s01_points[0])), 50.0}, //S01
    {s02_points, (int)(sizeof(s02_points) / sizeof(s02_points[0])), 50.0}, //S02
    {s03_points, (int)(sizeof(s03_points) / sizeof(s03_points[0])), 50.0}, //S03
    {s04_points, (int)(sizeof(s04_points) / sizeof(s04_points[0])), 50.0}, //S04
    {s05_points, (int)(sizeof(s05_points) / sizeof(s05_points[0])), 50.0}, //S05
    {s06_points, (int)(sizeof(s06_points) / sizeof(s06_points[0])), 50.0}, //S06
    {s07_points, (int)(sizeof(s07_points) / sizeof(s07_points[0])), 50.0}, //S07
    {s08_points, (int)(sizeof(s08_points) / sizeof(s08_points[0])), 50.0}, //S08
    {s09_points, (int)(sizeof(s09_points) / sizeof(s09_points[0])), 50.0}, //S09
    {s10_points, (int)(sizeof(s10_points) / sizeof(s10_points[0])), 50.0}, //S10
    {s11_points, (int)(sizeof(s11_points) / sizeof(s11_points[0])), 50.0}, //S11
    {s12_points, (int)(sizeof(s12_points) / sizeof(s12_points[0])), 50.0}, //S12
};

//Installation assignments: 2026-11-19.
//Rows: FL, FR, RL, RR. Columns: hip roll, hip pitch, knee pitch.
static const ServoId installed_servo_ids[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    {SERVO_S01, SERVO_S02, SERVO_S03}, //FL
    {SERVO_S04, SERVO_S05, SERVO_S06}, //FR
    {SERVO_S07, SERVO_S08, SERVO_S09}, //RL
    {SERVO_S10, SERVO_S11, SERVO_S12}  //RR
};

//Fusion inspection 2026-11-19: Chassis v88, left v18, right v47; FK verified.
//Rows: FL, FR, RL, RR. Columns: roll, hip pitch, relative knee pitch.
static const double cad_degrees_to_radians = 3.141592653589793 / 180.0;
static const JointCadReference joint_cad_references[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    { //FL: {q_0 [rad], s_q}; angles below are written in degrees for readability
        {-30.0 * cad_degrees_to_radians, +1},
        {-104.5669876439 * cad_degrees_to_radians, +1},
        {134.1685941347 * cad_degrees_to_radians, -1}
    },
    { //FR
        {30.0 * cad_degrees_to_radians, -1},
        {-104.5669876439 * cad_degrees_to_radians, +1},
        {134.1685943331 * cad_degrees_to_radians, -1}
    },
    { //RL
        {-30.0 * cad_degrees_to_radians, +1},
        {-104.5669876439 * cad_degrees_to_radians, +1},
        {134.1685941347 * cad_degrees_to_radians, -1}
    },
    { //RR
        {30.0 * cad_degrees_to_radians, -1},
        {-104.5669876439 * cad_degrees_to_radians, +1},
        {134.1685943331 * cad_degrees_to_radians, -1}
    }
};

const ServoCalibration *servo_calibration_for_id(int servo_id)
{
    if (servo_id < SERVO_S01 || servo_id > SERVO_S12) return NULL;
    return &calibrations[servo_id - 1]; //S01 is calibrations[0]
}

//Leg index:   0 = front left, 1 = front right, 2 = rear left, 3 = rear right.
//Joint index: 0 = hip roll,   1 = hip pitch,   2 = knee pitch.
//Returns the installed servo ID; invalid indices return SERVO_UNASSIGNED.
ServoId servo_id_for_joint(int leg, int joint)
{
    if (leg < 0 || leg >= SERVO_LEG_COUNT) return SERVO_UNASSIGNED;
    if (joint < 0 || joint >= SERVO_JOINT_COUNT) return SERVO_UNASSIGNED;
    return installed_servo_ids[leg][joint];
}

const JointCadReference *joint_cad_reference_for_joint(int leg, int joint)
{
    if (leg < 0 || leg >= SERVO_LEG_COUNT) return NULL;
    if (joint < 0 || joint >= SERVO_JOINT_COUNT) return NULL;
    return &joint_cad_references[leg][joint];
}
