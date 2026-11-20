#ifndef SERVO_CONFIGURATION_H
#define SERVO_CONFIGURATION_H

#include "../ServoCalibration/servo_calibration.h"

//Physical motor IDs match the spreadsheet: S01 is 1, S12 is 12.
typedef enum {
    SERVO_UNASSIGNED = 0,
    SERVO_S01 = 1, SERVO_S02, SERVO_S03,
    SERVO_S04, SERVO_S05, SERVO_S06,
    SERVO_S07, SERVO_S08, SERVO_S09,
    SERVO_S10, SERVO_S11, SERVO_S12
} ServoId;

enum { SERVO_COUNT = 12 };

//Same leg order as locomotion; the column order is roll, pitch, knee pitch.
typedef enum {
    SERVO_LEG_FL = 0, SERVO_LEG_FR, SERVO_LEG_RL, SERVO_LEG_RR,
    SERVO_LEG_COUNT
} ServoLeg;

typedef enum {
    SERVO_JOINT_HIP_ROLL = 0, SERVO_JOINT_HIP_PITCH, SERVO_JOINT_KNEE_PITCH,
    SERVO_JOINT_COUNT
} ServoJoint;

//CAD reference belongs to the joint, independently of the installed servo ID.
typedef struct {
    double model_home_rad; //q_0: mathematical angle at CAD home [rad]
    int model_away_sign;   //s_q: +1 increases q away from home, -1 decreases it
} JointCadReference;

//Return this physical servo's stored table; NULL for invalid/unassigned IDs.
//The pointer is read-only and remains valid; the table is not copied.
const ServoCalibration *servo_calibration_for_id(int servo_id);

//Leg/joint -> installed motor ID. Invalid indices return SERVO_UNASSIGNED.
ServoId servo_id_for_joint(int leg, int joint);

//Read-only reference for this leg/joint; NULL for invalid indices.
const JointCadReference *joint_cad_reference_for_joint(int leg, int joint);

//a_0, s_a, physical joint limits and GPIO pins are not configured here.
//s_q is not sigma: the completed JointMapping will use sigma = s_q*s_a.
#endif // SERVO_CONFIGURATION_H
