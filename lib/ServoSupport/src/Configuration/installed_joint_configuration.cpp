/*
 * Installed joint configuration: keeps the captured endpoint angles and builds
 * each mapping from the existing CAD q_0/s_q plus measured a_0/s_a.
 * No ADC reads, capture state, PWM commands or walking calculations are changed.
 */
#include "installed_joint_configuration.h"
#include <math.h>
#include <stddef.h>

//Source: leg_endpoints.json exported 2026-11-20 at 18:49:46 UTC.
//Same individual servos after the wire repair; keep their existing calibration tables.
//Rows: FL, FR, RL, RR. Columns: hip roll, hip pitch, knee pitch.
//Entries: {available, HOME [deg], OTHER END [deg]}.
static const InstalledJointCalibration installed_joints[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    { //FL: S01, S02, S03
        {1, 133.81785714, 65.75625000},
        {1, 155.16147309, 22.86071429},
        {1,  37.40333333, 165.82500000}
    },
    { //FR: S04, S05, S06
        {1,  68.94117647, 138.88750000},
        {1,  14.59310345, 159.80000000},
        {1, 161.88125000, 17.57916667}
    },
    { //RL: S07, S08, S09
        {1, 151.41562500, 85.41875000},
        {1, 156.50000000, 24.64642857},
        {1,  13.53333333, 164.48928571}
    },
    { //RR: S10, S11, S12; not captured yet
        {0, NAN, NAN},
        {0, NAN, NAN},
        {0, NAN, NAN}
    }
};

const InstalledJointCalibration *installed_joint_calibration_for_joint(int leg, int joint)
{
    if (leg < 0 || leg >= SERVO_LEG_COUNT) return NULL;
    if (joint < 0 || joint >= SERVO_JOINT_COUNT) return NULL;
    if (!installed_joints[leg][joint].available) return NULL;
    return &installed_joints[leg][joint];
}

int installed_joint_mapping_for_joint(int leg, int joint, JointMapping *mapping)
{
    const InstalledJointCalibration *measured = installed_joint_calibration_for_joint(leg, joint);
    const JointCadReference *cad = joint_cad_reference_for_joint(leg, joint);
    if (mapping == NULL || measured == NULL || cad == NULL) return 0;
    if (!isfinite(measured->servo_home_deg) || !isfinite(measured->servo_other_deg)) return 0;
    if (measured->servo_home_deg == measured->servo_other_deg) return 0;
    if (!isfinite(cad->model_home_rad)) return 0;
    if (cad->model_away_sign != 1 && cad->model_away_sign != -1) return 0;

    //s_a = sign(a_OTHER - a_HOME): does the servo angle increase away from home?
    int s_a = (measured->servo_other_deg > measured->servo_home_deg) ? +1 : -1;

    //q_0 comes from CAD, a_0 from HOME, and sigma = s_q*s_a.
    JointMapping result = {
        cad->model_home_rad,
        measured->servo_home_deg,
        cad->model_away_sign * s_a
    };
    *mapping = result;
    return 1;
}
