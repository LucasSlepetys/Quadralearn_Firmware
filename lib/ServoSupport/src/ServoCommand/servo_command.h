/*
 * ServoCommand: prepares checked q -> a -> pulse-width results for a joint or leg.
 * Also reports usable limits after supplied clearances from the measured stops.
 * Pure calculations only: no PWM, GPIO access, sampling or saved controller state.
 */
#ifndef SERVO_COMMAND_H
#define SERVO_COMMAND_H

#include "../Configuration/servo_configuration.h"
#include "../KinematicsTypes/kinematics_types.h"

typedef struct {
    double home_deg;  //Positive clearance inward from measured HOME [servo degrees]
    double other_deg; //Positive clearance inward from measured OTHER END [servo degrees]
} ServoStopClearance; //Supplied by the caller; no default physical margins are chosen here.

typedef struct {
    double servo_min_deg;
    double servo_max_deg;
    double joint_min_rad;
    double joint_max_rad;
} ServoCommandLimits; //Sorted limits in the physical and mathematical angle conventions.

typedef struct {
    int leg;
    int joint;
    ServoId servo_id;
    double joint_angle_rad; //Requested mathematical q: alpha, theta_1 or theta_2
    double servo_angle_deg; //Corresponding physical a
    double pulse_us;        //Calculated pulse width; nothing is sent
    double pwm_frequency_hz; //Frequency used by this servo's calibration table
} ServoCommand;

typedef struct {
    ServoCommand joint[SERVO_JOINT_COUNT]; //Hip roll, hip pitch, knee pitch
} LegServoCommand;

//All functions return 1 on success, 0 on failure; outputs stay unchanged on failure.
//A successful calculation does not prove physical clearance, speed or wiring.

int servo_command_limits(
    int leg,
    int joint,
    const ServoStopClearance *clearance,
    ServoCommandLimits *limits
);

int servo_command_prepare(
    int leg,
    int joint,
    double joint_angle_rad,
    const ServoStopClearance *clearance,
    ServoCommand *command
);

//Only returns the new leg command if ALL THREE joint requests pass.
int servo_command_prepare_leg(
    int leg,
    LegJointAngles angles,
    const ServoStopClearance clearance[SERVO_JOINT_COUNT],
    LegServoCommand *command
);

#endif // SERVO_COMMAND_H
