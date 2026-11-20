/*
 * JointMapping: converts physical servo angles a [deg] to model angles q [rad],
 * and back, using q_0, a_0 and sigma. Also converts supplied servo limits.
 * Pure maths with 1:1 angular coupling; no feedback reading or PWM output.
 */
#ifndef JOINT_MAPPING_H
#define JOINT_MAPPING_H

//One installed joint: q means alpha, theta_1 or theta_2 in our model.
typedef struct {
    double model_home_rad; //q_0: model angle at the CAD home reference //In matheamtical model frame
    double servo_home_deg; //a_0: servo angle at that same physical position
    int direction_sign;   //sigma = s_q*s_a; must be +1 or -1
} JointMapping;


//Return 1 on success, 0 for invalid inputs; outputs stay unchanged on failure.

//a [deg] -> q [rad]: physical servo angle to our mathematical joint angle.
int joint_angle_from_servo(
    const JointMapping *mapping,
    double servo_angle_deg,
    double *joint_angle_rad
);

//q [rad] -> a [deg]: our mathematical joint angle to the physical servo angle.
int servo_angle_from_joint(
    const JointMapping *mapping,
    double joint_angle_rad,
    double *servo_angle_deg
);

//Convert both usable servo endpoints, then sort the resulting model limits.
//Supply min < max with stop clearance already included; use distinct outputs.
//Calculates limits only. Checking a later command against them is a separate step.
int joint_limits_from_servo_limits(
    const JointMapping *mapping,
    double servo_min_deg,
    double servo_max_deg,
    double *joint_min_rad,
    double *joint_max_rad
);

#endif // JOINT_MAPPING_H
