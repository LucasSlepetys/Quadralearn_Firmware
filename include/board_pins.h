#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "Configuration/servo_configuration.h"

//Shield schematic: HIP = hip roll, KNEE = hip pitch, ANKLE = knee pitch.
//Rows: FL, FR, RL, RR. Columns: hip roll, hip pitch, knee pitch.
static const int servo_feedback_gpio[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    { 4,  5,  6}, //FL: S01, S02, S03
    { 1,  19, 11}, //FR: S04, S05, S06
    {17,  8,  9}, //RL: S07, S08, S09
    {20, 2, 12}  //RR: S10, S11, S12
};

//No PWM pins are configured by this reader yet

#endif // BOARD_PINS_H
