/* PWM wiring from the shield schematic. Confirm each physical joint on the bench.
 * Feedback pins are separate in board_pins.h (including the measured cable swap).
 */
#ifndef SERVO_PWM_PINS_H
#define SERVO_PWM_PINS_H
#include "Configuration/servo_configuration.h"

//GPIO used by ServoPwm for each joint output; these are not feedback inputs.
//Rows: FL, FR, RL, RR. Columns: hip roll, hip pitch, knee pitch.
static const int servo_pwm_gpio[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    { 7, 15, 16}, //FL: S01, S02, S03
    {42, 41, 40}, //FR: S04, S05, S06
    {18,  3, 46}, //RL: S07, S08, S09
    {45, 48, 47}  //RR: S10, S11, S12
};
#endif
