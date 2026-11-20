/* Adjustable three-joint hold timing. Clearances are entered per joint in Python.
 * USB timeouts and the joint allow-list stay in servo_bench_config.h.
 */
#ifndef LEG_CONTROL_CONFIG_H
#define LEG_CONTROL_CONFIG_H
#include "Firmware/LegControl/leg_control.h"

static const LegControlConfig leg_hold_config = {
    //Read all three ADC inputs every 5 ms; no waiting inside the reader.
    5,
    //Average 20 rounds (~100 ms). Increase for more averaging, but more delay.
    20,
    //Invalid or older feedback disables the active leg. Must exceed a sample window.
    250,
    //Keep prepared outputs LOW for 30 ms before enabling; do not set below 25 ms.
    30
};
#endif
