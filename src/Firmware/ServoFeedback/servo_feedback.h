#ifndef SERVO_FEEDBACK_H
#define SERVO_FEEDBACK_H

#include "FeedbackTypes/feedback_types.h"
#include "Configuration/servo_configuration.h"

//Configure one ADC input; this module does not generate PWM.
void servo_feedback_initialize(int gpio);

//Read N samples from one GPIO; no joint mapping or endpoint storage.
//Invalid sample count returns NAN fields without reading the ADC.
ServoFeedbackReading servo_feedback_read(
    int gpio,
    int sample_count,
    unsigned long sample_interval_ms
);

//Configure only this leg's three feedback inputs. Return 0 for an invalid leg.
int servo_feedback_initialize_leg(int leg);

//Read roll, pitch and knee in each round, then wait before the next round.
//Return 0 for invalid arguments; an invalid count/leg fills readings with NAN.
int servo_feedback_read_leg(
    int leg,
    int sample_count,
    unsigned long sample_interval_ms,
    ServoFeedbackReading readings[SERVO_JOINT_COUNT]
);

#endif // SERVO_FEEDBACK_H
