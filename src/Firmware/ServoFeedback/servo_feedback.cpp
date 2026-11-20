/*
 * Hardware feedback reader: GPIO -> mean/min/max voltage [V].
 * Reusable for joint feedback later; no angle conversion or endpoint storage.
 * Uses short waits while sampling; scheduling may be adapted for walking.
 */

#include "servo_feedback.h"
#include <Arduino.h>
#include <math.h>
#include "board_pins.h"

void servo_feedback_initialize(int gpio)
{
    pinMode(gpio, INPUT); //No pull-up or pull-down on the measured signal.
    analogReadResolution(12);
    analogSetPinAttenuation(gpio, ADC_11db); //S3 nominal ADC range: 0 to 3.1 V.
}

ServoFeedbackReading servo_feedback_read(
    int gpio,
    int sample_count,
    unsigned long sample_interval_ms
) {
    ServoFeedbackReading reading = {NAN, NAN, NAN};
    if (sample_count <= 0) return reading;

    double sum_V = 0;
    for (int i = 0; i < sample_count; i++) {
        //ADC-calibrated millivolts -> volts at the filter output, as in our tables.
        double V = analogReadMilliVolts(gpio) / 1000.0;
        sum_V += V;
        if (i == 0 || V < reading.min_V) reading.min_V = V;
        if (i == 0 || V > reading.max_V) reading.max_V = V;
        if (i + 1 < sample_count) delay(sample_interval_ms); //! blocking code
    }

    reading.mean_V = sum_V / sample_count; //V_mean = (V_1 + ... + V_N) / N.
    return reading;
}

int servo_feedback_initialize_leg(int leg)
{
    if (leg < 0 || leg >= SERVO_LEG_COUNT) return 0;
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        servo_feedback_initialize(servo_feedback_gpio[leg][joint]);
    }
    return 1;
}

int servo_feedback_read_leg(
    int leg,
    int sample_count,
    unsigned long sample_interval_ms,
    ServoFeedbackReading readings[SERVO_JOINT_COUNT]
) {
    if (readings == NULL) return 0;
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        ServoFeedbackReading empty = {NAN, NAN, NAN};
        readings[joint] = empty;
    }
    if (leg < 0 || leg >= SERVO_LEG_COUNT || sample_count <= 0) return 0;

    double sum_V[SERVO_JOINT_COUNT] = {0};
    for (int i = 0; i < sample_count; i++) {
        for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
            double V = analogReadMilliVolts(servo_feedback_gpio[leg][joint]) / 1000.0;
            sum_V[joint] += V;
            if (i == 0 || V < readings[joint].min_V) readings[joint].min_V = V;
            if (i == 0 || V > readings[joint].max_V) readings[joint].max_V = V;
        }
        if (i + 1 < sample_count) delay(sample_interval_ms); //One wait per three-joint round.
    }

    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        readings[joint].mean_V = sum_V[joint] / sample_count;
    }
    return 1; //EndpointCapture checks voltage validity afterwards.
}
