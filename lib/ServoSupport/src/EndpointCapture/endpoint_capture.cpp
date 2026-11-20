/*
 * Calibration workflow: validates readings, prepares a whole-leg preview,
 * and stores accepted HOME/OTHER END records in RAM.
 * These records will configure joint mapping and limits; normal walking does
 * not need to repeat this capture workflow. Export handles the computer backup.
 */

#include "endpoint_capture.h"
#include <math.h>
#include <stddef.h>

int endpoint_capture_from_reading(
    const ServoCalibration *calibration,
    ServoFeedbackReading feedback,
    EndpointCapture *capture
) {
    if (capture == NULL) return 0;
    if (!isfinite(feedback.mean_V) || !isfinite(feedback.min_V)
        || !isfinite(feedback.max_V)) return 0;
    //Averaging identical samples can round a few digits beyond min/max.
    const double rounding_V = 1e-12;
    if (feedback.min_V > feedback.max_V || feedback.mean_V < feedback.min_V - rounding_V
        || feedback.mean_V > feedback.max_V + rounding_V) return 0;
    double mean_V = fmin(fmax(feedback.mean_V, feedback.min_V), feedback.max_V);

    double angle_deg;
    double unused_angle_deg;
    //The mean AND both extremes must be inside the calibration table.
    if (!servo_angle_from_feedback(calibration, mean_V, &angle_deg)
        || !servo_angle_from_feedback(calibration, feedback.min_V, &unused_angle_deg)
        || !servo_angle_from_feedback(calibration, feedback.max_V, &unused_angle_deg)) return 0;

    capture->available = 1;
    capture->feedback = feedback;
    capture->servo_angle_deg = angle_deg; //Same familiar V -> a interpolation.
    return 1;
}

int leg_endpoint_capture_from_readings(
    int leg,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT],
    LegEndpointCapture *capture
) {
    if (leg < 0 || leg >= SERVO_LEG_COUNT || readings == NULL || capture == NULL) return 0;
    LegEndpointCapture next = {0};
    next.leg = leg;
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        ServoId id = servo_id_for_joint(leg, joint);
        if (!endpoint_capture_from_reading(servo_calibration_for_id(id), readings[joint], &next.joint[joint])) return 0;
    }
    next.available = 1;
    *capture = next; //Nothing is copied unless ALL three joints passed.
    return 1;
}

void endpoint_capture_cancel(EndpointCaptureState *state)
{
    if (state == NULL) return;
    state->pending_endpoint = CAPTURE_NONE;
    state->preview.available = 0;
}

int endpoint_capture_preview(
    EndpointCaptureState *state,
    int leg,
    EndpointKind endpoint,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT]
) {
    if (state == NULL) return 0;
    endpoint_capture_cancel(state); //A failed attempt must not leave a stale preview ready to SAVE.
    if (endpoint != CAPTURE_HOME && endpoint != CAPTURE_OTHER) return 0;
    if (!leg_endpoint_capture_from_readings(leg, readings, &state->preview)) return 0;
    state->pending_endpoint = endpoint;
    return 1;
}

int endpoint_capture_save(EndpointCaptureState *state)
{
    if (state == NULL || !state->preview.available) return 0;
    int leg = state->preview.leg; //SAVE belongs to the captured leg, not a later selection.
    if (leg < 0 || leg >= SERVO_LEG_COUNT) return 0;
    if (state->pending_endpoint == CAPTURE_HOME) state->home[leg] = state->preview;
    else if (state->pending_endpoint == CAPTURE_OTHER) state->other[leg] = state->preview;
    else return 0;
    endpoint_capture_cancel(state);
    return 1;
}
