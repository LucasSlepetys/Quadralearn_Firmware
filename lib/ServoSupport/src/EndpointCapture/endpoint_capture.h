#ifndef ENDPOINT_CAPTURE_H
#define ENDPOINT_CAPTURE_H

#include "FeedbackTypes/feedback_types.h"
#include "ServoCalibration/servo_calibration.h"
#include "Configuration/servo_configuration.h"

typedef struct {
    int available; //0 until a valid reading is recorded.
    ServoFeedbackReading feedback; //Keep the voltage and spread with the angle.
    double servo_angle_deg; //Physical angle a, not the mathematical joint angle.
} EndpointCapture;

typedef struct {
    int available; //1 only when all three joints have valid captures.
    int leg;       //FL=0, FR=1, RL=2, RR=3
    EndpointCapture joint[SERVO_JOINT_COUNT]; //Roll, pitch, knee pitch
} LegEndpointCapture;

typedef enum { CAPTURE_NONE = 0, CAPTURE_HOME, CAPTURE_OTHER } EndpointKind;

typedef struct {
    LegEndpointCapture home[SERVO_LEG_COUNT];
    LegEndpointCapture other[SERVO_LEG_COUNT];
    LegEndpointCapture preview; //Frozen until SAVE, CANCEL or a new preview.
    EndpointKind pending_endpoint;
} EndpointCaptureState; //Initialize with {0}. RAM only; no hardware access.

//Prepare a capture from one batch. Return 0 without changing it if invalid.
int endpoint_capture_from_reading(
    const ServoCalibration *calibration,
    ServoFeedbackReading feedback,
    EndpointCapture *capture
);

//Build three records in a temporary value; output is unchanged on failure.
int leg_endpoint_capture_from_readings(
    int leg,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT],
    LegEndpointCapture *capture
);

//Replace the pending preview only. Failure cancels an older pending preview.
int endpoint_capture_preview(
    EndpointCaptureState *state,
    int leg,
    EndpointKind endpoint,
    const ServoFeedbackReading readings[SERVO_JOINT_COUNT]
);

//Accept all three preview records into HOME or OTHER END. Return 0 if none.
int endpoint_capture_save(EndpointCaptureState *state);

//Discard the pending preview; saved endpoints stay unchanged.
void endpoint_capture_cancel(EndpointCaptureState *state);

#endif // ENDPOINT_CAPTURE_H
