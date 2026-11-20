/*
 * Calibration interface: handles Python requests and sends readings/capture results.
 * Calls ServoFeedback and EndpointCapture; POSE also calls MeasuredLegPose.
 * Existing capture commands are unchanged. Optional routing exits the bench first.
 * Used for calibration and inspection, not required by the walking controller.
 */

#include "capture_link.h"
#include "../ServoFeedback/servo_feedback.h"
#include "feedback_test_config.h"
#include "board_pins.h"
#include "MeasuredLegPose/measured_leg_pose.h"
#include "Configuration/installed_joint_configuration.h"
#include "Configuration/robot_model.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

//Requests: QLCAP1 id operation leg token\n. Responses: one JSON object per line.
static void begin_reply(unsigned long id, const char *type, int ok, int leg)
{
    Serial.printf("{\"protocol\":\"leg-capture/1\",\"id\":%lu,\"type\":\"%s\",\"ok\":%s,\"leg\":%d",
                  id, type, ok ? "true" : "false", leg);
}

static void print_number(double value)
{
    if (isfinite(value)) Serial.printf("%.8f", value);
    else Serial.printf("null"); //JSON has no NAN/Infinity values.
}

static void print_joint(int leg, int joint, ServoFeedbackReading reading, int valid, double angle)
{
    Serial.printf("{\"servo\":%d,\"gpio\":%d,\"valid\":%s,\"mean_V\":",
                  (int)servo_id_for_joint(leg, joint), servo_feedback_gpio[leg][joint], valid ? "true" : "false");
    print_number(reading.mean_V);
    Serial.printf(",\"min_V\":"); print_number(reading.min_V);
    Serial.printf(",\"max_V\":"); print_number(reading.max_V);
    Serial.printf(",\"spread_mV\":"); print_number(1000.0 * (reading.max_V - reading.min_V));
    Serial.printf(",\"angle_deg\":"); print_number(valid ? angle : NAN);
    Serial.printf("}");
}

static void print_readings(int leg, const ServoFeedbackReading readings[SERVO_JOINT_COUNT])
{
    Serial.printf("[");
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        if (joint > 0) Serial.printf(",");
        EndpointCapture value = {0};
        int valid = endpoint_capture_from_reading(servo_calibration_for_id(servo_id_for_joint(leg, joint)), readings[joint], &value);
        print_joint(leg, joint, readings[joint], valid, value.servo_angle_deg);
    }
    Serial.printf("]");
}

static void print_saved(int leg, const LegEndpointCapture *capture)
{
    if (!capture->available) { Serial.printf("null"); return; }
    Serial.printf("[");
    for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
        if (joint > 0) Serial.printf(",");
        const EndpointCapture *value = &capture->joint[joint];
        print_joint(leg, joint, value->feedback, value->available, value->servo_angle_deg);
    }
    Serial.printf("]");
}

static void error_reply(unsigned long id, int leg, const char *message)
{
    begin_reply(id, "error", 0, leg);
    Serial.printf(",\"message\":\"%s\"}\n", message);
}

//POSE sends model angles [rad] and H/P/K/F [m], all from the same sampling window.
static void print_vector(const double values[3])
{
    Serial.printf("[");
    for (int axis = 0; axis < 3; axis++) {
        if (axis > 0) Serial.printf(",");
        print_number(values[axis]);
    }
    Serial.printf("]");
}

static void pose_reply(unsigned long id, int leg, const ServoFeedbackReading readings[SERVO_JOINT_COUNT])
{
    MeasuredLegPose pose;
    int valid = measured_leg_pose_from_readings(leg, readings, &pose);
    begin_reply(id, "pose", valid, leg);
    Serial.printf(",\"joints\":"); print_readings(leg, readings);
    Serial.printf(",\"frame\":\"H\",\"angle_unit\":\"rad\",\"length_unit\":\"m\",\"ball_radius_m\":");
    print_number(robot_ball_radius_m);
    if (!valid) {
        int configured = 1;
        for (int joint = 0; joint < SERVO_JOINT_COUNT; joint++) {
            if (installed_joint_calibration_for_joint(leg, joint) == NULL) configured = 0;
        }
        Serial.printf(",\"q_rad\":null,\"points_H_m\":null,\"message\":\"%s\"}\n",
                      configured ? "Invalid feedback; no pose reconstructed" : "Installed endpoints not configured for this leg");
        return;
    }
    double q[3] = {pose.angles.alpha_rad, pose.angles.theta_1_rad, pose.angles.theta_2_rad};
    Serial.printf(",\"q_rad\":"); print_vector(q);
    Serial.printf(",\"points_H_m\":{\"H\":"); print_vector(pose.points_H.H);
    Serial.printf(",\"P\":"); print_vector(pose.points_H.P);
    Serial.printf(",\"K\":"); print_vector(pose.points_H.K);
    Serial.printf(",\"F\":"); print_vector(pose.points_H.F);
    Serial.println("}}");
}

static void process_request(CaptureLink *link, EndpointCaptureState *captures, const char *line)
{
    unsigned long id = 0, token = 0;
    char operation[16], extra;
    int leg = -1;
    int count = sscanf(line, "QLCAP1 %lu %15s %d %lu %c", &id, operation, &leg, &token, &extra);
    if (count != 4 || id == 0 || id > 2147483647UL || leg < 0 || leg >= SERVO_LEG_COUNT) {
        endpoint_capture_cancel(captures);
        error_reply(id <= 2147483647UL ? id : 0, -1, "Invalid request");
        return;
    }
    if (strcmp(operation, "HELLO") == 0) {
        endpoint_capture_cancel(captures); //A new UI session cannot accept an unseen old preview.
        begin_reply(id, "hello", 1, leg);
        Serial.println("}");
    } else if (strcmp(operation, "GET") == 0) {
        begin_reply(id, "records", 1, leg);
        Serial.printf(",\"home\":"); print_saved(leg, &captures->home[leg]);
        Serial.printf(",\"other\":"); print_saved(leg, &captures->other[leg]);
        Serial.println("}");
    } else if (strcmp(operation, "POSE") == 0) {
        if (link->active_leg != leg) {
            servo_feedback_initialize_leg(leg);
            link->active_leg = leg;
        }
        ServoFeedbackReading readings[SERVO_JOINT_COUNT];
        servo_feedback_read_leg(leg, feedback_sample_count, feedback_sample_interval_ms, readings);
        pose_reply(id, leg, readings); //No preview, accept, cancel or saved-record changes.
    } else if (strcmp(operation, "READ") == 0 || strcmp(operation, "HOME") == 0 || strcmp(operation, "OTHER") == 0) {
        if (link->active_leg != leg) {
            servo_feedback_initialize_leg(leg);
            link->active_leg = leg;
        }
        ServoFeedbackReading readings[SERVO_JOINT_COUNT];
        servo_feedback_read_leg(leg, feedback_sample_count, feedback_sample_interval_ms, readings);
        if (strcmp(operation, "READ") == 0) {
            begin_reply(id, "reading", 1, leg);
        } else {
            EndpointKind endpoint = strcmp(operation, "HOME") == 0 ? CAPTURE_HOME : CAPTURE_OTHER;
            int valid = endpoint_capture_preview(captures, leg, endpoint, readings);
            link->preview_token++;
            if (link->preview_token == 0) link->preview_token++;
            begin_reply(id, "preview", valid, leg);
            Serial.printf(",\"endpoint\":\"%s\",\"token\":%lu", endpoint == CAPTURE_HOME ? "home" : "other", link->preview_token);
        }
        Serial.printf(",\"joints\":"); print_readings(leg, readings);
        Serial.println("}");
    } else if (strcmp(operation, "ACCEPT") == 0) {
        if (!captures->preview.available || captures->preview.leg != leg || token == 0 || token != link->preview_token) {
            error_reply(id, leg, "Preview no longer matches; capture again");
            return;
        }
        EndpointKind endpoint = captures->pending_endpoint;
        if (!endpoint_capture_save(captures)) { error_reply(id, leg, "No valid preview"); return; }
        begin_reply(id, "accepted", 1, leg);
        Serial.printf(",\"endpoint\":\"%s\",\"token\":%lu}\n", endpoint == CAPTURE_HOME ? "home" : "other", token);
    } else if (strcmp(operation, "CANCEL") == 0) {
        endpoint_capture_cancel(captures);
        begin_reply(id, "cancelled", 1, leg);
        Serial.println("}");
    } else {
        error_reply(id, leg, "Unknown operation");
    }
}

void capture_link_initialize(CaptureLink *link)
{
    if (link == NULL) return;
    CaptureLink empty = {0};
    *link = empty;
    link->active_leg = -1; //Configure inputs only when a leg is first read.
    begin_reply(0, "boot", 1, -1);
    Serial.println("}");
}

void capture_link_update(CaptureLink *link, EndpointCaptureState *captures)
{
    if (link == NULL || captures == NULL) return;
    for (int bytes = 0; bytes < 256 && Serial.available() > 0; bytes++) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (link->overflow) {
                if (link->other_request) link->other_request(NULL, link->other_context);
                endpoint_capture_cancel(captures);
                error_reply(0, -1, "Request too long");
            } else if (link->length > 0) {
                link->line[link->length] = '\0';
                if (!link->other_request || !link->other_request(link->line, link->other_context)) {
                    process_request(link, captures, link->line);
                }
            }
            link->length = 0;
            link->overflow = 0;
        } else if (!link->overflow) {
            if (c < 32 || c > 126 || link->length >= (int)sizeof(link->line) - 1) link->overflow = 1;
            else link->line[link->length++] = (char)c;
        }
    }
}
