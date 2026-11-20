/*
 * Desktop capture tests: simulate ADC and serial to check whole-leg sampling,
 * HOME/OTHER preview and accept, and the Python link. No real board is accessed.
 */
#include "Arduino.h"
#include "EndpointCapture/endpoint_capture.h"
#include "Firmware/ServoFeedback/servo_feedback.h"
#include "Firmware/CaptureLink/capture_link.h"
#include "board_pins.h"
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

static int checks, failures, adc_count, delay_count, input_position;
static int gpio_reads[256], configured[64], resolution, bad_io;
static unsigned long delayed_ms, baud;
static uint32_t pin_mV[64];
static int outlier_gpio = -1;
static char input[256], output[20000];
TestSerial Serial;
void setup(void);
void loop(void);

//Expected wiring includes the confirmed FR/RR hip-pitch feedback swap.
static const int expected_gpio[4][3] = {{4,5,6},{1,19,11},{17,8,9},{20,2,12}};
static const int expected_id[4][3] = {{1,2,3},{4,5,6},{7,8,9},{10,11,12}};

static void check(int condition, const char *message)
{
    checks++;
    if (!condition) { failures++; ::printf("FAIL: %s\n%s\n", message, output); }
}
static int contains(const char *text) { return strstr(output, text) != NULL; }
void pinMode(int gpio, int mode)
{
    if (gpio < 0 || gpio >= 64 || mode != INPUT) { bad_io++; return; }
    configured[gpio]++;
}
void analogReadResolution(int bits) { resolution = bits; }
void analogSetPinAttenuation(int gpio, int attenuation)
{
    if (gpio < 0 || gpio >= 64 || attenuation != ADC_11db) bad_io++;
}
void delay(unsigned long ms) { delayed_ms += ms; delay_count++; }
unsigned long millis(void) { return delayed_ms; }
uint32_t analogReadMilliVolts(int gpio)
{
    if (gpio < 0 || gpio >= 64 || adc_count >= 256) { bad_io++; return 0; }
    gpio_reads[adc_count++] = gpio;
    if (gpio == outlier_gpio) { outlier_gpio = -1; return 0; }
    return pin_mV[gpio];
}
void TestSerial::begin(unsigned long value) { baud = value; }
void TestSerial::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(output + strlen(output), sizeof(output) - strlen(output), format, args);
    va_end(args);
}
void TestSerial::println(const char *text) { this->printf("%s\n", text); }
int TestSerial::available(void) { return input[input_position] != '\0'; }
int TestSerial::read(void) { return available() ? input[input_position++] : -1; }

static void clear_io(void)
{
    adc_count = 0; delayed_ms = 0; delay_count = 0;
    output[0] = '\0';
}
static void set_command(const char *command)
{
    clear_io();
    snprintf(input, sizeof(input), "%s", command);
    input_position = 0;
}
static void run(CaptureLink *link, EndpointCaptureState *state, const char *command)
{
    set_command(command);
    capture_link_update(link, state);
}
static void table_readings(int leg, int row, ServoFeedbackReading readings[3])
{
    for (int joint = 0; joint < 3; joint++) {
        const ServoCalibration *table = servo_calibration_for_id(expected_id[leg][joint]);
        double V = table->points[row].feedback_V;
        ServoFeedbackReading reading = {V, V, V};
        readings[joint] = reading;
    }
}

int main(int argc, char **argv)
{
    //Test-only bridge for Python integration checks; uses the real firmware and fake ADC.
    if (argc == 2 && strcmp(argv[1], "--bridge") == 0) {
        for (int gpio = 0; gpio < 64; gpio++) pin_mV[gpio] = 1500;
        setup(); ::printf("%s", output); fflush(stdout);
        char incoming[256];
        while (fgets(incoming, sizeof(incoming), stdin)) {
            int gpio; unsigned int value;
            if (sscanf(incoming, "TEST %d %u", &gpio, &value) == 2 && gpio >= 0 && gpio < 64) {
                pin_mV[gpio] = value; ::printf("TEST_OK\n"); fflush(stdout); continue;
            }
            set_command(incoming); loop(); ::printf("%s", output); fflush(stdout);
        }
        return 0;
    }

    EndpointCaptureState state = {0};
    ServoFeedbackReading readings[3];

    //Every leg must use its own three servo tables and keep independent HOME/OTHER records.
    check(!endpoint_capture_save(&state), "empty state cannot SAVE");
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            check(servo_feedback_gpio[leg][joint] == expected_gpio[leg][joint], "feedback GPIO matches confirmed wiring");
            check(servo_id_for_joint(leg, joint) == expected_id[leg][joint], "servo ID matches joint assignment");
        }
        table_readings(leg, 9, readings);
        check(endpoint_capture_preview(&state, leg, CAPTURE_HOME, readings), "all three HOME readings preview");
        check(!state.home[leg].available, "preview does not save HOME");
        check(endpoint_capture_save(&state), "SAVE accepts whole HOME");
        check(state.pending_endpoint == CAPTURE_NONE, "SAVE consumes preview");
        table_readings(leg, 6, readings);
        check(endpoint_capture_preview(&state, leg, CAPTURE_OTHER, readings), "whole OTHER END previews");
        check(endpoint_capture_save(&state), "whole OTHER END saves");
        for (int joint = 0; joint < 3; joint++) {
            check(fabs(state.home[leg].joint[joint].servo_angle_deg - 90) < 1e-8, "correct HOME table lookup for every joint");
            check(fabs(state.other[leg].joint[joint].servo_angle_deg - 60) < 1e-8, "correct OTHER table lookup for every joint");
        }
    }

    //Failure at joint 0, 1 or 2 must never partly overwrite an existing leg record.
    for (int leg = 0; leg < 4; leg++) {
        for (int bad_joint = 0; bad_joint < 3; bad_joint++) {
            LegEndpointCapture before_home[4], before_other[4];
            memcpy(before_home, state.home, sizeof(before_home));
            memcpy(before_other, state.other, sizeof(before_other));
            table_readings(leg, 12, readings);
            check(endpoint_capture_preview(&state, leg, CAPTURE_HOME, readings), "prepare an older valid pending preview");
            readings[bad_joint].min_V = 0; //Mean still valid, but one sample is outside the table.
            check(!endpoint_capture_preview(&state, leg, CAPTURE_HOME, readings), "one invalid extreme rejects whole leg");
            check(!endpoint_capture_save(&state), "rejection clears old preview so SAVE cannot reuse it");
            check(memcmp(before_home, state.home, sizeof(before_home)) == 0 && memcmp(before_other, state.other, sizeof(before_other)) == 0,
                  "failed preview preserves all 24 endpoint records");
        }
    }
    table_readings(0, 12, readings);
    endpoint_capture_preview(&state, 0, CAPTURE_HOME, readings);
    endpoint_capture_cancel(&state);
    check(!endpoint_capture_save(&state) && state.home[0].joint[0].servo_angle_deg == 90, "CANCEL preserves saved HOME");
    check(!endpoint_capture_preview(&state, -1, CAPTURE_HOME, readings), "invalid leg rejected");
    check(!endpoint_capture_preview(&state, 0, CAPTURE_NONE, readings), "invalid endpoint kind rejected");
    check(!endpoint_capture_preview(&state, 0, CAPTURE_HOME, NULL), "missing readings rejected");
    check(!endpoint_capture_preview(NULL, 0, CAPTURE_HOME, readings), "missing state rejected");

    //Keep the original single-joint numerical checks, including averaging roundoff at a boundary.
    EndpointCapture one = {0};
    ServoFeedbackReading good = {1.475, 1.470, 1.480};
    check(endpoint_capture_from_reading(servo_calibration_for_id(1), good, &one), "single-joint conversion retained");
    ServoFeedbackReading invalid[] = {{NAN,1,2},{1,NAN,2},{1,0,INFINITY},{1,2,3},{3,1,2},{0,0,0},{3,3,3}};
    for (unsigned int i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) {
        check(!endpoint_capture_from_reading(servo_calibration_for_id(1), invalid[i], &one), "invalid individual batch rejected");
        check(one.servo_angle_deg == 90, "invalid batch preserves prior individual result");
    }
    ServoFeedbackReading rounded = {2.830 + 1e-15, 2.830, 2.830};
    check(endpoint_capture_from_reading(servo_calibration_for_id(1), rounded, &one) && one.servo_angle_deg == 180,
          "tiny averaging roundoff does not reject the table boundary");

    //ADC reads are interleaved: roll/pitch/knee repeated 20 times, with only 19 waits.
    for (int gpio = 0; gpio < 64; gpio++) pin_mV[gpio] = 1000 + 10 * gpio;
    for (int leg = 0; leg < 4; leg++) {
        clear_io();
        check(servo_feedback_initialize_leg(leg), "leg ADC initialization succeeds");
        check(servo_feedback_read_leg(leg, 20, 5, readings), "leg reader returns three windows");
        check(adc_count == 60 && delay_count == 19 && delayed_ms == 95, "shared window has 60 samples and 95 ms scheduled waits");
        int order_ok = 1;
        for (int i = 0; i < 60; i++) if (gpio_reads[i] != expected_gpio[leg][i % 3]) order_ok = 0;
        check(order_ok, "all sampling rounds use the correct three pins in order");
        for (int joint = 0; joint < 3; joint++) {
            double V = pin_mV[expected_gpio[leg][joint]] / 1000.0;
            check(fabs(readings[joint].mean_V - V) < 1e-10 && readings[joint].min_V == V && readings[joint].max_V == V,
                  "each joint keeps its own mean and extrema");
        }
    }
    clear_io();
    check(!servo_feedback_read_leg(4, 20, 5, readings) && adc_count == 0 && isnan(readings[0].mean_V), "invalid leg reads no pins");
    check(!servo_feedback_read_leg(0, 0, 5, readings) && adc_count == 0 && isnan(readings[2].mean_V), "zero samples produce no fabricated readings");
    check(!servo_feedback_read_leg(0, 20, 5, NULL), "missing output rejected");
    check(!servo_feedback_initialize_leg(-1), "invalid initialization rejected");
    clear_io();
    servo_feedback_read_leg(0, 1, 5, readings);
    check(adc_count == 3 && delay_count == 0, "one sample per joint needs no between-round delay");
    clear_io();
    ServoFeedbackReading single = servo_feedback_read(4, 3, 5);
    check(adc_count == 3 && delayed_ms == 10 && fabs(single.mean_V - 1.040) < 1e-10, "old single-pin reader remains usable");

    //The Python transport uses the same capture functions, with no terminal menu.
    CaptureLink link = {0};
    EndpointCaptureState session = {0};
    clear_io(); capture_link_initialize(&link);
    check(contains("\"type\":\"boot\"") && !contains("Commands"), "startup is JSON without a terminal menu");
    run(&link, &session, "QLCAP1 1 HELLO 0 0\n");
    check(contains("\"type\":\"hello\"") && contains("\"id\":1"), "handshake identifies protocol");
    run(&link, &session, "QLCAP1 2 READ 0 0\r\n");
    check(adc_count == 60 && contains("\"servo\":1,\"gpio\":4") && contains("\"servo\":3,\"gpio\":6"), "READ returns three FL identities");
    check(!session.home[0].available, "READ does not capture");
    run(&link, &session, "QLCAP1 3 HOME 0 0\n");
    double frozen = session.preview.joint[0].feedback.mean_V;
    check(contains("\"type\":\"preview\"") && contains("\"token\":1") && !session.home[0].available, "HOME previews with token");
    pin_mV[4] = 2152;
    run(&link, &session, "QLCAP1 4 READ 0 0\n");
    check(session.preview.joint[0].feedback.mean_V == frozen, "live readings leave preview frozen");
    run(&link, &session, "QLCAP1 5 ACCEPT 0 99\n");
    check(contains("\"ok\":false") && !session.home[0].available, "wrong token cannot accept");
    run(&link, &session, "QLCAP1 6 ACCEPT 1 1\n");
    check(contains("\"ok\":false") && !session.home[1].available, "wrong leg cannot accept");
    run(&link, &session, "QLCAP1 7 ACCEPT 0 1\n");
    check(adc_count == 0 && session.home[0].joint[0].feedback.mean_V == frozen, "ACCEPT copies preview without sampling");
    run(&link, &session, "QLCAP1 8 ACCEPT 0 1\n");
    check(contains("\"ok\":false"), "consumed preview cannot accept twice");
    run(&link, &session, "QLCAP1 9 OTHER 0 0\n");
    run(&link, &session, "QLCAP1 10 ACCEPT 0 2\n");
    check(session.other[0].available && session.other[0].joint[0].feedback.mean_V != frozen, "OTHER is independent");
    run(&link, &session, "QLCAP1 11 GET 0 0\n");
    check(adc_count == 0 && contains("\"home\":[") && contains("\"other\":["), "GET returns saved arrays without sampling");
    run(&link, &session, "QLCAP1 12 GET 1 0\n");
    check(contains("\"home\":null") && contains("\"other\":null"), "unrecorded endpoints use null");
    for (int leg = 1; leg < 4; leg++) {
        char command[80], identity[40];
        snprintf(command, sizeof(command), "QLCAP1 20 HOME %d 0\n", leg);
        run(&link, &session, command);
        check(session.preview.leg == leg && adc_count == 60, "request routes to selected leg");
        for (int joint = 0; joint < 3; joint++) {
            snprintf(identity, sizeof(identity), "\"servo\":%d,\"gpio\":%d", expected_id[leg][joint], expected_gpio[leg][joint]);
            check(contains(identity), "JSON identity matches selected joint");
        }
        snprintf(command, sizeof(command), "QLCAP1 21 ACCEPT %d %lu\n", leg, link.preview_token);
        run(&link, &session, command);
        check(session.home[leg].available, "selected leg accepted");
    }
    LegEndpointCapture saved = session.home[0];
    run(&link, &session, "QLCAP1 30 HOME 0 0\n");
    unsigned long old_token = link.preview_token;
    outlier_gpio = 5;
    run(&link, &session, "QLCAP1 31 HOME 0 0\n");
    check(contains("\"ok\":false") && contains("\"valid\":false") && contains("\"angle_deg\":null"), "bad joint invalidates preview without JSON NaN");
    check(contains("\"servo\":1") && contains("\"servo\":3") && contains("\"min_V\":0.00000000"), "rejection still reports all voltages");
    char command[80];
    snprintf(command, sizeof(command), "QLCAP1 32 ACCEPT 0 %lu\n", old_token);
    run(&link, &session, command);
    check(contains("\"ok\":false") && memcmp(&saved, &session.home[0], sizeof(saved)) == 0, "invalid capture cannot overwrite HOME or accept stale preview");
    run(&link, &session, "QLCAP1 33 HOME 0 0\n");
    run(&link, &session, "QLCAP1 34 CANCEL 0 0\n");
    check(!session.preview.available && contains("\"type\":\"cancelled\""), "CANCEL invalidates preview");
    run(&link, &session, "QLCAP1 35 HOME 0 0\n");
    run(&link, &session, "QLCAP1 36 HELLO 0 0\n");
    check(!session.preview.available && session.home[0].available, "HELLO clears unseen preview but keeps accepted RAM records");
    run(&link, &session, "QLCAP1 37 HO");
    check(adc_count == 0 && output[0] == '\0', "partial request waits for newline");
    run(&link, &session, "ME 0 0\n");
    check(session.preview.available, "fragmented request completes");
    run(&link, &session, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACCEPT\n");
    check(contains("Request too long") && !session.preview.available, "overlong request clears preview");
    run(&link, &session, "QLCAP1 38 READ 4 0\n");
    check(contains("Invalid request") && adc_count == 0, "invalid leg never reaches ADC");
    run(&link, &session, "QLCAP1 39 READ 0 0 EXTRA\n");
    check(contains("Invalid request") && adc_count == 0, "extra fields rejected");
    run(&link, &session, "HOME\n");
    check(contains("Invalid request") && adc_count == 0, "old terminal commands removed");
    run(&link, &session, "QLCAP1 40 WRONG 0 0\n");
    check(contains("Unknown operation"), "unknown operation returns error");

    //POSE is read-only even while a calibration preview is waiting for acceptance.
    run(&link, &session, "QLCAP1 41 HOME 0 0\n");
    EndpointCaptureState before_pose = session;
    unsigned long before_token = link.preview_token;
    pin_mV[1] = 1570; pin_mV[19] = 1450; pin_mV[11] = 1570;
    run(&link, &session, "QLCAP1 42 POSE 1 0\n");
    check(adc_count == 60, "POSE samples each of the three joints once per round");
    check(contains("\"type\":\"pose\"") && contains("\"ok\":true") && contains("\"frame\":\"H\""), "POSE identifies the fixed hip frame");
    check(contains("\"q_rad\":[") && contains("\"points_H_m\":{\"H\":[0.00000000,0.00000000,0.00000000]"), "POSE includes angles and FK with hip roll at the origin");
    check(strlen(output) < 4096, "POSE fits the existing Python response size limit");
    check(memcmp(&before_pose, &session, sizeof(session)) == 0 && link.preview_token == before_token,
          "POSE preserves every saved endpoint, pending preview and preview token");
    pin_mV[19] = 0;
    run(&link, &session, "QLCAP1 43 POSE 1 0\n");
    check(contains("\"ok\":false") && contains("\"q_rad\":null,\"points_H_m\":null") && contains("\"mean_V\":0.00000000"),
          "bad pose keeps raw feedback visible without invented angles or points");
    run(&link, &session, "QLCAP1 44 POSE 3 0\n");
    check(contains("\"ok\":false") && contains("Installed endpoints not configured"), "RR reports missing calibration");
    check(memcmp(&before_pose, &session, sizeof(session)) == 0 && link.preview_token == before_token,
          "failed POSE requests also preserve calibration state");

    run(&link, &session, "\n\r\n");
    check(output[0] == '\0' && adc_count == 0, "no unsolicited live output");
    clear_io(); setup();
    check(baud == 115200 && contains("\"type\":\"boot\""), "actual main starts USB data interface");
    set_command(""); loop();
    check(adc_count == 0 && output[0] == '\0', "actual main waits for Python");
    check(bad_io == 0, "only input GPIO access occurred");
    ::printf("Capture-link host checks: %d passed, %d failed. Simulated ADC/serial only; no board accessed.\n", checks-failures, failures);
    return failures ? 1 : 0;
}
