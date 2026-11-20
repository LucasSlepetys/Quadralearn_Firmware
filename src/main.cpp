#include <Arduino.h>
#include "feedback_test_config.h"
#include "EndpointCapture/endpoint_capture.h"
#include "Firmware/CaptureLink/capture_link.h"
#include "Firmware/ServoPwm/servo_pwm.h"
#include "Firmware/ServoBench/servo_bench.h"
#include "Firmware/BenchLink/bench_link.h"
#include "leg_control_config.h"
#include "servo_bench_config.h"

//Saved HOME/OTHER END for four legs, plus one pending preview. RAM only.
static EndpointCaptureState captures = {0};
static CaptureLink capture_link_state = {0};
static ServoBench bench = {0};
static LegControl leg_control = {0}; //Measured feedback and commands for the selected whole leg.
static BenchLink bench_link = {0};

void setup(void)
{
    Serial.setTxBufferSize(bench_serial_tx_buffer_bytes); //Queue normal whole-leg telemetry without waiting for each byte.
    Serial.begin(feedback_serial_baud);
    delay(500); //Allow the feedback filters to settle.
    capture_link_initialize(&capture_link_state);
    servo_pwm_initialize(); //No outputs attached or enabled at startup.
    servo_bench_initialize(&bench);
    leg_control_initialize(&leg_control, 1, &leg_hold_config); //FR inputs only; PWM remains off.
    bench_link_initialize(&bench_link, &bench, &leg_control);
    capture_link_state.other_request = bench_link_dispatch;
    capture_link_state.other_context = &bench_link;
}

void loop(void)
{
    bench_link_update(&bench_link);
    //The inactive controller must not sample or run its shutdown policy.
    if (bench_link.whole_leg) leg_control_update(&leg_control);
    else servo_bench_update(&bench);
    capture_link_update(&capture_link_state, &captures); //One serial reader for both UIs.
    delay(1); //Bench samples are scheduled without blocking the command loop.
}
