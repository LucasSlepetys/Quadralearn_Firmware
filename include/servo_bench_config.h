/* Bench-only choices. Walking will use ServoCommand/ServoPwm without this file.
 * Endpoint clearances are entered explicitly in Python; no physical defaults.
 */
#ifndef SERVO_BENCH_CONFIG_H
#define SERVO_BENCH_CONFIG_H
#include "Configuration/servo_configuration.h"

//1 allows enabling PWM for this joint; 0 keeps it read-only in the bench.
//Rows: FL, FR, RL, RR. Columns: hip roll, hip pitch, knee pitch.
static const int bench_joint_allowed[SERVO_LEG_COUNT][SERVO_JOINT_COUNT] = {
    {0, 0, 0},
    {1, 1, 1}, //First test: FR hip pitch, S05, PWM41 / feedback19.
    {0, 0, 0},
    {0, 0, 0}
};

//Largest mathematical joint-angle change allowed in one Move request [deg].
static const double bench_max_step_deg = 10.0;
//Maximum commanded joint speed [deg/s]. Moves are checked using the quintic
//peak speed: 1.875*abs(angle_change)/duration, not just the average speed.
static const double bench_max_speed_deg_s = 5.0;
//Shortest accepted movement duration [s]; rejects overly quick test moves.
static const double bench_min_move_s = 2.0;
//Longest accepted movement duration [s]; this does not limit holding time.
static const double bench_max_move_s = 10.0;
//Maximum gap between regular Python requests before PWM is disabled [ms].
static const unsigned long bench_link_timeout_ms = 1000;
//Maximum age of the last completed valid feedback window [ms].
//Older readings prevent enabling and disable an active bench output.
static const unsigned long bench_feedback_timeout_ms = 250;
//Python requests live status at this interval [ms]; also keeps the bench link alive.
static const unsigned long bench_ui_status_interval_ms = 150;
//Python reply timeout and displayed-status age limit [ms]. Sent by the ESP32;
//keep status interval < reply timeout < link timeout to leave communication time.
static const unsigned long bench_ui_reply_timeout_ms = 750;
//UART transmit storage [bytes], set before Serial.begin(). Keeps normal three-joint
//status replies from blocking ADC sampling while the bytes travel over USB-to-UART.
//This is communication buffering, not a movement limit; do not reduce below 4096.
static const unsigned int bench_serial_tx_buffer_bytes = 8192;
//Minimum time between ADC samples of the selected joint [ms]; no blocking wait.
static const unsigned long bench_sample_interval_ms = 5;
//ADC samples combined into one mean/min/max voltage reading before converting V -> a -> q.
static const int bench_sample_count = 20;
//Minimum time between new angle/pulse targets during a move [ms].
//The hardware continues generating PWM between these updates.
static const unsigned long bench_output_interval_ms = 20;
#endif
