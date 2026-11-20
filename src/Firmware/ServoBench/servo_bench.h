/* Optional single-joint bench coordinator. Delete/omit this module for walking;
 * keep ServoCommand, ServoPwm, mapping and feedback for the final controller.
 */
#ifndef SERVO_BENCH_H
#define SERVO_BENCH_H
#include "ServoCommand/servo_command.h"
#include "FeedbackTypes/feedback_types.h"

typedef enum { BENCH_DISABLED, BENCH_ARMING, BENCH_HOLDING, BENCH_MOVING } ServoBenchMode;
typedef struct {
    int leg, joint, configured, feedback_valid;
    ServoBenchMode mode;
    ServoStopClearance clearance;
    ServoCommandLimits limits;
    ServoCommand command;
    ServoFeedbackReading feedback, window;
    double measured_a_deg, measured_q_rad;
    double q_start_rad, q_target_rad, move_duration_s, sum_V;
    int samples;
    unsigned long last_sample_ms, feedback_ms, last_request_ms;
    unsigned long move_start_ms, arm_start_ms, last_output_ms;
    const char *message;
} ServoBench;

void servo_bench_initialize(ServoBench *bench);
void servo_bench_close(ServoBench *bench); //Disable and forget the test selection/settings.
int servo_bench_select(ServoBench *bench, int leg, int joint);
int servo_bench_configure(ServoBench *bench, double home_deg, double other_deg);
int servo_bench_arm(ServoBench *bench);
int servo_bench_move(ServoBench *bench, double delta_q_rad, double duration_s);
void servo_bench_update(ServoBench *bench);
void servo_bench_stop(ServoBench *bench);
void servo_bench_disable(ServoBench *bench, const char *reason);
#endif
