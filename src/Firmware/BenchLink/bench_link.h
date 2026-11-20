/* Optional bench USB protocol. Shares the existing line reader; never captures
 * endpoints or writes calibration. Python sends requests, ESP32 times movement.
 */
#ifndef BENCH_LINK_H
#define BENCH_LINK_H
#include "../ServoBench/servo_bench.h"
#include "../LegControl/leg_control.h"
typedef struct {
    ServoBench *bench;
    LegControl *leg_control;
    int whole_leg; //Exactly one controller owns PWM/sampling at a time.
    int active;
    unsigned long last_id, last_request_ms;
} BenchLink;
void bench_link_initialize(BenchLink *link, ServoBench *bench, LegControl *leg_control);
int bench_link_dispatch(const char *line, void *context);
void bench_link_update(BenchLink *link);
#endif
