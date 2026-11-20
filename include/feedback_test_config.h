#ifndef FEEDBACK_TEST_CONFIG_H
#define FEEDBACK_TEST_CONFIG_H

//20 rounds, 5 ms apart; each round reads all three joints of the selected leg.
static const int feedback_sample_count = 20;
static const unsigned long feedback_sample_interval_ms = 5;
static const unsigned long feedback_serial_baud = 115200;

#endif // FEEDBACK_TEST_CONFIG_H
