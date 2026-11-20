#ifndef FEEDBACK_TYPES_H
#define FEEDBACK_TYPES_H

//Voltage data only; usable on the desktop or ESP32.
typedef struct {
    double mean_V; //Average filtered-output voltage [V]
    double min_V;  //Lowest sample in this window [V]
    double max_V;  //Highest sample in this window [V]
} ServoFeedbackReading;

#endif // FEEDBACK_TYPES_H
