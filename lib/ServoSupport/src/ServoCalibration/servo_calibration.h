#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H
/*
1 row in servo calibration table for each motor
*/
typedef struct {
    double servo_angle_deg; //a_j: measured physical servo angle through feedback
    double feedback_V;      //V_j: measured feedback voltage
    double duty_percent;    //D_j: measured duty cycle, e.g. 6.6 means 6.6% - For this specific angle
} ServoCalibrationPoint;

/*
Contains 
//Array of rows //A.k.a one table
//Number of rows in this table of (a_j, V_j, D_j)
//Use the frequency used during calibration
*/
typedef struct {
    const ServoCalibrationPoint *points; //Array of rows //A.k.a one table
    int point_count; //Number of rows in this table of (a_j, V_j, D_j)
    double pwm_frequency_hz; //Use the frequency used during calibration
} ServoCalibration;

//Rows use increasing servo angles and strictly increasing or decreasing voltages.
//Return 1 on success, 0 for invalid data or values outside the table.
//On failure the output is unchanged. No extrapolation or hardware access.

//V -> a: from feedback voltage value to physical servo angle
int servo_angle_from_feedback(
    const ServoCalibration *calibration,
    double feedback_V,
    double *servo_angle_deg
);

//a -> V: expected table voltage, useful for desktop tests, not a sensor reading. - From measured angle to expected voltage
int servo_feedback_from_angle(
    const ServoCalibration *calibration,
    double servo_angle_deg,
    double *feedback_V
);

// From calculated physical angle to pulse width to produce that servo angle
//a -> D -> pulse: calculate pulse width [us]; does not send a PWM command.
int servo_pulse_from_angle(
    const ServoCalibration *calibration,
    double servo_angle_deg,
    double *pulse_us
);

#endif // SERVO_CALIBRATION_H
