/*
 * Reusable calibration maths: voltage <-> physical servo angle using its table.
 * Also converts a physical servo angle to pulse width; does not generate PWM.
 * Used by calibration now and by feedback/output conversion during locomotion later.
 */

#include "servo_calibration.h"
#include <math.h>
#include <stddef.h>

//Check the table before dividing by any interval width.
static int calibration_is_valid(const ServoCalibration *calibration)
{
    // ! Check: did we receive a table description, does it point to rows, and are there at least two rows?
    if (calibration == NULL || calibration->points == NULL || calibration->point_count < 2) return 0; 

    //! Check: Frequency is finite and positive
    if (!isfinite(calibration->pwm_frequency_hz) || calibration->pwm_frequency_hz <= 0) return 0;

    const ServoCalibrationPoint *points = calibration->points;
    int feedback_increases = points[1].feedback_V > points[0].feedback_V; //Which voltage direction the table uses

    for (int j = 0; j < calibration->point_count; j++) {
        if (!isfinite(points[j].servo_angle_deg) || !isfinite(points[j].feedback_V)
            || !isfinite(points[j].duty_percent)) return 0; //! Check: All row values are finite
        if (points[j].duty_percent <= 0 || points[j].duty_percent >= 100) return 0; //! Check: Duty is greater than 0% and less than 100%

        if (j > 0) {
            if (points[j].servo_angle_deg <= points[j - 1].servo_angle_deg) return 0; //! Check: Servo angles increase from row to row
            if (feedback_increases && points[j].feedback_V <= points[j - 1].feedback_V) return 0; //! Check: Voltages strictly increase or strictly decrease throughout
            if (!feedback_increases && points[j].feedback_V >= points[j - 1].feedback_V) return 0; //! Check: Voltages strictly increase or strictly decrease throughout
        }
    }

    return 1; // Everything checks out
}

//voltage measured -> physical angle
int servo_angle_from_feedback(
    const ServoCalibration *calibration,
    double feedback_V,
    double *servo_angle_deg
) {
    if (servo_angle_deg == NULL || !isfinite(feedback_V) || !calibration_is_valid(calibration)) return 0;

    for (int j = 0; j < calibration->point_count - 1; j++) {
        double V_j = calibration->points[j].feedback_V;
        double V_next = calibration->points[j + 1].feedback_V;

        //checks if input lies between these two rows
        //If it does then use familiar equation
        if (feedback_V >= fmin(V_j, V_next) && feedback_V <= fmax(V_j, V_next)) { 
            double a_j = calibration->points[j].servo_angle_deg;
            double a_next = calibration->points[j + 1].servo_angle_deg;

            //a = a_j + (V - V_j)/(V_next - V_j) * (a_next - a_j)
            double a = a_j + (feedback_V - V_j) / (V_next - V_j) * (a_next - a_j);
            if (!isfinite(a)) return 0;
            *servo_angle_deg = a; //Puts calculated angle back to servo_angle_deg
            return 1;
        }
    }

    return 0; //Voltage is outside the measured table
}

//Same process as servo_angle_from_feedback()
// physical angle --> expected voltage measurent
// Used for desktop tests only
int servo_feedback_from_angle(
    const ServoCalibration *calibration,
    double servo_angle_deg,
    double *feedback_V
) {
    if (feedback_V == NULL || !isfinite(servo_angle_deg) || !calibration_is_valid(calibration)) return 0;

    for (int j = 0; j < calibration->point_count - 1; j++) {
        double a_j = calibration->points[j].servo_angle_deg;
        double a_next = calibration->points[j + 1].servo_angle_deg;

        if (servo_angle_deg >= a_j && servo_angle_deg <= a_next) {
            double V_j = calibration->points[j].feedback_V;
            double V_next = calibration->points[j + 1].feedback_V;

            //V = V_j + (a - a_j)/(a_next - a_j) * (V_next - V_j)
            double V = V_j + (servo_angle_deg - a_j) / (a_next - a_j) * (V_next - V_j);
            if (!isfinite(V)) return 0;
            *feedback_V = V;
            return 1;
        }
    }

    return 0; //Angle is outside the measured table
}

//Same process as servo_angle_from_feedback() 
int servo_pulse_from_angle(
    const ServoCalibration *calibration,
    double servo_angle_deg,
    double *pulse_us
) {
    if (pulse_us == NULL || !isfinite(servo_angle_deg) || !calibration_is_valid(calibration)) return 0;

    for (int j = 0; j < calibration->point_count - 1; j++) {
        double a_j = calibration->points[j].servo_angle_deg;
        double a_next = calibration->points[j + 1].servo_angle_deg;

        if (servo_angle_deg >= a_j && servo_angle_deg <= a_next) {
            double D_j = calibration->points[j].duty_percent;
            double D_next = calibration->points[j + 1].duty_percent;

            //D = D_j + (a - a_j)/(a_next - a_j) * (D_next - D_j)
            double D = D_j + (servo_angle_deg - a_j) / (a_next - a_j) * (D_next - D_j);

            //pulse_us = (D/100) * (1000000/f_PWM); at 50 Hz, pulse_us = 200*D
            double pulse = (D / 100.0) * (1000000.0 / calibration->pwm_frequency_hz);
            if (!isfinite(pulse) || pulse <= 0) return 0;
            *pulse_us = pulse;
            return 1;
        }
    }

    return 0; //Angle is outside the measured table
}
