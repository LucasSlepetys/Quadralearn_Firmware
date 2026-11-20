/* LegControl: scheduled three-joint feedback and coordinated leg outputs.
 * Reuses V -> a -> q and q -> a -> pulse; no gait, USB parser or PID here.
 */
#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H
#include "MeasuredLegPose/measured_leg_pose.h"
#include "ServoCommand/servo_command.h"

typedef struct {
    unsigned long sample_interval_ms; //Time between three-channel ADC rounds
    int sample_count;                 //Rounds averaged before publishing feedback
    unsigned long feedback_timeout_ms;
    unsigned long preload_ms;         //At least the driver's 25-ms preload
} LegControlConfig;

typedef enum { LEG_DISABLED, LEG_ARMING, LEG_HOLDING } LegControlMode;

typedef struct {
    int leg; //FL=0, FR=1, RL=2, RR=3; -1 means no leg selected
    LegControlConfig config;
    LegControlMode mode;
    int configured[3];
    ServoStopClearance clearance[3];
    ServoCommandLimits limits[3];
    ServoFeedbackReading feedback[3], window[3];
    double sum_V[3];
    int samples, has_feedback, feedback_valid, feedback_problem_joint;
    unsigned long last_sample_ms, feedback_ms, arm_start_ms;
    MeasuredLegPose measured; //Last complete V -> a -> q -> FK result
    LegServoCommand command;  //Last complete prepared/sent command
    const char *message;
    int problem_joint;        //0..2 for a joint-specific reason; -1 for the whole leg
} LegControl;

//Call on a new/disabled state. Initialization never enables an output.
int leg_control_initialize(LegControl *control, int leg, const LegControlConfig *config);
int leg_control_configure_joint(LegControl *control, int joint, double home_deg, double other_deg);
int leg_control_feedback_is_fresh(const LegControl *control);
void leg_control_update(LegControl *control); //One short ADC round when due; never delay().
int leg_control_enable_hold(LegControl *control); //Use measured q; prepare all, then enable in update().
//For a later trajectory caller: it must supply smoothly timed/speed-limited targets.
//Not exposed by the hold-only UI. Invalid targets leave the previous command intact.
int leg_control_set_target(LegControl *control, LegJointAngles angles);
void leg_control_disable(LegControl *control, const char *reason, int problem_joint);
#endif
