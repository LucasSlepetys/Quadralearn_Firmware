/*
 * Measured-pose desktop tests: compare V -> a -> q -> FK against known numerical
 * answers. Invalid readings must not become a new pose. No board or PWM is used.
 */
#include "MeasuredLegPose/measured_leg_pose.h"
#include "Configuration/robot_model.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static int checks, failures;
static const double deg_to_rad = 3.141592653589793 / 180.0;

static void check(int condition, const char *message)
{
    checks++;
    if (!condition) { failures++; printf("FAIL: %s\n", message); }
}

static int near(double actual, double expected)
{
    return isfinite(actual) && fabs(actual - expected) < 1e-8;
}

static void make_readings(const double voltage[3], ServoFeedbackReading readings[3])
{
    for (int joint = 0; joint < 3; joint++) {
        ServoFeedbackReading reading = {voltage[joint], voltage[joint], voltage[joint]};
        readings[joint] = reading;
    }
}

static void check_points(LegPointsHip actual, const double expected[4][3], const char *message)
{
    const double *points[4] = {actual.H, actual.P, actual.K, actual.F};
    for (int point = 0; point < 4; point++) {
        for (int axis = 0; axis < 3; axis++) check(near(points[point][axis], expected[point][axis]), message);
    }
}

int main(void)
{
    //The recorded FR HOME voltages must reconstruct the CAD home angles and shape.
    const double fr_home_V[3] = {1.082, 0.2566, 2.5201};
    const double fr_home_a[3] = {68.94117647, 14.59310345, 161.88125};
    const double fr_home_points[4][3] = {
        {0, 0, 0},
        {-0.021750000000, -0.010155444566, -0.017410254038},
        {-0.095126314980, -0.024969581012, -0.003945417044},
        {-0.064702241539, -0.017245860909, -0.061323292686}
    };
    ServoFeedbackReading readings[3];
    MeasuredLegPose pose = {0};
    make_readings(fr_home_V, readings);
    check(measured_leg_pose_from_readings(SERVO_LEG_FR, readings, &pose), "FR HOME converts successfully");
    for (int joint = 0; joint < 3; joint++) check(near(pose.servo_angle_deg[joint], fr_home_a[joint]), "FR HOME uses the correct servo calibration table");
    check(near(pose.angles.alpha_rad, 30.0 * deg_to_rad), "FR HOME alpha = +30 degrees");
    check(near(pose.angles.theta_1_rad, -104.5669876439 * deg_to_rad), "FR HOME theta_1 matches CAD");
    check(near(pose.angles.theta_2_rad, 134.1685943331 * deg_to_rad), "FR HOME theta_2 matches CAD");
    check_points(pose.points_H, fr_home_points, "FR HOME FK matches the independently calculated hip-frame coordinates");

    //An interior pose exercises all three signs away from HOME, not just the references.
    const double interior_V[3] = {1.57, 1.45, 1.57}; //S04=100, S05=90, S06=100 degrees
    const double interior_points[4][3] = {
        {0, 0, 0},
        {-0.021750000000, -0.017681800933, -0.009674911668},
        {-0.058690190873, -0.025001154512, -0.075755872885},
        {-0.016585015272, -0.047828056125, -0.120293356655}
    };
    make_readings(interior_V, readings);
    check(measured_leg_pose_from_readings(SERVO_LEG_FR, readings, &pose), "FR interior pose converts");
    check(near(pose.angles.alpha_rad, -1.05882353 * deg_to_rad), "FR hip roll decreases as its servo angle increases");
    check(near(pose.angles.theta_1_rad, -29.1600910939 * deg_to_rad), "FR hip pitch increases away from HOME");
    check(near(pose.angles.theta_2_rad, 72.2873443331 * deg_to_rad), "FR knee decreases away from HOME");
    check_points(pose.points_H, interior_points, "FR interior FK matches the independently calculated coordinates");

    //The same function must select left-side geometry and each left leg's own tables.
    const double left_home_V[2][3] = {{2.15345, 2.3811, 0.61105}, {2.39265, 2.501, 0.2424}};
    const double left_home_points[4][3] = {
        {0, 0, 0},
        {-0.021750000000, 0.011021469970, -0.017910254038},
        {-0.095126314980, 0.026268619117, -0.004695417044},
        {-0.064702241724, 0.018544898962, -0.062073292777}
    };
    const int left_legs[2] = {SERVO_LEG_FL, SERVO_LEG_RL};
    for (int i = 0; i < 2; i++) {
        make_readings(left_home_V[i], readings);
        check(measured_leg_pose_from_readings(left_legs[i], readings, &pose), "left HOME converts using the same function");
        check(near(pose.angles.alpha_rad, -30.0 * deg_to_rad), "left HOME roll has the opposite sign to FR");
        check_points(pose.points_H, left_home_points, "left HOME uses the measured left-side offsets");
    }

    //Exact simple geometry catches axis/sign errors independently of the measured fixture.
    LegJointAngles straight = {0, 0, 0};
    const double d[3] = {0.01, -0.02, -0.03};
    LegPointsHip points = leg_forward_kinematics(straight, 0.1, 0.2, d, -0.01, -0.02);
    const double straight_expected[4][3] = {{0,0,0},{0.01,-0.02,-0.03},{0.01,-0.03,-0.13},{0.01,-0.05,-0.33}};
    check_points(points, straight_expected, "zero-angle FK points downward with the supplied offsets");
    straight.alpha_rad = 90.0 * deg_to_rad;
    points = leg_forward_kinematics(straight, 0.1, 0.2, d, -0.01, -0.02);
    const double rolled_expected[4][3] = {{0,0,0},{0.01,0.03,-0.02},{0.01,0.13,-0.03},{0.01,0.33,-0.05}};
    check_points(points, rolled_expected, "positive roll uses the right-hand rule about x_H");

    //Missing/bad measurements must not overwrite the caller's previous result.
    pose.angles.alpha_rad = 123.0;
    make_readings(fr_home_V, readings);
    for (int joint = 0; joint < 3; joint++) {
        ServoFeedbackReading saved = readings[joint];
        readings[joint].min_V = 0; //Mean remains valid, but the sampling window does not.
        check(!measured_leg_pose_from_readings(SERVO_LEG_FR, readings, &pose), "an invalid extreme rejects the entire pose");
        check(pose.angles.alpha_rad == 123.0, "failed conversion preserves the previous output");
        readings[joint] = saved;
    }
    check(!measured_leg_pose_from_readings(SERVO_LEG_RR, readings, &pose), "RR does not invent missing installed references");
    check(!measured_leg_pose_from_readings(-1, readings, &pose), "negative leg index is rejected");
    check(!measured_leg_pose_from_readings(4, readings, &pose), "leg index above RR is rejected");
    check(!measured_leg_pose_from_readings(1, NULL, &pose), "missing readings are rejected");
    check(!measured_leg_pose_from_readings(1, readings, NULL), "missing output is rejected");
    readings[0].mean_V = NAN;
    check(!measured_leg_pose_from_readings(1, readings, &pose), "NaN feedback is rejected");
    check(pose.angles.alpha_rad == 123.0, "all failures leave the caller's output unchanged");

    printf("Measured pose checks: %d passed, %d failed. Simulated voltages only; no board accessed.\n", checks - failures, failures);
    return failures ? 1 : 0;
}
