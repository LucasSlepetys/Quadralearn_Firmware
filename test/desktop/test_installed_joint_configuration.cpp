/*
 * Desktop tests: check the saved measurements, mapping directions and V -> a -> q
 * conversions without an ESP32. Each FAIL names the joint and the failed expectation.
 * PASS verifies software/data consistency, not physical alignment or safe motor motion.
 ! & "C:\Users\lucas\Desktop\Locomotion Firmware\test\run_desktop_tests.ps1"
 */
#include "Configuration/installed_joint_configuration.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static int checks, failures;
static const double deg_to_rad = 3.141592653589793 / 180.0;

typedef struct {
    const char *name;
    int leg, joint, servo_id;
    double home_deg, other_deg;
    double home_V, other_V;
    double model_home_deg, model_other_deg;
    int sigma;
} ExpectedJoint;

//Independent expectations from the JSON export and the CAD references.
//The model OTHER values are calculated in advance, not by the function under test.
static const ExpectedJoint expected[] = {
    {"FL hip roll",   0,0,1, 133.81785714,65.75625000, 2.15345,1.08710, -30.0,38.06160714,-1},
    {"FL hip pitch",  0,1,2, 155.16147309,22.86071429, 2.38110,0.36005, -104.5669876439,27.7337711561,-1},
    {"FL knee pitch", 0,2,3, 37.40333333,165.82500000, 0.61105,2.55155, 134.1685941347,5.7469274647,-1},
    {"FR hip roll",   1,0,4, 68.94117647,138.88750000, 1.08200,2.15220, 30.0,-39.94632353,-1},
    {"FR hip pitch",  1,1,5, 14.59310345,159.80000000, 0.25660,2.48680, -104.5669876439,40.6399089061,+1},
    {"FR knee pitch", 1,2,6, 161.88125000,17.57916667, 2.52010,0.29095, 134.1685943331,-10.1334889969,+1},
    {"RL hip roll",   2,0,7, 151.41562500,85.41875000, 2.39265,1.37670, -30.0,35.99687500,-1},
    {"RL hip pitch",  2,1,8, 156.50000000,24.64642857, 2.50100,0.40505, -104.5669876439,27.2865837861,-1},
    {"RL knee pitch", 2,2,9, 13.53333333,164.48928571, 0.24240,2.59285, 134.1685941347,-16.7873582453,-1}
};

static void check(int condition, const char *name, const char *meaning)
{
    checks++;
    if (!condition) {
        failures++;
        printf("FAIL: %s | %s\n", name, meaning);
    }
}

static int near(double actual, double wanted)
{
    return isfinite(actual) && fabs(actual - wanted) < 1e-7;
}

static void test_recorded_joint(const ExpectedJoint *e)
{
    const InstalledJointCalibration *measured = installed_joint_calibration_for_joint(e->leg, e->joint);
    JointMapping mapping = {0};
    int mapped = installed_joint_mapping_for_joint(e->leg, e->joint, &mapping);
    check(measured != NULL, e->name, "captured endpoints are available");
    check(mapped, e->name, "a mapping can be built");
    if (measured == NULL || !mapped) return; //Avoid using a missing result after a FAIL.

    //A failure here means a stored value, motor assignment or mapping direction changed.
    check(servo_id_for_joint(e->leg, e->joint) == e->servo_id, e->name, "uses the expected physical servo");
    check(measured->available == 1, e->name, "record is explicitly configured");
    check(near(measured->servo_home_deg, e->home_deg), e->name, "HOME matches the export");
    check(near(measured->servo_other_deg, e->other_deg), e->name, "OTHER END matches the export");
    check(near(mapping.servo_home_deg, e->home_deg), e->name, "a_0 uses measured HOME");
    check(near(mapping.model_home_rad, e->model_home_deg * deg_to_rad), e->name, "q_0 uses CAD home in radians");
    check(mapping.direction_sign == e->sigma, e->name, "sigma has the expected sign");

    //Feed the actual captured voltages through the correct servo table, then JointMapping.
    //This checks the complete numerical path needed for later feedback visualization.
    const ServoCalibration *table = servo_calibration_for_id(e->servo_id);
    double a = NAN, q = NAN;
    check(servo_angle_from_feedback(table, e->home_V, &a) && near(a, e->home_deg),
          e->name, "HOME voltage converts to the exported servo angle");
    check(joint_angle_from_servo(&mapping, a, &q) && near(q, e->model_home_deg * deg_to_rad),
          e->name, "HOME feedback reconstructs CAD q_0");
    check(servo_angle_from_feedback(table, e->other_V, &a) && near(a, e->other_deg),
          e->name, "OTHER voltage converts to the exported servo angle");
    check(joint_angle_from_servo(&mapping, a, &q) && near(q, e->model_other_deg * deg_to_rad),
          e->name, "OTHER feedback gives the expected mathematical endpoint");

    //Check the reverse direction at both endpoints and an interior pose.
    //A failure indicates a sign, reference or degree/radian conversion problem.
    check(servo_angle_from_joint(&mapping, e->model_home_deg * deg_to_rad, &a) && near(a, e->home_deg),
          e->name, "model home converts back to measured servo HOME");
    check(servo_angle_from_joint(&mapping, e->model_other_deg * deg_to_rad, &a) && near(a, e->other_deg),
          e->name, "model OTHER converts back to measured servo OTHER");
    double a_mid = (e->home_deg + e->other_deg) / 2.0;
    double q_mid = (e->model_home_deg + e->model_other_deg) / 2.0 * deg_to_rad;
    check(joint_angle_from_servo(&mapping, a_mid, &q) && near(q, q_mid),
          e->name, "an interior servo pose has the expected model angle");
    check(servo_angle_from_joint(&mapping, q, &a) && near(a, a_mid),
          e->name, "the interior pose survives the round trip");

    printf("  %-13s S%02d | q_0=%8.3f deg | a_0=%8.3f deg | sigma=%+d\n",
           e->name, e->servo_id, mapping.model_home_rad / deg_to_rad,
           mapping.servo_home_deg, mapping.direction_sign);
}

static void test_missing_joint(int leg, int joint, const char *name)
{
    //An absent record must fail explicitly, without returning a fake zero reference.
    JointMapping untouched = {1.25, 85.0, -1};
    check(installed_joint_calibration_for_joint(leg, joint) == NULL, name, "no measurements are returned");
    check(!installed_joint_mapping_for_joint(leg, joint, &untouched), name, "no mapping is fabricated");
    check(untouched.model_home_rad == 1.25 && untouched.servo_home_deg == 85.0 && untouched.direction_sign == -1,
          name, "failed request leaves the caller's output unchanged");
}

static void test_mapping_errors(void)
{
    //Invalid mathematical inputs must not overwrite a previous valid result.
    JointMapping mapping = {0.5, 90.0, -1};
    double value = 123.0;
    check(!joint_angle_from_servo(NULL, 90.0, &value) && value == 123.0, "mapping inputs", "missing mapping is rejected");
    check(!joint_angle_from_servo(&mapping, NAN, &value) && value == 123.0, "mapping inputs", "NaN servo angle is rejected");
    check(!servo_angle_from_joint(&mapping, INFINITY, &value) && value == 123.0, "mapping inputs", "infinite model angle is rejected");
    check(!joint_angle_from_servo(&mapping, 90.0, NULL), "mapping inputs", "missing output is rejected");
    mapping.direction_sign = 0;
    check(!servo_angle_from_joint(&mapping, 0.5, &value) && value == 123.0, "mapping inputs", "sigma must be +1 or -1");

    //Synthetic limits test ordering only; these values are not installed robot limits.
    mapping.direction_sign = -1;
    double q_min = NAN, q_max = NAN;
    check(joint_limits_from_servo_limits(&mapping, 80.0, 100.0, &q_min, &q_max)
          && near(q_min, 0.5 - 10.0 * deg_to_rad) && near(q_max, 0.5 + 10.0 * deg_to_rad),
          "limit conversion", "negative sigma still returns sorted model limits");
    mapping.direction_sign = +1;
    check(joint_limits_from_servo_limits(&mapping, 80.0, 100.0, &q_min, &q_max)
          && near(q_min, 0.5 - 10.0 * deg_to_rad) && near(q_max, 0.5 + 10.0 * deg_to_rad),
          "limit conversion", "positive sigma returns sorted model limits");
    double before_min = q_min, before_max = q_max;
    check(!joint_limits_from_servo_limits(&mapping, 100.0, 80.0, &q_min, &q_max)
          && q_min == before_min && q_max == before_max,
          "limit conversion", "reversed input limits fail without changing outputs");
}

int main(void)
{
    printf("Installed joint tests: saved measurements -> mapping -> mathematical angles.\n");
    for (unsigned int i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        test_recorded_joint(&expected[i]);
    }
    test_missing_joint(SERVO_LEG_RR, SERVO_JOINT_HIP_ROLL, "RR hip roll");
    test_missing_joint(SERVO_LEG_RR, SERVO_JOINT_HIP_PITCH, "RR hip pitch");
    test_missing_joint(SERVO_LEG_RR, SERVO_JOINT_KNEE_PITCH, "RR knee pitch");
    test_missing_joint(-1, 0, "negative leg index");
    test_missing_joint(SERVO_LEG_COUNT, 0, "leg index too large");
    test_missing_joint(0, -1, "negative joint index");
    test_missing_joint(0, SERVO_JOINT_COUNT, "joint index too large");
    check(!installed_joint_mapping_for_joint(0, 0, NULL), "configuration", "missing output is rejected");
    test_mapping_errors();

    printf("Installed joint checks: %d passed, %d failed. No board accessed or PWM generated.\n",
           checks - failures, failures);
    return failures ? 1 : 0;
}
