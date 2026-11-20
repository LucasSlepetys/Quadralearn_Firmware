/*
 * ServoCommand desktop tests: q -> a -> pulse widths, with example stop clearances.
 * Known numerical answers and deliberate invalid inputs exercise the output path.
 * Nothing is sent to a servo. PASS does not verify PWM wiring or physical motion.
 */
#include "ServoCommand/servo_command.h"
#include "Configuration/installed_joint_configuration.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static int checks, failures;
static const double deg_to_rad = 3.141592653589793 / 180.0;
static const ServoStopClearance example_clearance = {2.0, 3.0}; //TEST ONLY, not chosen robot settings.

typedef struct {
    const char *name;
    int leg, joint, servo_id;
    double q_at_100_deg, pulse_at_100_us;
    double q_at_95_deg, pulse_at_95_us;
} ExpectedCommand;

//q values calculated from the saved references; pulses calculated from measured duty rows.
//At 50 Hz, pulse = 200*D. The 95-degree cases interpolate halfway between 90 and 100.
static const ExpectedCommand expected[] = {
    {"FL hip roll",   0,0,1,   3.817857140000,1460,   8.817857140000,1410},
    {"FL hip pitch",  0,1,2, -49.405514553900,1380, -44.405514553900,1330},
    {"FL knee pitch", 0,2,3,  71.571927464700,1400,  76.571927464700,1340},
    {"FR hip roll",   1,0,4,  -1.058823530000,1420,   3.941176470000,1370},
    {"FR hip pitch",  1,1,5, -19.160091093900,1440, -24.160091093900,1390},
    {"FR knee pitch", 1,2,6,  72.287344333100,1420,  67.287344333100,1370},
    {"RL hip roll",   2,0,7,  21.415625000000,1460,  26.415625000000,1400},
    {"RL hip pitch",  2,1,8, -48.066987643900,1460, -43.066987643900,1390},
    {"RL knee pitch", 2,2,9,  47.701927464700,1440,  52.701927464700,1380}
};

static void check(int condition, const char *name, const char *meaning)
{
    checks++;
    if (!condition) { failures++; printf("FAIL: %s | %s\n", name, meaning); }
}

static int near(double actual, double wanted)
{
    return isfinite(actual) && fabs(actual - wanted) < 1e-7;
}

static int same_command(const ServoCommand *a, const ServoCommand *b)
{
    return a->leg == b->leg && a->joint == b->joint && a->servo_id == b->servo_id
        && a->joint_angle_rad == b->joint_angle_rad && a->servo_angle_deg == b->servo_angle_deg
        && a->pulse_us == b->pulse_us && a->pwm_frequency_hz == b->pwm_frequency_hz;
}

static void test_joint(const ExpectedCommand *e)
{
    ServoCommand command = {0};
    ServoCommandLimits limits = {0};
    int valid = servo_command_limits(e->leg, e->joint, &example_clearance, &limits);
    check(valid, e->name, "usable limits can be calculated from captured endpoints");
    if (!valid) return;

    //Known setpoints check the actual selected motor, radians/degrees, and its PWM table.
    check(servo_command_prepare(e->leg, e->joint, e->q_at_100_deg * deg_to_rad, &example_clearance, &command),
          e->name, "100-degree servo target prepares successfully");
    check(command.leg == e->leg && command.joint == e->joint && command.servo_id == e->servo_id,
          e->name, "command carries the correct leg, joint and servo identity");
    check(near(command.joint_angle_rad, e->q_at_100_deg * deg_to_rad), e->name, "requested model angle is retained");
    check(near(command.servo_angle_deg, 100.0), e->name, "q converts to the expected physical servo angle");
    check(near(command.pulse_us, e->pulse_at_100_us), e->name, "pulse at a measured table knot is correct");
    check(command.pwm_frequency_hz == 50.0, e->name, "frequency comes from the 50-Hz calibration");
    printf("  %-13s S%02d | q=%8.3f deg -> a=%6.2f deg -> %7.2f us (calculated only)\n",
           e->name, command.servo_id, e->q_at_100_deg, command.servo_angle_deg, command.pulse_us);

    check(servo_command_prepare(e->leg, e->joint, e->q_at_95_deg * deg_to_rad, &example_clearance, &command),
          e->name, "interior table interpolation prepares successfully");
    check(near(command.servo_angle_deg, 95.0) && near(command.pulse_us, e->pulse_at_95_us),
          e->name, "95-degree interpolation gives the known pulse width");

    //Usable bounds are inclusive; even a small request beyond them must fail, not clip.
    check(servo_command_prepare(e->leg, e->joint, limits.joint_min_rad, &example_clearance, &command),
          e->name, "exact lower usable model bound is accepted");
    check(command.servo_angle_deg >= limits.servo_min_deg && command.servo_angle_deg <= limits.servo_max_deg,
          e->name, "boundary conversion remains inside the physical usable bounds");
    check(servo_command_prepare(e->leg, e->joint, limits.joint_max_rad, &example_clearance, &command),
          e->name, "exact upper usable model bound is accepted");
    ServoCommand before = command;
    check(!servo_command_prepare(e->leg, e->joint, limits.joint_min_rad - 1e-8, &example_clearance, &command),
          e->name, "target below usable range is rejected");
    check(same_command(&command, &before), e->name, "rejected lower target preserves the previous output");
    check(!servo_command_prepare(e->leg, e->joint, limits.joint_max_rad + 1e-8, &example_clearance, &command),
          e->name, "target above usable range is rejected");
    check(same_command(&command, &before), e->name, "rejected upper target preserves the previous output");

    //Measured mechanical stops themselves are excluded by the positive clearances.
    JointMapping mapping;
    const InstalledJointCalibration *measured = installed_joint_calibration_for_joint(e->leg, e->joint);
    double q_other = 0;
    installed_joint_mapping_for_joint(e->leg, e->joint, &mapping);
    joint_angle_from_servo(&mapping, measured->servo_other_deg, &q_other);
    check(!servo_command_prepare(e->leg, e->joint, mapping.model_home_rad, &example_clearance, &command),
          e->name, "raw HOME stop is not an allowed command");
    check(!servo_command_prepare(e->leg, e->joint, q_other, &example_clearance, &command),
          e->name, "raw OTHER stop is not an allowed command");
}

static void test_clearances(void)
{
    ServoCommandLimits limits = {0};
    //Different HOME/OTHER margins catch mistakes when HOME is the larger servo angle.
    check(servo_command_limits(1, 0, &example_clearance, &limits)
          && near(limits.servo_min_deg, 70.94117647) && near(limits.servo_max_deg, 135.8875)
          && near(limits.joint_min_rad, -36.94632353 * deg_to_rad) && near(limits.joint_max_rad, 28.0 * deg_to_rad),
          "FR margins", "HOME minimum and reversed model sign are handled correctly");
    check(servo_command_limits(0, 0, &example_clearance, &limits)
          && near(limits.servo_min_deg, 68.75625) && near(limits.servo_max_deg, 131.81785714)
          && near(limits.joint_min_rad, -28.0 * deg_to_rad) && near(limits.joint_max_rad, 35.06160714 * deg_to_rad),
          "FL margins", "HOME maximum still applies each margin to its named endpoint");

    //These deliberately invalid settings must never produce limits or pulse commands.
    ServoStopClearance invalid[] = {{0,3},{2,0},{-1,3},{2,-1},{NAN,3},{2,NAN},{INFINITY,3},{2,INFINITY},
                                    {40,40},{1e-300,3},{2,1e-300}};
    for (unsigned int i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        ServoCommandLimits before = limits;
        check(!servo_command_limits(1, 0, &invalid[i], &limits), "bad clearance", "missing, nonfinite or overlapping clearance is rejected");
        check(limits.servo_min_deg == before.servo_min_deg && limits.servo_max_deg == before.servo_max_deg
              && limits.joint_min_rad == before.joint_min_rad && limits.joint_max_rad == before.joint_max_rad,
              "bad clearance", "failed limits calculation leaves all outputs unchanged");
    }
    const InstalledJointCalibration *measured = installed_joint_calibration_for_joint(1, 0);
    ServoStopClearance no_space = {1.0, fabs(measured->servo_other_deg - measured->servo_home_deg) - 1.0};
    check(!servo_command_limits(1, 0, &no_space, &limits), "bad clearance", "margins consuming the entire travel are rejected");
    check(!servo_command_limits(1, 0, NULL, &limits), "bad clearance", "no clearance input is rejected");
    check(!servo_command_limits(1, 0, &example_clearance, NULL), "bad clearance", "missing limits output is rejected");
}

static void test_invalid_commands(void)
{
    ServoCommand before = {99, 88, SERVO_UNASSIGNED, 0.25, 12.0, 1234.0, 50.0};
    ServoCommand command = before;
    const double invalid_angles[] = {NAN, INFINITY, -INFINITY, 1e300, -1e300};
    for (unsigned int i = 0; i < sizeof(invalid_angles) / sizeof(invalid_angles[0]); i++) {
        check(!servo_command_prepare(1, 0, invalid_angles[i], &example_clearance, &command), "bad command", "invalid or far-outside model angle is rejected");
        check(same_command(&command, &before), "bad command", "failed angle request leaves all fields unchanged");
    }
    const int invalid_indices[][2] = {{-1,0},{4,0},{0,-1},{0,3},{3,0},{3,1},{3,2}};
    for (unsigned int i = 0; i < sizeof(invalid_indices) / sizeof(invalid_indices[0]); i++) {
        check(!servo_command_prepare(invalid_indices[i][0], invalid_indices[i][1], 0, &example_clearance, &command),
              "bad identity", "invalid indices or unconfigured RR cannot produce a command");
        check(same_command(&command, &before), "bad identity", "failed identity request preserves previous output");
    }
    check(!servo_command_prepare(1, 0, 0, NULL, &command), "bad command", "missing clearance is rejected");
    check(same_command(&command, &before), "bad command", "missing clearance preserves output");
    check(!servo_command_prepare(1, 0, 0, &example_clearance, NULL), "bad command", "missing command output is rejected");
}

static void test_leg_commands(void)
{
    const ServoStopClearance clearance[3] = {{2,3},{2,3},{2,3}}; //TEST ONLY.
    for (int leg = 0; leg < 3; leg++) {
        const ExpectedCommand *e = &expected[3 * leg];
        LegJointAngles angles = {e[0].q_at_100_deg * deg_to_rad, e[1].q_at_100_deg * deg_to_rad, e[2].q_at_100_deg * deg_to_rad};
        LegServoCommand command = {0};
        check(servo_command_prepare_leg(leg, angles, clearance, &command), "whole leg", "one generic function prepares FL, FR and RL");
        for (int joint = 0; joint < 3; joint++) {
            check(command.joint[joint].servo_id == e[joint].servo_id
                  && near(command.joint[joint].servo_angle_deg, 100)
                  && near(command.joint[joint].pulse_us, e[joint].pulse_at_100_us),
                  "whole leg", "each result uses its own servo table and identity");
        }

        //Fail at each possible joint. Earlier successful joint conversions must not leak out.
        LegServoCommand before = command;
        for (int bad_joint = 0; bad_joint < 3; bad_joint++) {
            LegJointAngles bad = angles;
            if (bad_joint == 0) bad.alpha_rad = NAN;
            else if (bad_joint == 1) bad.theta_1_rad = NAN;
            else bad.theta_2_rad = NAN;
            check(!servo_command_prepare_leg(leg, bad, clearance, &command), "whole leg", "one bad joint rejects the entire leg command");
            for (int joint = 0; joint < 3; joint++) {
                check(same_command(&command.joint[joint], &before.joint[joint]), "whole leg", "all three previous commands remain unchanged");
            }
        }
    }
    LegJointAngles angles = {0};
    LegServoCommand command = {0};
    check(!servo_command_prepare_leg(3, angles, clearance, &command), "whole leg", "RR remains unconfigured");
    check(!servo_command_prepare_leg(1, angles, NULL, &command), "whole leg", "missing clearance array is rejected");
    check(!servo_command_prepare_leg(1, angles, clearance, NULL), "whole leg", "missing leg output is rejected");
}

int main(void)
{
    printf("ServoCommand: calculated pulses only. Test clearances: HOME=2 deg, OTHER=3 deg.\n");
    for (unsigned int i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) test_joint(&expected[i]);
    test_clearances();
    test_invalid_commands();
    test_leg_commands();
    printf("Servo command checks: %d passed, %d failed. No PWM generated or hardware accessed.\n", checks - failures, failures);
    return failures ? 1 : 0;
}
