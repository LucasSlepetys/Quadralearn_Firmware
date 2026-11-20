# ServoCommand

This module prepares pulse-width VALUES. It does not generate PWM, configure pins,
move servos, read feedback or change calibration. It is not called by the firmware
loop yet. The calibration UI and live feedback viewer remain unchanged.

## Inputs and outputs

- `servo_command_limits(leg, joint, clearance, limits)` returns usable bounds in
  servo degrees and mathematical radians.
- `servo_command_prepare(leg, joint, q, clearance, command)` prepares one joint.
  The result contains leg/joint/servo identity, requested q, corresponding a,
  pulse width in microseconds and the calibration frequency in Hz.
- `servo_command_prepare_leg(leg, angles, clearance[3], command)` prepares all
  three joints. `angles` is our familiar `LegJointAngles` structure. No new leg
  result is returned unless all three requests pass.

Every function returns 1 on success or 0 on failure. Check that return value
before using the result. Failure leaves the caller's previous output unchanged.
These functions remember nothing between calls.

## Where the equations appear

1. Read installed HOME and OTHER END, plus q_0, a_0 and sigma from the existing
   configuration and mapping functions.
2. Let s_a = sign(a_OTHER - a_HOME). Move inward from each measured endpoint:

       a_home_usable  = a_HOME  + s_a * clearance.home_deg
       a_other_usable = a_OTHER - s_a * clearance.other_deg

   Sort these two values to obtain a_min and a_max. Both clearances must be
   finite and positive, and their sum must leave a nonzero interval. The usable
   endpoints must be covered by the servo's calibration table.
3. Convert those bounds with `joint_limits_from_servo_limits()`. Reject a requested
   q outside the resulting mathematical interval.
4. `servo_angle_from_joint()` applies:

       a = a_0 + sigma * (q - q_0) * 180/pi

5. `servo_pulse_from_angle()` interpolates the measured duty percentage D(a), then:

       pulse_us = (D/100) * (1000000/f_PWM)

   For our 50-Hz tables, pulse_us = 200*D.

Out-of-range requests are rejected, not changed to the nearest allowed target.
After q has passed its bounds check, at most tiny conversion roundoff is removed
from a at an exact boundary; this is not a general target-clipping policy.

## Clearances are explicit inputs

No production clearance values have been selected. The desktop test uses HOME=2
degrees and OTHER=3 degrees only to exercise the maths. Those are not verified
physical settings and are not added to the installed configuration.

HOME and OTHER retain their original measured values. With positive clearances,
commands at the raw mechanical endpoints themselves are rejected. The same code
handles HOME at either the smaller or larger physical servo angle.

Example from the test for FR hip roll (S04):

    requested q = -1.05882353 degrees, supplied to the function in radians
    physical a  = 100 degrees
    measured D  = 7.1 percent
    pulse width = 1420 microseconds at 50 Hz

This is a calculated example, not a command sent to hardware.

## Reuse later

The functions accept any leg/joint index. FL, FR and RL use their own stored
references and tables; RR fails until its installed measurements are configured.
Later the coordinator will prepare all enabled leg commands before giving any
pulses to a separate ServoPwm driver. No Arduino or PWM API belongs in this module.

This step checks position bounds and calibration coverage. It does not check
joint speed, acceleration, feedback freshness, collision clearance between parts,
tracking error or physical PWM wiring. Motion timing and hardware checks belong
to the next stages already planned.

## Desktop test

Run `test/run_desktop_tests.ps1` from the firmware project, using the working C++
terminal. After building, `test/desktop/test_servo_command.cpp` has its own executable:

    .\.pio\desktop_tests\servo_command_test.exe

The printed examples show q -> a -> pulse for nine servos. The checks cover known
table knots, interpolation, endpoint direction, exact usable bounds, requests just
outside the bounds, missing calibration and all-or-nothing whole-leg results.

## Planned reusable Python pin/direction test

This interface will be added with the PWM bench stage, not this calculation-only
step. Extend the live feedback viewer while keeping `calibration_ui.py` unchanged.

- Select leg and joint; show servo ID and the expected configured PWM GPIO.
- Start disabled. Read the current angle and prepare its initial holding command.
- Explicitly enable only the selected test joint; request a small bounded move
  in either direction using the same ServoCommand functions.
- Show requested model angle, physical angle, pulse width and measured feedback.
- Provide separate stop/hold and disable-output controls.
- Let the operator confirm which physical joint actually moved and its direction.
  Feedback alone cannot prove the wiring label matches the physical assembly.
- Export the observations with servo identity, pin, test values and date so the
  same test can be repeated after wiring changes or repairs. Do not automatically
  overwrite calibration or mark a channel verified from a software-only result.

Actual hardware limits and initial PWM behavior must be checked before those
movement controls are enabled. Python will request bounded tests; the ESP32 will
retain the command checks even if the interface disconnects.
