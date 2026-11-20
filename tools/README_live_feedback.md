# Live measured leg view

This separate window reconstructs a leg from its feedback. It does not command
motors, start locomotion, capture endpoints or change the installed calibration.
The existing `calibration_ui.py` is unchanged and remains the endpoint-capture tool.

## Run in VS Code

1. Export any unsaved calibration captures before uploading: those captures are
   RAM-only and a reset/upload can clear them. The installed mapping constants
   already copied from your export are independent of that capture RAM.
2. Close the calibration window and Serial Monitor. Build and upload this firmware
   using your working USB-to-UART connection. The new viewer needs the `POSE` request.
3. Open `tools/visualize_feedback.py` in VS Code. Select the **quadruped** Python
   interpreter, then choose **Run Python File in Terminal**. Use the terminal with
   quadruped activated for the graph; use your working C++ terminal for C++ tests.
4. Select the correct COM port and click **Connect**. FR is selected initially.
5. Move one joint slowly by hand. Compare the physical servo angle `a`, the model
   angle `q`, and the leg drawing. Use **Live readings** to pause or **Read once**.

If activation is not automatic, in a PowerShell terminal:

```powershell
. 'C:\Users\lucas\miniconda3\shell\condabin\conda-hook.ps1'
conda activate quadruped
python 'C:\Users\lucas\Desktop\Locomotion Firmware\tools\visualize_feedback.py'
```

The quadruped environment already has the plotting and USB libraries. For another
environment, install `tools/requirements_visualization.txt` using that environment's
Python. The original calibration requirements file is unchanged.

Only one application can own the COM port. Disconnect/close this viewer before
using the calibration window or Serial Monitor again. The packaged calibration
app still uses the same existing capture protocol and commands.

## What you see

- The table shows servo identity/GPIO, mean voltage, spread, physical angle in
  degrees, and mathematical angle in both radians and degrees.
- H is hip roll, P is hip pitch, K is the knee and F is the ball centre. The sphere
  uses the measured ball radius. The graph connects H -> P -> K -> F.
- Every coordinate is in the fixed hip-roll frame H: x forward, y left, z up.
  The hip is fixed at the origin. Moving the entire leg assembly in your hand
  does not move that origin; this is not a measurement of the body's world pose.
- The three feedback channels are read in interleaved rounds, not simultaneously.
  The current 20-round window has 95 ms of scheduled waits plus ADC overhead.
  Move slowly for this visual comparison; this is not a fast feedback controller.
- Invalid feedback clears the drawing while keeping the voltage rows visible.
  A disconnected or stale live reading also clears the old pose. Pausing live
  readings intentionally freezes and labels the last sample.

At the recorded FR HOME position, expect approximately:

| Joint | Physical servo a [deg] | Mathematical q [deg] |
|---|---:|---:|
| Hip roll | 68.94 | +30.00 |
| Hip pitch | 14.59 | -104.57 |
| Knee pitch | 161.88 | +134.17 |

Actual readings will vary. These are reference values, not commands to send.
Feedback measures servo rotation; visual agreement still depends on the installed
reference, geometry and mechanical coupling being correct.

## Where the maths runs

`servo_feedback_read_leg()` -> `measured_leg_pose_from_readings()` -> USB -> graph

Inside `measured_leg_pose_from_readings()`:

1. `leg_endpoint_capture_from_readings()` validates the same voltage windows used
   by the calibration display and performs V -> a. Despite its name, this helper
   only creates a temporary result; it does not save, preview or accept endpoints.
2. `installed_joint_mapping_for_joint()` obtains q_0, a_0 and sigma for that joint.
3. `joint_angle_from_servo()` applies q = q_0 + sigma*(a - a_0)*pi/180.
4. `leg_forward_kinematics()` reconstructs the four H-frame points using CAD geometry.

`KinematicsTypes` holds the same small geometry/angle structures as the desktop
model, avoiding a dependency on its full IK/gait header chain. `robot_model` and
the FK equations are copied from that project. The desktop project is untouched.

CaptureLink adds the read-only `POSE` operation. All existing commands and reply
fields retain their behavior. `capture_client.py` validates the extra POSE fields;
the new Python viewer only draws the coordinates and converts radians to degrees
for display. `main.cpp`, `calibration_ui.py`, EndpointCapture, ServoFeedback,
ServoCalibration and installed calibration data are unchanged.

The same calculation accepts any leg index. FL and RL are also configured; RR
reports missing installed endpoints. To draw the whole robot later, put each
leg in the body frame using X_B = h_B + X_H, then apply an assumed or estimated
body pose if a world-frame view is desired. No new calibration/FK equations are needed.

## Tests without the robot

Run `test/run_desktop_tests.ps1` first in the working C++ terminal. This builds the
fake firmware used by both Python test programs. In the quadruped terminal run:

```powershell
python 'C:\Users\lucas\Desktop\Locomotion Firmware\test\test_python_interface.py'
python 'C:\Users\lucas\Desktop\Locomotion Firmware\test\test_feedback_visualization.py'
```

The first tests the unchanged calibration workflow. The second checks the live
viewer against real C++ results with simulated ADC values, including invalid data,
leg changes and stale/disconnected readings. No physical port or PWM is used.
