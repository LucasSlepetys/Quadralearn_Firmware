# Step 1: supported whole-leg hold

This adds coordinated holding, not a walking trajectory. The ESP32 reads the
three FR joints, calculates their current mathematical angles, and holds those
three positions when you explicitly enable. It does not send CAD HOME angles.

## Run the test

1. Build/upload the updated **Locomotion Firmware** project in PlatformIO.
   Close the serial monitor before opening Python. Boot generates no PWM.
2. Run `tools/servo_bench_ui.py` with the quadruped interpreter, as before:

   ```powershell
   & 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' '.\tools\servo_bench_ui.py'
   ```

3. Support the robot/leg off the floor with servo power disconnect accessible.
   Start with the joints inside their usable ranges, not at the mechanical stops.
4. Connect using USB-to-UART. Select **Whole leg hold**, **FR**, then click
   **Read selected joint** (the button submits the selected mode and leg).
5. Check the three live voltage/angle rows: S04, S05, S06; PWM42/41/40;
   feedback1/19/11. No outputs are enabled just by selecting or reading.
6. Enter each joint's HOME and OTHER clearance, in physical servo degrees,
   then **Apply all three clearances**. Use your chosen physical margins;
   the five-degree margins in tests are examples, not installed defaults.
7. Tick the supported-test acknowledgement. The explanation beside Enable
   names any unmet condition. Click **Enable hold here** once.
8. Expect ARMING briefly (30 ms in the firmware), then HOLDING for the three
   joints. Some correction can occur because feedback/PWM calibration and
   load are imperfect. Observe a short supported hold first, then **DISABLE PWM**.

There is no Move button in whole-leg mode. Stop / hold keeps the current holds;
while ARMING it cancels preparation. Disable stops all three leg signals, but
does not disconnect the servo power supply. Do not interpret this test as
floor-walking or load/thermal validation.

## How it works

`main.cpp` calls only the selected controller: existing `ServoBench` for a
single joint, or new `LegControl` for a whole leg. `BenchLink` handles mode
selection and USB timeout; the existing `CaptureLink` remains the sole serial
reader. A new calibration session exits the bench and disables outputs first.

`leg_control_update()` takes one short ADC round for all three joints when due.
It averages 20 rounds, keeping mean/min/max, and calls
`measured_leg_pose_from_readings()`:

    V -> servo table -> a -> installed JointMapping -> q -> FK

`leg_control_enable_hold()` captures that complete measured pose once, then
calls `servo_command_prepare_leg()`:

    q = {alpha, theta_1, theta_2}
    a = a0 + sigma*(q-q0)*180/pi
    pulse_us = (calibrated duty_percent/100)*(1000000/50)

All three angle/pulse calculations must succeed before any PWM preparation.
All three pulses are prepared while their signals remain LOW. After the
configured preload time, the update enables the three channels. Writes occur
sequentially; this is not a promise of perfectly simultaneous electrical edges.

Later feedback updates the displayed measured pose, but does not overwrite the
saved hold target. There is no new external feedback PID or automatic drift
correction. The servos' internal controllers perform their own position hold.

## Files and functions

| File | Responsibility |
| --- | --- |
| `src/Firmware/LegControl/leg_control.h/.cpp` | Reusable three-joint feedback, measured-pose hold and complete-leg targets. |
| `include/leg_control_config.h` | Commented sampling, freshness and preload settings. |
| `src/Firmware/BenchLink/bench_link.h/.cpp` | Existing link extended with whole-leg read/configure/hold/disable; owns USB timeout and permission policy. |
| `src/main.cpp` | Owns both states and updates exactly one active controller. |
| `tools/leg_hold_panel.py` | Three-row readings, clearances and blocking explanations inside the existing window. |
| `tools/servo_bench_ui.py`, `tools/bench_client.py` | Mode selection and whole-leg requests/replies; the single-joint test remains available. |
| `test/desktop/test_leg_control.cpp`, `test/test_leg_control_ui.py` | Separate fake-hardware and UI/protocol checks. |

The main LegControl functions are:

- `initialize`: configure this leg's ADC inputs, start disabled, clear saved state.
  Caller must use it only for a new/disabled controller.
- `configure_joint`: validate/store one joint's HOME/OTHER clearances while off.
- `update`: schedule ADC windows and finish delayed PWM enabling.
- `enable_hold`: capture measured q, check all targets, preload the three pulses.
- `set_target`: reusable complete-leg target interface for the next stage.
  It is **not exposed by the hold UI**. The future caller must generate smoothly
  timed, speed-limited targets; this function does not plan a trajectory or add
  the manual bench's per-click limit.
- `disable`: turn off this leg's three outputs and save an explicit reason.

Saved state: leg/configuration, three clearance/limit pairs, three feedback
windows and timestamps, latest complete measured pose, last complete command,
DISABLED/ARMING/HOLDING, reason and problem-joint index. No endpoints are recaptured
or stored by this module. Clearances are RAM settings and reset on selection.

## Settings you can change

- `include/leg_control_config.h`: sample interval 5 ms, 20 rounds, feedback
  timeout 250 ms, preload 30 ms. Comments explain each. The initialization
  checks that the timeout exceeds the nominal sample window and preload is
  at least the driver's 25 ms; scheduling/USB margins still matter.
- `include/servo_bench_config.h`: existing per-joint allow-list, communication
  timeouts and UI polling. FR `{1,1,1}` permits its three outputs; this change
  did not widen that list. Other legs can be selected/read but remain blocked
  unless all three permissions and installed calibrations are available.
- The same header now sets an **8192-byte UART transmit buffer**, configured
  before Serial.begin. A normal whole-leg reply is about 1.8 KB (~0.16 seconds
  at 115200 baud). Buffering lets normal replies travel in the background while
  the loop samples feedback. It is not a movement limit or an unlimited queue.
- Six endpoint-clearance fields are entered in Python, independently per joint.
  The single-joint 10-degree step, 5-degree/s speed and move duration limits
  remain unchanged; they do not apply to this hold-only mode.

## Exactly what can block or stop the hold

| Condition | Result and recovery |
| --- | --- |
| Mode/leg not submitted, request pending, acknowledgement unticked | UI blocks Enable and explains why. Background status does not block clicks. |
| Any selected joint's allow-list entry is 0 | Whole-leg Enable rejected; read-only until deliberately configured and uploaded. |
| Missing/invalid calibration or any missing/invalid clearance | Enable blocked. Apply valid positive margins for all three joints within calibrated travel. |
| Any starting q outside its usable limits | Entire Enable request rejected before PWM preparation; reposition the supported leg while off. |
| No complete valid/fresh feedback | Enable blocked; inspect voltages and the reported joint. All three angles are published as a complete pose. |
| Selection/mode/clearance change while active | Request rejected; current hold stays active. Disable first. |
| Bad future `set_target` value | Entire target rejected with no new writes; previous complete hold remains. |
| Feedback invalid or older than 250 ms while active | All three selected-leg outputs disabled; reason identifies the joint when possible. Fix the reading and explicitly re-enable. |
| No link request for 1000 ms, malformed/repeated request or expired session | Active controller disabled; reconnect and enable explicitly. Python reply timeout is 750 ms. |
| PWM prepare/enable/write failure on any joint | All three selected-leg outputs disabled, including any already prepared/enabled. Inspect the reported joint/error. |
| Explicit Disable, closing/reconnecting, or entering calibration | Outputs intentionally disabled. No automatic restart. |

Feedback validity uses the existing voltage-table/mean/min/max checks and
installed mapping. There is no new feedback-error threshold, temperature/current
check, automatic maximum hold time, collision detector or physical speed proof.
The allowed angle range checks commanded targets; measured error alone is not
a new shutdown rule. Turning PWM off is a supported-bench policy, not yet a
policy for a load-bearing standing robot.

## Tests and what passing means

Run the isolated C++ coordinator test in a normal PowerShell terminal:

```powershell
.\test\run_leg_control_tests.ps1
```

For the UI tests, first build the fake-board bridge, then run Python:

```powershell
.\test\run_bench_tests.ps1
& 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' '.\test\test_leg_control_ui.py'
```

`run_desktop_tests.ps1` also includes the new C++ test. These tests live outside
the embedded build. They check measured holds, no partial writes after a bad
target, second-channel failures, isolation from other legs, no sampling delays,
invalid/stale feedback, ownership changes, link loss and UI gates. All hardware
is fake; passing does not demonstrate actual wiring or loaded hold accuracy.

After the supported three-joint hold is observed, stage 2 will connect the
existing locomotion mathematics. No gait/alignment/height/start-stop trajectory
code has been added by this stage.
