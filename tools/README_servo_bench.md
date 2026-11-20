# Single-joint servo bench

The same window now has a **Whole leg hold** mode. See [the step-1 guide](README_leg_hold.md).
The single-joint controls below remain available; disable PWM before changing modes.

First physical test: **FR hip pitch / S05**. Expected PWM is **GPIO41**;
feedback is **GPIO19**. Use the USB-to-UART connection: GPIO19/20 are also the
native USB data pins. The PWM pin assignment comes from the shield schematic;
physical identity and movement direction still need to be observed.

The firmware and Python window are implemented and software-tested. They have
not been uploaded or used to move your robot by this change.

## Settings and persistent errors

All adjustable motion limits come from `include/servo_bench_config.h`: maximum
step, maximum peak speed, and minimum/maximum duration. The ESP32 sends these
values to Python; the window displays **Firmware limits** and uses those exact
values. There is no separate 2-degree cap in Python anymore. Endpoint ranges and
the allowed-joint flag also continue to come from the firmware.

The header now also holds `bench_ui_status_interval_ms` (live status requests)
and `bench_ui_reply_timeout_ms` (reply timeout/status freshness). Keep status
interval < reply timeout < link timeout, with room for USB transmission. Existing
feedback and link timeout settings stay in this same header.

After changing the header, rebuild/upload and reconnect Python. The Python
window does not read a local header that might differ from the uploaded firmware.
This updated window requires firmware that reports these settings; an old build
produces an explicit upload/reconnect message rather than using guessed limits.

For a quintic move, peak speed = 1.875 * step_degrees / duration_seconds. With a
10-degree step limit and 5 deg/s peak-speed limit, a 10-degree move needs at
least 3.75 seconds. **10 degrees over 4 seconds** passes these motion limits,
provided the final joint angle is also inside its usable endpoint bounds.

**Last error** is separate from live status. Input errors, rejected requests,
connection failures and unexpected output shutdowns remain visible as status
changes. Clear error only clears the displayed message; it sends no motor
command. Error messages are also included in exported observations.

Python still contains protocol buffer sizes, the four-leg/three-joint identity
checks, and connection startup defaults. These are not motion limits: the
initial reply timeout applies only before the ESP32 can report its settings.
The 20-ms Tk event loop services the window; the ESP32 independently times motion.

## Why buttons blinked and another joint could not enable

Background STATUS used to make the client busy and disable buttons until the
reply arrived. Repeating that every 150 ms made controls blink and interfered
with clicks. A busy click could also disconnect Python, which requested OFF.
Now a button click can replace a pending STATUS. Its late reply is ignored by
request ID. An extra click while a real action is pending is ignored with an
explanation; it does not disconnect, queue movement, or disable PWM. Stop/Disable
retain priority. Unchanged controls are no longer reconfigured every UI tick.

There is a separate intentional restriction: `bench_joint_allowed` currently
contains all three FR joints. A joint whose entry is 0 allows readings, but not PWM.
The UI now states this next to Enable. The table has rows FL, FR, RL, RR and
columns hip roll, hip pitch, knee pitch; a 1 permits enabling that joint. The
current FR row is `{1, 1, 1}`. That is a deliberate
configuration change followed by rebuild/upload, not a Python setting. The
whole-leg hold addition preserves this allow-list and the motion limits.

To change selection: Disable -> choose leg/joint -> Read selected joint ->
enter/apply that joint's clearances -> tick the supported-test acknowledgement
-> Enable. Selection clears the previous joint's clearances and acknowledgement.
An allowed joint can be re-enabled after Disable without reconnecting. OFF now
refreshes a live session's request timer. An already expired session is never
revived by OFF; Python closes it once and asks you to reconnect instead of
continuing to print repeated rejection errors.

## Checks: what blocks a command, and what disables PWM

The UI displays the first unmet Enable condition. Rejected input and actual
shutdown are different: a normal rejected request leaves the previous hold or
move unchanged. It does not automatically stop an in-progress movement.
**Stop / hold** freezes the last commanded angle; **Disable** stops pulses.

| Check | Effect | How to resolve it |
| --- | --- | --- |
| Connected, fresh status and matching dropdown/acknowledged selection | Blocks Enable/Move in Python | Connect, wait for status, or click Read selected joint. |
| A command reply is pending | Blocks another ordinary action; no queue or disconnect | Wait for that reply. Background STATUS does not block clicks. Stop/Disable can interrupt. |
| Selected joint's `bench_joint_allowed` is 0 | Blocks Enable; read-only | Intentionally change the header entry and upload when ready to test that joint. |
| Supported-test acknowledgement is unticked | Blocks Enable in Python only | Tick it after arranging the supported test. Not a feedback sensor or firmware interlock. |
| Clearances missing/invalid | Blocks Enable | Both must be finite, strictly positive, and sum to less than the measured endpoint separation. Apply them. |
| Installed endpoints, q0/sign mapping, or servo table missing/inconsistent | Blocks clearance setup/Enable and q -> pulse conversion | Complete/correct that joint's configuration. RR currently lacks installed endpoints. |
| Usable endpoint range outside the servo PWM calibration table | Blocks clearance setup/Enable | Use a measured, calibrated usable range; no table extrapolation is performed. |
| Starting measured q outside the displayed usable range | Blocks Enable | With PWM off and the leg supported, position the joint inside that range. |
| Enable when already active; selection/clearance change while active | Rejects request, keeps current target | Disable first if changing joints or clearances. Use Move when already holding. |
| Move while not HOLDING | Rejects request | Enable first, wait for the move to finish, or use Stop / hold. |
| Step zero/nonfinite or larger than `bench_max_step_deg` | Rejects move, keeps current target | Enter a finite positive step within the displayed limit; +/- chooses direction. |
| Duration outside `bench_min_move_s` to `bench_max_move_s` | Rejects move, keeps current target | Adjust Duration. Initial UI duration is now read from the firmware minimum. |
| Peak speed `1.875*abs(step)/duration` exceeds `bench_max_speed_deg_s` | Rejects move, keeps current target | Increase Duration or reduce Step. |
| New q outside usable endpoints or cannot convert to calibrated pulse | Rejects move, keeps current target | Reduce Step, change direction, or fix the calibration/configuration. |
| Wrong request selection or unknown operation | Rejects request, keeps current target | Select/read the intended joint; use the matching bench software. |
| Invalid feedback | Blocks Enable and disables active PWM | Check live voltage, wiring and the joint's calibration. Mean AND min/max of the 20-sample window must be finite, consistent and within the voltage table. There is no separate spread threshold. |
| Feedback older than `bench_feedback_timeout_ms` (currently 250 ms) | Blocks Enable and disables active PWM | Restore sampling; enable explicitly only after fresh valid feedback returns. |
| No accepted-session request within `bench_link_timeout_ms` (currently 1000 ms) | Disables PWM and expires the session | Reconnect, apply clearances, then explicitly enable. A late command cannot restart it. |
| Python reply timeout (currently 750 ms), serial failure, ESP32 restart, or malformed/inconsistent response | Python closes the link and requests OFF; firmware timeout is the fallback | Inspect the persistent error and reconnect after correcting the link. |
| Malformed bench request, invalid/repeated request ID or expired session | Firmware disables PWM; session requires reconnect | Use the matching client; reconnect. Ordinary finite but out-of-range move inputs are handled as rejections above. |
| PWM preparation/enable/write failure or invalid intermediate q -> pulse result | Disables PWM | Read the error and inspect driver/configuration. It never retries movement automatically. |
| New HELLO connection, calibration request or explicit OFF | Disables PWM intentionally | Re-enable explicitly when returning to the bench. Calibration remains a separate read/capture workflow. |

The low-level PWM driver also rejects wrong leg/servo identity, nonfinite or
invalid pulse width, frequency other than the calibrated 50 Hz, and an output
not prepared before enabling. It waits for pulse preload (bench: 30 ms) before
enabling and reports hardware API failures. A reported disable failure asks you
to disconnect servo power. These are driver validity checks, not extra hidden
motion limits. PWM-off does not disconnect the servo's electrical supply.

`ServoBench` and `BenchLink` contain the temporary bench gates/timeouts.
`ServoCommand` contains the reusable endpoint and q -> a -> pulse validation.
`ServoPwm` contains hardware validity checks. Desktop assertions and mock
hardware stay in `test/` and are not included in the robot's firmware.

## What stays separate

- `ServoCommand` still performs q -> a -> calibrated pulse, with installed limits.
- `Firmware/ServoPwm` is the reusable 50-Hz MCPWM driver.
- `Firmware/ServoBench`, `Firmware/BenchLink`, `servo_bench_config.h` and this
  Python window are optional bench tools, not part of the walking maths.
- `test/` contains fake hardware, assertions and desktop/Python tests. None of
  those files are compiled into ESP32 firmware by PlatformIO.
- `calibration_ui.py`, `capture_client.py`, the live FK viewer, calibration
  tables, installed measurements and desktop walking project are unchanged.

The calibration window still reads/captures exactly as before. A calibration
request disables bench outputs before its normal processing. Close the bench
window before opening calibration: only one program can own the COM port.
Captured values are not overwritten by bench tests or observation exports.

## First test

1. Support the leg so disabling PWM cannot cause it to fall. Keep the servo
   supply disconnect accessible. Other joints receive no PWM from this bench.
2. Build/upload from the normal PlatformIO project when you are ready. Nothing
   enables on boot, connection, selection or applying clearances.
3. In VS Code choose the quadruped Python interpreter. Open
   `tools/servo_bench_ui.py` and choose **Run Python File in Terminal**.
   Alternatively, from the project terminal:

   ```powershell
   & 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' '.\tools\servo_bench_ui.py'
   ```

   Close the PlatformIO Serial Monitor first. Only `pyserial` and built-in Tk
   are required; the existing `tools/requirements.txt` installs pyserial.
4. Connect to the USB-to-UART COM port. FR / Hip pitch is selected initially.
   Confirm the live voltage/angle responds to S05 and the window reports
   PWM41, feedback19, S05. Move the supported joint manually to an interior
   position while PWM is disabled; do not start at either mechanical stop.
5. Enter **positive inward clearances**, in physical servo degrees, from HOME
   and OTHER END; click **Apply clearances**. These are allowances for the
   assembled hardware, not corrections to q_0. No production values have been
   assumed. The 5-degree values in tests are software examples, not verified
   physical margins. The displayed usable ranges are what the ESP32 will enforce.
6. First check the signal with the servo signal disconnected and a scope on
   GPIO41. Keep feedback powered. Tick the supported-test acknowledgement and
   click **Enable hold here**. Check 50 Hz / 20 ms period and the displayed
   pulse width. Verify **DISABLE PWM** leaves the signal LOW. Disable output
   and power down before reconnecting the servo signal.
7. With the leg supported and feedback valid again, enable the initial hold.
   It uses the current measured angle; it does not send a default 1500-us pulse.
   Some initial correction is possible because feedback/PWM calibration and
   load are imperfect. Observe this hold before requesting movement.
8. Choose Step and Duration within the displayed firmware limits. Duration
   initially uses the configured minimum (currently 2 seconds). Click **Move +** once.
   For this joint sigma=+1: mathematical q, physical a, and the calibrated pulse
   should increase. Observe which actual joint moves. Compare after the feedback
   filter has settled. Test **Move -**, then **Stop / hold** and **DISABLE PWM**.
9. With output disabled, record the observed joint/direction and export the
   observation JSON. This report does not update calibration or automatically
   mark wiring verified. Report the physical result before enabling more joints.

Stop/hold freezes the last commanded angle and continues pulses. Disable stops
pulses; it does not switch servo power off. A lost link disables after 1 second;
invalid feedback or feedback older than 250 ms also disables. These are supported
bench policies, not a promise that turning off PWM is safe for a standing robot.
There is no extra position PID or automatic tracking-error correction here.
Software cannot detect every wiring mistake, mechanical collision or stuck servo.

## How the files connect

`main.cpp` owns the bench state and calls its update alongside the existing line
reader. `CaptureLink` has an optional dispatch callback; old requests retain their
old format. `BenchLink` handles only `QLBENCH1` bench requests and replies in JSON.

`ServoBench` samples the selected joint once every 5 ms using the existing
`servo_feedback_read(..., 1, 0)` function. It forms 20-sample windows without
blocking waits. The mean/min/max are validated using the existing endpoint
reading helper, without saving any endpoint. ServoCalibration and JointMapping
then convert V -> a -> q. Raw mean voltage remains visible if conversion fails.

On Enable, the controller prepares the pulse for measured q, holds its output
LOW for 30 ms while the compare value loads, then enables it. On a move:

    u = min(1, elapsed_time / movement_duration)
    F(u) = 10u^3 - 15u^4 + 6u^5
    q = q_start + (q_target - q_start) F(u)

Every 20 ms it calls ServoCommand, then ServoPwm with the new checked pulse.
The allowed step, duration and peak speed come from `servo_bench_config.h`.
Python shows them, and the ESP32 enforces them even if Python is bypassed.
Only one joint can be enabled by this bench. Currently its allow-list contains
all three FR joints in the current header; other selections remain read-only.

The output driver uses two MCPWM units, each with three A/B pairs. A timer is
initialized once, so preparing another output does not reset its active neighbour.
Disable forces only that output LOW; it does not stop a shared timer. Pulses are
rounded to whole microseconds. The driver has basic identity/frequency/API checks;
the bench owns its timing and timeout policies.

## Tests and what passing means

From a normal PowerShell terminal (the working compiler terminal, without Conda):

```powershell
.\test\run_desktop_tests.ps1
```

To run only the new PWM/bench C++ tests:

```powershell
.\test\run_bench_tests.ps1
```

To test the Python controls against the compiled fake-board bridge:

```powershell
& 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' '.\test\test_servo_bench_ui.py'
```

The checks verify preload-before-enable, all 12 routing entries, shared-timer
isolation, current-position hold, quintic interpolation, limits, stopping,
invalid/stale feedback, disconnect/reconnect, rejected commands, and UI buttons.
They also cover clicks during background polling, Disable -> select another
joint -> return/re-enable, readable blocking reasons, rejected commands keeping
the existing hold, and an expired session disconnecting without an error loop.
They use fake ADC/MCPWM/serial; no real COM port is opened. They do not prove
electrical pulse timing, actual joint identity, mechanical clearance or motion.

For later walking, call ServoCommand and ServoPwm from the locomotion coordinator.
The test files can remain unused. Removing the bench later also means removing
its three includes/state/update hooks in main.cpp and the optional routing setup;
it does not require changing the walking mathematics or calibration modules.
