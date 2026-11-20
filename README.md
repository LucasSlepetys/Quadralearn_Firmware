# Python leg endpoint interface

A separate [live feedback viewer](tools/README_live_feedback.md) now reconstructs
the measured leg in 3D. The calibration window and its HOME/OTHER workflow below
are unchanged. Use one serial application at a time.

[ServoCommand](lib/ServoSupport/src/ServoCommand/README.md) now prepares checked
joint-angle-to-pulse-width values in desktop tests. It is not connected to PWM or
the firmware loop. No hardware operating clearances are chosen by its test examples.

This project reads servo feedback and records whole-leg HOME / OTHER END positions.
It generates no PWM and does not run locomotion. The terminal command interface
has been removed: Python is now the user interface, and the ESP32 sends structured
USB data through CaptureLink.

## Run it in VS Code

First upload the updated firmware. The previous terminal firmware cannot talk to
this Python interface. Copy any old RAM capture results before uploading/resetting.

1. Open **Locomotion Firmware** in VS Code. Close Serial Monitor, then use PlatformIO
   **Build** and **Upload** through the CH343 USB-to-UART connection.
2. Keep Serial Monitor closed. Only one application can use that COM port at once.
3. Open `tools/calibration_ui.py`.
4. Press Ctrl+Shift+P, choose **Python: Select Interpreter**, and select:

   `C:\Users\lucas\miniconda3\envs\quadruped\python.exe`

5. Right-click inside the Python editor and choose **Run Python File in Terminal**,
   or use the Python run triangle at the top right. Do not use PlatformIO Upload
   or a C++ build task to launch the Python file.
6. Select the CH343 port in the window, then click **Connect**. COM8 was used in the
   earlier test, but select the current number shown by Windows.

PowerShell alternative, from anywhere:

```powershell
& 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' 'C:\Users\lucas\Desktop\Locomotion Firmware\tools\calibration_ui.py'
```

The existing quadruped environment has Tkinter and pyserial 3.5 installed. If using
another Python environment, install `tools/requirements.txt` into that interpreter:

```powershell
python -m pip install -r tools/requirements.txt
```

USB control lines are set inactive before opening, but some USB drivers can still
reset the board on opening/closing a port. Captures are RAM-only and may be lost.
GPIO19/20 are feedback inputs, so use the CH343 bridge rather than native USB.

## Use the window

1. Select FL, FR, RL or RR. The live table shows three joints with servo IDs, GPIOs,
   mean/min/max voltage, spread and physical servo angle. Use **Read once** if live
   updates are paused. Move joints manually with the feedback circuitry powered;
   this program does not command them.
2. Position ALL THREE joints at home. Hold the leg stationary and click
   **Preview HOME · 3 joints**.
3. Inspect the separate frozen preview table. Live readings continue in their own
   table and cannot overwrite that snapshot. Check all three physical positions
   and voltage rows.
4. Click **Accept preview** to record all three together, or **Discard**.
5. Position all three joints at their other endpoints, click **Preview OTHER END ·
   3 joints**, inspect, then accept.
6. The bottom table shows the selected leg's accepted endpoints, read back from
   the ESP32. Change legs and repeat as needed.
7. Click **Export all legs to JSON…** to save a computer backup. The interface
   fetches all four legs from the board; unrecorded endpoints are explicitly null.

Accept means accepted into ESP32 RAM, not permanent flash. Reset/power loss clears
RAM. Export stores a file on the computer; it does not add EEPROM/NVS persistence
or automatically restore records into the robot. Disconnect the interface before
another upload. A reconnect discards an unseen pending preview but retains accepted
RAM records if the board did not reset.

A preview with any invalid joint is rejected. All three voltage rows remain visible,
invalid rows are red, and Accept is disabled. The ESP32 independently rejects invalid
or mismatched accepts, even if a different application sends a request.

## Files and responsibilities

```text
include/
    board_pins.h                    Feedback wiring for all four legs
    feedback_test_config.h          Sample count, interval, serial baud
lib/ServoSupport/src/
    Configuration/                  Servo IDs/tables, CAD references and installed endpoints
    JointMapping/                   Existing physical angle a <-> model angle q maths
    ServoCalibration/               Existing V <-> physical angle maths
    FeedbackTypes/                  Shared mean/min/max voltage structure
    EndpointCapture/                Existing preview/accept state and validation
src/
    main.cpp                        Owns capture state and runs the USB link
    Firmware/ServoFeedback/         Existing three-joint ADC sampling
    Firmware/CaptureLink/           Structured requests/replies for Python
tools/
    calibration_ui.py              Tkinter window, display and button actions
    capture_client.py              USB request/reply matching and timeout handling
    requirements.txt               pyserial dependency
test/
    run_desktop_tests.ps1           Compile/run C++ tests with fake ADC/serial
    desktop/                       Fake hardware and C++ test harness
    test_python_interface.py        Client and Tk button tests using that same harness
```

CaptureConsole was removed, along with its terminal menu, command parsing and
human-readable print helpers. There is only one firmware capture implementation.
Python does not contain calibration tables, interpolation, joint mapping or FK.

Data flow:

`Python request -> CaptureLink -> ServoFeedback -> EndpointCapture -> JSON reply -> Python display`

The shared libraries and ServoFeedback calculations are unchanged. The desktop
Locomotion code tester project is unchanged.

## What happens when you preview and accept?

- The button sends a request naming the leg and endpoint.
- `servo_feedback_read_leg()` reads roll, pitch, knee in each of 20 rounds. There
  are 20 samples per joint, with 19 waits of 5 ms (95 ms plus ADC overhead). The
  channels are sequential, not exactly simultaneous, so keep the whole leg still.
- `endpoint_capture_preview()` uses the existing per-servo calibration tables and
  checks all three voltage windows. Only a complete valid group becomes a preview.
- CaptureLink attaches a preview token, and Python keeps the returned snapshot
  separate from later live readings.
- Accept sends that leg and token back. CaptureLink checks that they still match
  the pending preview, then calls `endpoint_capture_save()`. It does not resample.
- Python refreshes the accepted records from the ESP32 after the acknowledgement.

`main.cpp` owns `EndpointCaptureState`: HOME/OTHER records for four legs and one
pending preview. CaptureLink remembers only the incoming line, active ADC leg and
preview token. Python remembers displayed readings and the preview token. The
board remains the authority for accepted records and calibration results.

The means/minima/maxima must be finite, ordered and inside each servo's calibration
range. There is no invented spread threshold. A valid voltage does not prove the
physical joint is at its mechanical endpoint or that the wiring is correct.
Outputs are physical servo angles a [degrees], not mathematical joint angles q.

## Minimal USB protocol (for understanding the code)

Each request is one bounded line:

`QLCAP1 request_id operation leg_index preview_token`

Operations are HELLO, READ, HOME, OTHER, ACCEPT, CANCEL and GET. Leg indices are
FL=0, FR=1, RL=2, RR=3. Only ACCEPT uses a nonzero preview token. Replies are JSON
lines tagged `leg-capture/1` and include the request ID and leg. Invalid numerical
values are JSON null. This is a data interface, not a second terminal UI.

Python keeps one outstanding request at a time, does not retry accepts automatically,
rejects mismatched replies and clears the displayed preview on a failed connection
or detected board reset. Live snapshots are requested approximately every 0.4 s
when no user action is waiting. Opening the window does not open a serial port;
only clicking Connect does that.

## Desktop checks without the robot

The installed joint configuration contains the FL, FR and RL measurements from
`leg_endpoints.json` exported on 2026-11-20 at 18:49:46 UTC. RR remains unavailable.
`installed_joint_calibration_for_joint()` returns the stored HOME/OTHER angles.
`installed_joint_mapping_for_joint()` combines HOME with the existing CAD reference:

`a_0 = a_HOME; s_a = sign(a_OTHER - a_HOME); sigma = s_q * s_a`

JointMapping then uses `q = q_0 + sigma * (a - a_0) * pi/180`, or its inverse.
These functions do not yet run in the capture interface; its displayed angles
remain physical servo angles. The new settings are compiled constants, independent
of capture RAM. A later capture/export does not automatically replace them.
Raw OTHER/HOME endpoints are retained for later limit handling; stop clearance and
motor-command checks are not implemented by this configuration.

From this firmware project folder:

```powershell
.\test\run_desktop_tests.ps1
& 'C:\Users\lucas\miniconda3\envs\quadruped\python.exe' '.\test\test_python_interface.py'
```

The PowerShell script runs two separate C++ programs: capture/link checks using
fake hardware, then installed-joint checks using the captured voltages and known
expected angles. A FAIL names the failing expectation. See `test/README` for what
each test proves and how to run only the installed-joint program.

The Python tests exercise the real Tk buttons against the compiled C++ code with
fake ADC samples. They never open a physical COM port. The fake command used to
change a test voltage exists only in the host test executable, not the firmware.
These tests do not prove the real USB connection or all twelve physical channels.

References for the dependencies:
- pySerial API: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
- Tkinter: https://docs.python.org/3/library/tkinter.html
