"""Separate single-joint PWM bench. Existing calibration UI is not modified.
Run with the quadruped interpreter; only pyserial and built-in Tk are needed.
"""
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("Install pyserial: python -m pip install -r tools/requirements.txt")

from bench_client import BenchClient, BenchBusyError
from leg_hold_panel import LegHoldPanel

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("Hip roll", "Hip pitch", "Knee pitch")


def open_bench_serial(name):
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.2)
    port.dtr = False
    port.rts = False
    port.port = name
    try:
        port.open()
    except Exception:
        port.close()
        raise
    return port


class ServoBenchApp:
    def __init__(self, root, serial_factory=open_bench_serial, ports_factory=list_ports.comports, connect_delay_ms=1500):
        self.root = root
        self.serial_factory = serial_factory
        self.ports_factory = ports_factory
        self.connect_delay_ms = connect_delay_ms
        self.client = None
        self.handshake_job = None
        self.tick_job = None
        self.status = None
        self.last_reply = 0.0
        self.next_poll = 0.0
        self.records = []
        self.root.title("Servo bench - joint test / whole-leg hold")
        self.root.minsize(850, 750)
        self.control_scope = tk.StringVar(value="Single joint")
        self.displayed_scope = None
        self.port_name = tk.StringVar()
        self.leg = tk.StringVar(value="FR")
        self.joint = tk.StringVar(value="Hip pitch")
        self.home_margin = tk.StringVar()
        self.other_margin = tk.StringVar()
        self.step_deg = tk.StringVar(value="1.0")
        self.duration_s = tk.StringVar()  #HELLO supplies the configured minimum.
        self.supported = tk.BooleanVar(value=False)
        self.message = tk.StringVar(value="Disconnected. No output is enabled by opening this window.")
        self.last_error = tk.StringVar(value="No errors.")
        self.last_error_detail = ""
        self.motion_limits = tk.StringVar(value="Motion limits will be read from the connected ESP32.")
        self.enable_reason = tk.StringVar(value="Connect to read the joint and its enable conditions.")
        self.identity = tk.StringVar(value="First test: FR hip pitch / S05 / expected PWM41, feedback19")
        self.limits = tk.StringVar(value="Enter inward clearances from both measured mechanical stops [servo degrees].")
        self.observed = tk.StringVar(value="Not tested")
        self.direction = tk.StringVar(value="Not tested")
        self.notes = tk.StringVar()
        self.direction_help = tk.StringVar(value="q is the mathematical joint angle; a is the physical servo angle.")
        self.values = {name: tk.StringVar(value="--") for name in
                       ("Voltage", "Spread", "Measured a", "Measured q", "Command a", "Command q", "Pulse", "State")}
        self._build()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick_job = self.root.after(20, self.tick)

    def _build(self):
        panel = ttk.Frame(self.root, padding=16)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="Servo PWM bench", font=("Segoe UI", 17, "bold")).pack(anchor="w")
        ttk.Label(panel, text="Calibration records stay unchanged. Support the leg; keep servo power disconnect accessible.").pack(anchor="w", pady=(4, 12))
        row = ttk.Frame(panel); row.pack(fill="x")
        self.port_box = ttk.Combobox(row, textvariable=self.port_name, width=18, state="readonly"); self.port_box.pack(side="left")
        ttk.Button(row, text="Refresh ports", command=self.refresh_ports).pack(side="left", padx=5)
        self.connect_button = ttk.Button(row, text="Connect", command=self.toggle_connection); self.connect_button.pack(side="left")
        row = ttk.Frame(panel); row.pack(fill="x", pady=12)
        self.scope_box = ttk.Combobox(row, textvariable=self.control_scope, values=("Single joint", "Whole leg hold"), state="readonly", width=16)
        self.scope_box.pack(side="left", padx=(0, 5))
        self.scope_box.bind("<<ComboboxSelected>>", lambda event: self.update_controls())
        self.leg_box = ttk.Combobox(row, textvariable=self.leg, values=LEGS, state="readonly", width=8); self.leg_box.pack(side="left")
        self.joint_box = ttk.Combobox(row, textvariable=self.joint, values=JOINTS, state="readonly", width=15); self.joint_box.pack(side="left", padx=5)
        self.leg_box.bind("<<ComboboxSelected>>", lambda event: self.update_controls())
        self.joint_box.bind("<<ComboboxSelected>>", lambda event: self.update_controls())
        self.select_button = ttk.Button(row, text="Read selected joint", command=self.select_joint); self.select_button.pack(side="left")
        ttk.Label(panel, textvariable=self.identity).pack(anchor="w")

        self.feedback_area = ttk.Frame(panel); self.feedback_area.pack(fill="x")
        self.single_panel = ttk.Frame(self.feedback_area); self.single_panel.pack(fill="x")
        self.leg_panel = LegHoldPanel(self.feedback_area, self.configure_leg)
        grid = ttk.LabelFrame(self.single_panel, text="Live feedback and command", padding=10); grid.pack(fill="x", pady=10)
        for i, (name, var) in enumerate(self.values.items()):
            r, c = divmod(i, 4)
            ttk.Label(grid, text=name).grid(row=2*r, column=c, sticky="w", padx=10)
            ttk.Label(grid, textvariable=var, font=("Segoe UI", 11, "bold")).grid(row=2*r+1, column=c, sticky="w", padx=10, pady=(0, 8))
            grid.columnconfigure(c, weight=1)
        row = ttk.Frame(self.single_panel); row.pack(fill="x")
        ttk.Label(row, text="HOME clearance [deg]").pack(side="left")
        self.home_entry = ttk.Entry(row, textvariable=self.home_margin, width=8); self.home_entry.pack(side="left", padx=5)
        ttk.Label(row, text="OTHER clearance [deg]").pack(side="left")
        self.other_entry = ttk.Entry(row, textvariable=self.other_margin, width=8); self.other_entry.pack(side="left", padx=5)
        self.config_button = ttk.Button(row, text="Apply clearances", command=self.configure); self.config_button.pack(side="left")
        ttk.Label(self.single_panel, textvariable=self.limits, wraplength=800).pack(anchor="w", pady=8)
        ttk.Checkbutton(panel, text="Leg is supported; ready for the selected test (single joint or all three holds)",
                        variable=self.supported, command=self.update_controls).pack(anchor="w")
        row = ttk.Frame(panel); row.pack(fill="x", pady=10)
        self.arm_button = ttk.Button(row, text="Enable hold here", command=lambda: self.send("ARM")); self.arm_button.pack(side="left")
        self.stop_button = ttk.Button(row, text="Stop / hold", command=lambda: self.send("STOP")); self.stop_button.pack(side="left", padx=5)
        self.off_button = ttk.Button(row, text="DISABLE PWM", command=lambda: self.send("OFF")); self.off_button.pack(side="left")
        ttk.Label(panel, textvariable=self.enable_reason, wraplength=800).pack(anchor="w", pady=(0, 8))
        self.move_row = ttk.Frame(panel); self.move_row.pack(fill="x", pady=(0, 8))
        row = self.move_row
        ttk.Label(row, text="Step [model deg]").pack(side="left")
        ttk.Entry(row, textvariable=self.step_deg, width=7).pack(side="left", padx=5)
        ttk.Label(row, text="Duration [s]").pack(side="left")
        ttk.Entry(row, textvariable=self.duration_s, width=7).pack(side="left", padx=5)
        self.minus_button = ttk.Button(row, text="Move -", command=lambda: self.move(-1)); self.minus_button.pack(side="left", padx=5)
        self.plus_button = ttk.Button(row, text="Move +", command=lambda: self.move(1)); self.plus_button.pack(side="left")
        self.motion_limits_label = ttk.Label(panel, textvariable=self.motion_limits, wraplength=800)
        self.motion_limits_label.pack(anchor="w", pady=(0, 4))
        ttk.Label(panel, textvariable=self.direction_help, wraplength=800).pack(anchor="w")
        ttk.Separator(panel).pack(fill="x", pady=10)
        row = ttk.Frame(panel); row.pack(fill="x")
        ttk.Label(row, text="Observed joint").pack(side="left")
        labels = ["Not tested"] + [f"{leg} {joint}" for leg in LEGS for joint in JOINTS]
        ttk.Combobox(row, textvariable=self.observed, values=labels, state="readonly", width=20).pack(side="left", padx=5)
        ttk.Combobox(row, textvariable=self.direction, values=("Not tested", "Matches prediction", "Opposite to prediction", "No movement"),
                     state="readonly", width=22).pack(side="left")
        ttk.Button(row, text="Export test record", command=self.export_record).pack(side="left", padx=5)
        row = ttk.Frame(panel); row.pack(fill="x", pady=8)
        ttk.Label(row, text="Notes").pack(side="left")
        ttk.Entry(row, textvariable=self.notes).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Label(panel, textvariable=self.message, wraplength=800, foreground="#154864").pack(anchor="w", pady=6)
        error_box = ttk.LabelFrame(panel, text="Last error - kept until cleared", padding=8)
        error_box.pack(fill="x", pady=(4, 0))
        self.clear_error_button = ttk.Button(error_box, text="Clear error", command=self.clear_error)
        self.clear_error_button.pack(side="right", padx=(8, 0))
        ttk.Label(error_box, textvariable=self.last_error, wraplength=680, foreground="#a12622").pack(side="left", fill="x", expand=True)
        self.update_controls()

    def refresh_ports(self):
        names = [port.device for port in self.ports_factory()]
        self.port_box["values"] = names
        if names and self.port_name.get() not in names:
            self.port_name.set(names[0])

    def toggle_connection(self):
        if self.client:
            self.disconnect("Disconnected; OFF requested. Board timeout is the fallback.")
            return
        try:
            if not self.port_name.get():
                raise ValueError("Choose the USB-to-UART COM port")
            self.client = BenchClient(self.serial_factory(self.port_name.get()))
            self.status = None
            self.supported.set(False)
            self.message.set("Connecting; outputs remain disabled...")
            self.handshake_job = self.root.after(self.connect_delay_ms, self.handshake)
        except Exception as error:
            self.show_error(str(error))
            self.disconnect(str(error))
        self.update_controls()

    def handshake(self):
        self.handshake_job = None
        if self.client:
            self.send("HELLO", callback=self.hello_reply)

    def hello_reply(self, reply):
        self.show_reply(reply)
        if reply["ok"]:
            if not self.duration_s.get():
                self.duration_s.set(f"{reply['settings']['min_move_s']:g}")
            self.select_joint()

    def selection(self):
        joint = -1 if self.control_scope.get() == "Whole leg hold" else JOINTS.index(self.joint.get())
        return LEGS.index(self.leg.get()), joint

    def select_joint(self):
        self.supported.set(False)
        self.home_margin.set("")
        self.other_margin.set("")
        self.leg_panel.clear()
        self.observed.set("Not tested")
        self.direction.set("Not tested")
        self.send("SELECT")

    def configure(self):
        try:
            home, other = float(self.home_margin.get()), float(self.other_margin.get())
            if home <= 0 or other <= 0 or not math.isfinite(home + other):
                raise ValueError("Enter positive finite clearances from both stops")
            self.send("CONFIG", home, other)
        except ValueError as error:
            self.show_error(str(error))

    def configure_leg(self):
        try:
            values = self.leg_panel.clearances()
        except ValueError as error:
            self.show_error(str(error))
            return
        #Only configuration is sent sequentially; never queue movements or enable.
        def apply(joint):
            def received(reply):
                self.show_reply(reply)
                if reply["ok"] and joint < 2 and self.client:
                    apply(joint + 1)
            self.send("LEG_CONFIG", *values[joint], callback=received, joint_override=joint)
        apply(0)

    def move(self, sign):
        try:
            step, duration = float(self.step_deg.get()), float(self.duration_s.get())
            if not self.status:
                raise ValueError("Connect and read a joint before requesting a move")
            limits = self.status["settings"]  #Actual uploaded C++ settings, not local copies.
            if not math.isfinite(step) or not 0 < step <= limits["max_step_deg"]:
                raise ValueError(f"Step must be greater than 0 and at most {limits['max_step_deg']:g} degrees (firmware limit).")
            if not math.isfinite(duration) or not limits["min_move_s"] <= duration <= limits["max_move_s"]:
                raise ValueError(f"Duration must be {limits['min_move_s']:g} to {limits['max_move_s']:g} seconds (firmware limits).")
            minimum_duration = 1.875 * step / limits["max_speed_deg_s"]
            if duration < minimum_duration:
                raise ValueError(f"This step needs at least {minimum_duration:.3f} s to keep quintic peak speed within "
                                 f"{limits['max_speed_deg_s']:g} deg/s. Increase Duration or reduce Step.")
            self.send("MOVE", sign * math.radians(step), duration)
        except ValueError as error:
            self.show_error(str(error))

    def send(self, operation, value1=0.0, value2=0.0, callback=None, joint_override=None):
        if not self.client:
            return
        try:
            #Commands use the board's acknowledged selection, not an unsubmitted dropdown.
            if operation in ("SELECT", "HELLO"):
                leg, joint = self.selection()
            elif self.status:
                leg, joint = self.status["leg"], self.status["joint"]
            else:
                leg, joint = -1, -1
            if operation == "SELECT" and joint == -1:
                operation = "LEG_SELECT"
            elif self.status and self.status.get("scope") == "leg" and operation in ("STATUS", "ARM", "STOP"):
                operation = "LEG_" + operation
            if joint_override is not None:
                joint = joint_override
            self.client.request(operation, leg, joint, callback or self.show_reply, value1, value2)
            if operation not in ("STATUS", "LEG_STATUS"):
                self.records.append({"time_utc": datetime.now(timezone.utc).isoformat(), "request": operation,
                                     "leg": leg, "joint": joint, "value1": value1, "value2": value2})
        except (BenchBusyError, ValueError) as error:
            self.show_error(str(error))  #A rejected click does not close the link.
        except Exception as error:
            self.show_error(str(error))
            self.disconnect(str(error))
        self.update_controls()

    def show_reply(self, reply):
        previous = self.status
        detail = reply["message"]
        if reply.get("scope") == "leg" and reply.get("problem_joint", -1) in range(3):
            detail = JOINTS[reply["problem_joint"]] + ": " + detail
        if not reply["ok"]:
            self.show_error("ESP32: " + detail)
        elif (previous and previous["mode"] != "disabled" and reply["mode"] == "disabled"
              and reply["message"] not in ("PWM disabled", "Arming cancelled", "Connected; outputs disabled", "Reading feedback; PWM disabled")):
            self.show_error("ESP32 disabled output: " + detail)
        if reply.get("session_active") is False:
            self.disconnect(reply["message"] + "; reconnect to continue.")
            return  #Do not keep polling an expired session and repeating errors.
        self.status = reply
        self.last_reply = time.monotonic()
        settings = reply["settings"]
        self.next_poll = self.last_reply + settings["status_interval_ms"] / 1000.0
        self.motion_limits.set(f"Firmware limits: step <= {settings['max_step_deg']:g} deg | "
                               f"peak speed <= {settings['max_speed_deg_s']:g} deg/s | "
                               f"duration {settings['min_move_s']:g}-{settings['max_move_s']:g} s")
        self.message.set(detail)
        self.records.append({"time_utc": datetime.now(timezone.utc).isoformat(), "status": dict(reply)})
        self.records = self.records[-2000:]
        if reply.get("scope") == "leg":
            self.identity.set(f"{LEGS[reply['leg']]} whole leg | roll + hip pitch + knee | state: {reply['mode'].upper()}")
            self.motion_limits.set("Whole-leg hold only. There are no Move or walking commands in this mode yet.")
            self.direction_help.set("Enable captures all three measured angles once; later feedback does not change those targets.")
            self.leg_panel.show_reply(reply)
            self.update_controls()
            return
        if reply["leg"] >= 0:
            self.identity.set(f"{LEGS[reply['leg']]} {JOINTS[reply['joint']]} | S{reply['servo']:02d} | "
                              f"PWM GPIO{reply['pwm_gpio']} | feedback GPIO{reply['feedback_gpio']} | 50 Hz")
        direction = "increase" if reply["sigma"] == 1 else "decrease"
        self.direction_help.set(f"Increasing mathematical q should {direction} physical servo a (sigma = {reply['sigma']:+d})."
                                if reply["sigma"] else "No installed joint mapping is available for this selection.")
        def fmt(key, unit, degrees=False):
            value = reply[key]
            if value is None:
                return "--"
            return f"{math.degrees(value) if degrees else value:.3f} {unit}"
        entries = {"Voltage": fmt("V", "V"), "Spread": fmt("spread_mV", "mV"),
                   "Measured a": fmt("a_measured_deg", "deg"), "Measured q": fmt("q_measured_rad", "deg", True),
                   "Command a": fmt("a_command_deg", "deg"), "Command q": fmt("q_command_rad", "deg", True),
                   "Pulse": fmt("pulse_us", "us"), "State": reply["mode"].upper()}
        if not reply["feedback_valid"]:
            entries["Measured a"] = entries["Measured q"] = "INVALID / STALE"
        for name, value in entries.items():
            self.values[name].set(value)
        if reply["configured"]:
            self.limits.set(f"Usable a: {reply['a_min_deg']:.2f} to {reply['a_max_deg']:.2f} deg | "
                            f"usable q: {math.degrees(reply['q_min_rad']):.2f} to {math.degrees(reply['q_max_rad']):.2f} deg")
        else:
            self.limits.set("Clearances not applied. These are servo degrees inward from HOME and OTHER END.")
        self.update_controls()

    def update_controls(self):
        whole = self.control_scope.get() == "Whole leg hold"
        if self.displayed_scope != whole:
            self.displayed_scope = whole
            if whole:
                self.single_panel.pack_forget()
                self.leg_panel.pack(fill="x")
                self.move_row.pack_forget()
            else:
                self.leg_panel.pack_forget()
                self.single_panel.pack(fill="x")
                self.move_row.pack(fill="x", pady=(0, 8), before=self.motion_limits_label)
            self.root.minsize(1000 if whole else 850, 800 if whole else 750)
        ready = bool(self.client and self.client.ready)
        available = ready and self.client.action_available
        fresh = bool(self.status and time.monotonic() - self.last_reply < self.status["settings"]["reply_timeout_ms"] / 1000.0)
        selected = fresh and self.selection() == (self.status["leg"], self.status["joint"])
        disabled = not fresh or self.status["mode"] == "disabled"
        holding = fresh and self.status["mode"] == "holding"
        leg_reason = self.leg_panel.blocking_reason(self.status) if selected and whole else None
        inside = (leg_reason is None) if selected and whole else bool(selected and self.status["configured"] and self.status["feedback_valid"]
                      and self.status["q_min_rad"] <= self.status["q_measured_rad"] <= self.status["q_max_rad"])
        def configure(widget, **options):
            #Do not repeatedly redraw unchanged controls or disturb an open dropdown.
            for key, value in options.items():
                if str(widget[key]) != str(value):
                    widget.configure(**{key: value})
        def state(widget, allowed):
            configure(widget, state="normal" if allowed else "disabled")
        configure(self.connect_button, text="Disconnect" if self.client else "Connect")
        configure(self.port_box, state="disabled" if self.client else "readonly")
        configure(self.scope_box, state="readonly" if disabled and (not ready or available) else "disabled")
        configure(self.leg_box, state="readonly" if disabled else "disabled")
        configure(self.joint_box, state="readonly" if disabled and not whole else "disabled")
        state(self.select_button, available and disabled)
        state(self.config_button, available and disabled and selected and not whole)
        state(self.leg_panel.apply_button, available and disabled and selected and whole)
        for entry in self.leg_panel.entries:
            state(entry, disabled)
        state(self.home_entry, disabled)
        state(self.other_entry, disabled)
        state(self.arm_button, available and selected and disabled and self.status["allowed"] and inside and self.supported.get())
        state(self.plus_button, available and selected and holding and not whole)
        state(self.minus_button, available and selected and holding and not whole)
        state(self.stop_button, ready and fresh and not disabled)
        state(self.off_button, bool(self.client))
        if not ready:
            reason = "Enable unavailable: connect first."
        elif not fresh:
            reason = "Enable unavailable: waiting for fresh ESP32 status."
        elif not selected:
            reason = "Click Read selected joint to apply the dropdown selection."
        elif not disabled:
            reason = "PWM is active. Disable it before selecting another joint."
        elif whole and leg_reason:
            reason = leg_reason
        elif not self.status["allowed"]:
            reason = "Read-only joint: bench_joint_allowed is 0 in include/servo_bench_config.h."
        elif not self.status["configured"]:
            reason = "Enable unavailable: enter both clearances and click Apply clearances."
        elif not self.status["feedback_valid"]:
            reason = "Enable unavailable: feedback is invalid or stale; check the voltage and calibration."
        elif not inside:
            reason = "Enable unavailable: measured angle is outside the displayed usable range."
        elif not self.supported.get():
            reason = "Enable unavailable: tick the supported-test acknowledgement."
        elif not available:
            reason = "Waiting for the previous command reply."
        else:
            reason = "Ready to enable a hold at the measured position."
        if self.enable_reason.get() != reason:
            self.enable_reason.set(reason)

    def tick(self):
        self.tick_job = None
        try:
            if self.client:
                self.client.poll()
                if self.client and self.client.ready and self.client.idle and self.status and self.status["leg"] >= 0:
                    if time.monotonic() >= self.next_poll:
                        self.send("STATUS")
        except Exception as error:
            self.show_error(str(error))
            self.disconnect(str(error))
        self.update_controls()
        self.tick_job = self.root.after(20, self.tick)

    def disconnect(self, message):
        if self.handshake_job:
            self.root.after_cancel(self.handshake_job)
            self.handshake_job = None
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.status = None
        self.supported.set(False)
        self.leg_panel.clear()
        for value in self.values.values():
            value.set("--")
        self.message.set(message)
        self.motion_limits.set("Motion limits will be read from the connected ESP32.")
        self.update_controls()

    def show_error(self, message):
        #Live status can change freely; only a new error or Clear changes this area.
        if message == self.last_error_detail:
            return
        self.last_error_detail = message
        self.last_error.set(f"{datetime.now().strftime('%H:%M:%S')}  {message}")
        self.records.append({"time_utc": datetime.now(timezone.utc).isoformat(), "error": message})

    def clear_error(self):
        self.last_error_detail = ""
        self.last_error.set("No errors.")  #Acknowledgement only; never enables or restarts PWM.

    def export_data(self):
        return {"format": "servo-bench-observation/1", "exported_utc": datetime.now(timezone.utc).isoformat(),
                "observed_joint": self.observed.get(), "observed_direction": self.direction.get(), "notes": self.notes.get(),
                "meaning": "Operator observations, not calibration or automatic physical verification",
                "last_error": self.last_error.get(),
                "records": self.records}

    def export_record(self):
        if self.status and self.status["mode"] != "disabled":
            self.show_error("Disable PWM before opening the export dialog")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="servo_bench_observation.json",
                                          filetypes=[("JSON record", "*.json")])
        if path:
            try:
                Path(path).write_text(json.dumps(self.export_data(), indent=2), encoding="utf-8")
                self.message.set(f"Saved observation: {path}")
            except OSError as error:
                self.show_error(str(error))

    def close(self):
        self.disconnect("Closed")
        if self.tick_job:
            self.root.after_cancel(self.tick_job)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    ServoBenchApp(root)
    root.mainloop()
