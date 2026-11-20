"""Whole-leg readings and per-joint clearances inside the existing bench window.
No serial connection, motor commands or calibration maths live in this panel.
"""
import math
import tkinter as tk
from tkinter import ttk

JOINTS = ("Hip roll", "Hip pitch", "Knee pitch")


class LegHoldPanel(ttk.Frame):
    def __init__(self, parent, apply_command):
        super().__init__(parent)
        self.home = [tk.StringVar() for _ in JOINTS]
        self.other = [tk.StringVar() for _ in JOINTS]
        self.entries = []
        self.values = []
        self.ranges = []
        headers = ("Joint", "Voltage", "Measured a", "Measured q", "Command q", "Pulse", "HOME gap", "OTHER gap")
        for col, title in enumerate(headers):
            ttk.Label(self, text=title).grid(row=0, column=col, padx=4, pady=4, sticky="w")
        for j, name in enumerate(JOINTS):
            row = 1 + 2*j
            ttk.Label(self, text=name).grid(row=row, column=0, sticky="w", padx=4)
            values = {key: tk.StringVar(value="--") for key in ("V", "a_measured_deg", "q_measured_rad", "q_command_rad", "pulse_us")}
            for col, var in enumerate(values.values(), 1):
                ttk.Label(self, textvariable=var, width=12).grid(row=row, column=col, sticky="w", padx=4)
            self.values.append(values)
            for col, var in ((6, self.home[j]), (7, self.other[j])):
                entry = ttk.Entry(self, textvariable=var, width=8)
                entry.grid(row=row, column=col, padx=4)
                self.entries.append(entry)
            info = tk.StringVar(value="Clearances not applied.")
            self.ranges.append(info)
            ttk.Label(self, textvariable=info).grid(row=row+1, column=0, columnspan=8, sticky="w", padx=4, pady=(0, 8))
        self.apply_button = ttk.Button(self, text="Apply all three clearances", command=apply_command)
        self.apply_button.grid(row=7, column=0, columnspan=3, sticky="w", padx=4, pady=5)
        ttk.Label(self, text="Gaps: physical servo degrees inward from each stop. No defaults are assumed.").grid(
            row=8, column=0, columnspan=8, sticky="w", padx=4, pady=(0, 8))

    def clear(self):
        for var in self.home + self.other:
            var.set("")
        for values in self.values:
            for var in values.values():
                var.set("--")
        for var in self.ranges:
            var.set("Clearances not applied.")

    def clearances(self):
        values = []
        for j, name in enumerate(JOINTS):
            try:
                home, other = float(self.home[j].get()), float(self.other[j].get())
            except ValueError:
                raise ValueError(f"{name}: enter both clearance values.") from None
            if not math.isfinite(home + other) or home <= 0 or other <= 0:
                raise ValueError(f"{name}: clearances must be positive finite servo degrees.")
            values.append((home, other))
        return values

    def show_reply(self, reply):
        for j, item in enumerate(reply["joints"]):
            for key, var in self.values[j].items():
                value = item[key]
                if value is not None and key.startswith("q_"):
                    value = math.degrees(value)
                unit = "V" if key == "V" else "us" if key == "pulse_us" else "deg"
                text = "--" if value is None else f"{value:.2f} {unit}"
                if var.get() != text:
                    var.set(text)
            prefix = f"S{item['servo']:02d} | PWM{item['pwm_gpio']} / FB{item['feedback_gpio']} | "
            if item["configured"]:
                text = prefix + f"usable q: {math.degrees(item['q_min_rad']):.2f} to {math.degrees(item['q_max_rad']):.2f} deg"
            else:
                text = prefix + "clearances not applied"
            self.ranges[j].set(text)

    @staticmethod
    def blocking_reason(reply):
        for j, item in enumerate(reply["joints"]):
            if not item["allowed"]:
                return f"{JOINTS[j]}: read-only; bench_joint_allowed is 0 in servo_bench_config.h."
            if not item["configured"]:
                return f"{JOINTS[j]}: enter clearances, then Apply all three clearances."
        if not reply["feedback_valid"]:
            j = reply["problem_joint"]
            prefix = JOINTS[j] if j in range(3) else "Whole leg"
            return f"{prefix}: feedback invalid or stale; check the voltage and calibration."
        for j, item in enumerate(reply["joints"]):
            if not item["q_min_rad"] <= item["q_measured_rad"] <= item["q_max_rad"]:
                return f"{JOINTS[j]}: measured angle is outside the displayed usable range."
        return None
