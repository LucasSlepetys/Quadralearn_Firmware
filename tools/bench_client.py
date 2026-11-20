"""Bench-only request/reply client. No calibration maths or automatic motion retries."""
import json
import math
import time

PROTOCOL = "servo-bench/1"
OPERATIONS = {"HELLO", "SELECT", "STATUS", "CONFIG", "ARM", "MOVE", "STOP", "OFF", "LEG_SELECT", "LEG_STATUS", "LEG_CONFIG", "LEG_ARM", "LEG_STOP"}


class BenchBusyError(RuntimeError):
    """An action is pending; ignore this extra click without disconnecting."""


def validate_settings(settings):
    if not isinstance(settings, dict):
        raise ValueError("Firmware does not report bench settings. Upload the updated firmware, then reconnect.")
    for key in ("max_step_deg", "max_speed_deg_s", "min_move_s", "max_move_s",
                "status_interval_ms", "reply_timeout_ms", "link_timeout_ms", "feedback_timeout_ms"):
        value = settings.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"Invalid firmware setting: {key}. Check servo_bench_config.h.")
    if settings["min_move_s"] > settings["max_move_s"]:
        raise ValueError("Firmware minimum duration exceeds maximum. Check servo_bench_config.h.")
    if not (settings["status_interval_ms"] < settings["reply_timeout_ms"] < settings["link_timeout_ms"]):
        raise ValueError("Set status interval < reply timeout < link timeout in servo_bench_config.h.")


def validate_reply(reply):
    validate_settings(reply.get("settings"))
    if reply.get("scope") == "leg":
        if type(reply.get("id")) is not int or type(reply.get("ok")) is not bool:
            raise ValueError("Invalid whole-leg reply")
        if type(reply.get("session_active")) is not bool or reply.get("mode") not in ("disabled", "arming", "holding"):
            raise ValueError("Invalid whole-leg session/state")
        leg = reply.get("leg")
        if type(leg) is not int or leg not in range(4) or reply.get("joint") != -1:
            raise ValueError("Invalid whole-leg selection")
        if type(reply.get("feedback_age_ms")) is not int or type(reply.get("problem_joint")) is not int or reply["problem_joint"] not in (-1, 0, 1, 2):
            raise ValueError("Invalid whole-leg timestamp or problem joint")
        joints = reply.get("joints")
        if not isinstance(joints, list) or len(joints) != 3:
            raise ValueError("Incomplete whole-leg status")
        for j, item in enumerate(joints):
            if not isinstance(item, dict) or item.get("leg") != leg or item.get("joint") != j:
                raise ValueError("Whole-leg joint identity mismatch")
            child = dict(item, id=reply["id"], ok=reply["ok"], mode=reply["mode"],
                         feedback_age_ms=reply["feedback_age_ms"], settings=reply["settings"])
            validate_reply(child)
        for key in ("allowed", "configured", "enabled", "feedback_valid"):
            if type(reply.get(key)) is not bool or reply[key] != all(j[key] for j in joints):
                raise ValueError("Inconsistent whole-leg status")
        return
    if "session_active" in reply and type(reply["session_active"]) is not bool:
        raise ValueError("Invalid bench session status")
    if type(reply.get("ok")) is not bool or reply.get("mode") not in ("disabled", "arming", "holding", "moving"):
        raise ValueError("Invalid bench status")
    for key in ("id", "leg", "joint", "servo", "pwm_gpio", "feedback_gpio", "feedback_age_ms"):
        if type(reply.get(key)) is not int:
            raise ValueError("Missing bench identity or timestamp")
    leg, joint = reply["leg"], reply["joint"]
    if (leg, joint) != (-1, -1) and (leg not in range(4) or joint not in range(3) or reply["servo"] != 3*leg + joint + 1):
        raise ValueError("Bench servo identity mismatch")
    for key in ("allowed", "configured", "enabled", "feedback_valid"):
        if type(reply.get(key)) is not bool:
            raise ValueError("Missing bench state")
    for key in ("V", "spread_mV", "a_measured_deg", "q_measured_rad", "q_command_rad", "a_command_deg",
                "pulse_us", "q_min_rad", "q_max_rad", "a_min_deg", "a_max_deg", "frequency_hz"):
        value = reply.get(key)
        if key not in reply or (value is not None and (type(value) not in (int, float) or not math.isfinite(value))):
            raise ValueError("Invalid bench number")
    if reply["frequency_hz"] != 50:
        raise ValueError("Unexpected PWM frequency")
    if type(reply.get("sigma")) is not int or reply["sigma"] not in (-1, 0, 1):
        raise ValueError("Missing direction mapping")
    if reply["feedback_valid"] and any(reply[k] is None for k in ("V", "a_measured_deg", "q_measured_rad")):
        raise ValueError("Missing measured angles")
    if reply["enabled"] and (reply["mode"] not in ("holding", "moving") or reply["pulse_us"] is None):
        raise ValueError("Inconsistent enabled status")


class BenchClient:
    def __init__(self, port, clock=time.monotonic):
        self.port = port
        self.clock = clock
        self.next_id = 1
        self.pending = None
        self.buffer = bytearray()
        self.ready = False
        self.reply_timeout_s = 3.0  #Connection bootstrap only; HELLO supplies the C++ value.

    @property
    def idle(self):
        return self.pending is None

    @property
    def action_available(self):
        #A live reading must not block a button click. Actions are never queued.
        return self.idle or self.pending[1] in ("STATUS", "LEG_STATUS")

    def request(self, operation, leg, joint, callback, value1=0.0, value2=0.0):
        if operation not in OPERATIONS or not all(math.isfinite(v) for v in (value1, value2)):
            raise ValueError("Invalid bench request")
        if not self.idle and operation not in ("OFF", "STOP", "LEG_STOP"):
            if not self.action_available or operation in ("STATUS", "LEG_STATUS"):
                raise BenchBusyError("Previous command is awaiting its reply; no extra command was sent.")
        #A click replaces pending STATUS; STOP/OFF can also replace an action.
        #Late replies are discarded by ID. No movement is retried or queued.
        request_id = self.next_id
        self.next_id += 1
        #Preserve the float: rounding 10 degrees upward could exceed an exact C++ limit.
        data = f"QLBENCH1 {request_id} {operation} {leg} {joint} {value1:.17g} {value2:.17g}\n".encode("ascii")
        if len(data) >= 96:
            raise ValueError("Bench request too long")
        if self.port.write(data) != len(data):
            raise OSError("Incomplete serial write")
        self.pending = (request_id, operation, leg, joint, callback, self.clock())

    def poll(self):
        if self.port.in_waiting:
            self.buffer.extend(self.port.read(min(self.port.in_waiting, 4096)))
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            if len(line) > 4096:
                raise ValueError("Oversized bench response")
            try:
                reply = json.loads(line)
            except (ValueError, UnicodeError):
                if line.startswith(b"{"):
                    raise ValueError("Damaged bench response")
                continue
            if not isinstance(reply, dict):
                continue
            if reply.get("protocol") == "leg-capture/1" and reply.get("type") == "boot" and self.ready:
                raise ConnectionError("ESP32 restarted; reconnect")
            if reply.get("protocol") != PROTOCOL:
                continue
            if self.pending is None or reply.get("id") != self.pending[0]:
                continue  #Late replies never acknowledge a newer movement.
            request_id, operation, leg, joint, callback, _ = self.pending
            validate_reply(reply)
            self.reply_timeout_s = reply["settings"]["reply_timeout_ms"] / 1000.0
            expected_joint = -1 if operation.startswith("LEG_") else joint
            if reply["ok"] and operation not in ("HELLO", "OFF") and (reply["leg"], reply["joint"]) != (leg, expected_joint):
                raise ValueError("Reply belongs to another joint/leg")
            self.pending = None
            if operation == "HELLO":
                self.ready = reply["ok"]
            if reply.get("session_active") is False:
                self.ready = False
            callback(reply)
        if len(self.buffer) > 4096:
            raise ValueError("Unterminated bench response")
        if self.pending and self.clock() - self.pending[-1] >= self.reply_timeout_s:
            raise TimeoutError("No bench reply; output will disable on the ESP32 timeout")

    def close(self):
        try:
            self.request("OFF", -1, -1, lambda reply: None)
        except Exception:
            pass  #ESP32 also enforces bench_link_timeout_ms independently.
        finally:
            self.pending = None
            self.ready = False
            self.port.close()
