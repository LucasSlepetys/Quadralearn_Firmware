"""Small USB request/reply client. No calibration or joint maths lives here."""
from collections import deque
import json
import math
import time

PROTOCOL = "leg-capture/1"
REPLY_TYPES = {"HELLO": "hello", "READ": "reading", "HOME": "preview",
               "OTHER": "preview", "ACCEPT": "accepted", "CANCEL": "cancelled", "GET": "records",
               "POSE": "pose"}


def validate_joints(rows):
    if not isinstance(rows, list) or len(rows) != 3:
        raise ValueError("Expected three joint readings from the ESP32.")
    for row in rows:
        if not isinstance(row, dict) or type(row.get("valid")) is not bool:
            raise ValueError("Invalid joint data.")
        for key in ("servo", "gpio"):
            if type(row.get(key)) is not int:
                raise ValueError("Missing joint identity.")
        for key in ("mean_V", "min_V", "max_V", "spread_mV", "angle_deg"):
            value = row.get(key)
            if value is None and not row["valid"]:
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("Invalid numerical data from the ESP32.")


def validate_pose(reply):
    """Check the additional POSE fields; existing calibration replies are unchanged."""
    validate_joints(reply.get("joints"))
    if (reply.get("frame"), reply.get("angle_unit"), reply.get("length_unit")) != ("H", "rad", "m"):
        raise ValueError("Unexpected pose frame or units.")
    leg = reply.get("leg")
    if type(leg) is not int or leg not in range(4):
        raise ValueError("Invalid pose leg identity.")
    if [row["servo"] for row in reply["joints"]] != list(range(3 * leg + 1, 3 * leg + 4)):
        raise ValueError("Pose servo identities do not match the selected leg.")
    if not reply["ok"]:
        if reply.get("q_rad") is not None or reply.get("points_H_m") is not None:
            raise ValueError("Invalid pose must not supply reconstructed points.")
        return
    if not all(row["valid"] for row in reply["joints"]):
        raise ValueError("Pose contains an invalid joint reading.")
    radius = reply.get("ball_radius_m")
    if type(radius) not in (int, float) or not math.isfinite(radius) or radius <= 0:
        raise ValueError("Invalid foot radius.")
    points = reply.get("points_H_m")
    if not isinstance(points, dict) or set(points) != {"H", "P", "K", "F"}:
        raise ValueError("Expected H/P/K/F pose points.")
    for vector in (reply.get("q_rad"), *points.values()):
        if not isinstance(vector, list) or len(vector) != 3:
            raise ValueError("Expected three coordinates or joint angles.")
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in vector):
            raise ValueError("Invalid pose numerical data.")


class CaptureClient:
    """One outstanding request at a time; no automatic retry of captures/accepts."""
    def __init__(self, port, clock=time.monotonic):
        self.port = port
        self.clock = clock
        self.queue = deque()
        self.pending = None
        self.buffer = bytearray()
        self.next_id = 1
        self.ready = False

    @property
    def idle(self):
        return self.pending is None and not self.queue

    def request(self, operation, leg, callback, token=0):
        if operation not in REPLY_TYPES or leg not in range(4):
            raise ValueError("Invalid interface request.")
        self.queue.append((operation, leg, token, callback))

    def close(self):
        self.queue.clear()
        self.pending = None
        self.ready = False
        self.port.close()

    def poll(self):
        waiting = self.port.in_waiting
        if waiting:
            self.buffer.extend(self.port.read(min(waiting, 8192)))
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            if len(line) > 4096:
                raise ValueError("Oversized response from the board.")
            try:
                reply = json.loads(line)
            except (ValueError, UnicodeError):
                if line.startswith(b"{"):
                    raise ValueError("Damaged response from the ESP32.")
                continue  #ESP32 boot logs may precede the protocol handshake.
            if not isinstance(reply, dict) or reply.get("protocol") != PROTOCOL:
                continue
            if reply.get("type") == "boot":
                if self.ready:
                    raise ConnectionError("The board restarted. Reconnect; RAM records may have cleared.")
                continue
            if reply.get("id") == 0 and reply.get("type") == "error":
                raise ValueError(reply.get("message", "Invalid USB request."))
            if self.pending is None or reply.get("id") != self.pending[0]:
                continue  #A late reply must not accept a different request.
            request_id, operation, leg, token, callback, _ = self.pending
            if type(reply.get("ok")) is not bool:
                raise ValueError("Missing acknowledgement from the ESP32.")
            if reply.get("type") != "error":
                if reply.get("type") != REPLY_TYPES[operation] or reply.get("leg") != leg:
                    raise ValueError("Reply does not match the requested leg or action.")
                if operation in ("READ", "HOME", "OTHER"):
                    validate_joints(reply.get("joints"))
                if operation == "POSE":
                    validate_pose(reply)
                if operation in ("HOME", "OTHER"):
                    expected = "home" if operation == "HOME" else "other"
                    if reply.get("endpoint") != expected or type(reply.get("token")) is not int or reply["token"] <= 0:
                        raise ValueError("Invalid preview identity.")
                    if reply["ok"] and not all(row["valid"] for row in reply["joints"]):
                        raise ValueError("Incomplete whole-leg preview.")
                if operation == "ACCEPT" and reply.get("token") != token:
                    raise ValueError("Accepted preview does not match the one displayed.")
                if operation == "GET":
                    for endpoint in ("home", "other"):
                        if endpoint not in reply:
                            raise ValueError("Incomplete saved-record response.")
                        if reply[endpoint] is not None:
                            validate_joints(reply[endpoint])
            self.pending = None
            if operation == "HELLO" and reply["ok"]:
                self.ready = True
            callback(reply)
        if len(self.buffer) > 4096:
            raise ValueError("Unterminated response from the board.")
        if self.pending and self.clock() - self.pending[-1] > 4.0:
            raise TimeoutError("No reply from the board. Check the port and upload the new interface firmware.")
        if self.pending is None and self.queue:
            operation, leg, token, callback = self.queue.popleft()
            request_id = self.next_id
            self.next_id += 1
            message = f"QLCAP1 {request_id} {operation} {leg} {token}\n".encode("ascii")
            if self.port.write(message) != len(message):
                raise ConnectionError("USB request was not sent completely.")
            self.pending = (request_id, operation, leg, token, callback, self.clock())
