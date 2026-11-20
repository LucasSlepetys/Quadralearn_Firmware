"""No physical port is opened. Tk buttons talk to the actual C++ code with a fake ADC."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from capture_client import CaptureClient, validate_joints
from calibration_ui import CalibrationApp


class FirmwarePort:
    """Test-only serial replacement backed by the compiled firmware's desktop harness."""
    def __init__(self):
        self.process = subprocess.Popen([str(ROOT / ".pio/desktop_tests/whole_leg_test.exe"), "--bridge"],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.buffer = bytearray(self.process.stdout.readline())
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size):
        data = self.buffer[:size]
        del self.buffer[:size]
        return bytes(data)

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()
        response = self.process.stdout.readline()
        if not response:
            raise OSError("Test firmware exited unexpectedly.")
        self.buffer.extend(response)
        return len(data)

    def voltage(self, gpio, mV):
        self.process.stdin.write(f"TEST {gpio} {mV}\n".encode())
        self.process.stdin.flush()
        if self.process.stdout.readline().strip() != b"TEST_OK":
            raise AssertionError("Fake ADC setup failed.")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.process.stdin.close()
        self.process.wait(timeout=3)
        self.process.stdout.close()
        self.process.stderr.close()


class DummyPort:
    def __init__(self):
        self.buffer = bytearray()
        self.sent = []

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, size):
        data = self.buffer[:size]
        del self.buffer[:size]
        return data

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def close(self):
        pass


class ClientTests(unittest.TestCase):
    def test_timeout_never_retries_accept(self):
        port = DummyPort()
        clock = [0.0]
        client = CaptureClient(port, clock=lambda: clock[0])
        client.request("ACCEPT", 0, lambda _: None, 7)
        client.poll()
        clock[0] = 5.0
        with self.assertRaises(TimeoutError):
            client.poll()
        self.assertEqual(len(port.sent), 1)

    def test_wrong_response_leg_and_missing_fields_rejected(self):
        for reply in ({"type": "hello", "ok": True, "leg": 1}, {"type": "hello", "leg": 0}):
            port = DummyPort()
            client = CaptureClient(port)
            client.request("HELLO", 0, lambda _: self.fail("Invalid reply reached UI"))
            client.poll()
            port.buffer.extend(json.dumps(dict(protocol="leg-capture/1", id=1, **reply)).encode() + b"\n")
            with self.assertRaises(ValueError):
                client.poll()

    def test_fragments_and_old_reply_ids(self):
        port = DummyPort()
        replies = []
        client = CaptureClient(port)
        client.request("HELLO", 0, replies.append)
        client.poll()
        packet = json.dumps(dict(protocol="leg-capture/1", id=1, type="hello", ok=True, leg=0)).encode() + b"\n"
        port.buffer.extend(packet[:20]); client.poll()
        self.assertFalse(replies)
        port.buffer.extend(packet[20:]); client.poll()
        self.assertTrue(client.ready)
        port.buffer.extend(packet); client.poll()
        self.assertEqual(len(replies), 1)

    def test_board_restart_invalidates_session(self):
        port = DummyPort(); client = CaptureClient(port); client.ready = True
        port.buffer.extend(b'{"protocol":"leg-capture/1","id":0,"type":"boot"}\n')
        with self.assertRaises(ConnectionError):
            client.poll()

    def test_nonfinite_or_incomplete_rows_rejected(self):
        with self.assertRaises(ValueError):
            validate_joints([])
        row = dict(valid=True, servo=1, gpio=4, mean_V=float("nan"), min_V=1, max_V=1, spread_mV=0, angle_deg=90)
        with self.assertRaises(ValueError):
            validate_joints([row] * 3)


class InterfaceTests(unittest.TestCase):
    def setUp(self):
        self.port = FirmwarePort()
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = CalibrationApp(self.root, serial_factory=lambda _: self.port,
                                  ports_factory=lambda: [], connect_delay_ms=0)
        self.app.live.set(False)
        self.app.port_name.set("TEST — simulated firmware")
        self.app.connect_button.invoke()
        self.root.update()
        self.pump()
        self.assertTrue(self.app.ready)

    def tearDown(self):
        self.app.close()
        self.port.close()

    def pump(self):
        for _ in range(40):
            if self.app.client:
                self.app.client.poll()
            self.root.update_idletasks()
            if not self.app.client or self.app.client.idle:
                return
        self.fail("Interface did not finish queued requests")

    def values(self, table, joint=0):
        return table.item(str(joint), "values")

    def test_live_preview_accept_uses_exact_cpp_snapshot(self):
        self.app.read_button.invoke(); self.pump()
        self.assertEqual(self.values(self.app.live_table)[1], "S01 / 4")
        self.app.home_button.invoke(); self.pump()
        preview = self.values(self.app.preview_table)
        self.assertEqual(str(self.app.accept_button['state']), "normal")
        self.port.voltage(4, 2152)
        self.app.read_button.invoke(); self.pump()
        self.assertNotEqual(self.values(self.app.live_table)[2], preview[2])
        self.assertEqual(self.values(self.app.preview_table), preview)
        self.app.accept_button.invoke(); self.pump()
        self.assertEqual(self.values(self.app.saved_table)[1], preview[2])
        self.assertEqual(self.values(self.app.saved_table)[2], preview[6])
        self.assertEqual(str(self.app.accept_button['state']), "disabled")

    def test_rejected_joint_visible_and_accept_disabled(self):
        self.port.voltage(5, 0)
        self.app.other_button.invoke(); self.pump()
        self.assertFalse(self.app.preview['ok'])
        self.assertEqual(self.values(self.app.preview_table, 1)[2], "0.0000")
        self.assertEqual(self.values(self.app.preview_table, 1)[7], "INVALID")
        self.assertEqual(str(self.app.accept_button['state']), "disabled")
        self.assertEqual(self.values(self.app.saved_table)[1], "Not recorded")

    def test_all_legs_and_selection_cancels_pending(self):
        self.app.home_button.invoke(); self.pump()
        self.app.leg_name.set("FR"); self.app.select_leg(); self.pump()
        self.assertIsNone(self.app.preview)
        self.assertEqual(str(self.app.accept_button['state']), "disabled")
        for leg, expected in (("FR", "S04 / 1"), ("RL", "S07 / 17"), ("RR", "S10 / 20")):
            self.app.leg_name.set(leg); self.app.select_leg(); self.pump()
            self.app.read_button.invoke(); self.pump()
            self.assertEqual(self.values(self.app.live_table)[1], expected)
            self.app.other_button.invoke(); self.pump()
            self.app.accept_button.invoke(); self.pump()
            self.assertNotEqual(self.values(self.app.saved_table)[3], "Not recorded")

    def test_discard_and_disconnect_clear_accept(self):
        self.app.home_button.invoke(); self.pump()
        self.app.cancel_button.invoke(); self.pump()
        self.assertIsNone(self.app.preview)
        self.app.home_button.invoke(); self.pump()
        self.app.connect_button.invoke()
        self.assertFalse(self.app.ready)
        self.assertEqual(str(self.app.accept_button['state']), "disabled")
        self.assertEqual(self.values(self.app.live_table)[2], "—")

    def test_export_fetches_all_four_legs_from_board(self):
        self.app.home_button.invoke(); self.pump()
        self.app.accept_button.invoke(); self.pump()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "endpoints.json"
            with patch("calibration_ui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.export_button.invoke(); self.pump()
            data = json.loads(path.read_text())
            self.assertEqual(set(data['legs']), {'FL', 'FR', 'RL', 'RR'})
            self.assertEqual(len(data['legs']['FL']['home']), 3)
            self.assertIsNone(data['legs']['RR']['home'])

    def test_window_layout_fits_requested_size(self):
        self.root.deiconify()
        self.root.update_idletasks()
        for widget in (self.app.live_table, self.app.preview_table, self.app.saved_table, self.app.export_button):
            bottom = widget.winfo_rooty() - self.root.winfo_rooty() + widget.winfo_height()
            right = widget.winfo_rootx() - self.root.winfo_rootx() + widget.winfo_width()
            self.assertLessEqual(bottom, self.root.winfo_height())
            self.assertLessEqual(right, self.root.winfo_width())
        self.root.withdraw()


if __name__ == "__main__":
    unittest.main(verbosity=2)
