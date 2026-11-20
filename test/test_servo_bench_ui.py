"""Exercise Python controls against the real C++ bench with fake ADC/PWM.
No COM port is opened. Passing tests do not verify actual motion or wiring.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tkinter as tk
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from bench_client import BenchClient, BenchBusyError, validate_reply, validate_settings
from servo_bench_ui import ServoBenchApp


class FirmwarePort:
    def __init__(self):
        self.process = subprocess.Popen([str(ROOT / ".pio/desktop_tests/servo_bench_test.exe"), "--bridge"],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.buffer = bytearray(self.process.stdout.readline())
        self.closed = False
        self.sent = []

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, count):
        data = self.buffer[:count]
        del self.buffer[:count]
        return bytes(data)

    def write(self, data):
        self.sent.append(data)
        self.process.stdin.write(data); self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise OSError("Fake firmware stopped")
        self.buffer.extend(line)
        return len(data)

    def control(self, command, expected):
        self.process.stdin.write((command + "\n").encode()); self.process.stdin.flush()
        if self.process.stdout.readline().strip() != expected:
            raise AssertionError("Fake hardware command failed")

    def tick(self, ms):
        self.control(f"TICK {ms}", b"TICK_OK")

    def voltage(self, mV):
        self.control(f"TEST 19 {mV}", b"TEST_OK")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.process.stdin.close()
        self.process.wait(timeout=3)
        self.process.stdout.close(); self.process.stderr.close()


class BenchProtocolTests(unittest.TestCase):
    def setUp(self):
        self.port = FirmwarePort()
        self.client = BenchClient(self.port)
        self.request("HELLO")
        self.request("SELECT")
        self.port.tick(110)

    def tearDown(self):
        self.client.close()

    def request(self, operation, value1=0, value2=0, leg=1, joint=1):
        replies = []
        self.client.request(operation, leg, joint, replies.append, value1, value2)
        self.client.poll()
        self.assertEqual(len(replies), 1)
        return replies[0]

    def arm(self):
        self.assertTrue(self.request("CONFIG", 5, 5)["ok"])  #Test values, not installed margins.
        self.assertTrue(self.request("ARM")["ok"])
        self.port.tick(35)
        return self.request("STATUS")

    def test_identity_voltage_and_current_position_hold(self):
        r = self.arm()
        self.assertEqual((r["servo"], r["pwm_gpio"], r["feedback_gpio"]), (5, 41, 19))
        self.assertEqual((r["mode"], r["enabled"], r["sigma"]), ("holding", True, 1))
        self.assertAlmostEqual(r["a_measured_deg"], 90)
        self.assertAlmostEqual(r["a_command_deg"], 90)
        self.assertAlmostEqual(r["q_command_rad"], r["q_measured_rad"])

    def test_move_and_stop_keep_the_last_command(self):
        start = self.arm()
        duration = max(start["settings"]["min_move_s"], 1.875/start["settings"]["max_speed_deg_s"])
        self.assertTrue(self.request("MOVE", math.radians(1), duration)["ok"])
        self.port.tick(500)
        middle = self.request("STATUS")
        u = 0.5/duration
        F = 10*u**3 - 15*u**4 + 6*u**5
        self.assertAlmostEqual(math.degrees(middle["q_command_rad"] - start["q_command_rad"]), F, delta=0.04)
        held = self.request("STOP")
        self.port.tick(300)
        self.assertEqual(self.request("STATUS")["q_command_rad"], held["q_command_rad"])
        self.assertFalse(self.request("OFF")["enabled"])

    def test_unknown_clearances_and_other_joint_cannot_arm(self):
        self.assertFalse(self.request("ARM")["ok"])
        self.request("SELECT", leg=1, joint=0)
        self.port.tick(110)
        self.request("CONFIG", 5, 5, joint=0)
        reply = self.request("ARM", joint=0)
        self.assertEqual(reply["ok"], reply["allowed"])  #FR roll may now be enabled in the user's header.
        self.assertFalse(reply["enabled"])

    def test_bad_feedback_remains_visible_and_disables(self):
        self.arm()
        self.port.voltage(0); self.port.tick(110)
        r = self.request("STATUS")
        self.assertFalse(r["enabled"])
        self.assertFalse(r["feedback_valid"])
        #The first invalid window can mix old and new samples; it still disables.
        self.assertLess(r["V"], 1.45)
        self.assertIsNone(r["a_measured_deg"])
        self.port.tick(110)
        self.assertEqual(self.request("STATUS")["V"], 0)

    def test_timeout_and_late_arm_need_new_handshake(self):
        self.arm()
        self.port.tick(1010)
        r = self.request("ARM")
        self.assertFalse(r["ok"])
        self.assertFalse(r["enabled"])

    def test_old_calibration_commands_keep_the_same_format(self):
        self.arm()
        self.port.write(b"QLCAP1 81 HELLO 0 0\n")
        reply = json.loads(self.port.read(self.port.in_waiting))
        self.assertEqual((reply["protocol"], reply["type"], reply["ok"]), ("leg-capture/1", "hello", True))
        self.assertFalse(self.request("ARM")["enabled"])

    def test_priority_disable_cancels_pending_movement_callback(self):
        settings = self.arm()["settings"]
        old, latest = [], []
        self.client.request("MOVE", 1, 1, old.append, math.radians(1), max(settings["min_move_s"], 1.875/settings["max_speed_deg_s"]))
        self.client.request("OFF", 1, 1, latest.append)
        self.client.poll()
        self.assertEqual(old, [])
        self.assertEqual(len(latest), 1)
        self.assertFalse(latest[0]["enabled"])

    def test_validation_rejects_wrong_identity_and_nonfinite_numbers(self):
        r = self.request("STATUS")
        bad = dict(r, servo=4)
        with self.assertRaises(ValueError): validate_reply(bad)
        bad = dict(r, V=float("nan"))
        with self.assertRaises(ValueError): validate_reply(bad)
        with self.assertRaises(ValueError): self.client.request("MOVE", 1, 1, lambda r: None, float("nan"), 1)

    def test_missing_reply_timeout_never_retries_motion(self):
        self.client.request("STATUS", 1, 1, lambda reply: None)
        self.port.buffer.clear()
        self.client.clock = lambda: self.client.pending[-1] + self.client.reply_timeout_s + 0.1
        count = len(self.port.sent)
        with self.assertRaises(TimeoutError): self.client.poll()
        self.assertEqual(len(self.port.sent), count)

    def test_reboot_requires_reconnect(self):
        self.port.buffer.extend(b'{"protocol":"leg-capture/1","type":"boot","id":0}\n')
        with self.assertRaises(ConnectionError): self.client.poll()

    def test_firmware_supplies_motion_and_communication_limits(self):
        r = self.request("STATUS")
        settings = r["settings"]
        validate_settings(settings)
        self.assertEqual(self.client.reply_timeout_s, settings["reply_timeout_ms"] / 1000.0)
        old_firmware = dict(r)
        del old_firmware["settings"]
        with self.assertRaisesRegex(ValueError, "Upload the updated firmware"):
            validate_reply(old_firmware)
        bad = dict(settings, max_step_deg=float("nan"))
        with self.assertRaises(ValueError): validate_settings(bad)

    def test_large_step_and_serial_precision_use_firmware_limits(self):
        self.arm()
        settings = self.request("STATUS")["settings"]
        step = min(10.0, settings["max_step_deg"])
        duration = max(settings["min_move_s"], 1.875*step/settings["max_speed_deg_s"] + 0.1)
        if duration > settings["max_move_s"]:
            self.skipTest("Configured speed/duration does not permit this boundary move")
        r = self.request("MOVE", math.radians(step), duration)
        self.assertTrue(r["ok"], r["message"])
        self.assertEqual(r["mode"], "moving")
        self.assertEqual(float(self.port.sent[-1].split()[5]), math.radians(step))

    def test_communication_settings_are_not_fixed_in_python(self):
        r = self.request("STATUS")
        r["settings"] = dict(r["settings"], status_interval_ms=200, reply_timeout_ms=600, link_timeout_ms=900)
        replies = []
        self.client.request("STATUS", 1, 1, replies.append)
        r["id"] = self.client.pending[0]
        self.port.buffer = bytearray((json.dumps(r) + "\n").encode())
        self.client.poll()
        self.assertEqual(self.client.reply_timeout_s, 0.6)

    def test_click_replaces_background_status_without_late_callback(self):
        #A delayed live reading cannot swallow the user's new joint selection.
        old, selected = [], []
        self.client.request("STATUS", 1, 1, old.append)
        self.assertFalse(self.client.idle)
        self.assertTrue(self.client.action_available)
        self.client.request("SELECT", 1, 0, selected.append)
        self.client.poll()
        self.assertEqual(old, [])
        self.assertEqual((selected[0]["leg"], selected[0]["joint"]), (1, 0))

    def test_pending_action_rejects_double_click_without_a_write(self):
        self.client.request("CONFIG", 1, 1, lambda r: None, 5, 5)
        count = len(self.port.sent)
        with self.assertRaises(BenchBusyError):
            self.client.request("ARM", 1, 1, lambda r: None)
        self.assertEqual(len(self.port.sent), count)
        self.assertFalse(self.client.action_available)

    def test_disable_is_a_heartbeat_but_does_not_revive_expired_session(self):
        #OFF near the timeout must leave time to select another joint normally.
        settings = self.request("STATUS")["settings"]
        self.port.tick(settings["link_timeout_ms"] - 100)
        self.assertTrue(self.request("OFF")["session_active"])
        self.port.tick(150)
        self.assertTrue(self.request("SELECT", joint=0)["ok"])
        self.port.tick(settings["link_timeout_ms"] + 10)
        self.assertFalse(self.request("OFF", joint=0)["session_active"])
        self.assertFalse(self.client.ready)

    def test_rejected_actions_preserve_hold_and_give_specific_reasons(self):
        start = self.arm()
        for operation, a, b, joint, reason in (
            ("MOVE", 1, 2, 1, "Step exceeds"),
            ("MOVE", 0.01, 0.001, 1, "Duration"),
            ("CONFIG", 5, 5, 1, "Disable PWM"),
            ("SELECT", 0, 0, 0, "Disable PWM"),
            ("ARM", 0, 0, 1, "already active"),
            ("MOVE", 0.01, 2, 0, "Selection mismatch"),
        ):
            r = self.request(operation, a, b, joint=joint)
            self.assertFalse(r["ok"])
            self.assertIn(reason, r["message"])
            self.assertTrue(r["enabled"])
            self.assertEqual(r["q_command_rad"], start["q_command_rad"])


class BenchWindowTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk(); self.root.withdraw()
        self.port = FirmwarePort()
        self.app = ServoBenchApp(self.root, serial_factory=lambda name: self.port,
                                 ports_factory=lambda: [SimpleNamespace(device="FAKE")], connect_delay_ms=0)
        self.app.toggle_connection(); self.root.update()
        for _ in range(3): self.app.client.poll()
        self.port.tick(110)
        self.app.send("STATUS"); self.app.client.poll()

    def tearDown(self):
        self.app.close()

    def arm(self):
        self.app.home_margin.set("5"); self.app.other_margin.set("5")
        self.app.configure(); self.app.client.poll()
        self.app.supported.set(True); self.app.update_controls()
        self.app.arm_button.invoke(); self.app.client.poll(); self.port.tick(35)
        self.app.send("STATUS"); self.app.client.poll()

    def test_default_selection_readings_and_controls(self):
        self.assertEqual(self.app.selection(), (1, 1))
        self.assertEqual(self.app.status["servo"], 5)
        self.assertEqual(str(self.app.arm_button["state"]), "disabled")
        self.assertEqual(str(self.app.plus_button["state"]), "disabled")
        self.assertIn("GPIO41", self.app.identity.get())
        self.assertIn("90.000", self.app.values["Measured a"].get())

    def test_buttons_make_single_joint_move_and_export_observations(self):
        self.app.home_margin.set("5"); self.app.other_margin.set("5")
        self.app.config_button.invoke(); self.app.client.poll()
        self.app.supported.set(True); self.app.update_controls()
        self.assertEqual(str(self.app.arm_button["state"]), "normal")
        self.app.arm_button.invoke(); self.app.client.poll(); self.port.tick(35)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertEqual(str(self.app.leg_box["state"]), "disabled")
        self.app.plus_button.invoke(); self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "moving")
        self.app.off_button.invoke(); self.app.client.poll()
        self.assertFalse(self.app.status["enabled"])
        self.app.observed.set("FR Hip pitch"); self.app.direction.set("Matches prediction")
        data = self.app.export_data()
        self.assertEqual(data["observed_joint"], "FR Hip pitch")
        self.assertTrue(any(r.get("request") == "MOVE" for r in data["records"]))
        json.dumps(data, allow_nan=False)

    def test_disconnect_clears_live_values_and_sends_off(self):
        self.app.disconnect("Test disconnect")
        self.assertIsNone(self.app.status)
        self.assertTrue(self.port.closed)
        self.assertIn(b" OFF ", self.port.sent[-1])
        self.assertTrue(all(value.get() == "--" for value in self.app.values.values()))

    def test_new_selection_clears_clearance_and_enable_agreement(self):
        self.app.home_margin.set("5"); self.app.supported.set(True)
        self.app.leg.set("RR"); self.app.joint.set("Knee pitch")
        self.app.select_joint(); self.app.client.poll()
        self.assertEqual(self.app.home_margin.get(), "")
        self.assertFalse(self.app.supported.get())
        self.assertFalse(self.app.status["allowed"])

    def test_unsubmitted_dropdown_cannot_arm_the_old_joint(self):
        self.app.home_margin.set("5"); self.app.other_margin.set("5")
        self.app.configure(); self.app.client.poll()
        self.app.supported.set(True); self.app.update_controls()
        self.assertEqual(str(self.app.arm_button["state"]), "normal")
        self.app.joint.set("Hip roll"); self.app.update_controls()
        self.assertEqual(str(self.app.arm_button["state"]), "disabled")
        self.assertEqual(str(self.app.config_button["state"]), "disabled")

    def test_local_error_survives_live_status_until_clear(self):
        self.arm()
        self.app.step_deg.set("-1")
        count = len(self.port.sent)
        self.app.move(1)
        error = self.app.last_error.get()
        self.assertIn("Step must", error)
        self.assertEqual(len(self.port.sent), count)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "holding")
        self.assertEqual(self.app.last_error.get(), error)
        self.assertEqual(self.app.export_data()["last_error"], error)
        count = len(self.port.sent)
        self.app.clear_error_button.invoke()
        self.assertEqual(self.app.last_error.get(), "No errors.")
        self.assertEqual(len(self.port.sent), count)  #Clear never sends a motor command.

    def test_board_rejection_survives_successful_status_reply(self):
        self.arm()
        self.app.send("MOVE", float("1"), 1)  #Bypass UI to exercise ESP32 rejection.
        self.app.client.poll()
        error = self.app.last_error.get()
        self.assertIn("ESP32:", error)
        self.assertFalse(self.app.status["ok"])
        self.assertTrue(self.app.status["enabled"])  #Rejected input no longer drops a valid hold.
        self.assertIn("Step exceeds", error)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertTrue(self.app.status["ok"])
        self.assertEqual(self.app.last_error.get(), error)

    def test_automatic_feedback_shutdown_is_reported_separately(self):
        self.arm()
        self.port.voltage(0); self.port.tick(110)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertIn("ESP32 disabled output", self.app.last_error.get())
        self.assertFalse(self.app.status["enabled"])

    def test_serial_failure_remains_in_error_area(self):
        with patch.object(self.port, "write", side_effect=OSError("Test cable disconnect")):
            self.app.send("STATUS")
        self.assertIsNone(self.app.client)
        self.assertIn("Test cable disconnect", self.app.last_error.get())
        self.app.message.set("Connecting...")
        self.assertIn("Test cable disconnect", self.app.last_error.get())

    def test_move_validation_uses_reported_values_not_literal_limits(self):
        #Use different values to prove that step, speed and BOTH durations follow C++.
        reply = dict(self.app.status)
        reply["settings"] = dict(reply["settings"], max_step_deg=12, max_speed_deg_s=4, min_move_s=2, max_move_s=12)
        self.app.show_reply(reply)
        with patch.object(self.app, "send") as send:
            self.app.step_deg.set("10"); self.app.duration_s.set("5")
            self.app.move(1)
            send.assert_called_once_with("MOVE", math.radians(10), 5)
            send.reset_mock()
            self.app.duration_s.set("2"); self.app.move(1)
            send.assert_not_called()
            self.assertIn("4.688", self.app.last_error.get())
            self.app.duration_s.set("11"); self.app.move(-1)
            send.assert_called_once_with("MOVE", -math.radians(10), 11)
            send.reset_mock()
            self.app.duration_s.set("13"); self.app.move(1)
            send.assert_not_called()
            self.assertIn("2 to 12", self.app.last_error.get())
        self.assertIn("step <= 12", self.app.motion_limits.get())

    def test_window_fits_with_persistent_error_panel(self):
        self.root.geometry("1000x850")
        self.root.update_idletasks()
        self.assertLessEqual(self.root.winfo_reqheight(), 850)

    def test_background_poll_does_not_blink_buttons_or_block_move(self):
        self.arm()
        self.app.send("STATUS")  #Leave the reply pending to reproduce the original flicker.
        self.assertEqual(str(self.app.plus_button["state"]), "normal")
        self.app.plus_button.invoke()
        self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "moving")
        self.assertEqual(self.app.last_error.get(), "No errors.")

    def test_disable_then_select_roll_during_background_poll(self):
        self.arm()
        self.app.off_button.invoke(); self.app.client.poll()
        self.app.send("STATUS")
        self.app.joint.set("Hip roll"); self.app.update_controls()
        self.assertEqual(str(self.app.select_button["state"]), "normal")
        self.app.select_button.invoke(); self.app.client.poll()
        self.assertEqual(self.app.status["joint"], 0)
        if self.app.status["allowed"]:
            self.assertIn("Apply clearances", self.app.enable_reason.get())
        else:
            self.assertIn("Read-only joint", self.app.enable_reason.get())
            self.assertIn("bench_joint_allowed", self.app.enable_reason.get())
        self.assertEqual(str(self.app.arm_button["state"]), "disabled")
        self.assertEqual(self.app.last_error.get(), "No errors.")
        #Return to the already allowed joint; applying clearances permits re-enabling.
        self.app.joint.set("Hip pitch"); self.app.select_joint(); self.app.client.poll()
        self.port.tick(110); self.app.send("STATUS"); self.app.client.poll()
        self.arm()
        self.assertTrue(self.app.status["enabled"])

    def test_busy_click_keeps_connection_and_previous_request(self):
        self.app.send("CONFIG", 5, 5)
        pending = self.app.client.pending
        count = len(self.port.sent)
        self.app.send("ARM")
        self.assertIsNotNone(self.app.client)
        self.assertEqual(self.app.client.pending, pending)
        self.assertEqual(len(self.port.sent), count)
        self.assertIn("no extra command was sent", self.app.last_error.get())
        self.app.client.poll()
        self.assertTrue(self.app.status["configured"])

    def test_expired_session_disconnects_once_instead_of_polling_errors(self):
        self.port.tick(self.app.status["settings"]["link_timeout_ms"] + 10)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertIsNone(self.app.client)
        self.assertIn("reconnect", self.app.last_error.get())
        count = len(self.port.sent)
        self.root.update()
        self.assertEqual(len(self.port.sent), count)

    def test_enable_reason_names_clearances_acknowledgement_and_starting_range(self):
        self.assertIn("Apply clearances", self.app.enable_reason.get())
        self.app.home_margin.set("5"); self.app.other_margin.set("5")
        self.app.configure(); self.app.client.poll()
        self.assertIn("acknowledgement", self.app.enable_reason.get())
        self.app.supported.set(True); self.app.update_controls()
        self.assertIn("Ready to enable", self.app.enable_reason.get())
        reply = dict(self.app.status, q_measured_rad=self.app.status["q_max_rad"] + 0.01)
        self.app.show_reply(reply)
        self.assertIn("outside", self.app.enable_reason.get())
        self.assertEqual(str(self.app.arm_button["state"]), "disabled")


if __name__ == "__main__":
    unittest.main(verbosity=2)
