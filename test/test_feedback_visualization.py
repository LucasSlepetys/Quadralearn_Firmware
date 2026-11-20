"""Test the live viewer with real C++ calculations and simulated ADC/USB only.

No physical serial port is opened. The calibration UI's existing tests stay separate.
"""
import copy
import math
from pathlib import Path
import sys
import time
import tkinter as tk
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from capture_client import CaptureClient, validate_pose
from visualize_feedback import FeedbackViewer
from test_python_interface import FirmwarePort


def set_fr_interior(port):
    #Exact calibration knots: S04=100 deg, S05=90 deg, S06=100 deg.
    for gpio, mV in ((1, 1570), (19, 1450), (11, 1570)):
        port.voltage(gpio, mV)


def get_pose(port, leg=1):
    replies = []
    client = CaptureClient(port)
    client.request("POSE", leg, replies.append)
    for _ in range(10):
        client.poll()
        if replies:
            return replies[0]
    raise AssertionError("No simulated POSE reply")


class PoseProtocolTests(unittest.TestCase):
    def setUp(self):
        self.port = FirmwarePort()
        set_fr_interior(self.port)

    def tearDown(self):
        self.port.close()

    def test_real_cpp_pose_response(self):
        reply = get_pose(self.port)
        self.assertTrue(reply["ok"])
        self.assertEqual([row["servo"] for row in reply["joints"]], [4, 5, 6])
        self.assertEqual([row["gpio"] for row in reply["joints"]], [1, 19, 11])
        self.assertEqual([row["angle_deg"] for row in reply["joints"]], [100, 90, 100])
        expected = (-1.05882353, -29.1600910939, 72.2873443331)
        for q, degrees in zip(reply["q_rad"], expected):
            self.assertAlmostEqual(math.degrees(q), degrees, places=5)
        for actual, wanted in zip(reply["points_H_m"]["F"], (-0.016585015272, -0.047828056125, -0.120293356655)):
            self.assertAlmostEqual(actual, wanted, places=7)
        self.assertEqual(reply["points_H_m"]["H"], [0, 0, 0])
        self.assertEqual(reply["ball_radius_m"], 0.0195)

    def test_invalid_reading_has_no_pose(self):
        self.port.voltage(19, 0)
        reply = get_pose(self.port)
        self.assertFalse(reply["ok"])
        self.assertIsNone(reply["q_rad"])
        self.assertIsNone(reply["points_H_m"])
        self.assertEqual(reply["joints"][1]["mean_V"], 0)

    def test_missing_rr_configuration_is_explicit(self):
        reply = get_pose(self.port, leg=3)
        self.assertFalse(reply["ok"])
        self.assertIn("not configured", reply["message"])

    def test_corrupted_pose_is_rejected(self):
        good = get_pose(self.port)
        for field, value in (("frame", "W"), ("angle_unit", "deg"), ("length_unit", "mm"),
                             ("q_rad", [0, 1]), ("q_rad", [0, float("nan"), 1]),
                             ("points_H_m", {}), ("ball_radius_m", -1)):
            bad = copy.deepcopy(good)
            bad[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_pose(bad)
        bad = copy.deepcopy(good)
        bad["joints"][0]["servo"] = 1
        with self.assertRaises(ValueError):
            validate_pose(bad)
        bad = copy.deepcopy(good)
        bad["ok"] = False
        with self.assertRaises(ValueError):
            validate_pose(bad)


class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.port = FirmwarePort()
        set_fr_interior(self.port)
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = FeedbackViewer(self.root, serial_factory=lambda _: self.port,
                                  ports_factory=lambda: [], connect_delay_ms=0)
        self.app.live.set(False)
        self.app.port_name.set("TEST - simulated firmware")
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
        self.fail("Viewer requests did not finish")

    def test_table_and_plot_use_received_values(self):
        self.assertEqual(self.app.leg_name.get(), "FR")
        row = self.app.table.item("0", "values")
        self.assertEqual(row[1], "S04 / 1")
        self.assertEqual(row[2], "1.5700")
        self.assertEqual(row[4], "100.00")
        self.assertAlmostEqual(float(row[6]), -1.06, places=2)
        x, y, z = self.app.line.get_data_3d()
        for i, name in enumerate(("H", "P", "K", "F")):
            self.assertEqual([x[i], y[i], z[i]], self.app.last_pose["points_H_m"][name])

    def test_moving_roll_changes_only_that_model_angle(self):
        before = self.app.last_pose
        self.port.voltage(1, 1710)  #S04 is now 110 degrees.
        self.app.read_button.invoke(); self.pump()
        after = self.app.last_pose
        self.assertAlmostEqual(after["q_rad"][0] - before["q_rad"][0], -math.pi / 18, places=7)
        self.assertEqual(after["q_rad"][1:], before["q_rad"][1:])
        self.assertEqual(after["points_H_m"]["F"][0], before["points_H_m"]["F"][0])
        self.assertNotEqual(after["points_H_m"]["F"][1:], before["points_H_m"]["F"][1:])

    def test_invalid_feedback_clears_graph_but_keeps_voltage(self):
        self.port.voltage(19, 0)
        self.app.read_button.invoke(); self.pump()
        self.assertIsNone(self.app.last_pose)
        self.assertEqual(len(self.app.line.get_data_3d()[0]), 0)
        self.assertEqual(self.app.table.item("1", "values")[2], "0.0000")
        self.assertEqual(self.app.table.item("1", "values")[4], "INVALID")

    def test_selection_ignores_previous_leg_response(self):
        self.app.read_once()  #FR request is still queued when the selection changes.
        self.app.leg_name.set("FL"); self.app.select_leg(); self.pump()
        self.assertEqual(self.app.last_pose["leg"], 0)
        self.assertEqual(self.app.table.item("0", "values")[1], "S01 / 4")
        self.app.leg_name.set("RR"); self.app.select_leg(); self.pump()
        self.assertIsNone(self.app.last_pose)
        self.assertIn("not configured", self.app.status.get())

    def test_stale_feedback_clears_pose(self):
        self.app.live.set(True)
        self.app.last_received = time.monotonic() - 2.0
        self.app.next_read = time.monotonic() + 10.0
        self.root.after_cancel(self.app.tick_job)
        self.app._tick()
        self.assertIsNone(self.app.last_pose)
        self.assertIn("stale", self.app.pose_title.get_text())

    def test_disconnect_clears_model_and_live_table(self):
        self.app.connect_button.invoke()
        self.assertFalse(self.app.ready)
        self.assertIsNone(self.app.last_pose)
        self.assertEqual(self.app.table.item("0", "values")[2], "--")

    def test_plot_renders_and_window_fits(self):
        self.root.deiconify()
        self.root.update_idletasks()
        for widget in (self.app.table, self.app.canvas.get_tk_widget(), self.app.read_button):
            bottom = widget.winfo_rooty() - self.root.winfo_rooty() + widget.winfo_height()
            right = widget.winfo_rootx() - self.root.winfo_rootx() + widget.winfo_width()
            self.assertLessEqual(bottom, self.root.winfo_height())
            self.assertLessEqual(right, self.root.winfo_width())
        self.app.canvas.draw()
        #A reproducible rendering for visual review; generated from fake ADC values.
        self.app.figure.savefig(ROOT / ".pio/desktop_tests/live_fr_test.png", dpi=130)
        self.root.withdraw()


if __name__ == "__main__":
    unittest.main(verbosity=2)
