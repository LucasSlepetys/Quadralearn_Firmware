"""Whole-leg buttons/protocol against actual C++ with fake ADC/PWM, never a COM port.
Tests explain whether a failure is selection, configuration, holding or recovery.
"""
import json
import unittest
import test_servo_bench_ui as bench_tests


class LegHoldTests(unittest.TestCase):
    def setUp(self):
        bench_tests.BenchWindowTests.setUp(self)
        self.app.control_scope.set("Whole leg hold"); self.app.update_controls()
        self.app.select_button.invoke(); self.app.client.poll()
        self.port.tick(110); self.app.send("STATUS"); self.app.client.poll()

    def tearDown(self):
        bench_tests.BenchWindowTests.tearDown(self)

    def configure(self):
        for j in range(3):
            self.app.leg_panel.home[j].set("5")
            self.app.leg_panel.other[j].set("5")
        self.app.leg_panel.apply_button.invoke()
        for _ in range(3):  #Each acknowledgement sends the next configuration.
            self.app.client.poll()

    def arm(self):
        self.configure()
        if not self.app.status["allowed"]:
            self.skipTest("Whole FR leg is not permitted by the current bench allow-list")
        self.app.supported.set(True); self.app.update_controls()
        self.assertEqual(str(self.app.arm_button["state"]), "normal")
        self.app.arm_button.invoke(); self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "arming")
        self.port.tick(35); self.app.send("STATUS"); self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "holding")

    def test_reading_reports_three_identities_without_enabling(self):
        r = self.app.status
        self.assertEqual(r["scope"], "leg")
        self.assertEqual([j["servo"] for j in r["joints"]], [4, 5, 6])
        self.assertEqual([j["pwm_gpio"] for j in r["joints"]], [42, 41, 40])
        self.assertEqual([j["feedback_gpio"] for j in r["joints"]], [1, 19, 11])
        self.assertFalse(any(j["enabled"] for j in r["joints"]))
        self.assertIn("clearances", self.app.enable_reason.get())
        self.assertEqual(str(self.app.plus_button["state"]), "disabled")

    def test_three_clearances_apply_in_order_without_enabling(self):
        self.configure()
        self.assertTrue(self.app.status["configured"])
        requests = [p.split() for p in self.port.sent if b" LEG_CONFIG " in p]
        self.assertEqual([int(p[4]) for p in requests], [0, 1, 2])
        self.assertFalse(self.app.status["enabled"])
        self.assertIn("acknowledgement", self.app.enable_reason.get())

    def test_invalid_clearance_prevents_partial_configuration(self):
        for j in range(3):
            self.app.leg_panel.home[j].set("5"); self.app.leg_panel.other[j].set("5")
        self.app.leg_panel.other[2].set("-1")
        before = len(self.port.sent)
        self.app.configure_leg()
        self.assertEqual(len(self.port.sent), before)
        self.assertIn("Knee pitch", self.app.last_error.get())

    def test_enable_holds_three_measured_angles_and_off_disables_three(self):
        self.arm()
        for j in self.app.status["joints"]:
            self.assertTrue(j["enabled"])
            self.assertAlmostEqual(j["q_command_rad"], j["q_measured_rad"])
        self.app.off_button.invoke(); self.app.client.poll()
        self.assertFalse(any(j["enabled"] for j in self.app.status["joints"]))
        self.assertEqual(self.app.status["mode"], "disabled")
        self.assertTrue(self.app.status["session_active"])

    def test_feedback_changes_do_not_chase_a_new_hold_angle(self):
        self.arm()
        old = [j["q_command_rad"] for j in self.app.status["joints"]]
        self.port.voltage(1500); self.port.tick(110)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertEqual([j["q_command_rad"] for j in self.app.status["joints"]], old)
        self.assertNotEqual(self.app.status["joints"][1]["q_measured_rad"], old[1])

    def test_bad_pitch_feedback_disables_whole_leg_and_names_pitch(self):
        self.arm()
        self.port.voltage(0); self.port.tick(110)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertFalse(any(j["enabled"] for j in self.app.status["joints"]))
        self.assertEqual(self.app.status["problem_joint"], 1)
        self.assertIn("Hip pitch", self.app.last_error.get())
        self.assertIn("disabled", self.app.last_error.get())
        self.port.voltage(1450); self.port.tick(220)
        self.app.send("STATUS"); self.app.client.poll()
        self.assertTrue(self.app.status["feedback_valid"])
        self.assertFalse(self.app.status["enabled"])  #Recovery does not auto-enable.

    def test_mode_switch_requires_disable_and_single_joint_still_works(self):
        self.arm()
        self.assertEqual(str(self.app.scope_box["state"]), "disabled")
        self.app.control_scope.set("Single joint")  #Bypass UI to test the board gate too.
        self.app.send("SELECT"); self.app.client.poll()
        self.assertFalse(self.app.status["ok"])
        self.assertTrue(self.app.status["enabled"])
        self.assertIn("Disable PWM", self.app.last_error.get())
        self.app.send("OFF"); self.app.client.poll()
        self.app.select_joint(); self.app.client.poll()
        self.port.tick(110); self.app.send("STATUS"); self.app.client.poll()
        self.assertEqual(self.app.status["joint"], 1)
        bench_tests.BenchWindowTests.arm(self)
        self.assertTrue(self.app.status["enabled"])

    def test_status_poll_does_not_disable_hold_button(self):
        self.configure(); self.app.supported.set(True); self.app.update_controls()
        self.app.send("STATUS")
        self.assertEqual(str(self.app.arm_button["state"]), "normal")
        self.app.arm_button.invoke(); self.app.client.poll()
        self.assertEqual(self.app.status["mode"], "arming")

    def test_link_timeout_stops_all_three_and_requires_reconnect(self):
        self.arm()
        self.port.tick(self.app.status["settings"]["link_timeout_ms"] + 10)
        replies = []
        self.app.client.request("LEG_STATUS", 1, -1, replies.append)
        self.app.client.poll()
        self.assertFalse(replies[0]["session_active"])
        self.assertFalse(any(j["enabled"] for j in replies[0]["joints"]))
        self.app.show_reply(replies[0])
        self.assertIsNone(self.app.client)

    def test_old_calibration_protocol_exits_the_whole_leg_controller(self):
        self.arm()
        self.port.write(b"QLCAP1 300 HELLO 0 0\n")
        r = json.loads(self.port.read(self.port.in_waiting))
        self.assertEqual((r["protocol"], r["type"], r["ok"]), ("leg-capture/1", "hello", True))
        replies = []
        self.app.client.request("LEG_ARM", 1, -1, replies.append)
        self.app.client.poll()
        self.assertFalse(replies[0]["ok"])
        self.assertFalse(replies[0]["enabled"])

    def test_whole_leg_reply_fits_client_buffer_and_window(self):
        self.configure()
        self.assertLess(len(json.dumps(self.app.status)), 4096)
        self.root.geometry("1100x900"); self.root.update_idletasks()
        self.assertLessEqual(self.root.winfo_reqwidth(), 1100)
        self.assertLessEqual(self.root.winfo_reqheight(), 900)


if __name__ == "__main__":
    unittest.main(verbosity=2)
