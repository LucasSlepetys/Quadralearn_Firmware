"""Live measured leg view. C++ supplies all calibration, mapping and FK results.

Python only handles the USB connection, angle display units and drawing.
This window never sends capture or motor commands.
"""
import math
import time
import tkinter as tk
from tkinter import ttk

try:
    import numpy as np
    import serial
    from serial.tools import list_ports
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError as exc:
    raise SystemExit("Use the quadruped interpreter, or install tools/requirements_visualization.txt") from exc

from capture_client import CaptureClient
from calibration_ui import open_serial  #Reuse only its connection helper; do not create a calibration window.

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("Hip roll / alpha", "Hip pitch / theta_1", "Knee pitch / theta_2")
POINTS = ("H", "P", "K", "F")


class FeedbackViewer:
    def __init__(self, root, serial_factory=open_serial, ports_factory=list_ports.comports, connect_delay_ms=1500):
        self.root = root
        self.serial_factory = serial_factory
        self.ports_factory = ports_factory
        self.connect_delay_ms = connect_delay_ms
        self.client = None
        self.ready = False
        self.closed = False
        self.handshake_job = None
        self.next_read = 0.0
        self.last_received = None
        self.last_pose = None
        self.port_name = tk.StringVar()
        self.leg_name = tk.StringVar(value="FR")
        self.live = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Close the calibration window and Serial Monitor, then connect to the board.")
        self.connection = tk.StringVar(value="Disconnected")
        self._build_window()
        self.refresh_ports()
        self._update_controls()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick_job = self.root.after(40, self._tick)

    @property
    def leg(self):
        return LEGS.index(self.leg_name.get())

    def _build_window(self):
        self.root.title("Quadruped - Live feedback leg view")
        self.root.geometry("1080x820")
        self.root.minsize(960, 720)
        page = ttk.Frame(self.root, padding=12)
        page.pack(fill="both", expand=True)
        ttk.Label(page, text="Live measured leg", font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(page, text="Feedback only | Fixed hip-roll frame H | x forward, y left, z up | No PWM").pack(anchor="w", pady=(2, 10))

        row = ttk.Frame(page)
        row.pack(fill="x")
        ttk.Label(row, text="USB port").pack(side="left", padx=(0, 6))
        self.port_box = ttk.Combobox(row, textvariable=self.port_name, width=43, state="readonly")
        self.port_box.pack(side="left")
        self.refresh_button = ttk.Button(row, text="Refresh", command=self.refresh_ports)
        self.refresh_button.pack(side="left", padx=6)
        self.connect_button = ttk.Button(row, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        ttk.Label(row, textvariable=self.connection).pack(side="right")

        row = ttk.Frame(page)
        row.pack(fill="x", pady=10)
        ttk.Label(row, text="Leg").pack(side="left", padx=(0, 6))
        self.leg_box = ttk.Combobox(row, textvariable=self.leg_name, values=LEGS, width=6, state="readonly")
        self.leg_box.pack(side="left")
        self.leg_box.bind("<<ComboboxSelected>>", self.select_leg)
        self.live_check = ttk.Checkbutton(row, text="Live readings", variable=self.live, command=self.live_changed)
        self.live_check.pack(side="left", padx=14)
        self.read_button = ttk.Button(row, text="Read once", command=self.read_once)
        self.read_button.pack(side="left")
        ttk.Label(row, text="Move slowly: each reading averages a short sampling window.").pack(side="left", padx=14)

        columns = ("joint", "servo", "voltage", "spread", "servo_angle", "model_rad", "model_deg")
        self.table = ttk.Treeview(page, columns=columns, show="headings", height=3, selectmode="none")
        headings = ("Joint", "Servo / GPIO", "Mean [V]", "Spread [mV]", "Servo a [deg]", "Model q [rad]", "Model q [deg]")
        for column, heading in zip(columns, headings):
            self.table.heading(column, text=heading)
            self.table.column(column, width=170 if column == "joint" else 120, anchor="center")
        self.table.tag_configure("invalid", foreground="#b33030")
        self.table.pack(fill="x")
        for joint, name in enumerate(JOINTS):
            self.table.insert("", "end", iid=str(joint), values=(name,) + ("--",) * 6)

        #The figure consumes the received H/P/K/F coordinates directly; no FK here.
        plot = ttk.Frame(page)
        plot.pack(fill="both", expand=True, pady=(8, 0))
        self.figure = Figure(figsize=(8, 4.4), dpi=100)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.figure.subplots_adjust(left=0.02, right=0.90, bottom=0.08, top=0.92)
        self.ax.set_xlabel("x_H [m] - forward")
        self.ax.set_ylabel("y_H [m] - left")
        self.ax.set_zlabel("z_H [m] - up")
        self.ax.set_xlim(-0.20, 0.20)
        self.ax.set_ylim(-0.20, 0.20)
        self.ax.set_zlim(-0.20, 0.20)
        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            axis.set_ticks([-0.15, 0, 0.15])
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_proj_type("ortho")
        self.ax.view_init(elev=22, azim=-55)
        for vector, colour in (((0.06, 0, 0), "#bf3737"), ((0, 0.06, 0), "#2a8b57"), ((0, 0, 0.06), "#3469bf")):
            self.ax.quiver(0, 0, 0, *vector, color=colour, arrow_length_ratio=0.15)
        self.line, = self.ax.plot([], [], [], "o-", color="#e87924", linewidth=3, markersize=6)
        self.labels = [self.ax.text(0, 0, 0, name, visible=False) for name in POINTS]
        self.ball = None
        self.pose_title = self.ax.set_title("FR - waiting for a measured pose")
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot)
        toolbar = NavigationToolbar2Tk(self.canvas, plot, pack_toolbar=False)
        toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.draw_idle()

        ttk.Label(page, text="H: hip roll | P: hip pitch | K: knee | F: ball centre. Body position and ground are not estimated.").pack(anchor="w")
        ttk.Label(page, textvariable=self.status, wraplength=940, foreground="#173c50").pack(anchor="w", pady=(4, 0))

    def refresh_ports(self):
        ports = sorted(self.ports_factory(), key=lambda port: ("CH343" not in port.description, port.device))
        values = [f"{port.device} - {port.description}" for port in ports]
        self.port_box["values"] = values
        if self.port_name.get() not in values:
            self.port_name.set(values[0] if values else "")

    def _update_controls(self):
        connected = self.client is not None
        self.connect_button["text"] = "Disconnect" if connected else "Connect"
        self.port_box["state"] = "disabled" if connected else "readonly"
        self.refresh_button["state"] = "disabled" if connected else "normal"
        self.read_button["state"] = "normal" if self.ready else "disabled"
        self.live_check["state"] = "normal" if self.ready else "disabled"

    def toggle_connection(self):
        if self.client:
            self.disconnect("Disconnected. Last pose cleared.")
            return
        port = self.port_name.get().split(" - ", 1)[0].strip()
        if not port:
            self.status.set("Select a USB-to-UART COM port first.")
            return
        try:
            self.client = CaptureClient(self.serial_factory(port))
            self.connection.set("Connecting...")
            self.status.set("Waiting for the board; keep other serial applications closed.")
            current = self.client
            self.handshake_job = self.root.after(self.connect_delay_ms, lambda: self._handshake(current))
            self._update_controls()
        except (OSError, serial.SerialException) as exc:
            self.disconnect(f"Could not connect: {exc}")

    def _handshake(self, client):
        self.handshake_job = None
        if self.client is client:
            client.request("HELLO", self.leg, self._connected)

    def _connected(self, reply):
        if not reply["ok"]:
            raise ConnectionError(reply.get("message", "Handshake failed."))
        self.ready = True
        self.connection.set("Connected - 115200 baud")
        self.next_read = 0.0
        self._update_controls()
        self.read_once()

    def _clear_pose(self, title, clear_table=True):
        self.last_pose = None
        self.last_received = None
        self.line.set_data_3d([], [], [])
        for label in self.labels:
            label.set_visible(False)
        if self.ball is not None:
            self.ball.remove()
            self.ball = None
        if clear_table:
            for joint, name in enumerate(JOINTS):
                self.table.item(str(joint), values=(name,) + ("--",) * 6, tags=())
        self.pose_title.set_text(f"{self.leg_name.get()} - {title}")
        self.canvas.draw_idle()

    def disconnect(self, message):
        if self.handshake_job is not None:
            self.root.after_cancel(self.handshake_job)
            self.handshake_job = None
        if self.client:
            try:
                self.client.close()
            except OSError:
                pass
        self.client = None
        self.ready = False
        self.connection.set("Disconnected")
        self._clear_pose("disconnected")
        self.status.set(message)
        self._update_controls()

    def select_leg(self, _event=None):
        self._clear_pose("waiting for a measured pose")
        self.status.set("Selected leg changed. RR has no installed endpoint configuration yet." if self.leg == 3
                        else "Waiting for the selected leg's measured pose.")
        self.next_read = 0.0
        self.read_once()

    def live_changed(self):
        self.next_read = 0.0
        if self.live.get():
            self._clear_pose("waiting for fresh feedback")
            self.status.set("Live readings resumed.")
        else:
            self.pose_title.set_text(f"{self.leg_name.get()} - last sample (live paused)")
            self.status.set("Live paused. The displayed sample is frozen; use Read once to refresh.")
            self.canvas.draw_idle()

    def read_once(self):
        if self.ready and self.client.idle:
            self.client.request("POSE", self.leg, self._pose_received)

    def _pose_received(self, reply):
        if reply.get("leg") != self.leg:
            self.read_once()
            return  #A response for the previous selection must not draw the new leg.
        self.next_read = time.monotonic() + 0.25
        if reply["type"] == "error":
            self.disconnect(reply.get("message", "POSE failed") + ". Upload the firmware with POSE support.")
            return
        q = reply["q_rad"] if reply["ok"] else None
        for joint, row in enumerate(reply["joints"]):
            a = f"{row['angle_deg']:.2f}" if row["valid"] else "INVALID"
            values = (JOINTS[joint], f"S{row['servo']:02d} / {row['gpio']}",
                      f"{row['mean_V']:.4f}" if row["mean_V"] is not None else "--",
                      f"{row['spread_mV']:.1f}" if row["spread_mV"] is not None else "--", a,
                      f"{q[joint]:.4f}" if q is not None else "--",
                      f"{math.degrees(q[joint]):.2f}" if q is not None else "--")
            self.table.item(str(joint), values=values, tags=() if row["valid"] else ("invalid",))
        if not reply["ok"]:
            self._clear_pose("pose unavailable", clear_table=False)
            self.status.set(reply.get("message", "No complete valid pose available."))
            return

        self.last_pose = reply
        self.last_received = time.monotonic()
        points = np.array([reply["points_H_m"][name] for name in POINTS])
        self.line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        for label, point in zip(self.labels, points):
            label.set_position((point[0] + 0.006, point[1]))
            label.set_3d_properties(point[2] + 0.006, zdir=None)
            label.set_visible(True)

        #Draw the received ball centre/radius; this is only sphere rendering.
        if self.ball is not None:
            self.ball.remove()
        u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 17), np.linspace(0, np.pi, 9))
        radius = reply["ball_radius_m"]
        centre = points[3]
        self.ball = self.ax.plot_wireframe(centre[0] + radius * np.cos(u) * np.sin(v),
                                          centre[1] + radius * np.sin(u) * np.sin(v),
                                          centre[2] + radius * np.cos(v), color="#667785", alpha=0.35, linewidth=0.6)
        suffix = "measured pose in H" if self.live.get() else "last sample (live paused)"
        self.pose_title.set_text(f"{self.leg_name.get()} - {suffix}")
        self.status.set(f"Received {time.strftime('%H:%M:%S')} | Angles and FK calculated in C++. Move one joint at a time to compare.")
        self.canvas.draw_idle()

    def _tick(self):
        if self.closed:
            return
        try:
            if self.client:
                self.client.poll()
            now = time.monotonic()
            if self.ready and self.live.get():
                if self.last_received is not None and now - self.last_received > 1.5:
                    self._clear_pose("feedback stale")
                    self.status.set("Waiting for fresh feedback; the old pose has been cleared.")
                if self.client.idle and now >= self.next_read:
                    self.read_once()
        except (OSError, ValueError, TimeoutError, serial.SerialException) as exc:
            self.disconnect(f"Connection stopped: {exc}")
        self.tick_job = self.root.after(40, self._tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.root.after_cancel(self.tick_job)
        self.disconnect("Closed")
        self.root.update_idletasks()  #Finish any queued canvas draw before destroying Tk.
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    viewer = FeedbackViewer(window)
    window.mainloop()
