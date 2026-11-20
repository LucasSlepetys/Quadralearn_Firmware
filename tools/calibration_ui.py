"""Whole-leg capture window. Run this file; use the quadruped Python interpreter."""
import json
from datetime import datetime, timezone
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("Install the USB dependency with: python -m pip install -r tools/requirements.txt")

from capture_client import CaptureClient

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("Hip roll", "Hip pitch", "Knee pitch")


def open_serial(port_name):
    #Set control lines before opening. Some USB drivers may still reset the board.
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.5)
    port.dtr = False
    port.rts = False
    port.port = port_name
    try:
        port.open()
    except Exception:
        port.close()
        raise
    return port


class CalibrationApp:
    def __init__(self, root, serial_factory=open_serial, ports_factory=list_ports.comports, connect_delay_ms=1500):
        self.root = root
        self.serial_factory = serial_factory
        self.ports_factory = ports_factory
        self.connect_delay_ms = connect_delay_ms
        self.handshake_job = None
        self.client = None
        self.ready = False
        self.busy = False
        self.preview = None
        self.records = {}
        self.next_read = 0.0
        self.closed = False
        self.port_name = tk.StringVar()
        self.leg_name = tk.StringVar(value="FL")
        self.live = tk.BooleanVar(value=True)
        self.connection_text = tk.StringVar(value="Disconnected")
        self.status = tk.StringVar(value="Connect to the CH343 USB-to-UART port after uploading the new firmware.")
        self.live_text = tk.StringVar(value="Live feedback — waiting for connection")
        self.preview_text = tk.StringVar(value="No preview. Position all three joints, then choose an endpoint.")
        self._build_window()
        self.refresh_ports()
        self._update_controls()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick_job = self.root.after(40, self._tick)

    @property
    def leg(self):
        return LEGS.index(self.leg_name.get())

    def _build_window(self):
        self.root.title("Quadruped · Leg endpoint capture")
        self.root.geometry("1080x790")
        self.root.minsize(980, 760)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10), padding=(10, 5))
        style.configure("Treeview", font=("Segoe UI", 10), rowheight=29)
        style.configure("Treeview.Heading", font=("Segoe UI", 10, "bold"))
        style.configure("Title.TLabel", font=("Segoe UI", 19, "bold"), foreground="#173c50")
        style.configure("Accent.TButton", foreground="#ffffff", background="#1b6d85")
        page = ttk.Frame(self.root, padding=18)
        page.pack(fill="both", expand=True)
        ttk.Label(page, text="Leg endpoint capture", style="Title.TLabel").pack(anchor="w")
        ttk.Label(page, text="Feedback only · No motor commands · All angles are physical servo angles").pack(anchor="w", pady=(2, 12))

        connection = ttk.Frame(page)
        connection.pack(fill="x")
        ttk.Label(connection, text="USB port").pack(side="left", padx=(0, 8))
        self.port_box = ttk.Combobox(connection, textvariable=self.port_name, width=43, state="readonly")
        self.port_box.pack(side="left")
        self.refresh_button = ttk.Button(connection, text="Refresh", command=self.refresh_ports)
        self.refresh_button.pack(side="left", padx=6)
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        ttk.Label(connection, textvariable=self.connection_text).pack(side="right")

        selection = ttk.Frame(page)
        selection.pack(fill="x", pady=12)
        ttk.Label(selection, text="Selected leg").pack(side="left", padx=(0, 8))
        self.leg_box = ttk.Combobox(selection, textvariable=self.leg_name, values=LEGS, width=7, state="readonly")
        self.leg_box.pack(side="left")
        self.leg_box.bind("<<ComboboxSelected>>", self.select_leg)
        self.live_check = ttk.Checkbutton(selection, text="Live readings", variable=self.live)
        self.live_check.pack(side="left", padx=18)
        self.read_button = ttk.Button(selection, text="Read once", command=self.read_once)
        self.read_button.pack(side="left")

        self.live_table = self._reading_table(page, self.live_text)
        actions = ttk.Frame(page)
        actions.pack(fill="x", pady=12)
        self.home_button = ttk.Button(actions, text="Preview HOME · 3 joints", command=lambda: self.capture("HOME"))
        self.home_button.pack(side="left")
        self.other_button = ttk.Button(actions, text="Preview OTHER END · 3 joints", command=lambda: self.capture("OTHER"))
        self.other_button.pack(side="left", padx=8)
        self.accept_button = ttk.Button(actions, text="Accept preview", style="Accent.TButton", command=self.accept_preview)
        self.accept_button.pack(side="right")
        self.cancel_button = ttk.Button(actions, text="Discard", command=self.discard_preview)
        self.cancel_button.pack(side="right", padx=8)
        self.preview_table = self._reading_table(page, self.preview_text)

        saved_frame = ttk.LabelFrame(page, text="Accepted endpoints for selected leg · ESP32 RAM", padding=8)
        saved_frame.pack(fill="x", pady=(12, 0))
        columns = ("joint", "home_v", "home_a", "other_v", "other_a")
        self.saved_table = ttk.Treeview(saved_frame, columns=columns, show="headings", height=3, selectmode="none")
        for column, label in zip(columns, ("Joint", "HOME voltage [V]", "HOME angle [°]", "OTHER voltage [V]", "OTHER angle [°]")):
            self.saved_table.heading(column, text=label)
            self.saved_table.column(column, width=170, anchor="center")
        self.saved_table.pack(fill="x")
        for index, joint in enumerate(JOINTS):
            self.saved_table.insert("", "end", iid=str(index), values=(joint, "—", "—", "—", "—"))
        footer = ttk.Frame(page)
        footer.pack(fill="x", pady=(10, 4))
        self.records_button = ttk.Button(footer, text="Refresh accepted endpoints", command=self.refresh_records)
        self.records_button.pack(side="left")
        self.export_button = ttk.Button(footer, text="Export all legs to JSON…", command=self.export_records)
        self.export_button.pack(side="left", padx=8)
        ttk.Label(page, text="RAM records are lost on reset or power loss. Export a computer backup before disconnecting.", foreground="#85541d").pack(anchor="w", pady=4)
        ttk.Label(page, textvariable=self.status, wraplength=1000, foreground="#173c50").pack(anchor="w", pady=4)

    def _reading_table(self, parent, title):
        frame = ttk.LabelFrame(parent, labelwidget=ttk.Label(parent, textvariable=title), padding=8)
        frame.pack(fill="x")
        columns = ("joint", "servo", "mean", "min", "max", "spread", "angle", "valid")
        tree = ttk.Treeview(frame, columns=columns, show="headings", height=3, selectmode="none")
        headings = ("Joint", "Servo / GPIO", "Mean [V]", "Min [V]", "Max [V]", "Spread [mV]", "Angle [°]", "Reading")
        for column, heading in zip(columns, headings):
            tree.heading(column, text=heading)
            tree.column(column, width=100 if column != "joint" else 125, anchor="center")
        tree.tag_configure("invalid", foreground="#b33030")
        tree.pack(fill="x")
        for index, joint in enumerate(JOINTS):
            tree.insert("", "end", iid=str(index), values=(joint,) + ("—",) * 7)
        return tree

    def refresh_ports(self):
        ports = sorted(self.ports_factory(), key=lambda p: ("CH343" not in p.description, p.device))
        values = [f"{p.device} — {p.description}" for p in ports]
        self.port_box["values"] = values
        if self.port_name.get() not in values:
            self.port_name.set(values[0] if values else "")

    def _update_controls(self):
        enabled = self.ready and not self.busy
        for button in (self.home_button, self.other_button, self.read_button, self.records_button, self.export_button):
            button["state"] = "normal" if enabled else "disabled"
        can_accept = enabled and self.preview is not None and self.preview["ok"]
        self.accept_button["state"] = "normal" if can_accept else "disabled"
        self.cancel_button["state"] = "normal" if enabled and self.preview is not None else "disabled"
        self.leg_box["state"] = "readonly" if enabled else "disabled"
        self.live_check["state"] = "normal" if self.ready else "disabled"
        self.port_box["state"] = "disabled" if self.client else "readonly"
        self.refresh_button["state"] = "disabled" if self.client else "normal"
        self.connect_button["text"] = "Disconnect" if self.client else "Connect"

    def toggle_connection(self):
        if self.client:
            self.disconnect("Disconnected. Displayed readings were cleared.")
            return
        port_name = self.port_name.get().split(" ")[0]
        if not port_name:
            self.status.set("Select the CH343 USB-to-UART port first.")
            return
        try:
            self.client = CaptureClient(self.serial_factory(port_name))
            self.connection_text.set("Connecting…")
            self.status.set("Waiting for the board. Serial Monitor must be closed.")
            current = self.client
            self.handshake_job = self.root.after(self.connect_delay_ms, lambda: self._handshake(current))
            self._update_controls()
        except (OSError, serial.SerialException) as exc:
            self.disconnect(f"Could not open {port_name}: {exc}. Close Serial Monitor and try again.")

    def _handshake(self, client):
        if self.client is client:
            client.request("HELLO", self.leg, self._connected)

    def _connected(self, reply):
        if not reply["ok"]:
            raise ConnectionError(reply.get("message", "Firmware handshake failed."))
        self.ready = True
        self.connection_text.set("Connected · 115200 baud")
        self.status.set("Position the selected leg and inspect its live voltages before previewing an endpoint.")
        self.refresh_records()

    def disconnect(self, message):
        if self.client:
            try:
                self.client.close()
            except OSError:
                pass
        self.client = None
        self.ready = False
        self.busy = False
        self.preview = None
        self.records.clear()
        self.connection_text.set("Disconnected")
        self.live_text.set("Live feedback — disconnected")
        self.preview_text.set("No preview. Reconnect to read the board's current records.")
        self._fill_readings(self.live_table, None)
        self._fill_readings(self.preview_table, None)
        self._fill_saved(None)
        self.status.set(message)
        self._update_controls()

    def select_leg(self, _event=None):
        if not self.ready or self.busy:
            return
        self.preview = None
        self.busy = True
        self._fill_readings(self.live_table, None)
        self._fill_readings(self.preview_table, None)
        self._fill_saved(None)
        self.preview_text.set("No preview. Changing legs discards the previous pending preview.")
        self.client.request("CANCEL", self.leg, lambda reply: self._get_records())
        self._update_controls()

    def _get_records(self):
        self.client.request("GET", self.leg, self._received_records)

    def refresh_records(self):
        if not self.ready:
            return
        self.busy = True
        self._update_controls()
        self._get_records()

    def _received_records(self, reply):
        self.busy = False
        if reply["ok"]:
            self.records[reply["leg"]] = reply
            if reply["leg"] == self.leg:
                self._fill_saved(reply)
        else:
            self.status.set(reply.get("message", "Could not read accepted records."))
        self._update_controls()

    def read_once(self):
        if self.ready and self.client.idle:
            self.client.request("READ", self.leg, self._reading)

    def _reading(self, reply):
        if reply["ok"] and reply["leg"] == self.leg:
            self._fill_readings(self.live_table, reply["joints"])
            self.live_text.set(f"Live feedback · {LEGS[self.leg]} · updated {time.strftime('%H:%M:%S')}")

    def capture(self, operation):
        if not self.ready or self.busy:
            return
        self.preview = None
        self.busy = True
        self._fill_readings(self.preview_table, None)
        self.preview_text.set("Taking a fresh whole-leg preview… hold all three joints still.")
        self.client.request(operation, self.leg, self._preview_received)
        self._update_controls()

    def _preview_received(self, reply):
        self.busy = False
        if reply["type"] == "preview":
            self.preview = reply
            endpoint = "HOME" if reply["endpoint"] == "home" else "OTHER END"
            self._fill_readings(self.preview_table, reply["joints"])
            self.preview_text.set(f"Frozen {endpoint} preview · {LEGS[reply['leg']]} · " + ("ready to inspect" if reply["ok"] else "REJECTED"))
            self.status.set("Inspect all three positions and voltage rows. Accept records this exact preview in RAM." if reply["ok"] else
                            "Capture rejected: an invalid joint is marked red. Accepted records are unchanged.")
        else:
            self.status.set(reply.get("message", "Preview failed."))
        self._update_controls()

    def accept_preview(self):
        if not self.ready or self.busy or not self.preview or not self.preview["ok"]:
            return
        self.busy = True
        self.client.request("ACCEPT", self.preview["leg"], self._accepted, self.preview["token"])
        self._update_controls()

    def _accepted(self, reply):
        self.preview = None
        if reply["ok"]:
            self.preview_text.set("Preview accepted into ESP32 RAM · all 3 joints together")
            self.status.set("Accepted. The ESP32 did not resample. Export a backup before reset or power loss.")
        else:
            self.preview_text.set("Preview no longer available — capture again")
            self.status.set(reply.get("message", "Accept failed; capture again."))
        self._get_records()

    def discard_preview(self):
        self.busy = True
        self.preview = None
        self._fill_readings(self.preview_table, None)
        self.client.request("CANCEL", self.leg, self._discarded)
        self._update_controls()

    def _discarded(self, reply):
        self.busy = False
        self.preview_text.set("Preview discarded. Accepted endpoints are unchanged.")
        self._update_controls()

    @staticmethod
    def _number(value, digits):
        return "—" if value is None else f"{value:.{digits}f}"

    def _fill_readings(self, tree, rows):
        for i, joint in enumerate(JOINTS):
            if rows is None:
                values, tags = (joint,) + ("—",) * 7, ()
            else:
                row = rows[i]
                values = (joint, f"S{row['servo']:02d} / {row['gpio']}", self._number(row['mean_V'], 4),
                          self._number(row['min_V'], 4), self._number(row['max_V'], 4), self._number(row['spread_mV'], 1),
                          self._number(row['angle_deg'], 2), "In table" if row['valid'] else "INVALID")
                tags = () if row['valid'] else ("invalid",)
            tree.item(str(i), values=values, tags=tags)

    def _fill_saved(self, record):
        for i, joint in enumerate(JOINTS):
            values = [joint]
            for endpoint in ("home", "other"):
                rows = record.get(endpoint) if record else None
                values.extend((self._number(rows[i]['mean_V'], 4), self._number(rows[i]['angle_deg'], 2)) if rows else ("Not recorded", "—"))
            self.saved_table.item(str(i), values=values)

    def export_records(self):
        path = filedialog.asksaveasfilename(title="Export accepted endpoints", defaultextension=".json",
                                          initialfile="leg_endpoints.json", filetypes=[("JSON backup", "*.json")])
        if not path or not self.ready:
            return
        self.busy = True
        self._update_controls()
        exported = {}

        def collect(reply):
            if not reply["ok"]:
                raise ValueError("Export failed: the board did not return its records.")
            exported[LEGS[reply["leg"]]] = {"home": reply["home"], "other": reply["other"]}
            if len(exported) == 4:
                data = {"format": "leg-endpoints/1", "exported_at": datetime.now(timezone.utc).isoformat(), "legs": exported}
                try:
                    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
                    self.status.set(f"Exported accepted records for all four legs to {path}")
                except OSError as exc:
                    self.status.set(f"Could not write the backup: {exc}")
                self.busy = False
                self._update_controls()

        for leg in range(4):
            self.client.request("GET", leg, collect)

    def _tick(self):
        if self.closed:
            return
        try:
            if self.client:
                self.client.poll()
                if self.ready and not self.busy and self.live.get() and self.client.idle and time.monotonic() >= self.next_read:
                    self.read_once()
                    self.next_read = time.monotonic() + 0.4
        except (OSError, ValueError, TimeoutError, KeyError, TypeError) as exc:
            self.disconnect(f"Connection stopped: {exc}")
        self.tick_job = self.root.after(40, self._tick)

    def close(self):
        self.closed = True
        for job in (self.tick_job, self.handshake_job):
            if job is not None:
                self.root.after_cancel(job)
        if self.client:
            try:
                self.client.close()
            except OSError:
                pass
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    app = CalibrationApp(window)
    window.mainloop()
