"""
config_gui.py — LipOpenSink_Passive configuration GUI (Tkinter + pyserial)

Features
--------
- Auto-connect: scans serial ports, probes with "VER?" and opens the matching device.
- Robust command layer with timeouts and telemetry noise filtering.
- Live telemetry panel (VIN, TEMPADC, Temperature, FanIdx, Duty, LOAD, PG, MODE).
- Parameter editor with type/units validation and inline tooltips (hover bubbles).
- LUT management (pull/push) kept minimal for clarity; extend as needed.
- Start/Stop logging handled by the host application (separate feature).

Requirements
------------
- Python 3.9+
- pyserial
- tkinter (ships with CPython on Windows/macOS; on Linux: `sudo apt install python3-tk`)

Usage
-----
    pip install pyserial
    python config_tool/config_gui.py

This GUI talks to firmware built from code.py/uart_cmd.py in this repo.

License: MIT
"""

from __future__ import annotations

import threading
import time
import queue
import re
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

import serial  # type: ignore
from serial.tools import list_ports  # type: ignore
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

APP_NAME = "LipOpenSink_Passive Config"
EXPECTED_NAME = "LipOpenSink_Passive"
MIN_FW = (1, 3, 0)  # minimal compatible firmware version (major, minor, patch)

CSV_HEADER = [
    "vin_v",
    "tempadc_v",
    "temp_c",
    "fan_idx",
    "duty_pct",
    "load",
    "pg",
    "mode",
]

PARAM_SCHEMA = {
    # key: (label, tooltip, validator)
    "MIN_FAN_DUTY": (
        "Min fan duty",
        "Duty applied below first temperature threshold. Range: 0.0 – 1.0",
        lambda s: 0.0 <= float(s) <= 1.0,
    ),
    "HYST_C": (
        "Hysteresis (°C)",
        "Temperature hysteresis for fan steps and load re-enable. Typical: 5°C",
        lambda s: 0.0 <= float(s) <= 50.0,
    ),
    "LOAD_TRIP_C": (
        "Load cutoff (°C)",
        "Temperature to disable LOAD_EN. LOAD re-enables at (cutoff - hysteresis)",
        lambda s: 20.0 <= float(s) <= 130.0,
    ),
    "FAST_DT_MS": (
        "Fast loop (ms)",
        "Control/ADC loop period in milliseconds. Typical: 5",
        lambda s: 1 <= int(float(s)) <= 100,
    ),
    "FC_TEMP_HZ": (
        "Temp LPF (Hz)",
        "IIR cutoff for temperature channel. Lower = more smoothing",
        lambda s: 0.0 <= float(s) <= 5.0,
    ),
    "FC_VIN_HZ": (
        "Vin LPF (Hz)",
        "IIR cutoff for VIN channel. Lower = more smoothing",
        lambda s: 0.0 <= float(s) <= 5.0,
    ),
    "RAMP_TIME_MS": (
        "Ramp time (ms)",
        "Time to move from current to new duty target",
        lambda s: 0 <= int(float(s)) <= 60000,
    ),
    "RAMP_STEP_MS": (
        "Ramp update (ms)",
        "Duty update interval during ramp",
        lambda s: 1 <= int(float(s)) <= 5000,
    ),
    "TELEM_RATE_MS": (
        "Telemetry (ms)",
        "Period of device telemetry. 0 disables. GUI sets this automatically",
        lambda s: 0 <= int(float(s)) <= 10000,
    ),
    "TELEM_FORMAT": (
        "Telemetry format",
        "CSV or HUMAN. GUI uses CSV for easier parsing",
        lambda s: s.strip().upper() in ("CSV", "HUMAN"),
    ),
    "TEMP_CAL_A": (
        "Temp cal. A",
        "Calibration coefficient A for temperature voltage: y=A*x+B",
        lambda s: -10.0 <= float(s) <= 10.0,
    ),
    "TEMP_CAL_B": (
        "Temp cal. B",
        "Calibration offset B for temperature voltage (Volts)",
        lambda s: -2.0 <= float(s) <= 2.0,
    ),
    "VIN_CAL_A": (
        "Vin cal. A",
        "Calibration coefficient A for VIN: y=A*x+B",
        lambda s: -10.0 <= float(s) <= 10.0,
    ),
    "VIN_CAL_B": (
        "Vin cal. B",
        "Calibration offset B for VIN (Volts)",
        lambda s: -5.0 <= float(s) <= 5.0,
    ),
}

PARAM_ORDER = list(PARAM_SCHEMA.keys())


# ------------------------------
# Utilities
# ------------------------------

def parse_version(vs: str) -> Tuple[int, int, int]:
    """Parse version string like '1.3.0' into a tuple.

    Args:
        vs: Version string.
    Returns:
        (major, minor, patch)
    """
    m = re.match(r"(\d+)\.(\d+)\.(\d+)", vs.strip())
    if not m:
        return (0, 0, 0)
    return tuple(int(x) for x in m.groups())  # type: ignore


# ------------------------------
# Serial device wrapper
# ------------------------------

class Dev:
    """Serial device wrapper for the firmware protocol.

    One background thread reads the serial port and *classifies lines*:
      - Telemetry (CSV or HUMAN) → telem_queue
      - Command responses (OK/ERR/VER/… and payload) → cmd_queue

    This prevents races between live telemetry and command requests.
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        # Two queues so telemetry doesn't steal command responses
        self.cmd_queue: "queue.Queue[str]" = queue.Queue()
        self.telem_queue: "queue.Queue[str]" = queue.Queue()
        self.rx_thread: Optional[threading.Thread] = None
        self.rx_stop = threading.Event()

    # --- lifecycle ---
    def open(self) -> None:
        """Open serial port and start RX thread."""
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout, write_timeout=1.0)
        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self) -> None:
        """Stop RX thread and close serial port."""
        self.rx_stop.set()
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    # --- low-level ---
    def write_line(self, line: str) -> None:
        """Write a line terminated with LF (device accepts CR/LF/CRLF)."""
        if not self.ser:
            raise RuntimeError("Port not open")
        data = (line.rstrip("\r\n") + "\n").encode("utf-8")
        self.ser.write(data)
        self.ser.flush()

    def _is_telem(self, s: str) -> bool:
        return ("," in s and s.count(",") >= 3) or s.startswith("VIN=")

    def _rx_loop(self) -> None:
        """Background read loop that classifies lines into cmd/telem queues."""
        assert self.ser is not None
        while not self.rx_stop.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                s = raw.decode("utf-8", errors="replace").strip()
                if not s:
                    continue
                if self._is_telem(s):
                    self.telem_queue.put(s)
                else:
                    self.cmd_queue.put(s)
            except Exception:
                time.sleep(0.05)

    def cmd(self, line: str, expect_multiline: bool = False, deadline_s: float = 2.0) -> Tuple[str, List[str]]:
        """Send a command and wait for a response from the *cmd_queue*.

        Telemetry is handled separately and never consumes these responses.
        """
        if not self.ser:
            raise RuntimeError("Port not open")
        # drain old responses (but leave telemetry alone)
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except Exception:
                break
        self.write_line(line)
        t0 = time.time()
        first = None
        payload: List[str] = []
        while time.time() - t0 < deadline_s:
            try:
                s = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if first is None:
                first = s
                if not expect_multiline:
                    return first, []
            else:
                payload.append(s)
                if s == "END":
                    break
        if first is None:
            raise RuntimeError("Timeout waiting for response")
        return first, payload

    # --- high-level helpers ---
    def telem_on(self, rate_ms: int = 250, fmt: str = "CSV") -> None:
        self.cmd(f"TELEM FORMAT {fmt}")
        self.cmd(f"TELEM RATE {rate_ms}")

    def telem_off(self) -> None:
        self.cmd("TELEM RATE 0")

    def get_params(self) -> Dict[str, str]:
        first, lines = self.cmd("GET PARAMS", expect_multiline=True, deadline_s=2.0)
        if not first.startswith("OK"):
            raise RuntimeError(first)
        out: Dict[str, str] = {}
        for s in lines:
            if s == "END":
                break
            if "=" in s:
                k, v = s.split("=", 1)
                out[k.strip()] = v.strip()
        return out

    def set_param(self, key: str, value: str) -> None:
        first, _ = self.cmd(f"SET {key} {value}")
        if not first.startswith("OK"):
            raise RuntimeError(first)

    def save(self) -> str:
        first, _ = self.cmd("SAVE", expect_multiline=False, deadline_s=3.0)
        if first.startswith("OK"):
            return first
        raise RuntimeError(first)

# ------------------------------
# Tk helpers: tooltip bubbles
# ------------------------------

class ToolTip:
    """Simple hover tooltip for Tk widgets."""

    def __init__(self, widget, text: str):
        self.widget = widget
        self.text = text
        self.tipwindow: Optional[tk.Toplevel] = None
        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, _event=None):
        if self.tipwindow or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
        self.tipwindow = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(
            tw,
            text=self.text,
            justify=tk.LEFT,
            relief=tk.SOLID,
            borderwidth=1,
            background="#ffffe0",
            padx=6,
            pady=4,
            font=("TkDefaultFont", 9),
            wraplength=320,
        )
        label.pack(ipadx=1)

    def hide(self, _event=None):
        if self.tipwindow:
            self.tipwindow.destroy()
            self.tipwindow = None


# ------------------------------
# Tk Application
# ------------------------------

class App(ttk.Frame):
    """Main Tk application for LipOpenSink_Passive configuration.

    - Auto-connect on startup (scans COM ports, asks VER?).
    - Live telemetry panel updated from a dedicated queue.
    - Parameter grid with validation and tooltips.
    - Calibration helpers for VIN and Temperature.
    """

    def __init__(self, master):
        super().__init__(master)
        self.master.title(APP_NAME)
        self.grid(sticky="nsew")
        for i in range(2):
            self.master.columnconfigure(i, weight=1)
        self.master.rowconfigure(0, weight=1)

        self.dev: Optional[Dev] = None
        self.telemetry_running = False
        self.telemetry_thread: Optional[threading.Thread] = None
        self.telemetry_stop = threading.Event()

        # Top bar: port selection + connect controls
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")

        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=6)
        bar.columnconfigure(3, weight=1)

        ttk.Label(bar, text="Port:").grid(row=0, column=0, padx=4)
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=20, values=self._list_ports())
        self.port_cb.grid(row=0, column=1, padx=4)
        ttk.Button(bar, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Label(bar, text="Baud:").grid(row=0, column=3, padx=4, sticky="e")
        ttk.Entry(bar, textvariable=self.baud_var, width=8).grid(row=0, column=4, padx=4)
        ttk.Button(bar, text="Auto-Connect", command=self.on_autoconnect).grid(row=0, column=5, padx=6)
        ttk.Button(bar, text="Connect", command=self.on_connect).grid(row=0, column=6, padx=6)
        ttk.Button(bar, text="Disconnect", command=self.on_disconnect).grid(row=0, column=7, padx=6)

        # Live Telemetry Panel
        telem = ttk.LabelFrame(self, text="Live Telemetry")
        telem.grid(row=1, column=0, sticky="nsew", padx=8, pady=6)
        self.telemlabels: Dict[str, tk.StringVar] = {}
        fields = [
            ("VIN (V)", "vin_v"),
            ("TEMPADC (V)", "tempadc_v"),
            ("TEMP (°C)", "temp_c"),
            ("FanIdx", "fan_idx"),
            ("Duty (%)", "duty_pct"),
            ("LOAD", "load"),
            ("PG", "pg"),
            ("Mode", "mode"),
        ]
        for r, (label, key) in enumerate(fields):
            ttk.Label(telem, text=label+":").grid(row=r, column=0, sticky="w", padx=6, pady=2)
            var = tk.StringVar(value="-")
            ttk.Label(telem, textvariable=var).grid(row=r, column=1, sticky="w", padx=6, pady=2)
            self.telemlabels[key] = var

        # Parameters Panel
        params_frame = ttk.LabelFrame(self, text="Parameters")
        params_frame.grid(row=1, column=1, sticky="nsew", padx=8, pady=6)
        self.param_vars: Dict[str, tk.StringVar] = {}
        for i, key in enumerate(PARAM_ORDER):
            label, tip, _validator = PARAM_SCHEMA[key]
            ttk.Label(params_frame, text=label+":").grid(row=i, column=0, sticky="e", padx=6, pady=2)
            var = tk.StringVar(value="")
            ent = ttk.Entry(params_frame, textvariable=var, width=14)
            ent.grid(row=i, column=1, sticky="w", padx=6, pady=2)
            self.param_vars[key] = var
            ToolTip(ent, f"{label}: {tip}\nKey: {key}")

        # Bottom bar: actions
        actions = ttk.Frame(self)
        actions.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        ttk.Button(actions, text="Refresh Params", command=self.on_refresh).grid(row=0, column=0, padx=4)
        ttk.Button(actions, text="Apply Params", command=self.on_apply).grid(row=0, column=1, padx=4)
        ttk.Button(actions, text="Save to Flash", command=self.on_save).grid(row=0, column=2, padx=4)
        ttk.Button(actions, text="Fan Auto", command=lambda: self._send('FAN AUTO')).grid(row=0, column=3, padx=4)
        ttk.Button(actions, text="Fan 40%", command=lambda: self._send('FAN DUTY 0.4')).grid(row=0, column=4, padx=4)
        ttk.Button(actions, text="Quiet", command=lambda: self._send('QUIET')).grid(row=0, column=5, padx=4)
        # Calibration buttons
        ttk.Button(actions, text="Cal VIN (1pt)", command=self.on_cal_vin_1pt).grid(row=0, column=6, padx=8)
        ttk.Button(actions, text="Cal VIN (2pt)", command=self.on_cal_vin_2pt).grid(row=0, column=7, padx=4)
        ttk.Button(actions, text="Cal TEMP (1pt)", command=self.on_cal_temp_1pt).grid(row=0, column=8, padx=8)

        # Status bar
        self.status = tk.StringVar(value="Disconnected")
        ttk.Label(self, textvariable=self.status, anchor="w").grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=(0,8))

        # Expand behavior
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

    # ---- port management ----
    def _list_ports(self) -> List[str]:
        return [p.device for p in list_ports.comports()]

    def _refresh_ports(self) -> None:
        self.port_cb["values"] = self._list_ports()

    # ---- connect / disconnect ----
    def on_connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning(APP_NAME, "Choose a serial port or use Auto-Connect")
            return
        try:
            self._open_dev(port)
            self.status.set(f"Connected on {port}")
        except Exception as e:
            messagebox.showerror("Connect", str(e))
            self.status.set("Disconnected")

    def on_autoconnect(self) -> None:
        """Scan COM ports, probe with VER?, connect when compatible."""
        ports = self._list_ports()
        if not ports:
            messagebox.showinfo(APP_NAME, "No serial ports found.")
            return
        for port in ports:
            try:
                tmp = Dev(port, int(self.baud_var.get()))
                tmp.open()
                # small settle
                time.sleep(0.2)
                # QUIET (in case telemetry is noisy)
                try:
                    tmp.cmd("QUIET", deadline_s=0.8)
                except Exception:
                    pass
                # Ask version
                first, _ = tmp.cmd("VER?", deadline_s=1.2)
                if first.startswith("VER "):
                    ver = first.split(" ", 1)[1].strip()
                    vtuple = parse_version(ver)
                    if vtuple >= MIN_FW:
                        # Found compatible device
                        tmp.close()
                        self.port_var.set(port)
                        self._open_dev(port)
                        self.status.set(f"Auto-connected on {port} (FW {ver})")
                        return
                tmp.close()
            except Exception:
                try:
                    tmp.close()
                except Exception:
                    pass
                continue
        messagebox.showwarning(APP_NAME, "No compatible device found.")

    def _open_dev(self, port: str) -> None:
        if self.dev:
            try: self.dev.close()
            except Exception: pass
        self.dev = Dev(port, int(self.baud_var.get()))
        self.dev.open()
        # sanity: QUIET + PING
        try: self.dev.cmd("QUIET", deadline_s=0.8)
        except Exception: pass
        first, _ = self.dev.cmd("PING", deadline_s=0.8)
        if not (first.startswith("PONG") or first.startswith("OK")):
            raise RuntimeError(f"Unexpected PING response: {first}")
        # Enable telemetry for live view
        self.dev.telem_on(rate_ms=250, fmt="CSV")
        self._start_telem_reader()
        self.on_refresh()

    def on_disconnect(self) -> None:
        self._stop_telem_reader()
        if self.dev:
            try:
                self.dev.telem_off()
                self.dev.close()
            except Exception:
                pass
        self.dev = None
        self.status.set("Disconnected")

    # ---- telemetry ----
    def _start_telem_reader(self) -> None:
        if self.telemetry_running:
            return
        self.telemetry_stop.clear()
        self.telemetry_thread = threading.Thread(target=self._telem_loop, daemon=True)
        self.telemetry_thread.start()
        self.telemetry_running = True

    def _stop_telem_reader(self) -> None:
        self.telemetry_stop.set()
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=1.0)
        self.telemetry_running = False

    def _telem_loop(self) -> None:
        assert self.dev is not None
        q = self.dev.telem_queue
        while not self.telemetry_stop.is_set():
            try:
                s = q.get(timeout=0.25)
            except queue.Empty:
                continue
            if not s:
                continue
            # Parse CSV telemetry only
            if "," in s and s.count(",") >= 3:
                parts = s.split(",")
                if len(parts) >= len(CSV_HEADER):
                    data = dict(zip(CSV_HEADER, parts))
                    self._update_telem_labels(data)

    def _update_telem_labels(self, data: Dict[str, str]) -> None:
        def setv(key, fmt=None):
            val = data.get(key, "-")
            if fmt:
                try:
                    val = fmt(float(val))
                except Exception:
                    pass
            self.telemlabels[key].set(str(val))
        # Numerical formatting
        setv("vin_v", lambda x: f"{x:.3f}")
        setv("tempadc_v", lambda x: f"{x:.3f}")
        setv("temp_c", lambda x: f"{x:.1f}")
        setv("fan_idx")
        setv("duty_pct", lambda x: f"{x:.0f}")
        setv("load")
        setv("pg")
        setv("mode")

    # ---- Calibration Helpers ----
    def _get_params_dict(self) -> Dict[str, float]:
        p = self.dev.get_params()
        out: Dict[str, float] = {}
        for k, v in p.items():
            try:
                out[k] = float(v) if k != "TELEM_FORMAT" else v
            except Exception:
                pass
        return out

    def _raw_volts(self) -> Tuple[float, float]:
        # Requires firmware support: RAW? → OK + one payload line: "vin_raw_v,tempadc_raw_v"
        first, lines = self.dev.cmd("RAW?", expect_multiline=True, deadline_s=1.0)
        if not first.startswith("OK"):
            raise RuntimeError(first)
        line = lines[0] if lines else ""
        parts = line.split(",")
        if len(parts) < 2:
            raise RuntimeError("RAW? bad payload")
        return float(parts[0]), float(parts[1])

    def _tempv_for(self, t_c: float) -> float:
        # Requires firmware support: TEMPV? <T> → OK + one payload line with volts
        first, lines = self.dev.cmd(f"TEMPV? {t_c}", expect_multiline=True, deadline_s=1.0)
        if not first.startswith("OK"):
            raise RuntimeError(first)
        line = lines[0] if lines else ""
        return float(line.strip())

    def on_cal_vin_1pt(self) -> None:
        if not self.dev:
            return
        try:
            s = tk.simpledialog.askstring(APP_NAME, "Enter actual VIN (Volts):")
            if not s:
                return
            v_true = float(s)
            p = self._get_params_dict()
            a = float(p.get("VIN_CAL_A", 1.0))
            b = float(p.get("VIN_CAL_B", 0.0))
            vin_raw, _ = self._raw_volts()
            # target: a*vin_raw + b_new = v_true ⇒ b_new = v_true - a*vin_raw
            b_new = v_true - a * vin_raw
            self.dev.set_param("VIN_CAL_B", str(b_new))
            self.on_refresh()
            messagebox.showinfo("Cal VIN (1pt)", f"VIN_CAL_B set to {b_new:.6f}")
        except Exception as e:
            messagebox.showerror("Cal VIN (1pt)", str(e))

    def on_cal_vin_2pt(self) -> None:
        if not self.dev:
            return
        try:
            s1 = tk.simpledialog.askstring(APP_NAME, "Enter actual VIN #1 (Volts), then click OK:")
            if not s1:
                return
            v1 = float(s1)
            vin_raw1, _ = self._raw_volts()
            messagebox.showinfo("Cal VIN (2pt)", "Now change to the second known VIN and press OK")
            s2 = tk.simpledialog.askstring(APP_NAME, "Enter actual VIN #2 (Volts), then click OK:")
            if not s2:
                return
            v2 = float(s2)
            vin_raw2, _ = self._raw_volts()
            if abs(vin_raw2 - vin_raw1) < 1e-6:
                raise RuntimeError("Raw readings too close; choose distinct points.")
            a_new = (v2 - v1) / (vin_raw2 - vin_raw1)
            b_new = v1 - a_new * vin_raw1
            self.dev.set_param("VIN_CAL_A", str(a_new))
            self.dev.set_param("VIN_CAL_B", str(b_new))
            self.on_refresh()
            messagebox.showinfo("Cal VIN (2pt)", f"VIN_CAL_A={a_new:.6f}\nVIN_CAL_B={b_new:.6f}")
        except Exception as e:
            messagebox.showerror("Cal VIN (2pt)", str(e))

    def on_cal_temp_1pt(self) -> None:
        if not self.dev:
            return
        try:
            s = tk.simpledialog.askstring(APP_NAME, "Enter actual Temperature (°C):")
            if not s:
                return
            t_true = float(s)
            p = self._get_params_dict()
            a = float(p.get("TEMP_CAL_A", 1.0))
            b = float(p.get("TEMP_CAL_B", 0.0))
            _, t_raw_v = self._raw_volts()
            v_target = self._tempv_for(t_true)
            # a*t_raw_v + b_new = v_target ⇒ b_new = v_target - a*t_raw_v
            b_new = v_target - a * t_raw_v
            self.dev.set_param("TEMP_CAL_B", str(b_new))
            self.on_refresh()
            messagebox.showinfo("Cal TEMP (1pt)", f"TEMP_CAL_B set to {b_new:.6f} V")
        except Exception as e:
            messagebox.showerror("Cal TEMP (1pt)", str(e))

    # ---- parameter flows ----
    def on_refresh(self) -> None:
        if not self.dev:
            return
        try:
            p = self.dev.get_params()
            for k, var in self.param_vars.items():
                var.set(str(p.get(k, "")))
            self.status.set("Parameters loaded")
        except Exception as e:
            messagebox.showerror("Refresh", str(e))

    def on_apply(self) -> None:
        if not self.dev:
            return
        try:
            # validate
            for k, var in self.param_vars.items():
                val = var.get().strip()
                label, tip, validator = PARAM_SCHEMA[k]
                if not validator(val):
                    raise ValueError(f"Invalid value for {k}: {val}\n{tip}")
            # push
            for k, var in self.param_vars.items():
                self.dev.set_param(k, var.get().strip())
            self.status.set("Parameters applied")
        except Exception as e:
            messagebox.showerror("Apply", str(e))

    def on_save(self) -> None:
        if not self.dev:
            return
        try:
            info = self.dev.save()
            messagebox.showinfo("Save", f"Saved: {info}")
        except Exception as e:
            messagebox.showerror("Save", str(e))

    # ---- misc ----
    def _send(self, line: str) -> None:
        if not self.dev:
            return
        try:
            self.dev.cmd(line)
        except Exception as e:
            messagebox.showerror("Command", str(e))


# ------------------------------
# main
# ------------------------------

def main() -> None:
    """Entry point."""
    root = tk.Tk()
    style = ttk.Style(root)
    if sys.platform.startswith("win"):
        style.theme_use("winnative")
    root.geometry("880x520")
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.on_disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
