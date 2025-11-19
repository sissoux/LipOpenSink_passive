# Licensed under CERN-OHL-S-2.0
# © 2025 ADSTech [e-ddy]
# https://ohwr.org/licences/

"""
uart_cmd.py — Dual-CDC UART + command server for E-DDY.

This module abstracts USB CDC I/O (console + data) and implements a small,
line-oriented ASCII protocol with a set of built-in commands. Application logic
(params, LUTs, telemetry control, persistence, etc.) is provided via a backend
object passed to `CommandServer`.

Key features:
- Reads from BOTH usb_cdc.console and usb_cdc.data (if available).
- Accepts CR, LF, or CRLF as line terminators.
- Writes responses to both ports using CRLF (avoids "stair-stepping").
- Minimal command parser with BEGIN/END block mode for LUT uploads.
- Helpers to write OK/ERR with optional payload and END.

Application must provide a backend object with the following callables:
- version() -> str
- status() -> dict[str, bool]                 # e.g., {"console": True, "data": True}
- get_params() -> list[str]                   # ["KEY=VALUE", ...]
- set_param(key: str, value: str) -> None
- get_lut_temp_to_duty() -> list[tuple[float,float]]
- set_lut_temp_to_duty(rows: list[tuple[float,float]]) -> None
- get_lut_adc_to_temp() -> list[tuple[int,float]]
- set_lut_adc_to_temp(rows: list[tuple[int,float]]) -> None
- fan_auto() -> None
- fan_duty(d: float) -> None
- telem_rate(ms: int) -> None
- telem_format(fmt: str) -> None              # "CSV" or "HUMAN"
- save_settings() -> tuple[bool, str]         # (ok, info)
- load_defaults() -> None
- reboot() -> None

The server will call these and handle I/O formatting.
"""

try:
    import usb_cdc
    try:
        # harmless if already enabled (e.g., via boot.py)
        usb_cdc.enable(console=True, data=True)
    except Exception:
        pass
except Exception:
    usb_cdc = None


class DualCDC:
    """Minimal dual-CDC UART abstraction with CRLF transmit and CR/LF tolerant RX."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.console = getattr(usb_cdc, "console", None) if usb_cdc else None
        self.data = getattr(usb_cdc, "data", None) if usb_cdc else None

    def read_into_buffer(self) -> None:
        """Drain available bytes from both CDC endpoints into the internal buffer."""
        if self.console and self.console.in_waiting:
            try:
                self.buf += self.console.read(self.console.in_waiting)
            except Exception:
                pass
        if self.data and self.data.in_waiting:
            try:
                self.buf += self.data.read(self.data.in_waiting)
            except Exception:
                pass

    def readline(self):
        """Return a decoded line (str) if a CR or LF is found, else None."""
        self.read_into_buffer()
        if not self.buf:
            return None
        cr = self.buf.find(b"\r")
        lf = self.buf.find(b"\n")
        idx = -1
        if cr != -1 and lf != -1:
            idx = cr if cr < lf else lf
        elif cr != -1:
            idx = cr
        elif lf != -1:
            idx = lf
        if idx == -1:
            return None
        line = self.buf[:idx]
        drop = 1
        if idx + 1 < len(self.buf) and self.buf[idx + 1:idx + 2] in (b"\r", b"\n"):
            drop = 2
        self.buf = self.buf[idx + drop :]
        try:
            return line.decode("utf-8", "replace")
        except Exception:
            return None

    def write_line(self, s: str) -> None:
        """Write a single line with CRLF. Broadcast to both endpoints; fallback to print()."""
        payload = (s + "\r\n").encode("utf-8")
        wrote = False
        if self.data:
            try:
                self.data.write(payload)
                wrote = True
            except Exception:
                pass
        if self.console:
            try:
                self.console.write(payload)
                wrote = True
            except Exception:
                pass
        if not wrote:
            # As a last resort, mirror to print so something is visible on REPL.
            print(s)

    def endpoints_connected(self) -> dict:
        """Return connection status for console and data endpoints."""
        return {
            "console": getattr(getattr(usb_cdc, "console", None), "connected", False),
            "data": getattr(getattr(usb_cdc, "data", None), "connected", False),
        }


class CommandServer:
    """
    Line-based command server.

    Parses ASCII commands terminated by CR/LF, calls backend methods, and writes
    OK/ERR responses. Supports block mode for LUT uploads:

    - SET LUT TEMP_TO_DUTY BEGIN <count>
      <idx,temp,duty>
      ...
      SET LUT TEMP_TO_DUTY END

    - SET LUT ADC_TO_TEMP_5C BEGIN <count>
      <idx,adc,temp>
      ...
      SET LUT ADC_TO_TEMP_5C END
    """

    def __init__(self, io: DualCDC, backend) -> None:
        """
        Args:
            io: DualCDC instance for I/O.
            backend: application backend with the required methods (see module docstring).
        """
        self.io = io
        self.backend = backend
        # block mode state
        self.rx_mode = None            # "TD" | "AT" | None
        self.rx_count_expected = 0
        self.rx_rows = []

    # ---------- helpers ----------
    def ok(self, lines=None, end=False):
        """Write OK; optional payload lines; optional END terminator."""
        self.io.write_line("OK")
        if lines:
            for s in lines:
                self.io.write_line(s)
        if end:
            self.io.write_line("END")

    def err(self, code: int, msg: str, end=False):
        """Write an error line. Example: ERR 400 PARSE"""
        self.io.write_line("ERR {} {}".format(code, msg))
        if end:
            self.io.write_line("END")

    # ---------- API ----------
    def poll(self) -> None:
        """Process at most one incoming line (non-blocking)."""
        line = self.io.readline()
        if line is None:
            return
        self._handle_line(line.strip())

    # ---------- internal ----------
    def _handle_line(self, s: str) -> None:
        if not s:
            return

        # --- Block receive mode for LUT uploads ---
        if self.rx_mode == "TD":
            up = s.upper()
            if up.startswith("SET LUT TEMP_TO_DUTY END"):
                # finalize
                try:
                    rows = []
                    for row in self.rx_rows:
                        idx_s, t_s, d_s = [x.strip() for x in row.split(",")]
                        rows.append((int(idx_s), float(t_s), float(d_s)))
                    if len(rows) != self.rx_count_expected:
                        self.err(400, "COUNT_MISMATCH")
                    else:
                        rows.sort(key=lambda x: x[0])
                        new_td = [(t, d) for (_, t, d) in rows]
                        self.backend.set_lut_temp_to_duty(new_td)
                        self.ok()
                except Exception as e:
                    self.err(400, "PARSE:{}".format(e))
                # reset mode
                self.rx_mode = None
                self.rx_rows = []
                self.rx_count_expected = 0
                return
            # normal row
            self.rx_rows.append(s)
            self.io.write_line("OK")
            return

        if self.rx_mode == "AT":
            up = s.upper()
            if up.startswith("SET LUT ADC_TO_TEMP_5C END"):
                try:
                    rows = []
                    for row in self.rx_rows:
                        idx_s, a_s, t_s = [x.strip() for x in row.split(",")]
                        rows.append((int(idx_s), int(a_s), float(t_s)))
                    if len(rows) != self.rx_count_expected:
                        self.err(400, "COUNT_MISMATCH")
                    else:
                        rows.sort(key=lambda x: x[0])
                        new_at = [(a, t) for (_, a, t) in rows]
                        self.backend.set_lut_adc_to_temp(new_at)
                        self.ok()
                except Exception as e:
                    self.err(400, "PARSE:{}".format(e))
                self.rx_mode = None
                self.rx_rows = []
                self.rx_count_expected = 0
                return
            self.rx_rows.append(s)
            self.io.write_line("OK")
            return

        # --- Stateless commands ---
        up = s.upper()

        if up == "PING":
            self.io.write_line("PONG")
            return

        if up == "QUIET":
            self.backend.telem_rate(0)
            self.ok()
            return

        if up == "VER?":
            self.io.write_line("VER {}".format(self.backend.version()))
            return

        if up == "STATUS?":
            st = self.backend.status()
            self.ok(
                [
                    "CONSOLE_CONNECTED={}".format(st.get("console", False)),
                    "DATA_CONNECTED={}".format(st.get("data", False)),
                ],
                end=True,
            )
            return

        if up == "HELP":
            self.ok(
                [
                    "PING",
                    "QUIET",
                    "VER?",
                    "HELP",
                    "STATUS?",
                    "GET PARAMS",
                    "SET <KEY> <VALUE>",
                    "GET LUT TEMP_TO_DUTY",
                    "SET LUT TEMP_TO_DUTY BEGIN <count>  (idx,temp,duty ... )  SET LUT TEMP_TO_DUTY END",
                    "GET LUT ADC_TO_TEMP_5C",
                    "SET LUT ADC_TO_TEMP_5C BEGIN <count> (idx,adc,temp ... ) SET LUT ADC_TO_TEMP_5C END",
                    "FAN AUTO",
                    "FAN DUTY <0.0..1.0>",
                    "TELEM RATE <ms>",
                    "TELEM FORMAT CSV|HUMAN",
                    "SAVE",
                    "DEFAULTS",
                    "REBOOT",
                ]
            )
            return

        if up == "GET PARAMS":
            self.ok(self.backend.get_params(), end=True)
            return

        if up.startswith("SET "):
            try:
                _, key, val = s.split(None, 2)
                self.backend.set_param(key, val)
                self.ok()
            except Exception as e:
                self.err(400, "PARSE:{}".format(e))
            return

        if up == "GET LUT TEMP_TO_DUTY":
            rows = self.backend.get_lut_temp_to_duty()
            out = ["COUNT {}".format(len(rows))]
            for i, (t, d) in enumerate(rows):
                out.append("{},{},{}".format(i, t, d))
            self.ok(out, end=True)
            return

        if up.startswith("SET LUT TEMP_TO_DUTY BEGIN"):
            try:
                cnt = int(s.split()[-1])
                self.rx_mode = "TD"
                self.rx_count_expected = cnt
                self.rx_rows = []
                self.ok()
            except Exception:
                self.err(400, "PARSE")
            return

        if up == "GET LUT ADC_TO_TEMP_5C":
            rows = self.backend.get_lut_adc_to_temp()
            out = ["COUNT {}".format(len(rows))]
            for i, (a, t) in enumerate(rows):
                out.append("{},{},{}".format(i, a, t))
            self.ok(out, end=True)
            return

        if up.startswith("SET LUT ADC_TO_TEMP_5C BEGIN"):
            try:
                cnt = int(s.split()[-1])
                self.rx_mode = "AT"
                self.rx_count_expected = cnt
                self.rx_rows = []
                self.ok()
            except Exception:
                self.err(400, "PARSE")
            return

        if up == "FAN AUTO":
            self.backend.fan_auto()
            self.ok()
            return

        if up.startswith("FAN DUTY"):
            try:
                v = float(s.split(None, 2)[2])
                self.backend.fan_duty(v)
                self.ok()
            except Exception:
                self.err(400, "PARSE")
            return

        if up.startswith("TELEM RATE"):
            try:
                v = int(float(s.split(None, 2)[2]))
                self.backend.telem_rate(v)
                self.ok()
            except Exception:
                self.err(400, "PARSE")
            return

        if up.startswith("TELEM FORMAT"):
            try:
                fmt = s.split(None, 2)[2].strip().upper()
                self.backend.telem_format(fmt)
                self.ok()
            except Exception:
                self.err(400, "PARSE")
            return

        if up == "SAVE":
            ok, info = self.backend.save_settings()
            if ok:
                self.ok([info])
            else:
                self.err(500, info)
            return

        if up == "DEFAULTS":
            self.backend.load_defaults()
            self.ok()
            return

        if up == "REBOOT":
            self.io.write_line("OK")
            self.backend.reboot()
            return

        self.err(400, "UNKNOWN")
