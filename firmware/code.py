"""
code.py — LipOpenSink_Passive main firmware (RP2040, CircuitPython).

Responsibilities:
- Hardware init (ADC, GPIO, PWM).
- Filtering (IIR), calibration (a*x+b), and temperature conversion (LUT).
- Fan policy with LUT + hysteresis + linear ramp; manual override support.
- Load cutoff with hysteresis; LED bonded to LOAD_EN.
- Telemetry generation (OFF by default).
- NVM-backed settings persistence with CRC and adaptive size.
- Integration with uart_cmd.CommandServer for UART protocol.

Requires:
- LUTs.py providing:
    - ADC_TO_TEMP_5C: list[(adc_count:int, temp_c:float)] sorted by decreasing adc
    - TEMP_TO_DUTY:   list[(temp_c:float, duty:float)]   sorted by increasing temp
- uart_cmd.py (this repo) for UART I/O and command parsing.

Optional:
- boot.py with `usb_cdc.enable(console=True, data=True)` to expose 2 CDC ports.

Author: LipOpenSink project
"""

import time
import math
import json
import struct
import microcontroller
import board
import analogio
import digitalio
import pwmio

from LUTs import ADC_TO_TEMP_5C as ADC_TO_TEMP_BOOT, TEMP_TO_DUTY as TEMP_TO_DUTY_BOOT
from uart_cmd import DualCDC, CommandServer

# -------------------------------
# Version / constants
# -------------------------------
FW_VERSION = "1.3.1"  # Bumped version: distinguishes builds with VIN bypass param & GUI sync improvements
VREF = 3.3
ADC_MAX = 65535
SETTINGS_PATH = "/settings.json"  # optional file fallback (read-only while mounted)
BROKEN_TEMP_CUTOFF_C = -25.0  # Safety: if measured temperature is below this, treat as broken NTC and open load

# -------------------------------
# Hardware setup
# -------------------------------
vinADC = analogio.AnalogIn(board.A0)   # GP26 ADC0 — input voltage sense
tempADC = analogio.AnalogIn(board.A1)  # GP27 ADC1 — temperature sense

psu_power_good = digitalio.DigitalInOut(board.GP6)  # PSU power good (pulled-up)
psu_power_good.direction = digitalio.Direction.INPUT
psu_power_good.pull = digitalio.Pull.UP

fan = pwmio.PWMOut(board.GP13, frequency=1000, duty_cycle=0)  # 1 kHz Fan PWM
# NEW: Fan tachometer input (external pull-up)
fan_tach = digitalio.DigitalInOut(board.GP12)
fan_tach.direction = digitalio.Direction.INPUT
load_en = digitalio.DigitalInOut(board.GP28)                   # Load enable
load_en.direction = digitalio.Direction.OUTPUT
led = digitalio.DigitalInOut(board.GP4)                        # LED bonded to LOAD_EN
led.direction = digitalio.Direction.OUTPUT

# -------------------------------
# Parameters (editable via UART)
# -------------------------------
params = {
    # Control / policy
    "HYST_C":        5.0,       # °C hysteresis for steps & cutoff re-enable
    "LOAD_TRIP_C":   85.0,      # °C cutoff threshold
    "MIN_FAN_DUTY":  0.20,      # duty (0..1) when below first temp threshold
    "FAST_DT_MS":    5,         # fast loop (ADC/filter/control)
    "SLOW_DT_MS":    250,       # reserved
    "FC_TEMP_HZ":    0.5,       # IIR cutoff for temperature (Hz)
    "FC_VIN_HZ":     0.5,       # IIR cutoff for Vin (Hz)
    "RAMP_TIME_MS":  1000,      # fan duty ramp duration
    "RAMP_STEP_MS":  50,        # fan duty ramp update period

    # Telemetry (OFF by default)
    "TELEM_RATE_MS": 0,         # 0 = off
    "TELEM_FORMAT":  "HUMAN",   # "CSV" or "HUMAN"

    # Calibration on volts: y = a*x + b
    "TEMP_CAL_A":    1.0,
    "TEMP_CAL_B":    0.0,
    "VIN_CAL_A":     (10.0/100.9),
    "VIN_CAL_B":     0.0,

    # Added parameters for load cutoff on voltage
    "LOAD_TRIP_V":   8.5,       # Voltage cutoff threshold (V)
    "HYST_V":        0.5,       # Voltage hysteresis (V)
    # Bypass VIN cutoff (1.0 = bypass enabled, 0.0 = enforce VIN cutoff). When bypassed, only temperature governs cutoff.
    "BYPASS_VIN_CUTOFF": 1.0,

    # NEW: Fan presence checking / LED blink
    "FAN_CHECK_MIN_DUTY": 0.30,   # Only check tach when commanded duty >= this
    "FAN_SPINUP_MS":     1500,    # Grace after starting fan before checking
    "FAN_TACH_TIMEOUT_MS": 600,   # Fault if no edge seen within this window
    "LED_BLINK_MS":       300,    # Blink period when in fan-fault
}

# -------------------------------
# LUTs (editable via UART)
# -------------------------------
TEMP_TO_DUTY = [(float(t), float(d)) for (t, d) in TEMP_TO_DUTY_BOOT]
TEMP_TO_DUTY.sort(key=lambda x: x[0])
ADC_TO_TEMP_5C = [(int(a), float(t)) for (a, t) in ADC_TO_TEMP_BOOT]
ADC_TO_TEMP_5C.sort(key=lambda x: -x[0])  # descending adc

# -------------------------------
# State variables
# -------------------------------
# Filters (start at raw readings)
raw_vin_f = vinADC.value
raw_temp_f = tempADC.value
last_fast_ns = time.monotonic_ns()

# Fan & load states
lut_index = 0
load_enabled = True
base_load_enabled = True  # NEW: temp/VIN policy before fan interlock
fan_fault = False         # NEW: fan fault flag
temp_fault_low = False    # NEW: broken sensor safety (NTC open → very low apparent temp)

# Fan ramp
duty_target = 0.0
duty_cmd = 0.0
ramp_start_duty = 0.0
ramp_target = 0.0
ramp_start_ms = 0
last_ramp_ms = 0
fan_manual = False  # set true by FAN DUTY; unset by FAN AUTO

# Telemetry
last_telem_ms = 0

# NEW: Fan tach / LED blink state
last_tach_val = False
last_tach_edge_ms = 0
fan_spin_request_ms = 0
last_led_blink_ms = 0
led_blink_state = False

# -------------------------------
# Utility functions
# -------------------------------
def temp_steps():
    """Return the list of temperature thresholds (°C) from TEMP_TO_DUTY."""
    return [t for (t, _) in TEMP_TO_DUTY]


def duties():
    """Return the list of duty values (0..1) from TEMP_TO_DUTY."""
    return [d for (_, d) in TEMP_TO_DUTY]


def clamp(v, lo, hi):
    """Clamp value v within [lo, hi]."""
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def iir_update(prev_y: float, x: float, fc_hz: float, dt_s: float) -> float:
    """
    First-order low-pass (exponential) with cutoff frequency fc_hz.

    Args:
        prev_y: previous filtered value.
        x: new raw sample.
        fc_hz: cutoff frequency (Hz); <=0 means passthrough.
        dt_s: sample interval in seconds.

    Returns:
        New filtered value.
    """
    if dt_s <= 0 or fc_hz <= 0:
        return x
    alpha = 1.0 - math.exp(-2.0 * math.pi * fc_hz * dt_s)
    if alpha < 0.0:
        alpha = 0.0
    if alpha > 1.0:
        alpha = 1.0
    return prev_y + alpha * (x - prev_y)

def calibrateTemp(rawTempValue: float) -> float:
    """
    Convert uncalibrated temperature in °C using the stored linear calibration.

    Args:
        rawTempValue: Temperature input (°C).

    Returns:
        Temperature in °C.
    """
    return params["TEMP_CAL_A"]*rawTempValue+params["TEMP_CAL_B"]

def temp_from_adc(raw_adc_count: int) -> float:
    """
    Convert ADC counts to °C using the 5°C LUT with linear interpolation.

    Args:
        raw_adc_count: ADC reading (0..65535).

    Returns:
        Temperature in °C.
    """
    if raw_adc_count >= ADC_TO_TEMP_5C[0][0]:
        return calibrateTemp(ADC_TO_TEMP_5C[0][1])
    if raw_adc_count <= ADC_TO_TEMP_5C[-1][0]:
        return calibrateTemp(ADC_TO_TEMP_5C[-1][1])
    for i in range(len(ADC_TO_TEMP_5C) - 1):
        a1, t1 = ADC_TO_TEMP_5C[i]
        a2, t2 = ADC_TO_TEMP_5C[i + 1]
        if a2 <= raw_adc_count <= a1:
            ratio = (raw_adc_count - a1) / (a2 - a1)
            return calibrateTemp(t1 + (t2 - t1) * ratio)
    return ADC_TO_TEMP_5C[-1][1]


def update_fan_policy(temp_c: float) -> None:
    """
    Update the current LUT index and target duty based on temperature
    with 5°C hysteresis between steps.

    Behavior:
      - Below first threshold → index 0 with MIN_FAN_DUTY.
      - Climb indices as temp crosses thresholds upwards.
      - Drop one index when temp falls by HYST_C below current threshold.
    """
    global lut_index, duty_target
    steps = temp_steps()
    n = len(steps)

    if temp_c < steps[0]:
        lut_index = 0
        duty_target = params["MIN_FAN_DUTY"]
        return

    while lut_index < n - 1 and temp_c >= steps[lut_index + 1]:
        lut_index += 1

    if lut_index > 0 and temp_c <= (steps[lut_index] - params["HYST_C"]):
        lut_index -= 1

    duty_target = duties()[lut_index]


def set_fan_target(new_target: float, now_ms: int) -> None:
    """
    Start a linear ramp towards the given target duty.

    Args:
        new_target: target duty in [0.0, 1.0].
        now_ms: monotonic time in milliseconds.
    """
    global ramp_start_duty, ramp_target, ramp_start_ms, duty_cmd, fan_spin_request_ms
    new_target = clamp(new_target, 0.0, 1.0)
    if new_target != ramp_target:
        ramp_start_duty = duty_cmd
        ramp_target = new_target
        ramp_start_ms = now_ms
    # NEW: Track spin request start for tach checking
    if new_target >= params["FAN_CHECK_MIN_DUTY"]:
        if fan_spin_request_ms == 0:
            fan_spin_request_ms = now_ms
    else:
        fan_spin_request_ms = 0


def service_fan_ramp(now_ms: int) -> None:
    """
    Update PWM duty along a linear ramp according to RAMP_TIME_MS and RAMP_STEP_MS.
    """
    global duty_cmd, last_ramp_ms
    if params["RAMP_TIME_MS"] <= 0:
        duty_cmd = ramp_target
    else:
        if now_ms - last_ramp_ms >= params["RAMP_STEP_MS"]:
            last_ramp_ms = now_ms
            elapsed = now_ms - ramp_start_ms
            if elapsed <= 0:
                duty_cmd = ramp_start_duty
            elif elapsed >= params["RAMP_TIME_MS"]:
                duty_cmd = ramp_target
            else:
                frac = elapsed / float(params["RAMP_TIME_MS"])
                duty_cmd = ramp_start_duty + frac * (ramp_target - ramp_start_duty)
    fan.duty_cycle = 0 if duty_cmd <= 0.0 else int(duty_cmd * 65535)


# -------------------------------
# NVM persistence (CRC-checked, adaptive)
# -------------------------------
_MAGIC = b"LOSP"  # LipOpenSink_Passive
_VER = 1  # settings schema version


def _crc16_ccitt(data, poly=0x1021, init=0xFFFF) -> int:
    """Return CRC-16/CCITT of the given bytes."""
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _nvm_capacity() -> int:
    """Return NVM capacity in bytes (0 if unavailable)."""
    try:
        return len(microcontroller.nvm)
    except Exception:
        return 0


def nvm_save(params_obj, td_lut, at_lut):
    """
    Persist settings into NVM; adapt content to available capacity.

    Tries (in order):
      1) params + TEMP_TO_DUTY + ADC_TO_TEMP_5C
      2) params + TEMP_TO_DUTY
      3) params only

    Returns:
        (ok: bool, info: str) describing what was saved or the reason of failure.
    """
    cap = _nvm_capacity()
    if cap <= 0:
        return False, "NVM_UNAVAILABLE"

    candidates = []
    candidates.append(
        (
            0x03,
            {
                "params": params_obj,
                "TEMP_TO_DUTY": [[float(t), float(d)] for (t, d) in td_lut],
                "ADC_TO_TEMP_5C": [[int(a), float(t)] for (a, t) in at_lut],
            },
        )
    )
    candidates.append(
        (
            0x01,
            {
                "params": params_obj,
                "TEMP_TO_DUTY": [[float(t), float(d)] for (t, d) in td_lut],
            },
        )
    )
    candidates.append((0x00, {"params": params_obj}))

    for flags, obj in candidates:
        payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        header_wo_crc = _MAGIC + bytes([_VER, flags]) + struct.pack(">H", len(payload))
        crc = _crc16_ccitt(payload)
        blob = header_wo_crc + struct.pack(">H", crc) + payload
        if len(blob) <= cap:
            buf = bytearray(cap)  # clear remainder
            buf[: len(blob)] = blob
            microcontroller.nvm[:] = buf
            if flags == 0x03:
                return True, "NVM_SAVED_ALL"
            if flags == 0x01:
                return True, "NVM_SAVED_PARAMS_TD"
            return True, "NVM_SAVED_PARAMS"
    return False, "NVM_TOO_SMALL"


def nvm_load():
    """
    Load settings from NVM if valid.

    Returns:
        dict with keys among {"params","TEMP_TO_DUTY","ADC_TO_TEMP_5C"} or None if missing/invalid.
    """
    cap = _nvm_capacity()
    if cap <= 8:
        return None
    mem = bytes(microcontroller.nvm[:])
    if mem[:4] != _MAGIC:
        return None
    ver = mem[4]
    flags = mem[5]
    ln = struct.unpack_from(">H", mem, 6)[0]
    if 8 + ln > len(mem):
        return None
    crc_stored = struct.unpack_from(">H", mem, 8)[0]
    payload = mem[10 : 10 + ln]
    if _crc16_ccitt(payload) != crc_stored:
        return None
    try:
        data = json.loads(payload.decode("utf-8"))
    except Exception:
        return None
    if not isinstance(data, dict):
        return None
    return data


def load_settings_from_any() -> bool:
    """
    Load settings from NVM first; fallback to SETTINGS_PATH if present.

    Returns:
        True on success; False if nothing could be loaded (defaults remain).
    """
    data = nvm_load()
    if data is None:
        try:
            with open(SETTINGS_PATH, "r") as f:
                data = json.load(f)
        except Exception:
            return False
    try:
        if "params" in data:
            for k, v in data["params"].items():
                if k in params:
                    params[k] = v
        if "TEMP_TO_DUTY" in data:
            TEMP_TO_DUTY[:] = [(float(t), float(d)) for (t, d) in data["TEMP_TO_DUTY"]]
            TEMP_TO_DUTY.sort(key=lambda x: x[0])
        if "ADC_TO_TEMP_5C" in data:
            ADC_TO_TEMP_5C[:] = [(int(a), float(t)) for (a, t) in data["ADC_TO_TEMP_5C"]]
            ADC_TO_TEMP_5C.sort(key=lambda x: -x[0])
        return True
    except Exception:
        return False


def save_settings_any():
    """
    Persist settings to NVM only (robust while USB mass-storage is mounted).

    Returns:
        (ok: bool, info: str)
    """
    return nvm_save(params, TEMP_TO_DUTY, ADC_TO_TEMP_5C)


def load_defaults_ram():
    """Restore default parameters and boot LUTs."""
    params.update(
        {
            "HYST_C": 5.0,
            "LOAD_TRIP_C": 85.0,
            "MIN_FAN_DUTY": 0.20,
            "FAST_DT_MS": 5,
            "SLOW_DT_MS": 250,
            "FC_TEMP_HZ": 0.5,
            "FC_VIN_HZ": 0.5,
            "RAMP_TIME_MS": 1000,
            "RAMP_STEP_MS": 50,
            "TELEM_RATE_MS": 0,  # OFF by default
            "TELEM_FORMAT": "HUMAN",
            "TEMP_CAL_A": 1.0,
            "TEMP_CAL_B": 0.0,
            "VIN_CAL_A": (10.0 / 100.9),
            "VIN_CAL_B": 0.0,
            "LOAD_TRIP_V": 8.5,   # Default voltage cutoff
            "HYST_V": 0.5,        # Default voltage hysteresis
            "BYPASS_VIN_CUTOFF": 1.0,  # Default: bypass VIN cutoff enabled
            # NEW advanced (not exposed in GUI by default)
            "FAN_CHECK_MIN_DUTY": 0.30,
            "FAN_SPINUP_MS":     1500,
            "FAN_TACH_TIMEOUT_MS": 600,
            "LED_BLINK_MS":       300,
        }
    )
    TEMP_TO_DUTY[:] = [(float(t), float(d)) for (t, d) in TEMP_TO_DUTY_BOOT]
    TEMP_TO_DUTY.sort(key=lambda x: x[0])
    ADC_TO_TEMP_5C[:] = [(int(a), float(t)) for (a, t) in ADC_TO_TEMP_BOOT]
    ADC_TO_TEMP_5C.sort(key=lambda x: -x[0])


# -------------------------------
# Backend glue for CommandServer
# -------------------------------
class Backend:
    """Adapter exposing firmware state/methods to the CommandServer."""

    def __init__(self, io: DualCDC) -> None:
        self.io = io

    # --- Introspection ---
    def version(self) -> str:
        """Return firmware version string for VER?."""
        return FW_VERSION

    def status(self) -> dict:
        """Return connection status for STATUS?."""
        return self.io.endpoints_connected()

    # --- Parameters ---
    def get_params(self):
        """Return parameters as KEY=VALUE lines for GET PARAMS."""
        lines = []
        keys = [
            "MIN_FAN_DUTY",
            "HYST_C",
            "LOAD_TRIP_C",
            # Also include voltage cutoff parameters so GUI stays in sync
            "LOAD_TRIP_V",
            "HYST_V",
            "BYPASS_VIN_CUTOFF",
            "FAST_DT_MS",
            "SLOW_DT_MS",
            "FC_TEMP_HZ",
            "FC_VIN_HZ",
            "RAMP_TIME_MS",
            "RAMP_STEP_MS",
            "TELEM_RATE_MS",
            "TELEM_FORMAT",
            "TEMP_CAL_A",
            "TEMP_CAL_B",
            "VIN_CAL_A",
            "VIN_CAL_B",
            # NEW advanced (not exposed in GUI by default)
            "FAN_CHECK_MIN_DUTY",
            "FAN_SPINUP_MS",
            "FAN_TACH_TIMEOUT_MS",
            "LED_BLINK_MS",
        ]
        for k in keys:
            lines.append("{}={}".format(k, params[k]))
        return lines

    def set_param(self, key: str, value: str) -> None:
        """Set a parameter from text (type inferred by key)."""
        if key not in params:
            raise KeyError("UNKNOWN_KEY")
        if key in ("FAST_DT_MS", "SLOW_DT_MS", "RAMP_TIME_MS", "RAMP_STEP_MS", "TELEM_RATE_MS",
                   "FAN_SPINUP_MS", "FAN_TACH_TIMEOUT_MS", "LED_BLINK_MS"):
            params[key] = int(float(value))
        elif key == "TELEM_FORMAT":
            v = value.strip().upper()
            if v not in ("CSV", "HUMAN"):
                raise ValueError("BAD_FORMAT")
            params[key] = v
        else:
            params[key] = float(value)

    # --- LUTs ---
    def get_lut_temp_to_duty(self):
        """Return a copy of TEMP_TO_DUTY for GET LUT TEMP_TO_DUTY."""
        return list(TEMP_TO_DUTY)

    def set_lut_temp_to_duty(self, rows):
        """Install a new TEMP_TO_DUTY LUT from rows[(temp,duty), ...]."""
        rows = [(float(t), float(d)) for (t, d) in rows]
        rows.sort(key=lambda x: x[0])
        if not rows:
            raise ValueError("EMPTY")
        TEMP_TO_DUTY[:] = rows

    def get_lut_adc_to_temp(self):
        """Return a copy of ADC_TO_TEMP_5C for GET LUT ADC_TO_TEMP_5C."""
        return list(ADC_TO_TEMP_5C)

    def set_lut_adc_to_temp(self, rows):
        """Install a new ADC_TO_TEMP_5C LUT from rows[(adc,temp), ...]."""
        rows = [(int(a), float(t)) for (a, t) in rows]
        rows.sort(key=lambda x: -x[0])
        if not rows:
            raise ValueError("EMPTY")
        ADC_TO_TEMP_5C[:] = rows

    # --- Fan / Telemetry / Persistence ---
    def fan_auto(self) -> None:
        """Return fan control to AUTO policy mode."""
        global fan_manual
        fan_manual = False

    def fan_duty(self, d: float) -> None:
        """Set manual fan target duty (ramps)."""
        global fan_manual, ramp_target, ramp_start_duty, ramp_start_ms, duty_cmd, fan_spin_request_ms
        fan_manual = True
        now = now_ms()
        d = clamp(d, 0.0, 1.0)
        ramp_start_duty = duty_cmd
        ramp_target = d
        ramp_start_ms = now
        # NEW: start spin request if above threshold
        fan_spin_request_ms = now if d >= params["FAN_CHECK_MIN_DUTY"] else 0

    def telem_rate(self, ms: int) -> None:
        """Set telemetry period in ms (0 = off)."""
        params["TELEM_RATE_MS"] = max(0, int(ms))

    def telem_format(self, fmt: str) -> None:
        """Set telemetry format ('CSV' or 'HUMAN')."""
        fmt = fmt.strip().upper()
        if fmt not in ("CSV", "HUMAN"):
            raise ValueError("BAD_FORMAT")
        params["TELEM_FORMAT"] = fmt

    def save_settings(self):
        """Persist settings to NVM."""
        return save_settings_any()

    def load_defaults(self) -> None:
        """Restore defaults in RAM."""
        load_defaults_ram()

    def reboot(self) -> None:
        """Soft reset the microcontroller."""
        import time as _t
        import microcontroller as _m
        _t.sleep(0.1)
        _m.reset()


# -------------------------------
# Boot: load settings (NVM first)
# -------------------------------
load_settings_from_any()

# -------------------------------
# Timers & helpers
# -------------------------------
def now_ms() -> int:
    """Return monotonic time in milliseconds."""
    return time.monotonic_ns() // 1_000_000


t_fast = now_ms()
last_telem_ms = t_fast
last_ramp_ms = t_fast
ramp_start_ms = t_fast
last_temp_c = 25.0
# NEW: initialize tach/blink timers
last_tach_edge_ms = t_fast
last_led_blink_ms = t_fast
last_tach_val = fan_tach.value

# -------------------------------
# Startup LED version blink
# -------------------------------
def _blink_n(n: int, on_ms: int = 150, off_ms: int = 150) -> None:
    try:
        n = int(n)
    except Exception:
        n = 0
    if n <= 0:
        return
    for i in range(n):
        led.value = True
        time.sleep(on_ms / 1000.0)
        led.value = False
        if i != n - 1:
            time.sleep(off_ms / 1000.0)


def _parse_major_minor(vs: str) -> tuple:
    major = 0
    minor = 0
    try:
        tokens = vs.replace("-", ".").split(".")
        for tok in tokens:
            try:
                val = int(tok)
            except Exception:
                continue
            if major == 0:
                major = val
            elif minor == 0:
                minor = val
                break
    except Exception:
        pass
    return major, minor


def blink_version() -> None:
    maj, minr = _parse_major_minor(FW_VERSION)
    _blink_n(maj)
    time.sleep(1.0)
    _blink_n(minr)
    time.sleep(0.2)
    led.value = False

# Perform version blink at startup (non-blocking thereafter)
blink_version()

# -------------------------------
# UART server
# -------------------------------
io = DualCDC()
server = CommandServer(io, Backend(io))
io.write_line("READY LipOpenSink_Passive (FW {})".format(FW_VERSION))

# -------------------------------
# Main loop
# -------------------------------
while True:
    # 1) Commands
    server.poll()

    # 2) Fast loop: acquisition + filtering + control + ramp
    t = now_ms()
    if t - t_fast >= params["FAST_DT_MS"]:
        now_ns = time.monotonic_ns()
        dt_s = (now_ns - last_fast_ns) / 1_000_000_000.0
        last_fast_ns = now_ns
        t_fast = t

        # ADC raw
        raw_vin = vinADC.value
        raw_tmp = tempADC.value

        # IIR on counts
        raw_vin_f = iir_update(raw_vin_f, raw_vin, params["FC_VIN_HZ"], dt_s)
        raw_temp_f = iir_update(raw_temp_f, raw_tmp, params["FC_TEMP_HZ"], dt_s)

        # Convert to calibrated volts
        vin_v = params["VIN_CAL_A"] * ((raw_vin_f * VREF) / ADC_MAX) + params["VIN_CAL_B"]
        tmp_v = (raw_temp_f * VREF) / ADC_MAX

        # Back to counts for LUT
        tmp_counts = int(clamp((tmp_v / VREF) * ADC_MAX, 0, ADC_MAX))

        # Temperature in °C
        last_temp_c = temp_from_adc(tmp_counts)
        # Broken sensor safety: NTC open (high-side) reads very low (≈ -40°C)
        temp_fault_low = (last_temp_c < BROKEN_TEMP_CUTOFF_C)

        # Policy
        if not fan_manual:
            update_fan_policy(last_temp_c)
            set_fan_target(duty_target, t)

        # Ramp to target
        service_fan_ramp(t)

        # NEW: Fan tach monitoring & fault detection
        cur_tach = fan_tach.value
        if cur_tach != last_tach_val:
            last_tach_val = cur_tach
            last_tach_edge_ms = t

        should_check_fan = (ramp_target >= params["FAN_CHECK_MIN_DUTY"]) or (duty_cmd >= params["FAN_CHECK_MIN_DUTY"]) or fan_manual
        if not should_check_fan:
            fan_fault = False
            fan_spin_request_ms = 0
        else:
            if fan_spin_request_ms == 0:
                fan_spin_request_ms = t
            if (t - fan_spin_request_ms) >= params["FAN_SPINUP_MS"]:
                fan_fault = (t - last_tach_edge_ms) > params["FAN_TACH_TIMEOUT_MS"]
            else:
                fan_fault = False

        # Updated load policy: independent temp and VIN cutoffs with bypass support.
        bypass_vin = params.get("BYPASS_VIN_CUTOFF", 1.0) >= 0.5
        if base_load_enabled:
            # Trip if temperature exceeds threshold OR (VIN below threshold AND not bypassed)
            if (last_temp_c >= params["LOAD_TRIP_C"]) or ((not bypass_vin) and (vin_v < (params["LOAD_TRIP_V"] - params["HYST_V"]))):
                base_load_enabled = False
        else:
            # Re-enable if temp falls sufficiently OR (VIN recovers above threshold AND not bypassed)
            if (last_temp_c <= (params["LOAD_TRIP_C"] - params["HYST_C"])) or ((not bypass_vin) and (vin_v >= params["LOAD_TRIP_V"])):
                base_load_enabled = True

        # NEW: Apply interlocks: no fan OR broken sensor → no load
        load_enabled = base_load_enabled and (not fan_fault) and (not temp_fault_low)
        load_en.value = load_enabled

        # NEW: LED blinking on fan fault, else mirror LOAD_EN
        if fan_fault and should_check_fan:
            if (t - last_led_blink_ms) >= params["LED_BLINK_MS"]:
                last_led_blink_ms = t
                led_blink_state = not led_blink_state
            led.value = led_blink_state
        else:
            led.value = load_enabled

    # 3) Telemetry (OFF by default)
    if params["TELEM_RATE_MS"] > 0 and (t - last_telem_ms >= params["TELEM_RATE_MS"]):
        last_telem_ms = t
        pg_status = "OK" if psu_power_good.value else "FAIL"
        mode = "MAN" if fan_manual else "AUTO"
        if params["TELEM_FORMAT"].upper() == "CSV":
            io.write_line(
                "{:.3f},{:.3f},{:.1f},{},{:.0f},{},{},{}".format(
                    params["VIN_CAL_A"] * ((raw_vin_f * VREF) / ADC_MAX) + params["VIN_CAL_B"],
                    params["TEMP_CAL_A"] * ((raw_temp_f * VREF) / ADC_MAX) + params["TEMP_CAL_B"],
                    last_temp_c,
                    lut_index,
                    duty_cmd * 100.0,
                    "ON" if load_enabled else "OFF",
                    pg_status,
                    mode,
                )
            )
        else:
            io.write_line(
                "VIN={:.3f}V TEMPADC={:.3f}V T={:.1f}C FanIdx={} Duty={:.0f}% LOAD={} PG={} MODE={}".format(
                    params["VIN_CAL_A"] * ((raw_vin_f * VREF) / ADC_MAX) + params["VIN_CAL_B"],
                    params["TEMP_CAL_A"] * ((raw_temp_f * VREF) / ADC_MAX) + params["TEMP_CAL_B"],
                    last_temp_c,
                    lut_index,
                    duty_cmd * 100.0,
                    "ON" if load_enabled else "OFF",
                    pg_status,
                    mode,
                )
            )

    time.sleep(0.001)
