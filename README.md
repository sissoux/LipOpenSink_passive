# LipOpenSink\_Passive

Smart passive load add-on for the ToolkitRC M8D charger. Bridges Channel 1 → Channel 2 into an external 3.3 Ω / 300 W resistor for higher discharge power. An onboard RP2040 supervises temperature, drives a 12 V fan (from the M8D’s USB-C PD), and cuts the load on error/thresholds. A simple USB configuration tool lets you tune behavior and monitor telemetry.

> Safety note: Ensure adequate cooling. Firmware enforces "no fan → no load": when the fan tach (GP12) shows no pulses while the fan should be running, the load is forced OFF and the LED blinks. Temperature and VIN thresholds also protect the load.

---

## Features

- Temperature monitoring with configurable filtering and calibration
- Fan control via LUT (temperature → PWM duty) with hysteresis and smooth ramping
- Load cutoff on over-temperature, with LED indicator
- Optional VIN-based load cutoff with hysteresis (configurable)
- Fan presence check via tach on GP12; interlock enforces "no fan → no load" with blinking LED
- Settings persistence in RP2040 NVM (CRC-checked, adaptive to capacity)
- USB configuration tool (CDC/serial) to edit parameters and view telemetry

---

## Repository structure

```text
.
├── firmware/               # RP2040 (CircuitPython)
│   ├── boot.py             # Optional: expose dual CDC ports early
│   ├── code.py             # Firmware entry point
│   ├── LUTs.py             # Lookup tables (ADC→Temp, Temp→Duty)
│   └── uart_cmd.py         # Dual-CDC + ASCII command server
│
├── config_tool/            # PC configuration utility (Python 3)
│   ├── config_gui.py       # Tkinter GUI: params, telemetry, calibration
│   └── requirements.txt    # Dependencies for the GUI
│
└── README.md
```

Note: Hardware CAD and 3D files are not included in this repository.

---

## Quick start

### Hardware

1. Assemble the module with the 3.3 Ω / 300 W resistor, fan, and RP2040 board.
2. Power the fan from the M8D USB-C (12 V PD) as designed in your hardware.
3. Wire the module in the M8D’s discharge bridge path (CH1 → resistor → CH2).

### Firmware (RP2040)

1. Install CircuitPython for your RP2040 board.
2. Copy `firmware/code.py` and `firmware/LUTs.py` to the CIRCUITPY drive.
	- Optional: also copy `firmware/boot.py` to expose both CDC endpoints on boot.
3. Power-cycle the board; defaults will run immediately (telemetry is OFF by default).
4. Optionally adjust settings with the configuration tool and save to NVM.

### Configuration tool (PC)

Install dependencies (Windows PowerShell example):

```
pip install -r config_tool/requirements.txt
```

Launch the GUI:

```
python config_tool/config_gui.py
```

In the GUI:

- Select the serial port (CDC/COM) and Auto-Connect or Connect.
- Use Refresh to load current parameters; adjust values and Apply; Save to flash when satisfied.
- Live telemetry (CSV mode) is shown in the panel; you can set manual fan duty for testing.

Currently, the GUI focuses on parameters, telemetry, and basic calibration. LUT editing (upload/download) is supported by the device’s ASCII protocol but not yet exposed in the GUI.

---

## What you can configure

- Thermal & safety: cutoff temperature, hysteresis, ADC filtering
- Fan behavior: minimum duty, ramp time/step, temperature → duty LUT (device-side)
- Calibration: VIN and temperature channels (a·x + b) — temperature offset/slope act on °C
- Telemetry: rate (ms) and format (CSV or human-readable)
- VIN-based load cutoff: `LOAD_TRIP_V` and `HYST_V`

---

## Updating LUTs (advanced)

LUTs can be retrieved and updated via the serial ASCII protocol implemented in `uart_cmd.py`:

- GET/SET LUT TEMP_TO_DUTY
- GET/SET LUT ADC_TO_TEMP_5C

Use a serial terminal to issue HELP for the exact command shapes (BEGIN/END block mode with indexed rows). GUI support for LUT editing is planned but not yet available.

---

## Typical workflow

1. Power the module from the M8D (USB-C 12 V) and flash the firmware.
2. Verify the fan ramps according to temperature requirements.
3. Connect the configuration tool; adjust calibration, thresholds, and telemetry rate.
4. Save to flash, run a controlled discharge, and iterate based on telemetry.

---

## Licenses

- Firmware & Configuration Tool: MIT
- Hardware and 3D files: not included in this repository

---

## Notes

- Firmware v1.3.0 monitors a Power-Good input but does not gate the load from it; load enable is controlled by temperature and VIN thresholds.
- Set the ToolkitRC M8D to discharge/bridge as per its documentation.
- Validate thermal performance for your specific mechanical design and environment before full-power operation.
