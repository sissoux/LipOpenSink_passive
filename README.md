# LipOpenSink\_Passive

**Smart passive load add-on for the ToolkitRC M8D charger.**
Bridges Channel 1 → Channel 2 into an external **3.3 Ω / 300 W** resistor for higher discharge power.
An onboard **RP2040** supervises temperature, drives a **12 V fan** (from the M8D’s USB-C PD), and **cuts the load** on over-temperature. A simple **USB configuration tool** lets you tune behavior and update lookup tables (LUTs). The whole assembly fits a **3D-printed housing**.

> **Safety note:** The load must **not** be used without the fan. The circuit only closes when the 12 V fan supply is present via USB-C PD.

---

## Features

* Temperature monitoring with configurable filtering and calibration
* Fan control via LUT (temperature → PWM duty) with hysteresis and smooth ramping
* Over-temperature cutoff for the load, with LED indicator bonded to `LOAD_EN`
* USB configuration tool (UART/CDC) to adjust parameters and update LUTs
* 3D-printable enclosure optimized for airflow and mounting

---

## Repository Structure

```markdown
.
├── firmware/               # RP2040 (CircuitPython)
│   ├── main.py             # Firmware entry point
│   └── LUTs.py             # Lookup tables (ADC→Temp, Temp→Duty)
│
├── config_tool/            # PC configuration utility (Python 3)
│   ├── config_gui.py       # Tkinter-based GUI
│   └── requirements.txt    # pyserial, pandas (tkinter usually preinstalled)
│
├── hardware/               # Hardware design files
│   ├── schematic.pdf
│   ├── pcb/                # CAD files (KiCad/Altium/etc.)
│   └── BOM.csv
│
├── 3d/                     # Mechanical design
│   ├── housing.3mf         # Print-ready enclosure
│   └── preview.png         # Render / photo
│
└── README.md
```

---

## Quick Start

### Hardware & Assembly

1. Print the enclosure in `3d/housing.3mf`.
2. Mount the **3.3 Ω / 300 W** resistor, the fan, and the RP2040 control board.
3. Connect the module to the **M8D USB-C** port (for **12 V** fan power via PD).
4. Wire the module in the M8D’s **discharge bridge** path (CH1 → external resistor → CH2).

### Firmware (RP2040)

1. Install **CircuitPython** for your RP2040 board.
2. Copy `firmware/main.py` and `firmware/LUTs.py` to the **CIRCUITPY** drive.
3. Power-cycle the board; defaults will run immediately.
4. (Optional) Adjust settings with the configuration tool and **save to flash**.

### Configuration Tool (PC)

Install dependencies:

```bash
pip install -r config_tool/requirements.txt
```

Launch the GUI:

```bash
python config_tool/config_gui.py
```

In the GUI:

* Select the **serial port** (CDC/COM) and **Connect**.
* **Refresh** to load current parameters and LUTs.
* Edit values, then **Apply** and **Save to flash** if desired.

---

## What You Can Configure

* **Thermal & Safety**: cutoff temperature, hysteresis, ADC filtering
* **Fan Behavior**: minimum duty, ramp time/step, temperature → duty LUT
* **Calibration**: Vin and Temp channels (`a·x + b` form)
* **Telemetry**: rate (ms) and format (CSV or human-readable)
* **Lookup Tables**: ADC → Temperature, Temperature → Duty; import/export CSV

---

## Typical Workflow

1. Assemble and power the module from the M8D (USB-C 12 V).
2. Flash the firmware and verify the fan spins when temperature requires it.
3. Connect the **config\_tool** and adjust calibration, LUTs, and thresholds.
4. **Save to flash**, test with a controlled discharge, and iterate.
5. Log telemetry to CSV if you want to validate thermal behavior.

---

## Licenses

* **Hardware:** CERN-OHL-P
* **Firmware & Configuration Tool:** MIT
* **3D Models:** CC-BY-SA 4.0

---

## Gallery

*Add photos/renders of the assembled ****LipOpenSink\_Passive**** here.*

---

## Notes

* The external load path is **interlocked** with fan power: no fan → no load.
* The **ToolkitRC M8D** must be set to discharge/bridge as per its documentation.
* Validate thermal performance with your specific print, resistor tolerance, and ambient airflow before full-power use.
