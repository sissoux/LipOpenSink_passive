# Licensed under CERN-OHL-S-2.0
# © 2025 ADSTech [e-ddy]
# https://ohwr.org/licences/

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {"time_s", "temp_c"}
    if not required.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {sorted(required)}")
    df = df.copy()
    df["time_s"] = pd.to_numeric(df["time_s"], errors="coerce")
    df["temp_c"] = pd.to_numeric(df["temp_c"], errors="coerce")
    if "fan_duty_pct" in df.columns:
        df["fan_duty_pct"] = pd.to_numeric(df["fan_duty_pct"], errors="coerce")
    df = df.dropna(subset=["time_s", "temp_c"]).reset_index(drop=True)
    return df


def simple_plot(csv_path: str, save_png: bool = True):
    df = load_csv(csv_path)
    max_temp = float(df["temp_c"].max())

    fig, ax = plt.subplots(figsize=(10, 5))
    line_temp, = ax.plot(df["time_s"], df["temp_c"], color="tab:red", label="Temperature (°C)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temp (°C)")
    ax.set_title(f"{os.path.basename(csv_path)} — Max T = {max_temp:.1f}°C")
    ax.grid(True, alpha=0.2)

    lines = [line_temp]
    labels = ["Temperature (°C)"]

    # Optional secondary axis for duty
    if "fan_duty_pct" in df.columns:
        ax2 = ax.twinx()
        line_duty, = ax2.plot(
            df["time_s"],
            df["fan_duty_pct"],
            color="tab:blue",
            linestyle=":",
            alpha=0.9,
            label="Fan duty (%)",
        )
        ax2.set_ylabel("Fan duty (%)", color="tab:blue")
        ax2.tick_params(axis='y', labelcolor='tab:blue')
        ax2.set_ylim(0, 100)
        lines.append(line_duty)
        labels.append("Fan duty (%)")

    ax.legend(lines, labels, loc="best")
    fig.tight_layout()

    if save_png:
        out_png = os.path.splitext(csv_path)[0] + "_plot.png"
        fig.savefig(out_png, dpi=150)
        print(f"Saved plot → {out_png}")

    print(f"Max temperature: {max_temp:.2f} °C")
    return {"max_temp_c": max_temp}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot temperature vs time and print max temperature.")
    parser.add_argument("csv", nargs="?", default="Double Cycle test 1.csv", help="Path to CSV file")
    parser.add_argument("--no-save", action="store_true", help="Don't save PNG next to CSV")
    args = parser.parse_args()

    simple_plot(args.csv, save_png=(not args.no_save))
