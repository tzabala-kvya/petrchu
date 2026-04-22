#!/usr/bin/env python3
"""
teststand_analysis.py
=====================
Post-test analysis toolkit for Test Stand CSV logs.

Load one or more telemetry CSVs, compute performance metrics,
generate publication-ready plots, and export a summary report.

Usage:
    python teststand_analysis.py logs/telem_20260420_143200_a1b2c3d4.csv
    python teststand_analysis.py logs/telem_*.csv --compare

Or import and use interactively:
    from teststand_analysis import TestRun, compare_runs
    run = TestRun("logs/telem_20260420_143200_a1b2c3d4.csv")
    run.summary()
    run.plot_discharge_profile()
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy import signal as sig

# =========================================================================
# CONSTANTS
# =========================================================================

SAMPLE_RATE_HZ = 10   # 10 Hz telemetry

DISCHARGE_STATES = {"DISCHARGE_AIR", "DISCHARGE_WATER", "DISCHARGE_BOTH"}
CHARGE_STATES    = {"CHARGE"}

# State code mapping (matches firmware)
STATE_MAP = {
    0: "BOOT_SELFTEST", 1: "CAL_PROMPT", 2: "CAL_ZEROING", 3: "CAL_REPLACE",
    4: "OFF", 5: "ARM", 6: "CHARGE", 7: "READY",
    8: "DISCHARGE_AIR", 9: "DISCHARGE_WATER", 10: "DISCHARGE_BOTH",
    11: "MANUAL_DIAG", 12: "SHUTDOWN", 13: "FAULT", 14: "ESTOP",
}


# =========================================================================
# TEST RUN CLASS
# =========================================================================

class TestRun:
    """Represents one logged test session."""

    def __init__(self, csv_path: str, events_path: str = None):
        self.path = csv_path
        self.name = os.path.basename(csv_path).replace(".csv", "")

        self.df = pd.read_csv(csv_path)
        self._prepare()

        self.events = None
        if events_path and os.path.exists(events_path):
            self.events = pd.read_csv(events_path)

    def _prepare(self):
        """Add convenient columns and clean up types."""
        df = self.df

        # Relative time in seconds from first sample
        if "timestamp_ms" in df.columns:
            df["t_s"] = (df["timestamp_ms"] - df["timestamp_ms"].iloc[0]) / 1000.0
        else:
            df["t_s"] = np.arange(len(df)) / SAMPLE_RATE_HZ

        # Ensure numeric
        for col in ["p_line_psi", "rpm_air", "rpm_water", "rpm_alternator",
                     "v_alt_V", "i_alt_A", "power_alt_W", "power_supply_W",
                     "power_compressor_W", "power_pump_W", "dp_dt_psi_per_s",
                     "energy_alt_J", "energy_supply_J",
                     "i_12v_A", "i_5v_A", "v_bus_V",
                     "i_compressor_A", "i_pump_A",
                     "water_level_cm", "temp_water_C"]:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors="coerce").fillna(0)

        # State name from code if not present
        if "state_name" not in df.columns and "state" in df.columns:
            df["state_name"] = df["state"].map(STATE_MAP).fillna("UNKNOWN")

    # -----------------------------------------------------------------
    # Segment extraction
    # -----------------------------------------------------------------

    def get_segments(self, state_names: set) -> list[pd.DataFrame]:
        """Extract contiguous segments where state_name is in the given set."""
        df = self.df
        mask = df["state_name"].isin(state_names)
        segments = []
        in_seg = False
        start = 0
        for i, val in enumerate(mask):
            if val and not in_seg:
                start = i
                in_seg = True
            elif not val and in_seg:
                segments.append(df.iloc[start:i].copy())
                in_seg = False
        if in_seg:
            segments.append(df.iloc[start:].copy())
        return segments

    def get_discharges(self) -> list[pd.DataFrame]:
        return self.get_segments(DISCHARGE_STATES)

    def get_charges(self) -> list[pd.DataFrame]:
        return self.get_segments(CHARGE_STATES)

    # -----------------------------------------------------------------
    # Scalar metrics
    # -----------------------------------------------------------------

    def summary(self) -> dict:
        """Compute key scalar metrics for the entire run."""
        df = self.df
        discharges = self.get_discharges()
        charges    = self.get_charges()

        s = {
            "file":              self.name,
            "duration_s":        df["t_s"].iloc[-1] if len(df) > 0 else 0,
            "total_samples":     len(df),
            "num_discharges":    len(discharges),
            "num_charges":       len(charges),
            "peak_pressure_psi": df["p_line_psi"].max(),
            "peak_rpm_air":      df["rpm_air"].max(),
            "peak_rpm_water":    df["rpm_water"].max(),
            "peak_rpm_alt":      df["rpm_alternator"].max(),
            "peak_power_alt_W":  df["power_alt_W"].max(),
            "total_energy_alt_J": df["energy_alt_J"].iloc[-1] if len(df) > 0 else 0,
            "total_energy_supply_J": df["energy_supply_J"].iloc[-1] if len(df) > 0 else 0,
            "v_bus_min":         df["v_bus_V"].min(),
            "v_bus_max":         df["v_bus_V"].max(),
            "fault_count":       (df["fault_list"] != "NONE").sum() if "fault_list" in df.columns else 0,
        }

        # Per-discharge metrics
        for i, seg in enumerate(discharges):
            dt = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
            s[f"discharge_{i}_duration_s"]     = round(dt, 2)
            s[f"discharge_{i}_peak_rpm_alt"]   = seg["rpm_alternator"].max()
            s[f"discharge_{i}_peak_power_W"]   = seg["power_alt_W"].max()
            s[f"discharge_{i}_energy_alt_J"]   = round(
                np.trapezoid(seg["power_alt_W"], seg["t_s"]), 2)
            s[f"discharge_{i}_p_start_psi"]    = seg["p_line_psi"].iloc[0]
            s[f"discharge_{i}_p_end_psi"]      = seg["p_line_psi"].iloc[-1]
            s[f"discharge_{i}_driving_source"]  = seg["driving_source"].mode().iloc[0] \
                if "driving_source" in seg.columns and len(seg) > 0 else "?"

        for i, seg in enumerate(charges):
            dt = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
            s[f"charge_{i}_duration_s"]    = round(dt, 2)
            s[f"charge_{i}_p_start_psi"]   = seg["p_line_psi"].iloc[0]
            s[f"charge_{i}_p_end_psi"]     = seg["p_line_psi"].iloc[-1]
            rate = (seg["p_line_psi"].iloc[-1] - seg["p_line_psi"].iloc[0]) / dt if dt > 0 else 0
            s[f"charge_{i}_rate_psi_per_s"] = round(rate, 2)

        return s

    def print_summary(self):
        s = self.summary()
        print(f"\n{'='*60}")
        print(f"  TEST RUN SUMMARY: {s['file']}")
        print(f"{'='*60}")
        print(f"  Duration:         {s['duration_s']:.1f} s ({s['total_samples']} samples)")
        print(f"  Charges:          {s['num_charges']}")
        print(f"  Discharges:       {s['num_discharges']}")
        print(f"  Peak pressure:    {s['peak_pressure_psi']:.1f} psi")
        print(f"  Peak RPM (alt):   {s['peak_rpm_alt']:.0f}")
        print(f"  Peak power (alt): {s['peak_power_alt_W']:.1f} W")
        print(f"  Total energy gen: {s['total_energy_alt_J']:.1f} J")
        print(f"  V_bus range:      {s['v_bus_min']:.2f} – {s['v_bus_max']:.2f} V")
        print(f"  Fault samples:    {s['fault_count']}")
        print()

    # -----------------------------------------------------------------
    # Step-response metrics from a discharge segment
    # -----------------------------------------------------------------

    def step_response_metrics(self, seg: pd.DataFrame,
                              col: str = "rpm_alternator") -> dict:
        """
        Compute step-response metrics for a signal during a discharge.
        Returns: rise_time, peak_time, settling_time, percent_overshoot,
                 steady_state_value, time_constant_tau.
        """
        y = seg[col].values
        t = seg["t_s"].values - seg["t_s"].values[0]

        if len(y) < 10 or y.max() < 1.0:
            return {}

        y_ss = np.mean(y[-max(10, len(y)//5):])   # last 20% as steady state
        y_peak = y.max()
        i_peak = np.argmax(y)

        # 10% and 90% thresholds
        y_10 = 0.10 * y_ss
        y_90 = 0.90 * y_ss

        i_10 = np.argmax(y >= y_10) if np.any(y >= y_10) else 0
        i_90 = np.argmax(y >= y_90) if np.any(y >= y_90) else len(y)-1

        rise_time = t[i_90] - t[i_10]
        peak_time = t[i_peak]

        # Settling time: last time signal exits ±5% band around steady state
        band = 0.05 * y_ss
        outside = np.where(np.abs(y - y_ss) > band)[0]
        settling_time = t[outside[-1]] - t[0] if len(outside) > 0 else 0.0

        pct_overshoot = ((y_peak - y_ss) / y_ss * 100.0) if y_ss > 0 else 0.0

        # Time constant estimate (63.2% of steady state)
        y_63 = 0.632 * y_ss
        i_63 = np.argmax(y >= y_63) if np.any(y >= y_63) else 0
        tau = t[i_63] - t[0]

        return {
            "rise_time_s":        round(rise_time, 3),
            "peak_time_s":        round(peak_time, 3),
            "settling_time_s":    round(settling_time, 3),
            "percent_overshoot":  round(pct_overshoot, 1),
            "steady_state_value": round(y_ss, 1),
            "time_constant_tau":  round(tau, 3),
        }

    # -----------------------------------------------------------------
    # Leak-down test (during READY state)
    # -----------------------------------------------------------------

    def leak_down_rate(self) -> float | None:
        """
        Compute pressure decay rate (psi/min) during READY hold.
        Returns None if no READY segment is long enough.
        """
        segs = self.get_segments({"READY"})
        if not segs:
            return None
        # Use longest READY segment
        seg = max(segs, key=len)
        if len(seg) < 30:  # need at least 3 seconds
            return None
        dt_s = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
        dp   = seg["p_line_psi"].iloc[-1] - seg["p_line_psi"].iloc[0]
        return round((dp / dt_s) * 60.0, 3)  # psi/min

    # -----------------------------------------------------------------
    # FFT of a signal during steady-state
    # -----------------------------------------------------------------

    def compute_fft(self, seg: pd.DataFrame, col: str = "rpm_alternator",
                    fs: float = SAMPLE_RATE_HZ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute single-sided FFT amplitude spectrum.
        Returns (freqs_hz, amplitudes).
        """
        y = seg[col].values - np.mean(seg[col].values)  # remove DC
        n = len(y)
        if n < 32:
            return np.array([]), np.array([])

        # Apply Hanning window to reduce spectral leakage
        window = np.hanning(n)
        y_windowed = y * window

        fft_vals = np.fft.rfft(y_windowed)
        freqs    = np.fft.rfftfreq(n, d=1.0/fs)
        amps     = 2.0 / n * np.abs(fft_vals)

        return freqs, amps

    # -----------------------------------------------------------------
    # Valve response time
    # -----------------------------------------------------------------

    def valve_response_times(self) -> list[dict]:
        """
        Detect valve open/close transitions and measure delay to
        pressure/current response.
        """
        df = self.df
        results = []

        for valve_col in ["valve_disch_air_cmd", "valve_disch_water_cmd",
                          "valve_charge_air_cmd", "valve_charge_water_cmd"]:
            if valve_col not in df.columns:
                continue

            # Find rising edges (0 -> 1)
            v = df[valve_col].values.astype(int)
            edges = np.where(np.diff(v) == 1)[0]

            for idx in edges:
                t_cmd = df["t_s"].iloc[idx]
                # Look for pressure response in next 2 seconds
                window = df.iloc[idx:idx+20]  # 2 s window
                if len(window) < 5:
                    continue

                p0 = window["p_line_psi"].iloc[0]
                # Find first sample where pressure deviates > 1 psi
                resp_idx = np.argmax(np.abs(window["p_line_psi"].values - p0) > 1.0)
                if resp_idx > 0:
                    t_resp = window["t_s"].iloc[resp_idx]
                    results.append({
                        "valve":       valve_col,
                        "t_cmd_s":     round(t_cmd, 3),
                        "delay_ms":    round((t_resp - t_cmd) * 1000, 1),
                    })

        return results

    # -----------------------------------------------------------------
    # PLOTTING
    # -----------------------------------------------------------------

    def plot_full_timeline(self, save_path: str = None):
        """Overview plot: pressure, RPM, power, state over entire run."""
        df = self.df
        fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
        fig.suptitle(f"Full Timeline — {self.name}", fontsize=12, fontweight="bold")

        # Pressure
        ax = axes[0]
        ax.plot(df["t_s"], df["p_line_psi"], color="tab:blue", linewidth=0.8)
        ax.set_ylabel("Pressure (psi)")
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="gray", linewidth=0.5)

        # RPM
        ax = axes[1]
        ax.plot(df["t_s"], df["rpm_air"],        label="Air",   color="tab:cyan",   linewidth=0.8)
        ax.plot(df["t_s"], df["rpm_water"],      label="Water", color="tab:blue",   linewidth=0.8)
        ax.plot(df["t_s"], df["rpm_alternator"], label="Alt",   color="tab:purple", linewidth=0.8)
        ax.set_ylabel("RPM")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # Power
        ax = axes[2]
        ax.plot(df["t_s"], df["power_alt_W"],        label="Alt out",    color="tab:green",  linewidth=0.8)
        ax.plot(df["t_s"], df["power_supply_W"],     label="Supply",     color="tab:orange", linewidth=0.8)
        ax.plot(df["t_s"], df["power_compressor_W"], label="Compressor", color="tab:red",    linewidth=0.8)
        ax.plot(df["t_s"], df["power_pump_W"],       label="Pump",       color="tab:cyan",   linewidth=0.8)
        ax.set_ylabel("Power (W)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # State
        ax = axes[3]
        ax.plot(df["t_s"], df["state"], drawstyle="steps-post",
                color="tab:gray", linewidth=1.0)
        ax.set_ylabel("State code")
        ax.set_xlabel("Time (s)")
        ax.set_yticks(range(15))
        ax.set_yticklabels([STATE_MAP.get(i, "?") for i in range(15)],
                           fontsize=6)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"  Saved: {save_path}")
        plt.show()

    def plot_discharge_profile(self, discharge_idx: int = 0,
                               save_path: str = None):
        """Detailed discharge analysis: P, RPM, power, current vs time."""
        discharges = self.get_discharges()
        if discharge_idx >= len(discharges):
            print(f"  No discharge #{discharge_idx} found.")
            return

        seg = discharges[discharge_idx]
        t = seg["t_s"].values - seg["t_s"].values[0]

        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f"Discharge #{discharge_idx} — {self.name}",
                     fontsize=12, fontweight="bold")

        # Pressure
        ax = axes[0]
        ax.plot(t, seg["p_line_psi"], color="tab:blue", linewidth=1)
        ax.set_ylabel("Pressure (psi)")
        ax.grid(True, alpha=0.3)

        # RPM
        ax = axes[1]
        ax.plot(t, seg["rpm_air"],        label="Air",   color="tab:cyan",   linewidth=1)
        ax.plot(t, seg["rpm_water"],      label="Water", color="tab:blue",   linewidth=1)
        ax.plot(t, seg["rpm_alternator"], label="Alt",   color="tab:purple", linewidth=1)
        ax.set_ylabel("RPM")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Power
        ax = axes[2]
        ax.plot(t, seg["power_alt_W"],    label="Alt out", color="tab:green", linewidth=1)
        ax.fill_between(t, 0, seg["power_alt_W"], alpha=0.15, color="tab:green")
        ax.set_ylabel("Power (W)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Current
        ax = axes[3]
        ax.plot(t, seg["i_12v_A"],        label="12V bus",    color="tab:orange", linewidth=1)
        ax.plot(t, seg["i_compressor_A"], label="Compressor", color="tab:red",    linewidth=0.8)
        ax.plot(t, seg["i_pump_A"],       label="Pump",       color="tab:cyan",   linewidth=0.8)
        ax.set_ylabel("Current (A)")
        ax.set_xlabel("Time since discharge start (s)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_pressure_vs_rpm(self, save_path: str = None):
        """Cross-plot: pressure vs alternator RPM during all discharges.
        Reveals the machine's pressure-speed characteristic curve."""
        discharges = self.get_discharges()
        if not discharges:
            print("  No discharge segments found.")
            return

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title(f"Pressure vs RPM — {self.name}", fontweight="bold")

        for i, seg in enumerate(discharges):
            src = seg["driving_source"].mode().iloc[0] if "driving_source" in seg.columns else "?"
            ax.scatter(seg["p_line_psi"], seg["rpm_alternator"],
                       s=4, alpha=0.5, label=f"Discharge {i} ({src})")

        ax.set_xlabel("Line Pressure (psi)")
        ax.set_ylabel("Alternator RPM")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_power_balance(self, save_path: str = None):
        """Stacked area: where power goes during a run."""
        df = self.df
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.set_title(f"Power Balance — {self.name}", fontweight="bold")

        ax.fill_between(df["t_s"], 0, df["power_compressor_W"],
                        alpha=0.4, label="Compressor", color="tab:red")
        ax.fill_between(df["t_s"], df["power_compressor_W"],
                        df["power_compressor_W"] + df["power_pump_W"],
                        alpha=0.4, label="Pump", color="tab:cyan")
        ax.plot(df["t_s"], df["power_alt_W"],
                label="Alt output", color="tab:green", linewidth=1.5)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Power (W)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_fft_rpm(self, discharge_idx: int = 0, save_path: str = None):
        """FFT of alternator RPM during a discharge — reveals vibration."""
        discharges = self.get_discharges()
        if discharge_idx >= len(discharges):
            print("  No discharge segment found.")
            return

        seg = discharges[discharge_idx]
        # Use the last 50% (more likely steady-state)
        half = seg.iloc[len(seg)//2:]
        if len(half) < 32:
            print("  Segment too short for FFT.")
            return

        freqs, amps = self.compute_fft(half, "rpm_alternator")

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.set_title(f"RPM Frequency Spectrum — Discharge #{discharge_idx}",
                     fontweight="bold")
        ax.plot(freqs, amps, color="tab:purple", linewidth=0.8)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Amplitude (RPM)")
        ax.set_xlim(0, SAMPLE_RATE_HZ / 2)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_discharge_overlay(self, save_path: str = None):
        """Overlay all discharges on the same time axis (t=0 at trigger).
        Shows repeatability."""
        discharges = self.get_discharges()
        if len(discharges) < 2:
            print("  Need at least 2 discharges for overlay.")
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        fig.suptitle(f"Discharge Overlay — {self.name}", fontweight="bold")

        for i, seg in enumerate(discharges):
            t = seg["t_s"].values - seg["t_s"].values[0]
            axes[0].plot(t, seg["p_line_psi"],      alpha=0.6, linewidth=0.8, label=f"#{i}")
            axes[1].plot(t, seg["rpm_alternator"],   alpha=0.6, linewidth=0.8, label=f"#{i}")

        axes[0].set_ylabel("Pressure (psi)")
        axes[0].legend(fontsize=7, ncol=4)
        axes[0].grid(True, alpha=0.3)

        axes[1].set_ylabel("Alternator RPM")
        axes[1].set_xlabel("Time since discharge start (s)")
        axes[1].grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_charge_profile(self, charge_idx: int = 0, save_path: str = None):
        """Charge curve: pressure vs time, with exponential fit overlay."""
        charges = self.get_charges()
        if charge_idx >= len(charges):
            print("  No charge segment found.")
            return

        seg = charges[charge_idx]
        t = seg["t_s"].values - seg["t_s"].values[0]
        p = seg["p_line_psi"].values

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.set_title(f"Charge Profile #{charge_idx} — {self.name}",
                     fontweight="bold")
        ax.plot(t, p, color="tab:blue", linewidth=1.2, label="Measured")

        # Attempt exponential fit:  P(t) = P_f * (1 - exp(-t/tau)) + P_0
        try:
            from scipy.optimize import curve_fit
            def exp_rise(t, p_f, tau, p_0):
                return p_f * (1.0 - np.exp(-t / tau)) + p_0
            popt, _ = curve_fit(exp_rise, t, p, p0=[p[-1], 5.0, p[0]],
                                maxfev=5000)
            p_fit = exp_rise(t, *popt)
            ax.plot(t, p_fit, "--", color="tab:orange", linewidth=1,
                    label=f"Fit: P_f={popt[0]:.1f}, τ={popt[1]:.2f}s")
        except Exception:
            pass  # fit failed, no big deal

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure (psi)")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_water_level(self, save_path: str = None):
        """Water level timeline with tank thresholds."""
        df = self.df
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.set_title(f"Water Level — {self.name}", fontweight="bold")

        ax.plot(df["t_s"], df["water_level_cm"], color="tab:blue", linewidth=1)
        ax.axhline(45.0, color="tab:green",  linestyle="--", linewidth=0.8, label="Tank Full")
        ax.axhline(8.0,  color="tab:orange", linestyle="--", linewidth=0.8, label="Low Warning")
        ax.axhline(3.0,  color="tab:red",    linestyle="--", linewidth=0.8, label="Tank Empty")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Water Level (cm)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_bus_voltage(self, save_path: str = None):
        """Bus voltage with valve actuation events overlaid — shows brownouts."""
        df = self.df
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.set_title(f"12V Bus Voltage — {self.name}", fontweight="bold")

        ax.plot(df["t_s"], df["v_bus_V"], color="tab:orange", linewidth=0.8)
        ax.axhline(11.0, color="tab:red", linestyle="--", linewidth=0.8,
                   label="Undervoltage threshold")

        # Mark valve transitions as vertical lines
        for col, color in [("valve_disch_air_cmd", "tab:cyan"),
                           ("valve_disch_water_cmd", "tab:blue")]:
            if col in df.columns:
                edges = np.where(np.diff(df[col].values.astype(int)) != 0)[0]
                for e in edges:
                    ax.axvline(df["t_s"].iloc[e], color=color, alpha=0.3, linewidth=0.5)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Voltage (V)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()


# =========================================================================
# MULTI-RUN COMPARISON
# =========================================================================

def compare_runs(csv_paths: list[str]):
    """Load multiple runs and produce comparison tables + overlay plots."""
    runs = [TestRun(p) for p in csv_paths]

    # Summary table
    summaries = [r.summary() for r in runs]
    df_summary = pd.DataFrame(summaries)
    print("\n=== MULTI-RUN COMPARISON ===\n")
    cols_show = ["file", "num_discharges", "peak_pressure_psi",
                 "peak_rpm_alt", "peak_power_alt_W", "total_energy_alt_J",
                 "v_bus_min"]
    print(df_summary[[c for c in cols_show if c in df_summary.columns]].to_string())

    # Overlay: peak RPM vs run index (trend)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    axes[0].bar(range(len(summaries)),
                [s.get("peak_rpm_alt", 0) for s in summaries],
                color="tab:purple", alpha=0.7)
    axes[0].set_xlabel("Run index")
    axes[0].set_ylabel("Peak alt RPM")
    axes[0].set_title("Peak RPM Across Runs")
    axes[0].grid(True, alpha=0.3)

    axes[1].bar(range(len(summaries)),
                [s.get("peak_power_alt_W", 0) for s in summaries],
                color="tab:green", alpha=0.7)
    axes[1].set_xlabel("Run index")
    axes[1].set_ylabel("Peak alt power (W)")
    axes[1].set_title("Peak Power Across Runs")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()

    return runs, df_summary


# =========================================================================
# ENTRY POINT
# =========================================================================

def main():
    parser = argparse.ArgumentParser(description="Test Stand Post-Analysis")
    parser.add_argument("files", nargs="+", help="Telemetry CSV file(s)")
    parser.add_argument("--compare", action="store_true",
                        help="Compare multiple runs side-by-side")
    parser.add_argument("--save-dir", default="plots",
                        help="Directory for saved plot images")
    args = parser.parse_args()

    # Expand globs
    paths = []
    for f in args.files:
        paths.extend(glob.glob(f))
    paths = sorted(set(paths))

    if not paths:
        print("No CSV files found.")
        sys.exit(1)

    os.makedirs(args.save_dir, exist_ok=True)

    if args.compare and len(paths) > 1:
        compare_runs(paths)
    else:
        for p in paths:
            run = TestRun(p)
            run.print_summary()

            # Step-response for first discharge
            discharges = run.get_discharges()
            if discharges:
                sr = run.step_response_metrics(discharges[0])
                if sr:
                    print("  Step-response (RPM, discharge #0):")
                    for k, v in sr.items():
                        print(f"    {k:25s} = {v}")

            # Leak-down
            ld = run.leak_down_rate()
            if ld is not None:
                print(f"  Leak-down rate: {ld:.3f} psi/min")

            # Valve response
            vr = run.valve_response_times()
            if vr:
                print("  Valve response times:")
                for v in vr:
                    print(f"    {v['valve']:30s} @ {v['t_cmd_s']:.1f}s → "
                          f"{v['delay_ms']:.1f} ms delay")

            # Plots
            base = os.path.splitext(os.path.basename(p))[0]
            run.plot_full_timeline(
                os.path.join(args.save_dir, f"{base}_timeline.png"))
            if discharges:
                run.plot_discharge_profile(0,
                    os.path.join(args.save_dir, f"{base}_discharge0.png"))
                run.plot_pressure_vs_rpm(
                    os.path.join(args.save_dir, f"{base}_p_vs_rpm.png"))
                run.plot_fft_rpm(0,
                    os.path.join(args.save_dir, f"{base}_fft_rpm.png"))
                if len(discharges) > 1:
                    run.plot_discharge_overlay(
                        os.path.join(args.save_dir, f"{base}_overlay.png"))
            charges = run.get_charges()
            if charges:
                run.plot_charge_profile(0,
                    os.path.join(args.save_dir, f"{base}_charge0.png"))
            run.plot_water_level(
                os.path.join(args.save_dir, f"{base}_water_level.png"))
            run.plot_bus_voltage(
                os.path.join(args.save_dir, f"{base}_vbus.png"))


if __name__ == "__main__":
    main()
