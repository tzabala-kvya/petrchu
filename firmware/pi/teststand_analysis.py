#!/usr/bin/env python3
"""
teststand_analysis.py
=====================
Post-test analysis for PetrChu test stand CSV logs.

Load one or more telemetry CSVs (produced by teststand_logger.py post-pivot),
compute headline metrics, and generate publication-ready plots. The headline
metric for this PoC is round-trip efficiency η; everything else is in support
of telling that story.

Usage:
    python teststand_analysis.py logs/telem_20260420_143200_a1b2c3d4.csv
    python teststand_analysis.py logs/telem_*.csv --compare

Or interactively:
    from teststand_analysis import TestRun, compare_runs
    run = TestRun("logs/telem_xxx.csv")
    run.print_summary()
    run.plot_full_timeline()
    run.plot_efficiency_summary()
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================================================================
# CONSTANTS
# =========================================================================

SAMPLE_RATE_HZ = 10   # 10 Hz telemetry from the Mega

DISCHARGE_STATES = {"DISCHARGE_AIR", "DISCHARGE_WATER", "DISCHARGE_BOTH"}
CHARGE_STATES    = {"CHARGE_AIR", "CHARGE_WATER", "CHARGE_BOTH"}
ENERGIZED_STATES = DISCHARGE_STATES | CHARGE_STATES

# State code -> name (matches firmware enum)
STATE_MAP = {
    0:  "BOOT_SELFTEST",
    1:  "OFF",
    2:  "ARMED_IDLE",
    3:  "CHARGE_AIR",
    4:  "CHARGE_WATER",
    5:  "CHARGE_BOTH",
    6:  "DISCHARGE_AIR",
    7:  "DISCHARGE_WATER",
    8:  "DISCHARGE_BOTH",
    9:  "SHUTDOWN",
    10: "FAULT",
    11: "ESTOP",
}

NUMERIC_COLUMNS = [
    "v_alt_V", "i_load_A", "power_alt_W",
    "i_pump_A", "i_comp_A", "power_pump_W", "power_comp_W",
    "p1_vessel_psi", "p2_motor_psi",
    "p_hydro_mbar", "temp_water_C", "depth_cm",
    "rpm_alt", "rpm_water",
    "water_valve_cmd_pct", "water_valve_fb_pct", "air_pulse_hz",
    "v_setpoint_V",
    "energy_alt_J", "energy_pump_J", "energy_comp_J", "energy_input_J",
    "eta_round_trip_pct", "dp1_dt_psi_per_s",
]


# =========================================================================
# TEST RUN
# =========================================================================

class TestRun:
    """One logged test session."""

    def __init__(self, csv_path: str, events_path: str = None):
        self.path = csv_path
        self.name = os.path.basename(csv_path).replace(".csv", "")
        self.df = pd.read_csv(csv_path)
        self._prepare()
        self.events = None
        if events_path and os.path.exists(events_path):
            self.events = pd.read_csv(events_path)

    def _prepare(self):
        df = self.df
        if "timestamp_ms" in df.columns:
            df["t_s"] = (df["timestamp_ms"] - df["timestamp_ms"].iloc[0]) / 1000.0
        else:
            df["t_s"] = np.arange(len(df)) / SAMPLE_RATE_HZ
        for col in NUMERIC_COLUMNS:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors="coerce").fillna(0.0)
        if "state_name" not in df.columns and "state" in df.columns:
            df["state_name"] = df["state"].map(STATE_MAP).fillna("UNKNOWN")

    # -----------------------------------------------------------------
    # Segments
    # -----------------------------------------------------------------

    def get_segments(self, state_names: set):
        """Contiguous segments where state_name is in the given set."""
        df = self.df
        mask = df["state_name"].isin(state_names)
        segs, in_seg, start = [], False, 0
        for i, val in enumerate(mask):
            if val and not in_seg:
                start, in_seg = i, True
            elif not val and in_seg:
                segs.append(df.iloc[start:i].copy())
                in_seg = False
        if in_seg:
            segs.append(df.iloc[start:].copy())
        return segs

    def get_discharges(self):
        return self.get_segments(DISCHARGE_STATES)

    def get_charges(self):
        return self.get_segments(CHARGE_STATES)

    # -----------------------------------------------------------------
    # Headline metrics
    # -----------------------------------------------------------------

    def round_trip_efficiency(self) -> float:
        """Final η (%) at end of run."""
        df = self.df
        if len(df) == 0:
            return 0.0
        if "eta_round_trip_pct" in df.columns:
            return float(df["eta_round_trip_pct"].iloc[-1])
        # Fallback: recompute from totals
        e_out = df["energy_alt_J"].iloc[-1] if "energy_alt_J" in df.columns else 0.0
        e_in  = df["energy_input_J"].iloc[-1] if "energy_input_J" in df.columns else 0.0
        return 100.0 * e_out / e_in if e_in > 0 else 0.0

    def summary(self) -> dict:
        df = self.df
        discharges = self.get_discharges()
        charges    = self.get_charges()

        s = {
            "file":               self.name,
            "duration_s":         round(df["t_s"].iloc[-1], 1) if len(df) else 0.0,
            "samples":            len(df),
            "num_charges":        len(charges),
            "num_discharges":     len(discharges),
            "peak_v_alt_V":       round(df["v_alt_V"].max(), 2)       if "v_alt_V" in df else 0.0,
            "peak_p1_psi":        round(df["p1_vessel_psi"].max(), 1) if "p1_vessel_psi" in df else 0.0,
            "peak_rpm_alt":       round(df["rpm_alt"].max(), 0)       if "rpm_alt" in df else 0.0,
            "peak_power_out_W":   round(df["power_alt_W"].max(), 1)   if "power_alt_W" in df else 0.0,
            "energy_out_J":       round(df["energy_alt_J"].iloc[-1], 1)   if "energy_alt_J" in df else 0.0,
            "energy_in_J":        round(df["energy_input_J"].iloc[-1], 1) if "energy_input_J" in df else 0.0,
            "eta_round_trip_pct": round(self.round_trip_efficiency(), 2),
            "fault_samples":      int((df["fault_list"] != "NONE").sum()) if "fault_list" in df.columns else 0,
        }

        for i, seg in enumerate(discharges):
            dur = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
            mode_name = seg["state_name"].mode().iloc[0]
            energy_out = float(np.trapezoid(seg["power_alt_W"], seg["t_s"])) \
                if "power_alt_W" in seg.columns else 0.0
            s[f"discharge_{i}_state"]      = mode_name
            s[f"discharge_{i}_duration_s"] = round(dur, 2)
            s[f"discharge_{i}_peak_V_alt"] = round(seg["v_alt_V"].max(), 2)
            s[f"discharge_{i}_mean_V_alt"] = round(seg["v_alt_V"].mean(), 2)
            s[f"discharge_{i}_peak_P_W"]   = round(seg["power_alt_W"].max(), 1)
            s[f"discharge_{i}_energy_J"]   = round(energy_out, 1)

        for i, seg in enumerate(charges):
            dur = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
            mode_name = seg["state_name"].mode().iloc[0]
            s[f"charge_{i}_state"]      = mode_name
            s[f"charge_{i}_duration_s"] = round(dur, 2)
            if "p1_vessel_psi" in seg.columns:
                p_start = seg["p1_vessel_psi"].iloc[0]
                p_end   = seg["p1_vessel_psi"].iloc[-1]
                rate    = (p_end - p_start) / dur if dur > 0 else 0.0
                s[f"charge_{i}_P1_start"] = round(p_start, 1)
                s[f"charge_{i}_P1_end"]   = round(p_end, 1)
                s[f"charge_{i}_rate_psi_per_s"] = round(rate, 2)

        return s

    def print_summary(self):
        s = self.summary()
        print(f"\n{'='*60}")
        print(f"  RUN SUMMARY: {s['file']}")
        print(f"{'='*60}")
        print(f"  Duration:           {s['duration_s']:.1f} s  ({s['samples']} samples)")
        print(f"  Charges:            {s['num_charges']}    Discharges: {s['num_discharges']}")
        print(f"  Peak V_alt:         {s['peak_v_alt_V']:.2f} V")
        print(f"  Peak P1 (vessel):   {s['peak_p1_psi']:.1f} psi")
        print(f"  Peak alt RPM:       {s['peak_rpm_alt']:.0f}")
        print(f"  Peak P_out:         {s['peak_power_out_W']:.1f} W")
        print(f"  Energy in/out:      {s['energy_in_J']:.1f} J in,  {s['energy_out_J']:.1f} J out")
        print(f"  η (round-trip):     {s['eta_round_trip_pct']:.2f} %")
        print(f"  Fault samples:      {s['fault_samples']}")
        print()

    # -----------------------------------------------------------------
    # V-regulation step-response on a discharge
    # -----------------------------------------------------------------

    def v_regulation_metrics(self, seg: pd.DataFrame) -> dict:
        """Settling time + steady-state error of V_alt vs setpoint."""
        if "v_alt_V" not in seg or "v_setpoint_V" not in seg or len(seg) < 10:
            return {}
        v = seg["v_alt_V"].values
        t = seg["t_s"].values - seg["t_s"].values[0]
        sp = float(seg["v_setpoint_V"].iloc[-1])
        if sp <= 0:
            return {}
        # Steady-state: mean of last 20 %
        tail = v[-max(10, len(v)//5):]
        v_ss = float(np.mean(tail))
        ss_err_pct = 100.0 * (v_ss - sp) / sp

        # Settling time: last time |V - sp| exited ±5 % of sp
        band = 0.05 * sp
        outside = np.where(np.abs(v - sp) > band)[0]
        settling_s = float(t[outside[-1]]) if len(outside) else 0.0

        return {
            "v_setpoint_V":         round(sp, 2),
            "v_ss_V":               round(v_ss, 2),
            "steady_state_err_pct": round(ss_err_pct, 2),
            "settling_time_s_pm5":  round(settling_s, 2),
            "v_max_V":              round(float(np.max(v)), 2),
            "v_min_V":              round(float(np.min(v)), 2),
        }

    # -----------------------------------------------------------------
    # Leak-down (P1 decay during ARMED_IDLE after a charge)
    # -----------------------------------------------------------------

    def leak_down_rate(self):
        idle_segs = self.get_segments({"ARMED_IDLE"})
        if not idle_segs:
            return None
        seg = max(idle_segs, key=len)
        if len(seg) < 30:
            return None
        if "p1_vessel_psi" not in seg.columns:
            return None
        dt_s = seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]
        dp = seg["p1_vessel_psi"].iloc[-1] - seg["p1_vessel_psi"].iloc[0]
        return round((dp / dt_s) * 60.0, 3)   # psi/min

    # =================================================================
    # PLOTS
    # =================================================================

    def plot_full_timeline(self, save_path: str = None):
        df = self.df
        fig, axes = plt.subplots(5, 1, figsize=(14, 11), sharex=True)
        fig.suptitle(f"Full timeline — {self.name}", fontsize=12, fontweight="bold")

        # V_alt vs target
        ax = axes[0]
        ax.plot(df["t_s"], df["v_alt_V"],     color="tab:green",  linewidth=0.8, label="V_alt")
        if "v_setpoint_V" in df.columns:
            ax.plot(df["t_s"], df["v_setpoint_V"], color="tab:orange",
                    linewidth=0.8, linestyle="--", label="target")
        ax.set_ylabel("V_alt (V)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # P1 / P2
        ax = axes[1]
        ax.plot(df["t_s"], df["p1_vessel_psi"], color="tab:purple", linewidth=0.8, label="P1 vessel")
        ax.plot(df["t_s"], df["p2_motor_psi"],  color="tab:orange", linewidth=0.8, label="P2 motor")
        ax.set_ylabel("Pressure (psi)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # Alt RPM
        ax = axes[2]
        ax.plot(df["t_s"], df["rpm_alt"], color="tab:purple", linewidth=0.8)
        ax.set_ylabel("Alt RPM")
        ax.grid(True, alpha=0.3)

        # Power in/out + cumulative energy (twin axis)
        ax = axes[3]
        ax.plot(df["t_s"], df["power_alt_W"],  color="tab:green",  linewidth=0.8, label="P_alt out")
        ax.plot(df["t_s"], df["power_pump_W"], color="tab:cyan",   linewidth=0.8, label="P_pump in")
        ax.plot(df["t_s"], df["power_comp_W"], color="tab:red",    linewidth=0.8, label="P_comp in")
        ax.set_ylabel("Power (W)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        if "eta_round_trip_pct" in df.columns:
            ax2 = ax.twinx()
            ax2.plot(df["t_s"], df["eta_round_trip_pct"],
                     color="tab:orange", linewidth=1.0, alpha=0.6, label="η (%)")
            ax2.set_ylabel("η (%)", color="tab:orange")
            ax2.tick_params(axis="y", colors="tab:orange")

        # State
        ax = axes[4]
        if "state" in df.columns:
            ax.plot(df["t_s"], df["state"], drawstyle="steps-post",
                    color="tab:gray", linewidth=1.0)
            ax.set_yticks(range(len(STATE_MAP)))
            ax.set_yticklabels([STATE_MAP[i] for i in range(len(STATE_MAP))],
                               fontsize=6)
        ax.set_ylabel("State")
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"  Saved: {save_path}")
        plt.show()

    def plot_efficiency_summary(self, save_path: str = None):
        """Headline plot for the PoC argument: cumulative energy in vs out + η."""
        df = self.df
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.set_title(f"Round-trip efficiency — {self.name}",
                     fontsize=12, fontweight="bold")

        ax.plot(df["t_s"], df["energy_input_J"], color="tab:red",
                linewidth=1.3, label="Energy in (pump + compressor)")
        ax.plot(df["t_s"], df["energy_alt_J"],   color="tab:green",
                linewidth=1.3, label="Energy out (alt)")
        ax.fill_between(df["t_s"], 0, df["energy_input_J"],
                        alpha=0.15, color="tab:red")
        ax.fill_between(df["t_s"], 0, df["energy_alt_J"],
                        alpha=0.15, color="tab:green")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Cumulative energy (J)")
        ax.legend(loc="upper left", fontsize=9)
        ax.grid(True, alpha=0.3)

        if "eta_round_trip_pct" in df.columns:
            ax2 = ax.twinx()
            ax2.plot(df["t_s"], df["eta_round_trip_pct"], color="tab:orange",
                     linewidth=1.5, label="η (%)")
            ax2.set_ylabel("η (%)", color="tab:orange")
            ax2.tick_params(axis="y", colors="tab:orange")
            eta_final = self.round_trip_efficiency()
            ax2.text(0.98, 0.95, f"η final = {eta_final:.2f} %",
                     transform=ax2.transAxes, ha="right", va="top",
                     fontsize=11, fontweight="bold", color="tab:orange",
                     bbox=dict(facecolor="white", alpha=0.8, edgecolor="tab:orange"))

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"  Saved: {save_path}")
        plt.show()

    def plot_discharge_profile(self, idx: int = 0, save_path: str = None):
        discharges = self.get_discharges()
        if idx >= len(discharges):
            print(f"  No discharge #{idx}.")
            return
        seg = discharges[idx]
        t = seg["t_s"].values - seg["t_s"].values[0]
        mode = seg["state_name"].mode().iloc[0]

        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f"Discharge #{idx} ({mode}) — {self.name}",
                     fontsize=12, fontweight="bold")

        # V_alt + target + ±5 % band
        ax = axes[0]
        ax.plot(t, seg["v_alt_V"], color="tab:green", linewidth=1.1, label="V_alt")
        if "v_setpoint_V" in seg.columns:
            sp = seg["v_setpoint_V"].values
            ax.plot(t, sp, color="tab:orange", linestyle="--", linewidth=1, label="target")
            ax.fill_between(t, 0.95*sp, 1.05*sp, color="tab:orange", alpha=0.1, label="±5%")
        ax.set_ylabel("V_alt (V)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Actuator output
        ax = axes[1]
        if mode in ("DISCHARGE_WATER", "DISCHARGE_BOTH"):
            ax.plot(t, seg["water_valve_cmd_pct"], color="tab:cyan",
                    linewidth=1, label="water valve cmd %")
            ax.plot(t, seg["water_valve_fb_pct"], color="tab:blue",
                    linewidth=1, linestyle=":", label="water valve fb %")
        if mode in ("DISCHARGE_AIR", "DISCHARGE_BOTH"):
            ax2 = ax.twinx()
            ax2.plot(t, seg["air_pulse_hz"], color="tab:purple",
                     linewidth=1, label="air pulse Hz")
            ax2.set_ylabel("Air pulse (Hz)", color="tab:purple")
            ax2.tick_params(axis="y", colors="tab:purple")
            ax2.legend(loc="upper right", fontsize=8)
        ax.set_ylabel("Water valve (%)")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)

        # Power
        ax = axes[2]
        ax.plot(t, seg["power_alt_W"], color="tab:green", linewidth=1.1, label="P_alt out")
        ax.fill_between(t, 0, seg["power_alt_W"], alpha=0.15, color="tab:green")
        ax.set_ylabel("Power (W)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Pressure (only meaningful for air discharge)
        ax = axes[3]
        if mode in ("DISCHARGE_AIR", "DISCHARGE_BOTH"):
            ax.plot(t, seg["p1_vessel_psi"], color="tab:purple", linewidth=1, label="P1 vessel")
            ax.plot(t, seg["p2_motor_psi"],  color="tab:orange", linewidth=1, label="P2 motor")
            ax.legend(fontsize=8)
        ax.set_ylabel("Pressure (psi)")
        ax.set_xlabel("Time since discharge start (s)")
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_charge_profile(self, idx: int = 0, save_path: str = None):
        charges = self.get_charges()
        if idx >= len(charges):
            print(f"  No charge #{idx}.")
            return
        seg = charges[idx]
        t = seg["t_s"].values - seg["t_s"].values[0]
        mode = seg["state_name"].mode().iloc[0]

        fig, ax = plt.subplots(figsize=(12, 5))
        ax.set_title(f"Charge #{idx} ({mode}) — {self.name}", fontweight="bold")

        if mode in ("CHARGE_AIR", "CHARGE_BOTH"):
            ax.plot(t, seg["p1_vessel_psi"], color="tab:purple",
                    linewidth=1.2, label="P1 vessel (psi)")
        if mode in ("CHARGE_WATER", "CHARGE_BOTH") and "depth_cm" in seg.columns:
            ax2 = ax.twinx()
            ax2.plot(t, seg["depth_cm"], color="tab:cyan",
                     linewidth=1.2, label="upper tank depth (cm)")
            ax2.set_ylabel("Tank depth (cm)", color="tab:cyan")
            ax2.tick_params(axis="y", colors="tab:cyan")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure (psi)")
        ax.legend(loc="upper left", fontsize=9)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_discharge_overlay(self, save_path: str = None):
        """Overlay all discharges on the same time axis. Shows repeatability."""
        discharges = self.get_discharges()
        if len(discharges) < 2:
            print("  Need at least 2 discharges for overlay.")
            return
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        fig.suptitle(f"Discharge overlay — {self.name}", fontweight="bold")
        for i, seg in enumerate(discharges):
            t = seg["t_s"].values - seg["t_s"].values[0]
            mode = seg["state_name"].mode().iloc[0]
            axes[0].plot(t, seg["v_alt_V"],     alpha=0.7, linewidth=0.9, label=f"#{i} ({mode})")
            axes[1].plot(t, seg["power_alt_W"], alpha=0.7, linewidth=0.9, label=f"#{i}")
        axes[0].set_ylabel("V_alt (V)")
        axes[0].legend(fontsize=7, ncol=2)
        axes[0].grid(True, alpha=0.3)
        axes[1].set_ylabel("P_alt (W)")
        axes[1].set_xlabel("Time since discharge start (s)")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_water_depth(self, save_path: str = None):
        df = self.df
        if "depth_cm" not in df.columns:
            return
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.set_title(f"Lower-tank depth — {self.name}", fontweight="bold")
        ax.plot(df["t_s"], df["depth_cm"], color="tab:cyan", linewidth=1)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Depth (cm)")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()

    def plot_dispatch(self, save_path: str = None):
        """Dispatch view: water FB position + air arm + air pulse Hz."""
        df = self.df
        both_mask = df["state_name"] == "DISCHARGE_BOTH"
        if not both_mask.any():
            print("  No DISCHARGE_BOTH segment found.")
            return
        seg = df[both_mask].copy()
        t = seg["t_s"].values - seg["t_s"].values[0]
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.set_title(f"Dispatch (BOTH) — {self.name}", fontweight="bold")
        ax.plot(t, seg["water_valve_fb_pct"], color="tab:blue",
                linewidth=1, label="Water valve FB %")
        ax.axhline(80, color="tab:gray", linestyle="--", linewidth=0.7,
                   label="add-air threshold (80%)")
        ax.set_ylabel("Water valve FB (%)")
        ax.set_xlabel("Time since BOTH start (s)")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(t, seg["air_pulse_hz"], color="tab:purple",
                 linewidth=1, label="Air pulse Hz")
        ax2.set_ylabel("Air pulse Hz", color="tab:purple")
        ax2.tick_params(axis="y", colors="tab:purple")
        if "air_arm_open" in seg.columns:
            # Shade where air is engaged
            engaged = seg["air_arm_open"].values.astype(int)
            edges = np.where(np.diff(engaged) != 0)[0]
            on = engaged[0] != 0
            last = 0
            for e in edges:
                if on:
                    ax.axvspan(t[last], t[e], color="tab:orange", alpha=0.07)
                on = not on
                last = e
            if on:
                ax.axvspan(t[last], t[-1], color="tab:orange", alpha=0.07,
                           label="air engaged")

        fig.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.show()


# =========================================================================
# MULTI-RUN COMPARISON
# =========================================================================

def compare_runs(csv_paths):
    runs = [TestRun(p) for p in csv_paths]
    summaries = [r.summary() for r in runs]
    df = pd.DataFrame(summaries)

    print("\n=== MULTI-RUN COMPARISON ===\n")
    cols = ["file", "duration_s", "num_discharges",
            "peak_v_alt_V", "peak_power_out_W",
            "energy_in_J", "energy_out_J", "eta_round_trip_pct"]
    show = [c for c in cols if c in df.columns]
    print(df[show].to_string(index=False))

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    axes[0].bar(range(len(summaries)),
                [s.get("eta_round_trip_pct", 0) for s in summaries],
                color="tab:orange", alpha=0.8)
    axes[0].set_xlabel("Run index")
    axes[0].set_ylabel("η round-trip (%)")
    axes[0].set_title("Round-trip efficiency")
    axes[0].grid(True, alpha=0.3)

    axes[1].bar(range(len(summaries)),
                [s.get("peak_power_out_W", 0) for s in summaries],
                color="tab:green", alpha=0.8)
    axes[1].set_xlabel("Run index")
    axes[1].set_ylabel("Peak P_alt (W)")
    axes[1].set_title("Peak output power")
    axes[1].grid(True, alpha=0.3)

    axes[2].bar(range(len(summaries)),
                [s.get("energy_out_J", 0) for s in summaries],
                color="tab:blue", alpha=0.8)
    axes[2].set_xlabel("Run index")
    axes[2].set_ylabel("Energy out (J)")
    axes[2].set_title("Total energy delivered")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()
    return runs, df


# =========================================================================
# ENTRY POINT
# =========================================================================

def main():
    p = argparse.ArgumentParser(description="PetrChu Test Stand Post-Analysis")
    p.add_argument("files", nargs="+", help="Telemetry CSV file(s)")
    p.add_argument("--compare", action="store_true",
                   help="Side-by-side compare across runs")
    p.add_argument("--save-dir", default="plots",
                   help="Directory for saved plot images")
    args = p.parse_args()

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
        return

    for csv_path in paths:
        run = TestRun(csv_path)
        run.print_summary()

        # V-regulation metrics for first discharge
        discharges = run.get_discharges()
        if discharges:
            vreg = run.v_regulation_metrics(discharges[0])
            if vreg:
                print("  V-regulation (discharge #0):")
                for k, v in vreg.items():
                    print(f"    {k:25s} = {v}")

        ld = run.leak_down_rate()
        if ld is not None:
            print(f"  Leak-down rate (ARMED_IDLE): {ld:.3f} psi/min")

        # Plots
        base = os.path.splitext(os.path.basename(csv_path))[0]
        sd = args.save_dir
        run.plot_full_timeline(os.path.join(sd, f"{base}_timeline.png"))
        run.plot_efficiency_summary(os.path.join(sd, f"{base}_efficiency.png"))
        if discharges:
            run.plot_discharge_profile(0, os.path.join(sd, f"{base}_discharge0.png"))
            if len(discharges) > 1:
                run.plot_discharge_overlay(os.path.join(sd, f"{base}_overlay.png"))
        charges = run.get_charges()
        if charges:
            run.plot_charge_profile(0, os.path.join(sd, f"{base}_charge0.png"))
        run.plot_water_depth(os.path.join(sd, f"{base}_depth.png"))
        if any(s == "DISCHARGE_BOTH" for s in run.df["state_name"].unique()):
            run.plot_dispatch(os.path.join(sd, f"{base}_dispatch.png"))


if __name__ == "__main__":
    main()
