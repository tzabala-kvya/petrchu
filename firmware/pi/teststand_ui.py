#!/usr/bin/env python3
"""
teststand_ui.py
===============
Real-time operator UI for the Water/Air Test Stand.

Tkinter + matplotlib strip charts.  Talks to the Mega through
teststand_logger.Logger.

Usage:
    python teststand_ui.py --port /dev/ttyACM0
    python teststand_ui.py --port COM3          # Windows

Layout:
    ┌─────────────────────────────────────────────────────┐
    │  STATE BADGE  │ P │ RPM_a │ RPM_w │ RPM_s │ V_alt  │  ← big numbers
    ├───────────────┴───┴───────┴───────┴───────┴────────┤
    │  Strip chart: Pressure + dP/dt                      │
    │  Strip chart: RPM (all 3) + driving source          │
    │  Strip chart: Power (supply, alt, comp, pump)       │
    ├─────────────────────────────────────────────────────┤
    │  I/O status indicators │ Event log (scrolling text) │
    ├─────────────────────────────────────────────────────┤
    │  Command buttons: ARM  CHARGE  DISCHARGE  STOP  ... │
    └─────────────────────────────────────────────────────┘
"""

import argparse
import sys
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from teststand_logger import Logger, STATE_NAMES, CAL_STATES

# =========================================================================
# CONSTANTS
# =========================================================================

REFRESH_MS       = 100   # UI update rate (10 Hz matches telemetry)
CHART_WINDOW_S   = 60    # rolling window on strip charts
CHART_POINTS     = 600   # 60 s × 10 Hz

# Color scheme
C_BG       = "#1a1a2e"
C_FG       = "#e0e0e0"
C_ACCENT   = "#0f3460"
C_GREEN    = "#2ecc71"
C_YELLOW   = "#f39c12"
C_RED      = "#e74c3c"
C_BLUE     = "#3498db"
C_PURPLE   = "#9b59b6"
C_ORANGE   = "#e67e22"
C_CYAN     = "#1abc9c"
C_DARK     = "#0d0d1a"

STATE_COLORS = {
    "BOOT_SELFTEST":    C_YELLOW,
    "CAL_PROMPT":       C_ORANGE,
    "CAL_ZEROING":      C_ORANGE,
    "CAL_REPLACE":      C_ORANGE,
    "OFF":              C_FG,
    "ARM":              C_GREEN,
    "CHARGE":           C_YELLOW,
    "READY":            C_GREEN,
    "DISCHARGE_AIR":    C_BLUE,
    "DISCHARGE_WATER":  C_CYAN,
    "DISCHARGE_BOTH":   C_PURPLE,
    "MANUAL_DIAG":      C_ORANGE,
    "SHUTDOWN":         C_YELLOW,
    "FAULT":            C_RED,
    "ESTOP":            C_RED,
}


# =========================================================================
# BIG NUMBER WIDGET
# =========================================================================

class BigNumber(tk.Frame):
    """A large numeric readout with label and optional bar/color coding."""

    def __init__(self, parent, label: str, unit: str = "",
                 warn_hi=None, crit_hi=None, fmt=".1f", width=120):
        super().__init__(parent, bg=C_BG, width=width)
        self.warn_hi = warn_hi
        self.crit_hi = crit_hi
        self.fmt     = fmt

        self._label = tk.Label(self, text=label, font=("Consolas", 9),
                               fg="#888", bg=C_BG)
        self._label.pack(anchor="w", padx=4)

        self._value = tk.Label(self, text="---", font=("Consolas", 22, "bold"),
                               fg=C_FG, bg=C_BG)
        self._value.pack(anchor="w", padx=4)

        self._unit = tk.Label(self, text=unit, font=("Consolas", 9),
                              fg="#888", bg=C_BG)
        self._unit.pack(anchor="w", padx=4)

    def set(self, value):
        if value is None or value == "":
            self._value.config(text="---", fg=C_FG)
            return
        try:
            v = float(value)
            txt = f"{v:{self.fmt}}"
        except (ValueError, TypeError):
            txt = str(value)
            self._value.config(text=txt, fg=C_FG)
            return

        color = C_FG
        if self.crit_hi is not None and v >= self.crit_hi:
            color = C_RED
        elif self.warn_hi is not None and v >= self.warn_hi:
            color = C_YELLOW
        self._value.config(text=txt, fg=color)


# =========================================================================
# IO STATUS PANEL
# =========================================================================

class IOStatusPanel(tk.Frame):
    """Grid of small LED-like indicators for actuator states."""

    def __init__(self, parent):
        super().__init__(parent, bg=C_BG)

        self._indicators = {}
        labels = [
            ("E-STOP",     "estop"),
            ("ARM",        "arm_switch"),
            ("CONTACTOR",  "contactor_cmd"),
            ("SOL AIR",    "sol_air_cmd"),
            ("SOL WATER",  "sol_water_cmd"),
            ("CHG AIR",    "valve_charge_air_cmd"),
            ("CHG WATER",  "valve_charge_water_cmd"),
            ("DSC AIR",    "valve_disch_air_cmd"),
            ("DSC WATER",  "valve_disch_water_cmd"),
            ("VENT",       "vent_cmd"),
            ("TANK FULL",  "tank_full"),
            ("TANK EMPTY", "tank_empty"),
        ]

        for i, (text, key) in enumerate(labels):
            row, col = divmod(i, 6)
            lbl = tk.Label(self, text=text, font=("Consolas", 8),
                           fg="#888", bg=C_BG, width=12)
            lbl.grid(row=row*2, column=col, padx=2, pady=0)
            dot = tk.Label(self, text="●", font=("Consolas", 14),
                           fg="#444", bg=C_BG)
            dot.grid(row=row*2+1, column=col, padx=2, pady=0)
            self._indicators[key] = dot

    def update(self, row: dict):
        for key, dot in self._indicators.items():
            val = row.get(key, 0)
            try:
                on = int(val) != 0
            except (ValueError, TypeError):
                on = False

            if key == "estop":
                dot.config(fg=C_RED if on else C_GREEN)
            elif key in ("tank_full", "tank_empty"):
                dot.config(fg=C_YELLOW if on else "#444")
            else:
                dot.config(fg=C_GREEN if on else "#444")


# =========================================================================
# STRIP CHART
# =========================================================================

class StripChart:
    """Matplotlib strip chart embedded in Tkinter."""

    def __init__(self, parent, title: str, y_label: str,
                 traces: list[tuple[str, str, str]],
                 y_range: tuple = None):
        """
        traces: list of (csv_key, legend_label, color)
        """
        self.traces  = traces
        self.y_range = y_range

        self.fig = Figure(figsize=(8, 1.8), dpi=100, facecolor=C_DARK)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_facecolor(C_DARK)
        self.ax.set_title(title, color=C_FG, fontsize=9, loc="left")
        self.ax.set_ylabel(y_label, color=C_FG, fontsize=8)
        self.ax.tick_params(colors="#888", labelsize=7)
        for spine in self.ax.spines.values():
            spine.set_color("#333")
        self.ax.grid(True, color="#333", linewidth=0.5, alpha=0.5)

        self._lines = {}
        for key, label, color in traces:
            line, = self.ax.plot([], [], color=color, linewidth=1.2,
                                label=label)
            self._lines[key] = line

        self.ax.legend(loc="upper left", fontsize=7, framealpha=0.3,
                       facecolor=C_DARK, edgecolor="#444",
                       labelcolor=C_FG)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.X, padx=4, pady=2)

        self.fig.tight_layout(pad=1.0)

    def update(self, history: list):
        if not history:
            return

        n = len(history)
        # X axis: seconds ago (most recent = 0, oldest = negative)
        t0 = history[-1].get("timestamp_ms", 0)
        xs = [(h.get("timestamp_ms", 0) - t0) / 1000.0 for h in history]

        for key, _, _ in self.traces:
            ys = []
            for h in history:
                v = h.get(key, 0)
                try:
                    ys.append(float(v) if v != "" else 0.0)
                except (ValueError, TypeError):
                    ys.append(0.0)
            self._lines[key].set_data(xs, ys)

        if xs:
            self.ax.set_xlim(min(xs), max(xs) + 0.1)

        if self.y_range:
            self.ax.set_ylim(*self.y_range)
        else:
            self.ax.relim()
            self.ax.autoscale_view(scaley=True, scalex=False)

        self.canvas.draw_idle()


# =========================================================================
# MAIN UI
# =========================================================================

class TestStandUI:
    def __init__(self, logger: Logger):
        self.logger = logger
        self.root = tk.Tk()
        self.root.title("Test Stand Operator Console")
        self.root.configure(bg=C_BG)
        self.root.geometry("1100x900")

        self._build_ui()
        self._schedule_refresh()

    def _build_ui(self):
        # ---- Top row: state badge + big numbers ----
        top = tk.Frame(self.root, bg=C_BG)
        top.pack(fill=tk.X, padx=6, pady=4)

        self._state_label = tk.Label(top, text="---", font=("Consolas", 20, "bold"),
                                     fg=C_FG, bg=C_ACCENT, width=18,
                                     relief=tk.RIDGE, borderwidth=2)
        self._state_label.pack(side=tk.LEFT, padx=4, ipady=8)

        self._nums = {}
        num_defs = [
            ("p_line_psi",      "PRESSURE",    "psi",   100, 120, ".0f"),
            ("rpm_air",         "RPM AIR",     "rpm",   None, None, ".0f"),
            ("rpm_water",       "RPM WATER",   "rpm",   None, None, ".0f"),
            ("rpm_alternator",  "RPM ALT",     "rpm",   None, None, ".0f"),
            ("v_alt_V",         "V_ALT",       "V",     None, None, ".1f"),
            ("power_alt_W",     "P_ALT",       "W",     None, None, ".1f"),
            ("v_bus_V",         "V_BUS",       "V",     11.5, None, ".1f"),
            ("water_level_cm",  "LEVEL",       "cm",    None, None, ".1f"),
        ]
        for key, label, unit, warn, crit, fmt in num_defs:
            bn = BigNumber(top, label, unit, warn_hi=warn, crit_hi=crit, fmt=fmt)
            bn.pack(side=tk.LEFT, padx=3)
            self._nums[key] = bn

        # ---- Fault bar ----
        self._fault_bar = tk.Label(self.root, text="", font=("Consolas", 10, "bold"),
                                   fg=C_RED, bg=C_DARK, anchor="w")
        self._fault_bar.pack(fill=tk.X, padx=6, pady=1)

        # ---- Calibration panel (shown only during CAL states) ----
        self._cal_frame = tk.Frame(self.root, bg="#2a1a00", relief=tk.RIDGE, bd=2)
        # Packed dynamically — starts hidden

        self._cal_title = tk.Label(self._cal_frame, text="",
                                   font=("Consolas", 14, "bold"),
                                   fg=C_ORANGE, bg="#2a1a00")
        self._cal_title.pack(pady=(8, 2))

        self._cal_msg = tk.Label(self._cal_frame, text="",
                                 font=("Consolas", 10), fg=C_FG, bg="#2a1a00",
                                 wraplength=800, justify=tk.LEFT)
        self._cal_msg.pack(pady=4, padx=12)

        self._cal_reading = tk.Label(self._cal_frame, text="",
                                     font=("Consolas", 11), fg=C_CYAN, bg="#2a1a00")
        self._cal_reading.pack(pady=2)

        cal_btn_frame = tk.Frame(self._cal_frame, bg="#2a1a00")
        cal_btn_frame.pack(pady=8)

        self._btn_cal_zero = tk.Button(
            cal_btn_frame, text="SENSOR IS DRY — ZERO NOW",
            font=("Consolas", 10, "bold"), fg=C_DARK, bg=C_ORANGE,
            width=30, command=lambda: self.logger.commander.cal_zero())
        self._btn_cal_zero.pack(side=tk.LEFT, padx=6)

        self._btn_cal_done = tk.Button(
            cal_btn_frame, text="SENSOR IS BACK IN WATER — GO",
            font=("Consolas", 10, "bold"), fg=C_DARK, bg=C_GREEN,
            width=30, command=lambda: self.logger.commander.cal_done())
        self._btn_cal_done.pack(side=tk.LEFT, padx=6)

        self._btn_cal_skip = tk.Button(
            cal_btn_frame, text="SKIP CALIBRATION",
            font=("Consolas", 10, "bold"), fg=C_DARK, bg=C_FG,
            width=20, command=lambda: self.logger.commander.cal_skip())
        self._btn_cal_skip.pack(side=tk.LEFT, padx=6)

        self._cal_visible = False

        # ---- Strip charts ----
        chart_frame = tk.Frame(self.root, bg=C_BG)
        chart_frame.pack(fill=tk.BOTH, expand=True, padx=2)

        self._chart_pressure = StripChart(
            chart_frame, "Pressure", "psi",
            [("p_line_psi",      "Line (psi)",  C_BLUE),
             ("dp_dt_psi_per_s", "dP/dt",       C_YELLOW)],
        )

        self._chart_rpm = StripChart(
            chart_frame, "RPM", "rpm",
            [("rpm_air",        "Air",       C_CYAN),
             ("rpm_water",      "Water",     C_BLUE),
             ("rpm_alternator", "Alternator", C_PURPLE)],
        )

        self._chart_power = StripChart(
            chart_frame, "Power", "W",
            [("power_alt_W",        "Alt Out",    C_GREEN),
             ("power_supply_W",     "Supply",     C_YELLOW),
             ("power_compressor_W", "Compressor", C_ORANGE),
             ("power_pump_W",       "Pump",       C_CYAN)],
        )

        # ---- Bottom section: IO + events + buttons ----
        bottom = tk.Frame(self.root, bg=C_BG)
        bottom.pack(fill=tk.X, padx=6, pady=4)

        # IO indicators
        io_frame = tk.LabelFrame(bottom, text="I/O Status",
                                 font=("Consolas", 9),
                                 fg=C_FG, bg=C_BG, bd=1)
        io_frame.pack(side=tk.LEFT, padx=4, anchor="n")
        self._io_panel = IOStatusPanel(io_frame)
        self._io_panel.pack(padx=4, pady=4)

        # Event log
        log_frame = tk.LabelFrame(bottom, text="Event Log",
                                  font=("Consolas", 9),
                                  fg=C_FG, bg=C_BG, bd=1)
        log_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)
        self._event_log = scrolledtext.ScrolledText(
            log_frame, height=6, font=("Consolas", 8),
            bg=C_DARK, fg=C_FG, insertbackground=C_FG,
            state=tk.DISABLED, wrap=tk.WORD)
        self._event_log.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # Command buttons
        btn_frame = tk.LabelFrame(self.root, text="Commands",
                                  font=("Consolas", 9),
                                  fg=C_FG, bg=C_BG, bd=1)
        btn_frame.pack(fill=tk.X, padx=6, pady=4)

        buttons = [
            ("ARM",             lambda: self._cmd("ARM"),             C_GREEN),
            ("CHARGE AIR",      lambda: self._cmd_charge(True, False), C_YELLOW),
            ("CHARGE WATER",    lambda: self._cmd_charge(False, True), C_YELLOW),
            ("CHARGE BOTH",     lambda: self._cmd_charge(True, True),  C_YELLOW),
            ("DISCHARGE AIR",   lambda: self._cmd("DISCHARGE_AIR"),  C_BLUE),
            ("DISCHARGE WATER", lambda: self._cmd("DISCHARGE_WATER"),C_CYAN),
            ("DISCHARGE BOTH",  lambda: self._cmd("DISCHARGE_BOTH"), C_PURPLE),
            ("STOP",            lambda: self._cmd("STOP"),           C_ORANGE),
            ("DISARM",          lambda: self._cmd("DISARM"),         C_FG),
            ("RESET",           lambda: self.logger.commander.request_reset(), C_RED),
        ]

        for text, cmd, color in buttons:
            b = tk.Button(btn_frame, text=text, command=cmd,
                          font=("Consolas", 9, "bold"),
                          fg=C_DARK, bg=color, activebackground=color,
                          width=14, relief=tk.RAISED, bd=2)
            b.pack(side=tk.LEFT, padx=3, pady=4)

        # ---- Status bar ----
        self._status = tk.Label(self.root, text="Connecting...",
                                font=("Consolas", 8), fg="#888", bg=C_DARK,
                                anchor="w")
        self._status.pack(fill=tk.X, padx=0, pady=0)

    def _cmd(self, mode: str):
        self.logger.commander.request_mode(mode)

    def _cmd_charge(self, air: bool, water: bool):
        self.logger.commander.set_charge_config(air, water)

    def _log_event(self, text: str):
        self._event_log.config(state=tk.NORMAL)
        self._event_log.insert(tk.END, text + "\n")
        self._event_log.see(tk.END)
        self._event_log.config(state=tk.DISABLED)

    def _refresh(self):
        row = self.logger.get_latest()
        if row:
            # State badge
            sname = row.get("state_name", "---")
            scolor = STATE_COLORS.get(sname, C_FG)
            self._state_label.config(text=sname, fg=scolor)

            # Big numbers
            for key, bn in self._nums.items():
                bn.set(row.get(key))

            # Fault bar
            faults = row.get("fault_list", "NONE")
            if faults != "NONE":
                self._fault_bar.config(
                    text=f"⚠ FAULTS: {faults}",
                    fg=C_RED, bg="#3a0a0a")
            else:
                self._fault_bar.config(text="", bg=C_DARK)

            # IO panel
            self._io_panel.update(row)

            # Calibration panel visibility and content
            sname = row.get("state_name", "")
            if sname in CAL_STATES:
                if not self._cal_visible:
                    self._cal_frame.pack(fill=tk.X, padx=6, pady=4,
                                         after=self._fault_bar)
                    self._cal_visible = True

                cal_v = row.get("cal_valid", 0)
                p_atm = row.get("cal_p_atm_mbar", "?")

                if sname == "CAL_PROMPT":
                    self._cal_title.config(text="SENSOR CALIBRATION REQUIRED")
                    self._cal_msg.config(
                        text="Remove the MS5837-02BA from water and let it read "
                             "ambient air pressure. Press 'SENSOR IS DRY' when ready.\n"
                             "Or press 'SKIP' to use stored/default offset "
                             "(water level readings may be inaccurate).")
                    self._cal_reading.config(
                        text=f"Current reading: {row.get('p_hydro_mbar', '?')} mbar  |  "
                             f"Stored offset: {p_atm} mbar  |  "
                             f"Cal valid: {'YES' if cal_v else 'NO'}")
                    self._btn_cal_zero.config(state=tk.NORMAL)
                    self._btn_cal_done.config(state=tk.DISABLED)
                    self._btn_cal_skip.config(state=tk.NORMAL)

                elif sname == "CAL_ZEROING":
                    self._cal_title.config(text="ZEROING — HOLD STILL")
                    self._cal_msg.config(
                        text="Averaging atmospheric pressure. Keep sensor in air, "
                             "do not move it. This takes about 5 seconds.")
                    self._cal_reading.config(
                        text=f"Reading: {row.get('p_hydro_mbar', '?')} mbar")
                    self._btn_cal_zero.config(state=tk.DISABLED)
                    self._btn_cal_done.config(state=tk.DISABLED)
                    self._btn_cal_skip.config(state=tk.DISABLED)

                elif sname == "CAL_REPLACE":
                    self._cal_title.config(text="CALIBRATION COMPLETE — REPLACE SENSOR")
                    self._cal_msg.config(
                        text=f"Atmospheric offset stored: {p_atm} mbar. "
                             "Place sensor back in water, then press 'GO'.")
                    self._cal_reading.config(text="")
                    self._btn_cal_zero.config(state=tk.DISABLED)
                    self._btn_cal_done.config(state=tk.NORMAL)
                    self._btn_cal_skip.config(state=tk.NORMAL)

            elif self._cal_visible:
                self._cal_frame.pack_forget()
                self._cal_visible = False

            # Status bar
            s = self.logger.stats
            self._status.config(
                text=f"  Session: {self.logger.session_id[:8]}  |  "
                     f"Rows: {s['rows']}  |  Errors: {s['errors']}  |  "
                     f"Driving: {row.get('driving_source', '?')}  |  "
                     f"Energy gen: {row.get('energy_alt_J', 0):.1f} J")

        # Events
        ev = self.logger.get_latest_event()
        if ev:
            self._log_event(
                f"[{ev.get('timestamp_iso', '?')[11:23]}] "
                f"{ev.get('event_type', '?')}: {ev.get('detail', '')}")

        # Charts (update at reduced rate to save CPU: every 5th call = 2 Hz)
        if not hasattr(self, '_chart_tick'):
            self._chart_tick = 0
        self._chart_tick += 1
        if self._chart_tick % 5 == 0:
            history = self.logger.get_history()
            self._chart_pressure.update(history)
            self._chart_rpm.update(history)
            self._chart_power.update(history)

    def _schedule_refresh(self):
        self._refresh()
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.logger.stop()


# =========================================================================
# ENTRY POINT
# =========================================================================

def main():
    parser = argparse.ArgumentParser(description="Test Stand Operator UI")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--test-id", default="")
    args = parser.parse_args()

    logger = Logger(port=args.port, baud=args.baud, test_id=args.test_id)
    logger.start()

    ui = TestStandUI(logger)
    ui.run()


if __name__ == "__main__":
    main()
