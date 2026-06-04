#!/usr/bin/env python3
"""
teststand_ui.py
===============
Real-time operator UI for the PetrChu test stand.

Three layouts auto-selected from the MODE switches on the operator panel:
  - MODE_WATER on, MODE_AIR off  -> WATER_ONLY view
  - MODE_AIR on,   MODE_WATER off -> AIR_ONLY view
  - both on                       -> BOTH view (water + air + dispatch)
  - both off                      -> IDLE view (instructions)

State transitions are driven by physical switches on the panel — the Pi can
NOT change state directly. The only commands the Pi sends are:
  - V setpoint override (persists in Mega EEPROM)
  - Manual override (diagnostic, lets the Pi drive actuators directly)

Tkinter + matplotlib strip charts.  Talks to the Mega through
teststand_logger.Logger.

Usage:
    python teststand_ui.py --port /dev/ttyACM0
    python teststand_ui.py --port COM3        # Windows dev
"""

import argparse
import sys
import tkinter as tk
from tkinter import ttk, scrolledtext
import tkinter.font as tkfont

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from teststand_logger import (
    Logger, STATE_NAMES, CHARGE_STATES, DISCHARGE_STATES,
    ui_mode_from_row,
)

# =========================================================================
# CONSTANTS
# =========================================================================

REFRESH_MS    = 100   # UI tick (10 Hz matches telemetry)
CHART_REDRAW_EVERY = 5   # redraw charts every 5 ticks = 2 Hz (saves CPU)

# Monospace family. Resolved at TestStandUI init to the platform's actual
# fixed-width font (Consolas on Windows; DejaVu Sans Mono on Pi/Linux).
# Hardcoding "Consolas" broke labels and layout on Raspberry Pi OS because
# the font is absent there and Tk fell back to a proportional font.
MONO_FAMILY = "Consolas"

# Dark theme
C_BG     = "#1a1a2e"
C_FG     = "#e0e0e0"
C_ACCENT = "#0f3460"
C_DARK   = "#0d0d1a"
C_GREEN  = "#2ecc71"
C_YELLOW = "#f39c12"
C_RED    = "#e74c3c"
C_BLUE   = "#3498db"
C_CYAN   = "#1abc9c"
C_PURPLE = "#9b59b6"
C_ORANGE = "#e67e22"
C_GRAY   = "#666"

STATE_COLORS = {
    "BOOT_SELFTEST":   C_YELLOW,
    "OFF":             C_FG,
    "ARMED_IDLE":      C_GREEN,
    "CHARGE_AIR":      C_YELLOW,
    "CHARGE_WATER":    C_YELLOW,
    "CHARGE_BOTH":     C_YELLOW,
    "DISCHARGE_AIR":   C_BLUE,
    "DISCHARGE_WATER": C_CYAN,
    "DISCHARGE_BOTH":  C_PURPLE,
    "SHUTDOWN":        C_YELLOW,
    "FAULT":           C_RED,
    "ESTOP":           C_RED,
}


# =========================================================================
# WIDGETS
# =========================================================================

class BigNumber(tk.Frame):
    """Large numeric readout with label, unit, optional warning thresholds."""

    def __init__(self, parent, label, unit="", fmt=".1f",
                 warn_hi=None, crit_hi=None, warn_lo=None, crit_lo=None):
        super().__init__(parent, bg=C_BG)
        self.fmt = fmt
        self.warn_hi = warn_hi
        self.crit_hi = crit_hi
        self.warn_lo = warn_lo
        self.crit_lo = crit_lo

        tk.Label(self, text=label, font=(MONO_FAMILY, 9), fg=C_GRAY,
                 bg=C_BG).pack(anchor="w", padx=4)
        self._value = tk.Label(self, text="---", font=(MONO_FAMILY, 22, "bold"),
                               fg=C_FG, bg=C_BG)
        self._value.pack(anchor="w", padx=4)
        tk.Label(self, text=unit, font=(MONO_FAMILY, 9), fg=C_GRAY,
                 bg=C_BG).pack(anchor="w", padx=4)

    def set(self, value):
        if value is None or value == "":
            self._value.config(text="---", fg=C_FG)
            return
        try:
            v = float(value)
            txt = f"{v:{self.fmt}}"
        except (ValueError, TypeError):
            self._value.config(text=str(value), fg=C_FG)
            return
        color = C_FG
        if   self.crit_hi is not None and v >= self.crit_hi: color = C_RED
        elif self.crit_lo is not None and v <= self.crit_lo: color = C_RED
        elif self.warn_hi is not None and v >= self.warn_hi: color = C_YELLOW
        elif self.warn_lo is not None and v <= self.warn_lo: color = C_YELLOW
        self._value.config(text=txt, fg=color)


class SwitchPanel(tk.Frame):
    """LED-style indicators for the 6 operator-panel switches + E-stop."""

    SWITCHES = [
        ("E-STOP",     "estop",      C_RED,   C_GREEN),   # active = bad
        ("ARM",        "arm",        C_GREEN, C_GRAY),
        ("MODE_AIR",   "mode_air",   C_BLUE,  C_GRAY),
        ("MODE_WATER", "mode_water", C_CYAN,  C_GRAY),
        ("CHARGE",     "charge",     C_YELLOW, C_GRAY),
        ("DISCHARGE",  "discharge",  C_PURPLE, C_GRAY),
    ]

    def __init__(self, parent):
        super().__init__(parent, bg=C_BG)
        self._dots = {}
        for i, (lbl, key, on_color, off_color) in enumerate(self.SWITCHES):
            tk.Label(self, text=lbl, font=(MONO_FAMILY, 8), fg=C_GRAY, bg=C_BG,
                     width=12).grid(row=0, column=i, padx=2)
            d = tk.Label(self, text="●", font=(MONO_FAMILY, 18), fg=off_color,
                         bg=C_BG)
            d.grid(row=1, column=i, padx=2)
            self._dots[key] = (d, on_color, off_color)

    def update(self, row):
        for key, (dot, on_c, off_c) in self._dots.items():
            on = int(row.get(key, 0)) != 0
            dot.config(fg=on_c if on else off_c)


class OutputPanel(tk.Frame):
    """LED-style indicators for actuator output states (from telemetry bitmap)."""

    OUTPUTS = [
        ("COMPRESSOR",  "compressor_on",    C_YELLOW),
        ("PUMP",        "pump_on",          C_YELLOW),
        ("AIR ARM",     "air_arm_open",     C_BLUE),
        ("WATER VALVE", "water_valve_open", C_CYAN),
        ("AIR PULSE",   "air_pulsing",      C_BLUE),
    ]

    def __init__(self, parent):
        super().__init__(parent, bg=C_BG)
        self._dots = {}
        for i, (lbl, key, on_c) in enumerate(self.OUTPUTS):
            tk.Label(self, text=lbl, font=(MONO_FAMILY, 8), fg=C_GRAY, bg=C_BG,
                     width=12).grid(row=0, column=i, padx=2)
            d = tk.Label(self, text="●", font=(MONO_FAMILY, 18), fg=C_GRAY,
                         bg=C_BG)
            d.grid(row=1, column=i, padx=2)
            self._dots[key] = (d, on_c)

    def update(self, row):
        for key, (dot, on_c) in self._dots.items():
            on = int(row.get(key, 0)) != 0
            dot.config(fg=on_c if on else C_GRAY)


class StripChart:
    """Matplotlib strip chart embedded in Tkinter."""

    def __init__(self, parent, title, y_label, traces, y_range=None, height=1.5):
        """traces: list of (key, label, color)"""
        self.traces  = traces
        self.y_range = y_range
        self.fig = Figure(figsize=(8, height), dpi=100, facecolor=C_DARK)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_facecolor(C_DARK)
        self.ax.set_title(title, color=C_FG, fontsize=9, loc="left")
        self.ax.set_ylabel(y_label, color=C_FG, fontsize=8)
        self.ax.tick_params(colors="#888", labelsize=7)
        for spine in self.ax.spines.values():
            spine.set_color("#333")
        self.ax.grid(True, color="#333", linewidth=0.5, alpha=0.5)

        self._lines = {}
        for key, lbl, color in traces:
            line, = self.ax.plot([], [], color=color, linewidth=1.2, label=lbl)
            self._lines[key] = line
        self.ax.legend(loc="upper left", fontsize=7, framealpha=0.3,
                       facecolor=C_DARK, edgecolor="#444", labelcolor=C_FG)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.X, padx=4, pady=2)
        self.fig.tight_layout(pad=0.8)

    def update(self, history):
        if not history:
            return
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
    def __init__(self, logger: Logger, fullscreen: bool = False):
        self.logger = logger
        self.root = tk.Tk()
        self.root.title("PetrChu Test Stand")
        self.root.configure(bg=C_BG)
        # Conservative default for Pi displays. Fullscreen mode (--fullscreen)
        # overrides this for the kiosk demo.
        self.root.geometry("1024x720")
        if fullscreen:
            self.root.attributes("-fullscreen", True)
            self.root.bind("<Escape>", lambda e: self.root.destroy())

        # Resolve a monospace font that exists on this platform. TkFixedFont
        # maps to Consolas on Windows and DejaVu Sans Mono on Pi/Linux.
        # Hardcoding "Consolas" caused fallback to a proportional font on the
        # Pi, which threw off label widths in the operator/output panels.
        global MONO_FAMILY
        try:
            MONO_FAMILY = tkfont.nametofont("TkFixedFont").actual("family")
        except Exception:
            pass

        self._mode  = None   # current UI mode string
        self._tick  = 0
        self._manual_on = False

        self._build_ui()

        # Force initial geometry computation so the bottom widgets actually
        # claim their space on first show. Without this, the dynamic mode
        # panel sometimes grabs all vertical space at first draw and the
        # bottom widgets are 0-height until the user resizes the window.
        self.root.update_idletasks()

        self._schedule_refresh()

    # ---------------------------------------------------------------------
    # UI construction
    # ---------------------------------------------------------------------

    def _build_ui(self):
        # IMPORTANT pack order: top widgets first (side=TOP), then bottom
        # widgets (side=BOTTOM) so they reserve their space BEFORE the
        # dynamic mode panel gets packed with expand=True later. If we pack
        # the mode panel first, it grabs all remaining space and the bottom
        # widgets render at 0 height until the window is resized.

        # --- Top widgets (state badge, fault bar) ---
        self._build_top_bar()

        self._fault_bar = tk.Label(self.root, text="",
                                   font=(MONO_FAMILY, 11, "bold"),
                                   fg=C_RED, bg=C_DARK, anchor="w")
        self._fault_bar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=1)

        # --- Bottom widgets (status footer, commands, status panels) ---
        # Pack BEFORE the mode panel ever shows. Order matters: the FIRST
        # side=BOTTOM widget ends up at the very bottom; subsequent ones
        # stack upward from there.
        self._status = tk.Label(self.root, text="Connecting...",
                                font=(MONO_FAMILY, 8), fg=C_GRAY, bg=C_DARK,
                                anchor="w")
        self._status.pack(side=tk.BOTTOM, fill=tk.X)

        self._build_command_bar()     # packs side=BOTTOM internally
        self._build_status_panels()   # packs side=BOTTOM internally

        # --- Dynamic mode-specific panels (created here, packed by _set_mode) ---
        self._build_idle_panel()
        self._build_water_panel()
        self._build_air_panel()

    def _build_top_bar(self):
        top = tk.Frame(self.root, bg=C_BG)
        top.pack(fill=tk.X, padx=6, pady=4)

        # State badge
        self._state_label = tk.Label(top, text="---",
                                     font=(MONO_FAMILY, 20, "bold"),
                                     fg=C_FG, bg=C_ACCENT, width=18,
                                     relief=tk.RIDGE, borderwidth=2)
        self._state_label.pack(side=tk.LEFT, padx=4, ipady=8)

        # Always-visible numbers: V_alt, target, power, efficiency
        self._top_nums = {}
        for key, lbl, unit, fmt, kwargs in [
            ("v_alt_V",          "V_ALT",   "V",   ".2f",
                {"warn_lo": 0.5}),
            ("v_setpoint_V",     "TARGET",  "V",   ".2f", {}),
            ("power_alt_W",      "P_OUT",   "W",   ".1f", {}),
            ("i_load_A",         "I_LOAD",  "A",   ".2f", {}),
            ("eta_round_trip_pct", "η",     "%",   ".1f", {}),
        ]:
            bn = BigNumber(top, lbl, unit, fmt=fmt, **kwargs)
            bn.pack(side=tk.LEFT, padx=3)
            self._top_nums[key] = bn

        # UI mode label (right-aligned)
        self._mode_label = tk.Label(top, text="MODE: ---",
                                    font=(MONO_FAMILY, 14, "bold"),
                                    fg=C_GRAY, bg=C_BG)
        self._mode_label.pack(side=tk.RIGHT, padx=12)

    def _build_idle_panel(self):
        self._idle_frame = tk.Frame(self.root, bg=C_ACCENT, height=200)
        # packed dynamically
        tk.Label(self._idle_frame, text="NO MODE SELECTED",
                 font=(MONO_FAMILY, 20, "bold"),
                 fg=C_YELLOW, bg=C_ACCENT).pack(pady=(20, 4))
        tk.Label(self._idle_frame,
                 text="Flip MODE_AIR or MODE_WATER on the operator panel\n"
                      "to begin. Combine both for DISCHARGE_BOTH.",
                 font=(MONO_FAMILY, 11), fg=C_FG, bg=C_ACCENT,
                 justify=tk.CENTER).pack(pady=4)
        self._idle_visible = False

    def _build_water_panel(self):
        """Water-mode panel: valve cmd/fb, depth, pump, water-side charts."""
        self._water_frame = tk.Frame(self.root, bg=C_BG)

        # Big numbers row
        row = tk.Frame(self._water_frame, bg=C_BG)
        row.pack(fill=tk.X, pady=4)
        self._water_nums = {}
        for key, lbl, unit, fmt, kwargs in [
            ("water_valve_cmd_pct", "VALVE CMD",  "%",     ".1f", {}),
            ("water_valve_fb_pct",  "VALVE FB",   "%",     ".1f", {}),
            ("depth_cm",            "TANK DEPTH", "cm",    ".1f",
                {"warn_lo": 10.0, "crit_lo": 5.0}),
            ("i_pump_A",            "I_PUMP",     "A",     ".2f", {}),
            ("power_pump_W",        "P_PUMP",     "W",     ".1f", {}),
            ("rpm_alt",             "ALT RPM",    "rpm",   ".0f", {}),
        ]:
            bn = BigNumber(row, lbl, unit, fmt=fmt, **kwargs)
            bn.pack(side=tk.LEFT, padx=3)
            self._water_nums[key] = bn

        # Strip charts
        self._chart_water_v = StripChart(
            self._water_frame, "V_alt (water)", "V",
            [("v_alt_V",      "V_alt",  C_GREEN),
             ("v_setpoint_V", "target", C_YELLOW)])
        self._chart_water_valve = StripChart(
            self._water_frame, "Water valve position", "%",
            [("water_valve_cmd_pct", "commanded", C_CYAN),
             ("water_valve_fb_pct",  "feedback",  C_BLUE)],
            y_range=(0, 105))
        self._chart_water_power = StripChart(
            self._water_frame, "Power flow", "W",
            [("power_alt_W",  "alt out", C_GREEN),
             ("power_pump_W", "pump in", C_ORANGE)])

    def _build_air_panel(self):
        """Air-mode panel: pulse Hz, P1/P2, compressor, air-side charts."""
        self._air_frame = tk.Frame(self.root, bg=C_BG)

        row = tk.Frame(self._air_frame, bg=C_BG)
        row.pack(fill=tk.X, pady=4)
        self._air_nums = {}
        for key, lbl, unit, fmt, kwargs in [
            ("air_pulse_hz",   "PULSE Hz",  "Hz",    ".2f", {}),
            ("p1_vessel_psi",  "P1 VESSEL", "psi",   ".1f",
                {"warn_hi": 70.0, "crit_hi": 85.0}),
            ("p2_motor_psi",   "P2 MOTOR",  "psi",   ".1f",
                {"warn_hi": 50.0, "crit_hi": 56.5}),
            ("i_comp_A",       "I_COMP",    "A",     ".2f", {}),
            ("power_comp_W",   "P_COMP",    "W",     ".1f", {}),
            ("rpm_alt",        "ALT RPM",   "rpm",   ".0f", {}),
        ]:
            bn = BigNumber(row, lbl, unit, fmt=fmt, **kwargs)
            bn.pack(side=tk.LEFT, padx=3)
            self._air_nums[key] = bn

        # Dispatch banner (only shown in BOTH mode)
        self._dispatch_label = tk.Label(self._air_frame, text="",
                                        font=(MONO_FAMILY, 11, "bold"),
                                        fg=C_FG, bg=C_ACCENT, anchor="w")
        self._dispatch_label.pack(fill=tk.X, padx=4, pady=2)

        # Strip charts
        self._chart_air_v = StripChart(
            self._air_frame, "V_alt (air)", "V",
            [("v_alt_V",      "V_alt",  C_GREEN),
             ("v_setpoint_V", "target", C_YELLOW)])
        self._chart_air_pulse = StripChart(
            self._air_frame, "Air pulse rate", "Hz",
            [("air_pulse_hz", "commanded", C_BLUE)],
            y_range=(0, 5.5))
        self._chart_air_pressure = StripChart(
            self._air_frame, "Air-chain pressure", "psi",
            [("p1_vessel_psi", "P1 vessel",      C_PURPLE),
             ("p2_motor_psi",  "P2 motor inlet", C_ORANGE)])

    def _build_status_panels(self):
        bottom = tk.Frame(self.root, bg=C_BG)
        bottom.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

        sw_frame = tk.LabelFrame(bottom, text="Operator panel",
                                 font=(MONO_FAMILY, 9), fg=C_FG, bg=C_BG, bd=1)
        sw_frame.pack(side=tk.LEFT, padx=4, anchor="n")
        self._switch_panel = SwitchPanel(sw_frame)
        self._switch_panel.pack(padx=4, pady=4)

        out_frame = tk.LabelFrame(bottom, text="Actuator outputs",
                                  font=(MONO_FAMILY, 9), fg=C_FG, bg=C_BG, bd=1)
        out_frame.pack(side=tk.LEFT, padx=4, anchor="n")
        self._output_panel = OutputPanel(out_frame)
        self._output_panel.pack(padx=4, pady=4)

        log_frame = tk.LabelFrame(bottom, text="Event log",
                                  font=(MONO_FAMILY, 9), fg=C_FG, bg=C_BG, bd=1)
        log_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)
        self._event_log = scrolledtext.ScrolledText(
            log_frame, height=5, font=(MONO_FAMILY, 8),
            bg=C_DARK, fg=C_FG, insertbackground=C_FG,
            state=tk.DISABLED, wrap=tk.WORD)
        self._event_log.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

    def _build_command_bar(self):
        bar = tk.LabelFrame(self.root, text="Commands (Pi -> Mega)",
                            font=(MONO_FAMILY, 9), fg=C_FG, bg=C_BG, bd=1)
        bar.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

        # V setpoint slider + apply button
        vsp_frame = tk.Frame(bar, bg=C_BG)
        vsp_frame.pack(side=tk.LEFT, padx=8, pady=4)
        tk.Label(vsp_frame, text="V setpoint", font=(MONO_FAMILY, 9),
                 fg=C_FG, bg=C_BG).pack(anchor="w")
        self._vsp_var = tk.DoubleVar(value=8.0)
        self._vsp_scale = tk.Scale(vsp_frame, from_=1.0, to=15.0,
                                   resolution=0.25, orient=tk.HORIZONTAL,
                                   variable=self._vsp_var, length=180,
                                   bg=C_BG, fg=C_FG, troughcolor=C_DARK,
                                   highlightthickness=0)
        self._vsp_scale.pack()
        tk.Button(vsp_frame, text="Apply (writes EEPROM)",
                  font=(MONO_FAMILY, 9, "bold"),
                  fg=C_DARK, bg=C_YELLOW,
                  command=self._apply_vsp,
                  width=22).pack(pady=2)

        # Manual override
        man_frame = tk.Frame(bar, bg=C_BG)
        man_frame.pack(side=tk.LEFT, padx=20, pady=4, fill=tk.Y)
        self._manual_var = tk.BooleanVar(value=False)
        tk.Checkbutton(man_frame, text="Manual override (diag only)",
                       variable=self._manual_var,
                       command=self._toggle_manual,
                       font=(MONO_FAMILY, 9, "bold"),
                       fg=C_RED, bg=C_BG,
                       selectcolor=C_DARK,
                       activebackground=C_BG,
                       activeforeground=C_RED).pack(anchor="w")
        tk.Label(man_frame,
                 text="When ON, PIDs are off in discharge states and the\n"
                      "sliders below drive the actuators directly.",
                 font=(MONO_FAMILY, 8), fg=C_GRAY, bg=C_BG,
                 justify=tk.LEFT).pack(anchor="w")

        # Manual sliders
        sl_frame = tk.Frame(bar, bg=C_BG)
        sl_frame.pack(side=tk.LEFT, padx=12, pady=4)

        tk.Label(sl_frame, text="Water valve %", font=(MONO_FAMILY, 9),
                 fg=C_FG, bg=C_BG).grid(row=0, column=0, sticky="w")
        self._water_man_var = tk.DoubleVar(value=0.0)
        self._water_slider = tk.Scale(sl_frame, from_=0, to=100, resolution=1,
                                      orient=tk.HORIZONTAL,
                                      variable=self._water_man_var, length=160,
                                      bg=C_BG, fg=C_FG, troughcolor=C_DARK,
                                      highlightthickness=0,
                                      command=lambda v: self._on_water_slider())
        self._water_slider.grid(row=0, column=1, padx=4)

        tk.Label(sl_frame, text="Air pulse Hz", font=(MONO_FAMILY, 9),
                 fg=C_FG, bg=C_BG).grid(row=1, column=0, sticky="w")
        self._air_man_var = tk.DoubleVar(value=0.0)
        self._air_slider = tk.Scale(sl_frame, from_=0, to=5, resolution=0.1,
                                    orient=tk.HORIZONTAL,
                                    variable=self._air_man_var, length=160,
                                    bg=C_BG, fg=C_FG, troughcolor=C_DARK,
                                    highlightthickness=0,
                                    command=lambda v: self._on_air_slider())
        self._air_slider.grid(row=1, column=1, padx=4)

        self._toggle_manual()   # set initial enable state

    # ---------------------------------------------------------------------
    # Command callbacks
    # ---------------------------------------------------------------------

    def _apply_vsp(self):
        v = float(self._vsp_var.get())
        self.logger.commander.set_v_setpoint(v)
        self._log_event(f"V setpoint -> {v:.2f} V (written to EEPROM)")

    def _toggle_manual(self):
        on = bool(self._manual_var.get())
        self._manual_on = on
        self.logger.commander.set_manual(on)
        for s in (self._water_slider, self._air_slider):
            s.config(state=tk.NORMAL if on else tk.DISABLED)
        if not on:
            # Send 0 to make sure Mega doesn't keep last manual values
            self.logger.commander.set_water_manual(0.0)
            self.logger.commander.set_air_manual(0.0)
        self._log_event(f"Manual override {'ON' if on else 'OFF'}")

    def _on_water_slider(self):
        if self._manual_on:
            self.logger.commander.set_water_manual(float(self._water_man_var.get()))

    def _on_air_slider(self):
        if self._manual_on:
            self.logger.commander.set_air_manual(float(self._air_man_var.get()))

    # ---------------------------------------------------------------------
    # Refresh
    # ---------------------------------------------------------------------

    def _log_event(self, text: str):
        self._event_log.config(state=tk.NORMAL)
        self._event_log.insert(tk.END, text + "\n")
        self._event_log.see(tk.END)
        self._event_log.config(state=tk.DISABLED)

    def _set_mode(self, mode: str):
        if mode == self._mode:
            return
        self._mode = mode
        # Hide everything mode-specific
        self._idle_frame.pack_forget()
        self._water_frame.pack_forget()
        self._air_frame.pack_forget()

        # Show what this mode needs (above the bottom command/status section).
        # Pack after the fault bar.
        anchor = self._fault_bar
        if mode == "WATER_ONLY":
            self._water_frame.pack(fill=tk.BOTH, expand=True,
                                   padx=4, pady=2, after=anchor)
            self._dispatch_label.config(text="")
        elif mode == "AIR_ONLY":
            self._air_frame.pack(fill=tk.BOTH, expand=True,
                                 padx=4, pady=2, after=anchor)
            self._dispatch_label.config(text="")
        elif mode == "BOTH":
            self._water_frame.pack(fill=tk.BOTH, expand=True,
                                   padx=4, pady=2, after=anchor)
            self._air_frame.pack(fill=tk.BOTH, expand=True,
                                 padx=4, pady=2, after=self._water_frame)
        else:  # IDLE
            self._idle_frame.pack(fill=tk.X, padx=4, pady=2, after=anchor)

        self._mode_label.config(text=f"MODE: {mode}")

    def _refresh(self):
        row = self.logger.get_latest()

        if row:
            sname = row.get("state_name", "---")
            self._state_label.config(text=sname,
                                     fg=STATE_COLORS.get(sname, C_FG))

            for key, bn in self._top_nums.items():
                bn.set(row.get(key))

            faults = row.get("fault_list", "NONE")
            if faults != "NONE":
                self._fault_bar.config(text=f"  FAULT: {faults}",
                                       fg=C_RED, bg="#3a0a0a")
            else:
                self._fault_bar.config(text="", bg=C_DARK)

            # Mode switching
            mode = ui_mode_from_row(row)
            self._set_mode(mode)

            # Per-mode big numbers
            if mode in ("WATER_ONLY", "BOTH"):
                for key, bn in self._water_nums.items():
                    bn.set(row.get(key))
            if mode in ("AIR_ONLY", "BOTH"):
                for key, bn in self._air_nums.items():
                    bn.set(row.get(key))

            # Dispatch banner (BOTH only)
            if mode == "BOTH":
                air_engaged = int(row.get("air_arm_open", 0)) != 0
                if air_engaged:
                    self._dispatch_label.config(
                        text="  DISPATCH: air ENGAGED — both sources contributing",
                        fg=C_GREEN, bg=C_ACCENT)
                else:
                    self._dispatch_label.config(
                        text="  DISPATCH: water only (air on standby until water saturates)",
                        fg=C_YELLOW, bg=C_ACCENT)
            else:
                self._dispatch_label.config(text="", bg=C_ACCENT)

            self._switch_panel.update(row)
            self._output_panel.update(row)

            # Status footer
            s = self.logger.stats
            self._status.config(
                text=f"  Session: {self.logger.session_id[:8]}  |  "
                     f"Rows: {s['rows']}  |  Errors: {s['errors']}  |  "
                     f"η: {row.get('eta_round_trip_pct', 0):.1f}%  |  "
                     f"Energy out: {row.get('energy_alt_J', 0):.1f} J  |  "
                     f"Energy in: {row.get('energy_input_J', 0):.1f} J")
        else:
            self._status.config(text="  Waiting for telemetry...")

        # Event log drain
        ev = self.logger.get_latest_event()
        if ev:
            ts = ev.get("timestamp_iso", "?")[11:23]
            self._log_event(f"[{ts}] {ev.get('event_type','?')}: {ev.get('detail','')}")

        # Charts at reduced rate
        self._tick += 1
        if self._tick % CHART_REDRAW_EVERY == 0:
            history = self.logger.get_history()
            mode = self._mode
            if mode in ("WATER_ONLY", "BOTH"):
                self._chart_water_v.update(history)
                self._chart_water_valve.update(history)
                self._chart_water_power.update(history)
            if mode in ("AIR_ONLY", "BOTH"):
                self._chart_air_v.update(history)
                self._chart_air_pulse.update(history)
                self._chart_air_pressure.update(history)

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
    p = argparse.ArgumentParser(description="PetrChu Test Stand UI")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--test-id", default="")
    p.add_argument("--mains-v", type=float, default=120.0,
                   help="AC mains voltage assumed for pump/comp power calc")
    p.add_argument("--fullscreen", action="store_true",
                   help="Launch full-screen (for HDMI kiosk demo)")
    args = p.parse_args()

    try:
        logger = Logger(port=args.port, baud=args.baud,
                        test_id=args.test_id, mains_v=args.mains_v)
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    logger.start()
    ui = TestStandUI(logger, fullscreen=args.fullscreen)
    ui.run()


if __name__ == "__main__":
    main()
