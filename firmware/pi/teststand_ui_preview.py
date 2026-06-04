#!/usr/bin/env python3
"""
teststand_ui_preview.py
=======================
Local preview for teststand_ui.py — no Mega, no Pi, no serial port.

Stubs pyserial (so importing teststand_logger doesn't require it being
installed), then runs teststand_ui.TestStandUI against a FakeLogger that
synthesises plausible telemetry matching the post-pivot protocol. The
commands the UI sends print to the console instead of going out a serial port.

Scenarios drive the operator-panel switch states over time, so the UI auto-
switches between WATER_ONLY / AIR_ONLY / BOTH / IDLE views via
ui_mode_from_row().

Usage:
    python teststand_ui_preview.py
    python teststand_ui_preview.py --scenario water    # water-only run
    python teststand_ui_preview.py --scenario air      # air-only run
    python teststand_ui_preview.py --scenario both     # combined discharge w/ dispatch
    python teststand_ui_preview.py --scenario fault    # surface a fault during discharge
    python teststand_ui_preview.py --scenario idle     # no MODE selected
"""

import argparse
import math
import random
import sys
import time
import types
import uuid
from collections import deque

# ---- Stub pyserial BEFORE teststand_logger gets imported ----
if "serial" not in sys.modules:
    fake_serial = types.ModuleType("serial")

    class _StubSerial:
        def __init__(self, *a, **kw): pass
        def close(self): pass
        def readline(self): return b""
        def write(self, *a, **kw): return 0
        def reset_input_buffer(self): pass
        @property
        def in_waiting(self): return 0

    fake_serial.Serial = _StubSerial
    fake_serial.SerialException = Exception
    sys.modules["serial"] = fake_serial

# Safe to import the UI now (which pulls in teststand_logger)
from teststand_ui import TestStandUI
from teststand_logger import STATE_NAMES


# =========================================================================
# FAKE COMMANDER — prints what the real one would have sent
# =========================================================================

class FakeCommander:
    def __init__(self, on_cmd):
        self._on = on_cmd

    def set_v_setpoint(self, v):     self._on("set_v_setpoint", v)
    def set_manual(self, on):        self._on("set_manual", on)
    def set_water_manual(self, pct): self._on("set_water_manual", pct)
    def set_air_manual(self, hz):    self._on("set_air_manual", hz)


# =========================================================================
# FAKE LOGGER — same shape TestStandUI expects
# =========================================================================
# Drives all the panel switches (arm, mode_air, mode_water, charge, discharge)
# over time as a sequence. The state name comes from synthesising the same
# logic the Mega's next_state() would apply.

class FakeLogger:
    """Synthesises telemetry rows matching the post-pivot Mega protocol."""

    def __init__(self, scenario: str = "water"):
        self.session_id   = uuid.uuid4().hex
        self.stats        = {"rows": 0, "errors": 0}
        self.commander    = FakeCommander(self._on_cmd)

        self._scenario    = scenario
        self._t0_ms       = int(time.time() * 1000)
        self._history     = deque(maxlen=600)
        self._pending_event = None
        self._prev_state  = None

        # Cumulative energy trackers
        self._energy_alt_J  = 0.0
        self._energy_pump_J = 0.0
        self._energy_comp_J = 0.0

        # Scenarios are sequences of (t_seconds, panel_state_dict).
        # panel_state_dict has: arm, mode_air, mode_water, charge, discharge
        # (estop is always 0 except in dedicated estop scenarios).
        self._schedule = self._build_schedule(scenario)

    # ---- Methods TestStandUI actually calls ----

    def get_latest(self):
        row = self._make_row()
        self._history.append(row)
        self.stats["rows"] += 1
        return row

    def get_latest_event(self):
        ev = self._pending_event
        self._pending_event = None
        return ev

    def get_history(self):
        return list(self._history)

    def stop(self):
        print("[FakeLogger] stop()")

    # ---- Internals ----

    def _on_cmd(self, name, arg):
        print(f"[CMD] {name}({arg!r})")

    def _build_schedule(self, scenario: str):
        """Each entry: (t_s, {arm, mode_air, mode_water, charge, discharge, fault?})."""
        if scenario == "water":
            return [
                (0,  dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),
                (3,  dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=0)),
                (6,  dict(arm=1, mode_air=0, mode_water=1, charge=1, discharge=0)),
                (25, dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=0)),
                (28, dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=1)),
                (80, dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=0)),
            ]
        if scenario == "air":
            return [
                (0,  dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),
                (3,  dict(arm=1, mode_air=1, mode_water=0, charge=0, discharge=0)),
                (6,  dict(arm=1, mode_air=1, mode_water=0, charge=1, discharge=0)),
                (25, dict(arm=1, mode_air=1, mode_water=0, charge=0, discharge=0)),
                (28, dict(arm=1, mode_air=1, mode_water=0, charge=0, discharge=1)),
                (80, dict(arm=1, mode_air=1, mode_water=0, charge=0, discharge=0)),
            ]
        if scenario == "both":
            return [
                (0,  dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),
                (3,  dict(arm=1, mode_air=1, mode_water=1, charge=0, discharge=0)),
                (6,  dict(arm=1, mode_air=1, mode_water=1, charge=1, discharge=0)),
                (30, dict(arm=1, mode_air=1, mode_water=1, charge=0, discharge=0)),
                (32, dict(arm=1, mode_air=1, mode_water=1, charge=0, discharge=1)),
                (110,dict(arm=1, mode_air=1, mode_water=1, charge=0, discharge=0)),
            ]
        if scenario == "fault":
            return [
                (0,  dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),
                (3,  dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=0)),
                (6,  dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=1)),
                (20, dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=1, fault=True)),
                (50, dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),  # cycle ARM to clear
                (52, dict(arm=1, mode_air=0, mode_water=1, charge=0, discharge=0)),
            ]
        # "idle"
        return [
            (0, dict(arm=0, mode_air=0, mode_water=0, charge=0, discharge=0)),
            (5, dict(arm=1, mode_air=0, mode_water=0, charge=0, discharge=0)),
        ]

    def _panel_state(self, t_rel: float) -> dict:
        cur = self._schedule[0][1]
        for ts, panel in self._schedule:
            if t_rel >= ts:
                cur = panel
        return cur

    def _derive_state_name(self, panel: dict, fault_active: bool) -> str:
        """Mirror the Mega's next_state() switch-logic for synthetic data."""
        if fault_active:
            return "FAULT"
        if not panel.get("arm", 0):
            return "OFF"
        air = panel.get("mode_air", 0)
        wat = panel.get("mode_water", 0)
        ch  = panel.get("charge", 0)
        di  = panel.get("discharge", 0)
        if ch and di:
            return "ARMED_IDLE"
        if ch:
            if air and wat: return "CHARGE_BOTH"
            if air:         return "CHARGE_AIR"
            if wat:         return "CHARGE_WATER"
            return "ARMED_IDLE"
        if di:
            if air and wat: return "DISCHARGE_BOTH"
            if air:         return "DISCHARGE_AIR"
            if wat:         return "DISCHARGE_WATER"
            return "ARMED_IDLE"
        return "ARMED_IDLE"

    def _make_row(self) -> dict:
        now_ms = int(time.time() * 1000)
        total_window = self._schedule[-1][0] + 10
        t_rel = ((now_ms - self._t0_ms) / 1000.0) % total_window
        panel = self._panel_state(t_rel)
        fault_active = bool(panel.get("fault", False))

        state = self._derive_state_name(panel, fault_active)

        # State change → emit event
        if state != self._prev_state:
            self._pending_event = {
                "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S.000",
                                               time.gmtime()),
                "timestamp_ms":  now_ms,
                "event_type":    "STATE_CHANGE",
                "detail":        f"{self._prev_state} -> {state}",
            }
            self._prev_state = state

        # Synthetic physics
        wob = math.sin(t_rel * 0.4)
        n   = lambda amp: random.uniform(-amp, amp)

        # Defaults
        v_alt = i_load = power_alt = 0.0
        p1 = p2 = 0.0
        rpm_alt = 0.0
        rpm_water = 0.0
        i_pump = i_comp = 0.0
        wvp = wfb = 0.0
        aph = 0.0
        depth_cm = 30.0 + n(0.1)
        air_arm = water_open = pump_on = comp_on = pulsing = 0
        fault_flags = 0
        fault_list = "NONE"

        if state == "FAULT":
            fault_flags = (1 << 3)   # F_LEAK_AIR for example
            fault_list = "LEAK_AIR"

        elif state == "CHARGE_AIR":
            ramp = min(1.0, (t_rel - 6) / 15.0)
            p1 = 5.0 + 75.0 * ramp + n(0.5)
            comp_on = 1
            i_comp = 5.5 * ramp + n(0.2)

        elif state == "CHARGE_WATER":
            pump_on = 1
            i_pump = 4.0 + n(0.2)
            depth_cm = min(45.0, 30.0 + (t_rel - 6) * 0.4)

        elif state == "CHARGE_BOTH":
            ramp = min(1.0, (t_rel - 6) / 15.0)
            p1 = 5.0 + 75.0 * ramp + n(0.5)
            pump_on = comp_on = 1
            i_comp = 5.5 * ramp + n(0.2)
            i_pump = 4.0 + n(0.2)
            depth_cm = min(45.0, 30.0 + (t_rel - 6) * 0.4)

        elif state == "DISCHARGE_WATER":
            # Closed-loop on V_alt -> 8 V setpoint. Show some ramp + settle.
            elapsed = max(0.0, t_rel - 28)
            v_alt = min(8.0, elapsed * 0.6) + 0.3 * wob + n(0.1)
            i_load = v_alt * 0.4
            rpm_alt = v_alt / 0.0162   # ~ V / (Kv * 1V/min^-1)... rough
            wvp = min(85.0, elapsed * 7.0) + n(1.0)
            wfb = max(0.0, wvp - 8.0 + 6.0 * wob)
            water_open = 1
            depth_cm = max(5.0, 45.0 - elapsed * 0.4)

        elif state == "DISCHARGE_AIR":
            elapsed = max(0.0, t_rel - 28)
            v_alt = min(4.0, elapsed * 0.4) + 0.2 * wob + n(0.08)
            i_load = v_alt * 0.3
            rpm_alt = v_alt / 0.0162
            aph = min(5.0, elapsed * 0.5) + n(0.1)
            air_arm = 1
            pulsing = 1 if aph > 0.3 else 0
            p1 = 70.0 - elapsed * 0.5 + n(0.5)
            p2 = max(0.0, 25.0 + 10.0 * wob)

        elif state == "DISCHARGE_BOTH":
            elapsed = max(0.0, t_rel - 32)
            v_alt = min(8.0, elapsed * 0.5) + 0.3 * wob + n(0.1)
            i_load = v_alt * 0.5
            rpm_alt = v_alt / 0.0162
            wvp = min(95.0, elapsed * 5.0) + n(1.0)
            wfb = max(0.0, wvp - 8.0 + 6.0 * wob)
            water_open = 1
            # Dispatch logic: air engages once water_fb > 80% for a bit
            if wfb > 80.0:
                air_arm = 1
                aph = min(4.0, (elapsed - 15) * 0.3) if elapsed > 15 else 0.0
                pulsing = 1 if aph > 0.3 else 0
                p1 = 60.0 - max(0.0, (elapsed - 15)) * 0.4 + n(0.5)
                p2 = max(0.0, 22.0 + 8.0 * wob)
            depth_cm = max(5.0, 45.0 - elapsed * 0.5)

        # Power calcs (mains assumed 120 V)
        mains_v = 120.0
        power_alt  = v_alt * i_load
        power_pump = i_pump * mains_v
        power_comp = i_comp * mains_v

        # Energy integrals (10 Hz tick ~ 0.1 s)
        self._energy_alt_J  += power_alt  * 0.1
        self._energy_pump_J += power_pump * 0.1
        self._energy_comp_J += power_comp * 0.1
        energy_in = self._energy_pump_J + self._energy_comp_J
        eta = (100.0 * self._energy_alt_J / energy_in) if energy_in > 0.1 else 0.0

        # dP1/dt (rough)
        dp1_dt = 0.0
        if self._history:
            prev = self._history[-1]
            dt = (now_ms - prev["timestamp_ms"]) / 1000.0
            if dt > 0:
                dp1_dt = (p1 - prev.get("p1_vessel_psi", p1)) / dt

        return {
            "timestamp_ms":  now_ms,
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S.000", time.gmtime()),

            "state":           list(STATE_NAMES.values()).index(state) if state in STATE_NAMES.values() else 0,
            "state_name":      state,
            "fault_flags_hex": f"0x{fault_flags:04X}",
            "fault_list":      fault_list,

            "estop":      0,
            "arm":        panel.get("arm", 0),
            "mode_air":   panel.get("mode_air", 0),
            "mode_water": panel.get("mode_water", 0),
            "charge":     panel.get("charge", 0),
            "discharge":  panel.get("discharge", 0),

            "v_alt_V":     round(v_alt, 2),
            "i_load_A":    round(i_load, 3),
            "power_alt_W": round(power_alt, 2),

            "i_pump_A":     round(i_pump, 3),
            "i_comp_A":     round(i_comp, 3),
            "power_pump_W": round(power_pump, 2),
            "power_comp_W": round(power_comp, 2),

            "p1_vessel_psi": round(p1, 2),
            "p2_motor_psi":  round(p2, 2),

            "p_hydro_mbar": round(1013.0 + depth_cm * 0.98, 2),
            "temp_water_C": round(22.0 + n(0.2), 1),
            "depth_cm":     round(depth_cm, 2),

            "rpm_alt":   round(rpm_alt, 1),
            "rpm_water": round(rpm_water, 1),

            "water_valve_cmd_pct": round(wvp, 1),
            "water_valve_fb_pct":  round(wfb, 1),
            "air_pulse_hz":        round(aph, 2),

            "compressor_on":    comp_on,
            "pump_on":          pump_on,
            "air_arm_open":     air_arm,
            "water_valve_open": water_open,
            "air_pulsing":      pulsing,

            "v_setpoint_V": 8.0 if state != "DISCHARGE_AIR" else 4.0,

            "energy_alt_J":  round(self._energy_alt_J, 2),
            "energy_pump_J": round(self._energy_pump_J, 2),
            "energy_comp_J": round(self._energy_comp_J, 2),
            "energy_input_J": round(energy_in, 2),
            "eta_round_trip_pct": round(eta, 2),

            "dp1_dt_psi_per_s": round(dp1_dt, 2),
        }


# =========================================================================
# ENTRY POINT
# =========================================================================

def main():
    p = argparse.ArgumentParser(description="Preview teststand UI locally")
    p.add_argument("--scenario",
                   choices=["water", "air", "both", "fault", "idle"],
                   default="water",
                   help="Telemetry pattern to simulate")
    p.add_argument("--fullscreen", action="store_true")
    args = p.parse_args()

    print(f"[Preview] scenario={args.scenario} — commands print to console")
    logger = FakeLogger(scenario=args.scenario)
    ui = TestStandUI(logger, fullscreen=args.fullscreen)
    ui.run()


if __name__ == "__main__":
    main()
