#!/usr/bin/env python3
"""
teststand_ui_preview.py
=======================
Local preview for teststand_ui.py — no Mega, no Pi, no serial port.

Stubs out pyserial (so importing teststand_logger doesn't require it being
installed), then runs teststand_ui.TestStandUI against a FakeLogger that
generates plausible synthetic telemetry. Buttons print to the console
instead of issuing real commands.

Usage:
    python teststand_ui_preview.py
    python teststand_ui_preview.py --scenario cal     # walk through CAL flow
    python teststand_ui_preview.py --scenario fault   # surface a fault
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

# Now safe to import the UI (which pulls in teststand_logger)
from teststand_ui import TestStandUI
from teststand_logger import STATE_NAMES, CAL_STATES


# =========================================================================
# FAKE COMMANDER — prints what the real one would have sent
# =========================================================================

class FakeCommander:
    def __init__(self, on_cmd):
        self._on_cmd = on_cmd

    def request_mode(self, mode):       self._on_cmd("request_mode", mode)
    def set_charge_config(self, a, w):  self._on_cmd("set_charge_config", (a, w))
    def request_reset(self):            self._on_cmd("request_reset", None)
    def cal_zero(self):                 self._on_cmd("cal_zero", None)
    def cal_done(self):                 self._on_cmd("cal_done", None)
    def cal_skip(self):                 self._on_cmd("cal_skip", None)


# =========================================================================
# FAKE LOGGER — same shape TestStandUI expects
# =========================================================================

class FakeLogger:
    def __init__(self, scenario: str = "normal"):
        self.session_id = uuid.uuid4().hex
        self.stats = {"rows": 0, "errors": 0}
        self.commander = FakeCommander(self._on_cmd)

        self._scenario = scenario
        self._t0_ms = int(time.time() * 1000)
        self._history = deque(maxlen=600)
        self._pending_event = None
        self._prev_state = None

        # Scenario-driven state schedule: (t_seconds_relative, state_name)
        if scenario == "cal":
            self._schedule = [
                (0,  "BOOT_SELFTEST"),
                (3,  "CAL_PROMPT"),
                (10, "CAL_ZEROING"),
                (16, "CAL_REPLACE"),
                (24, "OFF"),
                (28, "ARM"),
                (34, "CHARGE"),
                (50, "READY"),
            ]
        elif scenario == "fault":
            self._schedule = [
                (0,  "BOOT_SELFTEST"),
                (3,  "OFF"),
                (6,  "ARM"),
                (10, "CHARGE"),
                (20, "FAULT"),
            ]
        else:  # "normal"
            self._schedule = [
                (0,  "BOOT_SELFTEST"),
                (3,  "OFF"),
                (6,  "ARM"),
                (10, "CHARGE"),
                (25, "READY"),
                (30, "DISCHARGE_AIR"),
                (45, "DISCHARGE_WATER"),
                (60, "DISCHARGE_BOTH"),
                (80, "OFF"),
            ]

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

    def _current_state(self) -> str:
        t = (time.time() * 1000 - self._t0_ms) / 1000.0
        cur = self._schedule[0][1]
        for ts, name in self._schedule:
            if t >= ts:
                cur = name
        # Loop the schedule for endless preview
        total = self._schedule[-1][0] + 20
        if t > total:
            self._t0_ms = int(time.time() * 1000)
        return cur

    def _make_row(self) -> dict:
        now_ms = int(time.time() * 1000)
        t_rel = (now_ms - self._t0_ms) / 1000.0
        state = self._current_state()

        # State change → emit event
        if state != self._prev_state:
            self._pending_event = {
                "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S.000",
                                               time.gmtime()),
                "timestamp_ms": now_ms,
                "event_type": "STATE_CHANGE",
                "detail": f"{self._prev_state} -> {state}",
            }
            self._prev_state = state

        # Synthetic physics — gentle sine + noise, scaled by state
        wob = math.sin(t_rel * 0.4)
        noise = lambda amp: random.uniform(-amp, amp)

        if state in ("OFF", "BOOT_SELFTEST") or state in CAL_STATES:
            p_psi, rpm_air, rpm_water, rpm_alt = 14.5, 0, 0, 0
            v_alt, i_alt = 0.0, 0.0
            i_comp, i_pump = 0.0, 0.0
        elif state == "ARM":
            p_psi = 14.5
            rpm_air = rpm_water = rpm_alt = 0
            v_alt = i_alt = i_comp = i_pump = 0.0
        elif state == "CHARGE":
            ramp = min(1.0, (t_rel % 30) / 12.0)
            p_psi = 14.5 + 75 * ramp + noise(1.0)
            rpm_air = rpm_water = rpm_alt = 0
            v_alt = i_alt = 0.0
            i_comp = 6.0 * ramp + noise(0.2)
            i_pump = 4.5 * ramp + noise(0.2)
        elif state == "READY":
            p_psi = 95 + noise(0.5)
            rpm_air = rpm_water = rpm_alt = 0
            v_alt = i_alt = i_comp = i_pump = 0.0
        elif state == "DISCHARGE_AIR":
            p_psi = max(14.5, 95 - (t_rel - 30) * 4)
            rpm_alt = 2200 + 200 * wob
            rpm_air = rpm_alt
            rpm_water = 0
            v_alt = 13.5 + 0.3 * wob
            i_alt = 8.0 + noise(0.5)
            i_comp = i_pump = 0.0
        elif state == "DISCHARGE_WATER":
            p_psi = max(14.5, 95 - (t_rel - 45) * 3)
            rpm_alt = 1800 + 150 * wob
            rpm_water = rpm_alt
            rpm_air = 0
            v_alt = 12.8 + 0.3 * wob
            i_alt = 6.5 + noise(0.4)
            i_comp = i_pump = 0.0
        elif state == "DISCHARGE_BOTH":
            p_psi = max(14.5, 95 - (t_rel - 60) * 5)
            rpm_alt = 2500 + 200 * wob
            rpm_air = rpm_water = rpm_alt
            v_alt = 14.0 + 0.3 * wob
            i_alt = 11.0 + noise(0.6)
            i_comp = i_pump = 0.0
        elif state == "FAULT":
            p_psi = 0.0
            rpm_air = rpm_water = rpm_alt = 0
            v_alt = i_alt = i_comp = i_pump = 0.0
        else:
            p_psi = 14.5
            rpm_air = rpm_water = rpm_alt = 0
            v_alt = i_alt = i_comp = i_pump = 0.0

        v_bus = 12.1 + noise(0.05)
        i_12v = (i_comp + i_pump) + 0.3
        power_supply_W = v_bus * i_12v
        power_alt_W = v_alt * i_alt
        power_comp_W = v_bus * i_comp
        power_pump_W = v_bus * i_pump

        # dP/dt from history
        dp_dt = 0.0
        if self._history:
            prev = self._history[-1]
            dt = (now_ms - prev["timestamp_ms"]) / 1000.0
            if dt > 0:
                dp_dt = (p_psi - prev["p_line_psi"]) / dt

        # Energy accumulator (cheap)
        energy_alt_J = 0.0
        if self._history:
            energy_alt_J = self._history[-1].get("energy_alt_J", 0.0) + power_alt_W * 0.1

        # Water level — slowly drains during DISCHARGE_WATER/BOTH
        if self._history:
            prev_level = self._history[-1]["water_level_cm"]
        else:
            prev_level = 35.0
        if state in ("DISCHARGE_WATER", "DISCHARGE_BOTH"):
            level = max(0.0, prev_level - 0.05)
        elif state == "CHARGE":
            level = min(48.0, prev_level + 0.03)
        else:
            level = prev_level
        p_hydro = 1013.2 + (level / 1.0197) * 10  # rough back-calc

        # IO bitmap
        contactor = 1 if state in ("ARM", "CHARGE", "READY",
                                   "DISCHARGE_AIR", "DISCHARGE_WATER",
                                   "DISCHARGE_BOTH") else 0
        ch_air = 1 if state == "CHARGE" else 0
        ch_water = 1 if state == "CHARGE" else 0
        dsc_air = 1 if state in ("DISCHARGE_AIR", "DISCHARGE_BOTH") else 0
        dsc_water = 1 if state in ("DISCHARGE_WATER", "DISCHARGE_BOTH") else 0
        sol_air = dsc_air
        sol_water = dsc_water

        fault_list = "OVERPRESSURE" if state == "FAULT" else "NONE"

        return {
            "timestamp_ms": now_ms,
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S.000",
                                           time.gmtime()),
            "state_name": state,
            "fault_list": fault_list,
            "p_line_psi": p_psi,
            "p_hydro_mbar": p_hydro,
            "water_level_cm": level,
            "tank_full": 1 if level >= 45 else 0,
            "tank_empty": 1 if level <= 3 else 0,
            "rpm_air": rpm_air,
            "rpm_water": rpm_water,
            "rpm_alternator": rpm_alt,
            "v_bus_V": v_bus,
            "v_alt_V": v_alt,
            "i_alt_A": i_alt,
            "power_supply_W": power_supply_W,
            "power_alt_W": power_alt_W,
            "power_compressor_W": power_comp_W,
            "power_pump_W": power_pump_W,
            "energy_alt_J": energy_alt_J,
            "dp_dt_psi_per_s": dp_dt,
            "driving_source": ("BOTH" if state == "DISCHARGE_BOTH"
                               else "AIR" if state == "DISCHARGE_AIR"
                               else "WATER" if state == "DISCHARGE_WATER"
                               else "NONE"),
            "estop": 0,
            "arm_switch": 1 if contactor else 0,
            "contactor_cmd": contactor,
            "sol_air_cmd": sol_air,
            "sol_water_cmd": sol_water,
            "valve_charge_air_cmd": ch_air,
            "valve_charge_water_cmd": ch_water,
            "valve_disch_air_cmd": dsc_air,
            "valve_disch_water_cmd": dsc_water,
            "vent_cmd": 0,
            "cal_valid": 1,
            "cal_p_atm_mbar": 1013.2,
        }


# =========================================================================
# ENTRY POINT
# =========================================================================

def main():
    parser = argparse.ArgumentParser(description="Preview teststand UI locally")
    parser.add_argument("--scenario",
                        choices=["normal", "cal", "fault"],
                        default="normal",
                        help="Telemetry pattern to simulate")
    args = parser.parse_args()

    print(f"[Preview] scenario={args.scenario} — buttons print to console")
    logger = FakeLogger(scenario=args.scenario)
    ui = TestStandUI(logger)
    ui.run()


if __name__ == "__main__":
    main()
