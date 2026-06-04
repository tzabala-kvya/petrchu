#!/usr/bin/env python3
"""
teststand_logger.py
===================
Serial reader + CSV logger for the PetrChu test stand.

Reads line-delimited JSON telemetry from the Arduino Mega over USB-serial,
decodes, derives quantities (power, energy, efficiency), and writes a CSV.
Exposes a thread-safe history queue for the real-time UI.

Updated for the post-pivot Mega protocol:
  - Switch-driven state machine (12 states, no CAL_* — cal is a separate sketch)
  - New telemetry fields: V_alt + I_load + P1/P2 + MS5837 + alt RPM via encoder
    + water valve cmd/FB + air pulse Hz + 5 switches + V setpoint
  - New Pi-side commands: V setpoint override, manual override of water/air
    actuators (diagnostic only; normal control is from physical switches)

Usage (standalone):
    python teststand_logger.py --port /dev/ttyACM0 --baud 115200

Usage (imported by UI):
    from teststand_logger import Logger
    log = Logger(port="/dev/ttyACM0")
    log.start()
    row = log.get_latest()
"""

import argparse
import csv
import json
import os
import sys
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone

import serial

# =========================================================================
# CONSTANTS
# =========================================================================

BAUD_DEFAULT = 115200
LOG_DIR      = "logs"

# Assumed AC mains voltage for converting SCT-013 RMS amps -> input power.
# Override via Logger(mains_v=...) if the bench uses a different supply.
MAINS_V_DEFAULT = 120.0

# State name lookup (matches firmware enum in TestStandFirmware.ino)
STATE_NAMES = {
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

CHARGE_STATES    = {"CHARGE_AIR", "CHARGE_WATER", "CHARGE_BOTH"}
DISCHARGE_STATES = {"DISCHARGE_AIR", "DISCHARGE_WATER", "DISCHARGE_BOTH"}
ENERGIZED_STATES = CHARGE_STATES | DISCHARGE_STATES

# Fault bit decode (matches FaultBits enum in firmware)
FAULT_BITS = {
    0:  "ESTOP",
    1:  "OVERVOLTAGE",
    2:  "OVERSPEED",
    3:  "LEAK_AIR",
    4:  "P2_STUCK",
    5:  "V_DROOP_SEVERE",
    6:  "I2C_TIMEOUT",
    7:  "SENSOR_RANGE",
    8:  "PI_LOST",
    9:  "WATCHDOG",
    10: "SELFTEST",
    11: "OVERPRESSURE",
    12: "CAL_FAIL",
    13: "PUMP_CAVITATION",
}

# Output bitmap decode (matches commit_outputs() bits)
OUTPUT_BITS = {
    0: "compressor_on",
    1: "pump_on",
    2: "air_arm_open",
    3: "water_valve_open",   # >0.5% commanded
    4: "air_pulsing",        # >0.01 Hz commanded
}

# CSV column order
CSV_COLUMNS = [
    # Identifiers
    "timestamp_iso", "timestamp_ms", "session_id", "test_id",
    # State + faults
    "state", "state_name", "fault_flags_hex", "fault_list",
    # Switch positions (the operator-panel state)
    "estop", "arm", "mode_air", "mode_water", "charge", "discharge",
    # Output side (the controlled variable)
    "v_alt_V", "i_load_A", "power_alt_W",
    # Input side (efficiency calc)
    "i_pump_A", "i_comp_A", "power_pump_W", "power_comp_W",
    # Pressure
    "p1_vessel_psi", "p2_motor_psi",
    # Water tank (hydrostatic)
    "p_hydro_mbar", "temp_water_C", "depth_cm",
    # RPM
    "rpm_alt", "rpm_water",
    # Actuator commands + feedback
    "water_valve_cmd_pct", "water_valve_fb_pct", "air_pulse_hz",
    # Output bitmap decoded
    "compressor_on", "pump_on", "air_arm_open", "water_valve_open", "air_pulsing",
    # V setpoint (Pi-overridable, telemetered back so UI can show current)
    "v_setpoint_V",
    # Energy integrals (cumulative across session, J)
    "energy_alt_J", "energy_pump_J", "energy_comp_J", "energy_input_J",
    # Efficiency (running)
    "eta_round_trip_pct",
    # Derived
    "dp1_dt_psi_per_s",
]

EVENT_COLUMNS = [
    "timestamp_iso", "timestamp_ms", "session_id",
    "event_type", "detail",
]


# =========================================================================
# TELEMETRY DECODER
# =========================================================================

class TelemetryDecoder:
    """Decodes raw Mega JSON into a row dict with derived quantities."""

    def __init__(self, session_id: str, test_id: str = "",
                 mains_v: float = MAINS_V_DEFAULT):
        self.session_id = session_id
        self.test_id    = test_id
        self.mains_v    = mains_v

        # State for integrals / derivatives
        self._prev_t_ms        = None
        self._prev_p1_psi      = None
        self._energy_alt_J     = 0.0
        self._energy_pump_J    = 0.0
        self._energy_comp_J    = 0.0
        self._prev_state       = None
        self._prev_fault       = 0

    def decode(self, raw: dict):
        """
        raw: dict parsed from one Mega telemetry line, e.g.:
          {"t":12345,"st":1,"f":0,"o":0,
           "va":850,"il":120,"p1":7800,"p2":2500,
           "ph":10131,"tw":221,"d":150,
           "ra":7430,"rw":0,
           "ip":0,"ic":0,
           "wvp":420,"wfb":410,"aph":150,
           "es":0,"arm":1,"ma":0,"mw":1,"ch":0,"di":1,
           "vsp":800}
        Returns (row_dict, event_dict_or_None).
        """
        now_iso = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        t_ms    = raw.get("t", 0)

        # --- Unscale per telemetry contract ---
        v_alt   = raw.get("va", 0)  / 100.0
        i_load  = raw.get("il", 0)  / 1000.0
        p1_psi  = raw.get("p1", 0)  / 100.0
        p2_psi  = raw.get("p2", 0)  / 100.0
        ph_mbar = raw.get("ph", 0)  / 10.0
        tw_c    = raw.get("tw", 0)  / 10.0
        depth_cm = raw.get("d", 0)  / 10.0
        rpm_alt   = raw.get("ra", 0) / 10.0
        rpm_water = raw.get("rw", 0) / 10.0
        i_pump  = raw.get("ip", 0)  / 1000.0
        i_comp  = raw.get("ic", 0)  / 1000.0
        wvp     = raw.get("wvp", 0) / 10.0   # commanded valve %
        wfb_raw = raw.get("wfb", -10)
        wfb     = wfb_raw / 10.0 if wfb_raw >= 0 else None   # -1 = absent
        aph     = raw.get("aph", 0) / 100.0
        v_setpoint = raw.get("vsp", 800) / 100.0

        estop   = raw.get("es", 0)
        arm     = raw.get("arm", 0)
        m_air   = raw.get("ma", 0)
        m_water = raw.get("mw", 0)
        charge_sw    = raw.get("ch", 0)
        discharge_sw = raw.get("di", 0)

        fault_flags  = raw.get("f", 0)
        output_bits  = raw.get("o", 0)
        state_code   = raw.get("st", 0)

        state_name = STATE_NAMES.get(state_code, f"UNKNOWN_{state_code}")

        fault_list = "|".join(
            name for bit, name in FAULT_BITS.items()
            if fault_flags & (1 << bit)
        ) or "NONE"

        def has_bit(n): return 1 if (output_bits & (1 << n)) else 0

        # --- Power ---
        # Alt output is DC after rectifier; pump/comp are AC RMS.
        power_alt  = v_alt  * i_load
        power_pump = i_pump * self.mains_v
        power_comp = i_comp * self.mains_v

        # --- Energy integrals (trapezoidal, only between samples) ---
        if self._prev_t_ms is not None and t_ms > self._prev_t_ms:
            dt_s = (t_ms - self._prev_t_ms) / 1000.0
            self._energy_alt_J  += power_alt  * dt_s
            self._energy_pump_J += power_pump * dt_s
            self._energy_comp_J += power_comp * dt_s
        energy_input = self._energy_pump_J + self._energy_comp_J
        eta = (100.0 * self._energy_alt_J / energy_input) if energy_input > 0.1 else 0.0

        # --- dP1/dt ---
        dp1_dt = 0.0
        if (self._prev_p1_psi is not None and
                self._prev_t_ms is not None and t_ms > self._prev_t_ms):
            dt_s = (t_ms - self._prev_t_ms) / 1000.0
            if dt_s > 0:
                dp1_dt = (p1_psi - self._prev_p1_psi) / dt_s

        # --- Event detection ---
        event = None
        if self._prev_state is not None and state_code != self._prev_state:
            event = {
                "timestamp_iso": now_iso,
                "timestamp_ms":  t_ms,
                "session_id":    self.session_id,
                "event_type":    "STATE_CHANGE",
                "detail": f"{STATE_NAMES.get(self._prev_state, '?')} -> {state_name}",
            }
        if fault_flags != 0 and fault_flags != self._prev_fault:
            event = {
                "timestamp_iso": now_iso,
                "timestamp_ms":  t_ms,
                "session_id":    self.session_id,
                "event_type":    "FAULT",
                "detail":        fault_list,
            }

        # --- Update derivative state for next call ---
        self._prev_t_ms   = t_ms
        self._prev_p1_psi = p1_psi
        self._prev_state  = state_code
        self._prev_fault  = fault_flags

        row = {
            "timestamp_iso":   now_iso,
            "timestamp_ms":    t_ms,
            "session_id":      self.session_id,
            "test_id":         self.test_id,

            "state":           state_code,
            "state_name":      state_name,
            "fault_flags_hex": f"0x{fault_flags:04X}",
            "fault_list":      fault_list,

            "estop":      estop,
            "arm":        arm,
            "mode_air":   m_air,
            "mode_water": m_water,
            "charge":     charge_sw,
            "discharge":  discharge_sw,

            "v_alt_V":     round(v_alt, 2),
            "i_load_A":    round(i_load, 3),
            "power_alt_W": round(power_alt, 2),

            "i_pump_A":     round(i_pump, 3),
            "i_comp_A":     round(i_comp, 3),
            "power_pump_W": round(power_pump, 2),
            "power_comp_W": round(power_comp, 2),

            "p1_vessel_psi": round(p1_psi, 2),
            "p2_motor_psi":  round(p2_psi, 2),

            "p_hydro_mbar": round(ph_mbar, 2),
            "temp_water_C": round(tw_c, 1),
            "depth_cm":     round(depth_cm, 2),

            "rpm_alt":   round(rpm_alt, 1),
            "rpm_water": round(rpm_water, 1),

            "water_valve_cmd_pct": round(wvp, 1),
            "water_valve_fb_pct":  round(wfb, 1) if wfb is not None else "",
            "air_pulse_hz":        round(aph, 2),

            "compressor_on":     has_bit(0),
            "pump_on":           has_bit(1),
            "air_arm_open":      has_bit(2),
            "water_valve_open":  has_bit(3),
            "air_pulsing":       has_bit(4),

            "v_setpoint_V": round(v_setpoint, 2),

            "energy_alt_J":   round(self._energy_alt_J, 2),
            "energy_pump_J":  round(self._energy_pump_J, 2),
            "energy_comp_J":  round(self._energy_comp_J, 2),
            "energy_input_J": round(energy_input, 2),
            "eta_round_trip_pct": round(eta, 2),

            "dp1_dt_psi_per_s": round(dp1_dt, 2),
        }
        return row, event


# =========================================================================
# CSV WRITER
# =========================================================================

class CSVWriter:
    """Writes telemetry CSV and events CSV side-by-side."""

    def __init__(self, session_id: str, log_dir: str = LOG_DIR):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.telem_path = os.path.join(log_dir, f"telem_{ts}_{session_id[:8]}.csv")
        self.event_path = os.path.join(log_dir, f"events_{ts}_{session_id[:8]}.csv")

        self._telem_file = open(self.telem_path, "w", newline="")
        self._event_file = open(self.event_path, "w", newline="")
        self._telem_writer = csv.DictWriter(self._telem_file, fieldnames=CSV_COLUMNS)
        self._event_writer = csv.DictWriter(self._event_file, fieldnames=EVENT_COLUMNS)
        self._telem_writer.writeheader()
        self._event_writer.writeheader()
        self._telem_file.flush()
        self._event_file.flush()
        self._rows_written = 0

    def write_telemetry(self, row: dict):
        self._telem_writer.writerow(row)
        self._rows_written += 1
        if self._rows_written % 10 == 0:
            self._telem_file.flush()

    def write_event(self, event: dict):
        self._event_writer.writerow(event)
        self._event_file.flush()

    def close(self):
        self._telem_file.close()
        self._event_file.close()

    @property
    def rows_written(self):
        return self._rows_written


# =========================================================================
# COMMAND SENDER
# =========================================================================

class CommandSender:
    """
    Sends JSON commands to the Mega. The Pi has limited control authority:
    state transitions are driven by physical switches, not commands. The
    only things the Pi can do are:

      - Override V setpoint (vsp, centivolts; persists in Mega EEPROM)
      - Toggle manual mode (man) and directly drive water valve % / air
        pulse Hz (wvp, aph). Manual mode is for diagnostics only.
    """

    def __init__(self, ser: serial.Serial):
        self._ser  = ser
        self._seq  = 0
        self._lock = threading.Lock()

    def _send(self, obj: dict):
        with self._lock:
            self._seq += 1
            obj["seq"] = self._seq
            line = json.dumps(obj, separators=(",", ":")) + "\n"
            try:
                self._ser.write(line.encode("ascii"))
            except serial.SerialException:
                pass

    def set_v_setpoint(self, volts: float):
        """Persist in Mega EEPROM. Clamped 1.00 - 25.00 V."""
        cv = int(round(max(1.0, min(25.0, volts)) * 100))
        self._send({"vsp": cv})

    def set_manual(self, on: bool):
        self._send({"man": 1 if on else 0})

    def set_water_manual(self, pct: float):
        pct = max(0.0, min(100.0, pct))
        self._send({"wvp": int(round(pct))})

    def set_air_manual(self, hz: float):
        # Firmware divides by 100 (cHz to Hz). Clamp to [0, 5 Hz].
        hz = max(0.0, min(5.0, hz))
        self._send({"aph": int(round(hz * 100))})


# =========================================================================
# LOGGER (top-level thread-safe interface)
# =========================================================================

class Logger:
    """
    Opens serial, runs a reader thread, exposes the latest decoded row + a
    rolling history deque + a CommandSender.

    Usage:
        log = Logger(port="/dev/ttyACM0")
        log.start()
        row = log.get_latest()        # newest decoded row, or None
        hist = log.get_history()      # last N rows
        log.commander.set_v_setpoint(8.0)
        log.stop()
    """

    def __init__(self, port: str, baud: int = BAUD_DEFAULT,
                 test_id: str = "", history_len: int = 600,
                 mains_v: float = MAINS_V_DEFAULT):
        self.session_id = str(uuid.uuid4())
        self.test_id    = test_id
        self.port       = port
        self.baud       = baud

        self._decoder = TelemetryDecoder(self.session_id, test_id, mains_v=mains_v)
        self._csv     = CSVWriter(self.session_id)

        self._history      = deque(maxlen=history_len)  # 60 s @ 10 Hz default
        self._latest       = None
        self._latest_event = None
        self._lock         = threading.Lock()
        self._running      = False
        self._thread       = None

        # Serial open. If this fails, Logger init fails fast.
        self._ser = serial.Serial(port, baud, timeout=0.5)
        self.commander = CommandSender(self._ser)

        self._error_count = 0
        self._good_count  = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="serial_reader")
        self._thread.start()
        print(f"[Logger] started on {self.port} @ {self.baud}")
        print(f"[Logger] session: {self.session_id}")
        print(f"[Logger] CSV:     {self._csv.telem_path}")
        print(f"[Logger] Events:  {self._csv.event_path}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._csv.close()
        try:
            self._ser.close()
        except Exception:
            pass
        print(f"[Logger] stopped. {self._csv.rows_written} rows written.")

    def get_latest(self):
        with self._lock:
            return self._latest

    def get_latest_event(self):
        with self._lock:
            e = self._latest_event
            self._latest_event = None
            return e

    def get_history(self):
        with self._lock:
            return list(self._history)

    @property
    def stats(self):
        return {
            "good":   self._good_count,
            "errors": self._error_count,
            "rows":   self._csv.rows_written,
        }

    # --- internal ---

    def _reader_loop(self):
        while self._running:
            try:
                line = self._ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                # Mega prefixes diagnostic / boot messages with '#'; skip them.
                if line.startswith("#"):
                    continue
                raw = json.loads(line)
                row, event = self._decoder.decode(raw)
                with self._lock:
                    self._latest = row
                    self._history.append(row)
                    if event:
                        self._latest_event = event
                self._csv.write_telemetry(row)
                if event:
                    self._csv.write_event(event)
                self._good_count += 1
            except json.JSONDecodeError:
                self._error_count += 1
            except serial.SerialException as e:
                print(f"[Logger] serial error: {e}", file=sys.stderr)
                self._error_count += 1
                time.sleep(0.5)
            except Exception as e:
                print(f"[Logger] unexpected error: {e}", file=sys.stderr)
                self._error_count += 1


# =========================================================================
# UI MODE HELPER
# =========================================================================

def ui_mode_from_row(row: dict) -> str:
    """
    Decide which UI layout to show, based on the operator-panel MODE switches.
    Returns one of: "WATER_ONLY", "AIR_ONLY", "BOTH", "IDLE".
    """
    if not row:
        return "IDLE"
    a = bool(row.get("mode_air", 0))
    w = bool(row.get("mode_water", 0))
    if a and w: return "BOTH"
    if w:       return "WATER_ONLY"
    if a:       return "AIR_ONLY"
    return "IDLE"


# =========================================================================
# STANDALONE ENTRY POINT
# =========================================================================

def main():
    p = argparse.ArgumentParser(description="PetrChu Test Stand Logger")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    p.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    p.add_argument("--test-id", default="")
    p.add_argument("--mains-v", type=float, default=MAINS_V_DEFAULT,
                   help="AC mains voltage assumed for pump/comp power calc")
    args = p.parse_args()

    log = Logger(port=args.port, baud=args.baud,
                 test_id=args.test_id, mains_v=args.mains_v)
    log.start()
    try:
        while True:
            time.sleep(2.0)
            s = log.stats
            row = log.get_latest()
            if row:
                print(f"[{row['state_name']:16s}] "
                      f"V={row['v_alt_V']:5.2f}V "
                      f"I={row['i_load_A']:5.2f}A "
                      f"Palt={row['power_alt_W']:6.2f}W "
                      f"P1={row['p1_vessel_psi']:5.1f}psi "
                      f"P2={row['p2_motor_psi']:5.1f}psi "
                      f"η={row['eta_round_trip_pct']:5.1f}%  "
                      f"faults={row['fault_list']}  "
                      f"[{s['rows']} rows, {s['errors']} errs]")
            else:
                print(f"[Logger] waiting for data... ({s['errors']} parse errors)")
    except KeyboardInterrupt:
        print("\n[Logger] shutting down...")
        log.stop()


if __name__ == "__main__":
    main()
