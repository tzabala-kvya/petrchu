#!/usr/bin/env python3
"""
teststand_logger.py
===================
Serial reader + CSV logger for the Water/Air Test Stand.

Reads line-delimited JSON from the Arduino Mega over USB-serial,
decodes telemetry, computes derived quantities, writes to CSV,
and exposes a thread-safe data queue for the real-time UI.

Usage (standalone):
    python teststand_logger.py --port /dev/ttyACM0 --baud 115200

Usage (imported by UI):
    from teststand_logger import Logger
    log = Logger(port="/dev/ttyACM0")
    log.start()
    ...
    row = log.get_latest()
"""

import argparse
import csv
import json
import math
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
HEARTBEAT_INTERVAL_S = 1.0

# --- MS5837-02BA water-level calibration ---
# Range: 300–1200 mbar.  Resolution: ~0.01 mbar.
# 1 mbar ≈ 1.0197 cm water at 20 °C (ρ ≈ 998.2 kg/m³).
WATER_DENSITY_KG_M3 = 998.2
GRAVITY_M_S2         = 9.81
MBAR_TO_PA           = 100.0    # 1 mbar = 100 Pa

# Tank thresholds (cm) — adjust to your physical tank dimensions
TANK_FULL_CM    = 45.0
TANK_EMPTY_CM   = 3.0
TANK_WARNING_CM = 8.0

# --- State name lookup (matches firmware enum) ---
STATE_NAMES = {
    0:  "BOOT_SELFTEST",
    1:  "CAL_PROMPT",
    2:  "CAL_ZEROING",
    3:  "CAL_REPLACE",
    4:  "OFF",
    5:  "ARM",
    6:  "CHARGE",
    7:  "READY",
    8:  "DISCHARGE_AIR",
    9:  "DISCHARGE_WATER",
    10: "DISCHARGE_BOTH",
    11: "MANUAL_DIAG",
    12: "SHUTDOWN",
    13: "FAULT",
    14: "ESTOP",
}

CAL_STATES = {"CAL_PROMPT", "CAL_ZEROING", "CAL_REPLACE"}

# --- Fault flag decode ---
FAULT_BITS = {
    0:  "ESTOP",
    1:  "OVERPRESSURE",
    2:  "UNDERVOLTAGE",
    3:  "OVERCURRENT",
    4:  "I2C_TIMEOUT",
    5:  "SENSOR_RANGE",
    6:  "PI_LOST",
    7:  "WATCHDOG",
    8:  "SELFTEST",
    9:  "ILLEGAL_TRANS",
    10: "CAL_FAIL",
}

# --- Output bitmap decode (matches firmware #defines) ---
OUTPUT_BITS = {
    0: "valve_charge_air",
    1: "valve_charge_water",
    2: "valve_disch_air",
    3: "valve_disch_water",
    4: "vent",
    5: "contactor",
    6: "sol_air",
    7: "sol_water",
}

# --- CSV column order (updated for calibration + new sensors) ---
CSV_COLUMNS = [
    # Identifiers
    "timestamp_iso", "timestamp_ms", "session_id", "test_id",
    # State
    "state", "state_name", "fault_flags_hex", "fault_list",
    # Pressure & water level
    "p_line_psi", "p_hydro_mbar", "temp_water_C",
    "water_level_cm", "tank_full", "tank_empty", "tank_low_warn",
    # RPM
    "rpm_air", "rpm_water", "rpm_alternator",
    # Supply electrical
    "i_12v_A", "i_5v_A", "v_bus_V",
    "i_compressor_A", "i_pump_A",
    # Alternator output
    "v_alt_V", "i_alt_A",
    # Valve position
    "valve_pos_air_cmd_deg", "valve_pos_water_cmd_deg",
    "valve_pos_air_actual_deg", "valve_pos_water_actual_deg",
    # Discrete inputs
    "estop", "arm_switch", "manual_reset",
    # Command outputs (decoded from bitmap)
    "sol_air_cmd", "sol_water_cmd",
    "valve_charge_air_cmd", "valve_charge_water_cmd",
    "valve_disch_air_cmd", "valve_disch_water_cmd",
    "vent_cmd", "contactor_cmd",
    # Derived
    "power_supply_W", "power_alt_W",
    "power_compressor_W", "power_pump_W",
    "energy_supply_J", "energy_alt_J",
    "dp_dt_psi_per_s",
    "rpm_ratio_air", "rpm_ratio_water", "driving_source",
    # Calibration
    "cal_valid", "cal_p_atm_mbar",
]

EVENT_COLUMNS = [
    "timestamp_iso", "timestamp_ms", "session_id",
    "event_type", "detail",
]


# =========================================================================
# TELEMETRY DECODER
# =========================================================================

class TelemetryDecoder:
    """
    Decodes raw Mega JSON into a full row dict with derived quantities.

    The Mega sends compact JSON (values scaled to ints to avoid floats
    on the AVR side).  This class unscales, derives, and produces a
    dict keyed by CSV_COLUMNS.
    """

    def __init__(self, session_id: str, test_id: str = ""):
        self.session_id = session_id
        self.test_id    = test_id

        # State for numerical derivatives and integrals
        self._prev_p_psi  = None
        self._prev_t_ms   = None
        self._energy_supply_J = 0.0
        self._energy_alt_J    = 0.0
        self._prev_state       = None
        self._prev_fault       = 0

        # Atmospheric offset for water-level zeroing
        self._p_atm_mbar = None

    def set_atm_offset(self, mbar: float):
        """Call once at startup with tank empty to zero the level sensor."""
        self._p_atm_mbar = mbar

    def decode(self, raw: dict) -> dict:
        """
        raw: dict parsed from Mega JSON line, e.g.:
          {"t":123456,"st":3,"p":8520,"ph":10131,"tw":221,
           "ra":0,"rw":0,"rs":0,
           "i12":410,"i5":180,"vb":1207,
           "ic":320,"ip":150,
           "va":1380,"ia":520,
           "vpa":90,"vpw":0,"vpaa":-1,"vpwa":-1,
           "es":0,"arm":1,"mr":0,
           "f":0,"o":35}
        """
        now_iso = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        t_ms    = raw.get("t", 0)

        # --- Unscale sensor values ---
        p_line_psi    = raw.get("p", 0)   / 100.0
        p_hydro_mbar  = raw.get("ph", 0)  / 10.0
        temp_water_C  = raw.get("tw", 0)  / 10.0

        rpm_air       = raw.get("ra", 0)  / 10.0
        rpm_water     = raw.get("rw", 0)  / 10.0
        rpm_alt       = raw.get("rs", 0)  / 10.0

        i_12v         = raw.get("i12", 0) / 1000.0
        i_5v          = raw.get("i5", 0)  / 1000.0
        v_bus         = raw.get("vb", 0)  / 100.0
        i_comp        = raw.get("ic", 0)  / 1000.0
        i_pump        = raw.get("ip", 0)  / 1000.0

        v_alt         = raw.get("va", 0)  / 100.0
        i_alt         = raw.get("ia", 0)  / 1000.0

        vpa_cmd       = raw.get("vpa", -1)
        vpw_cmd       = raw.get("vpw", -1)
        vpa_act       = raw.get("vpaa", -1)
        vpwa_act      = raw.get("vpwa", -1)

        estop         = raw.get("es", 0)
        arm           = raw.get("arm", 0)
        manual_reset  = raw.get("mr", 0)

        fault_flags   = raw.get("f", 0)
        output_bitmap = raw.get("o", 0)
        state_code    = raw.get("st", 0)

        # --- Calibration fields from Mega ---
        cal_valid     = raw.get("cv", 0) != 0
        cal_p_atm     = raw.get("cp", 10132) / 10.0   # mbar × 10

        # --- State name ---
        state_name = STATE_NAMES.get(state_code, f"UNKNOWN_{state_code}")

        # --- Fault decode ---
        fault_list = "|".join(
            name for bit, name in FAULT_BITS.items()
            if fault_flags & (1 << bit)
        ) or "NONE"

        # --- Output bitmap decode ---
        def bit(n): return 1 if (output_bitmap & (1 << n)) else 0

        # --- Water level ---
        # --- Water level (using Mega's calibrated atm offset) ---
        p_atm = cal_p_atm if cal_valid else self._p_atm_mbar

        gauge_mbar = p_hydro_mbar - p_atm
        # P = ρgh  →  h = P / (ρg),  with P in Pa = mbar × 100
        water_level_cm = max(0.0,
            (gauge_mbar * MBAR_TO_PA) / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2) * 100.0
        )
        tank_full     = 1 if water_level_cm >= TANK_FULL_CM else 0
        tank_empty    = 1 if water_level_cm <= TANK_EMPTY_CM else 0
        tank_low_warn = 1 if water_level_cm <= TANK_WARNING_CM else 0

        # --- Derived electrical ---
        power_supply_W  = i_12v * v_bus
        power_alt_W     = v_alt * i_alt
        power_comp_W    = i_comp * v_bus
        power_pump_W    = i_pump * v_bus

        # --- Energy integration (trapezoidal) ---
        if self._prev_t_ms is not None and t_ms > self._prev_t_ms:
            dt_s = (t_ms - self._prev_t_ms) / 1000.0
            self._energy_supply_J += power_supply_W * dt_s
            self._energy_alt_J    += power_alt_W    * dt_s

        # --- dP/dt ---
        dp_dt = 0.0
        if (self._prev_p_psi is not None and
                self._prev_t_ms is not None and
                t_ms > self._prev_t_ms):
            dt_s = (t_ms - self._prev_t_ms) / 1000.0
            if dt_s > 0:
                dp_dt = (p_line_psi - self._prev_p_psi) / dt_s

        # --- RPM ratios ---
        rpm_ratio_air   = (rpm_air   / rpm_alt) if rpm_alt > 1.0 else 0.0
        rpm_ratio_water = (rpm_water / rpm_alt) if rpm_alt > 1.0 else 0.0

        # Driving source: whichever ratio is closest to 1.0
        if rpm_alt < 1.0:
            driving = "NONE"
        elif abs(rpm_ratio_air - 1.0) < 0.05 and abs(rpm_ratio_water - 1.0) < 0.05:
            driving = "BOTH"
        elif abs(rpm_ratio_air - 1.0) < abs(rpm_ratio_water - 1.0):
            driving = "AIR"
        else:
            driving = "WATER"

        # --- Event detection (state change) ---
        event = None
        if self._prev_state is not None and state_code != self._prev_state:
            prev_name = STATE_NAMES.get(self._prev_state, "?")
            event = {
                "timestamp_iso": now_iso,
                "timestamp_ms":  t_ms,
                "session_id":    self.session_id,
                "event_type":    "STATE_CHANGE",
                "detail":        f"{prev_name} -> {state_name}",
            }
        if fault_flags != 0 and (self._prev_state is not None and
                raw.get("f", 0) != self._prev_fault):
            event = {
                "timestamp_iso": now_iso,
                "timestamp_ms":  t_ms,
                "session_id":    self.session_id,
                "event_type":    "FAULT",
                "detail":        fault_list,
            }

        # --- Store for next iteration ---
        self._prev_p_psi  = p_line_psi
        self._prev_t_ms   = t_ms
        self._prev_state  = state_code
        self._prev_fault  = fault_flags

        # --- Build row ---
        row = {
            "timestamp_iso":    now_iso,
            "timestamp_ms":     t_ms,
            "session_id":       self.session_id,
            "test_id":          self.test_id,

            "state":            state_code,
            "state_name":       state_name,
            "fault_flags_hex":  f"0x{fault_flags:04X}",
            "fault_list":       fault_list,

            "p_line_psi":       round(p_line_psi, 2),
            "p_hydro_mbar":     round(p_hydro_mbar, 2),
            "temp_water_C":     round(temp_water_C, 1),
            "water_level_cm":   round(water_level_cm, 2),
            "tank_full":        tank_full,
            "tank_empty":       tank_empty,
            "tank_low_warn":    tank_low_warn,

            "rpm_air":          round(rpm_air, 1),
            "rpm_water":        round(rpm_water, 1),
            "rpm_alternator":   round(rpm_alt, 1),

            "i_12v_A":          round(i_12v, 3),
            "i_5v_A":           round(i_5v, 3),
            "v_bus_V":          round(v_bus, 2),
            "i_compressor_A":   round(i_comp, 3),
            "i_pump_A":         round(i_pump, 3),

            "v_alt_V":          round(v_alt, 2),
            "i_alt_A":          round(i_alt, 3),

            "valve_pos_air_cmd_deg":    vpa_cmd if vpa_cmd >= 0 else "",
            "valve_pos_water_cmd_deg":  vpw_cmd if vpw_cmd >= 0 else "",
            "valve_pos_air_actual_deg": vpa_act if vpa_act >= 0 else "",
            "valve_pos_water_actual_deg": vpwa_act if vpwa_act >= 0 else "",

            "estop":            estop,
            "arm_switch":       arm,
            "manual_reset":     manual_reset,

            "sol_air_cmd":              bit(6),
            "sol_water_cmd":            bit(7),
            "valve_charge_air_cmd":     bit(0),
            "valve_charge_water_cmd":   bit(1),
            "valve_disch_air_cmd":      bit(2),
            "valve_disch_water_cmd":    bit(3),
            "vent_cmd":                 bit(4),
            "contactor_cmd":            bit(5),

            "power_supply_W":       round(power_supply_W, 2),
            "power_alt_W":          round(power_alt_W, 2),
            "power_compressor_W":   round(power_comp_W, 2),
            "power_pump_W":         round(power_pump_W, 2),
            "energy_supply_J":      round(self._energy_supply_J, 2),
            "energy_alt_J":         round(self._energy_alt_J, 2),
            "dp_dt_psi_per_s":      round(dp_dt, 2),
            "rpm_ratio_air":        round(rpm_ratio_air, 3),
            "rpm_ratio_water":      round(rpm_ratio_water, 3),
            "driving_source":       driving,
            "cal_valid":            1 if cal_valid else 0,
            "cal_p_atm_mbar":       round(cal_p_atm, 2),
        }

        return row, event


# =========================================================================
# CSV WRITER
# =========================================================================

class CSVWriter:
    """Manages main telemetry CSV and events CSV."""

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
        # Flush every 10 rows (~1 second) to avoid data loss on crash
        if self._rows_written % 10 == 0:
            self._telem_file.flush()

    def write_event(self, event: dict):
        self._event_writer.writerow(event)
        self._event_file.flush()  # events are rare, always flush immediately

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
    """Formats and sends JSON commands to the Mega."""

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._seq = 0
        self._lock = threading.Lock()

    def _send(self, obj: dict):
        with self._lock:
            self._seq += 1
            obj["seq"] = self._seq
            line = json.dumps(obj, separators=(",", ":")) + "\n"
            self._ser.write(line.encode("ascii"))

    def heartbeat(self):
        self._send({"heartbeat": 1})

    def request_mode(self, mode: str):
        """mode: 'ARM', 'CHARGE', 'DISCHARGE_AIR', etc."""
        self._send({"mode": mode})

    def request_reset(self):
        self._send({"reset": 1})

    def set_charge_config(self, air: bool = True, water: bool = False):
        self._send({"mode": "CHARGE",
                     "charge_air": int(air),
                     "charge_water": int(water)})

    def set_manual_io(self, mask: int, value: int):
        self._send({"mode": "MANUAL",
                     "manual_mask": mask,
                     "manual_val": value})

    def set_valve_positions(self, air_deg: int = -1, water_deg: int = -1):
        cmd = {}
        if air_deg >= 0:   cmd["vpa"] = air_deg
        if water_deg >= 0: cmd["vpw"] = water_deg
        if cmd:
            self._send(cmd)

    def cal_zero(self):
        """Tell Mega the sensor is out of water — start zeroing."""
        self._send({"mode": "CAL_ZERO"})

    def cal_done(self):
        """Tell Mega the sensor is back in water — proceed to OFF."""
        self._send({"mode": "CAL_DONE"})

    def cal_skip(self):
        """Skip calibration (use EEPROM stored value or default)."""
        self._send({"mode": "CAL_SKIP"})


# =========================================================================
# LOGGER (main thread-safe interface)
# =========================================================================

class Logger:
    """
    Top-level logger: opens serial, runs reader thread, exposes data.

    Usage:
        log = Logger(port="/dev/ttyACM0")
        log.start()
        while True:
            row = log.get_latest()   # newest decoded row or None
            hist = log.get_history() # deque of last N rows
    """

    def __init__(self, port: str, baud: int = BAUD_DEFAULT,
                 test_id: str = "", history_len: int = 600):
        self.session_id = str(uuid.uuid4())
        self.test_id    = test_id
        self.port       = port
        self.baud       = baud

        self._decoder = TelemetryDecoder(self.session_id, test_id)
        self._csv     = CSVWriter(self.session_id)

        self._history      = deque(maxlen=history_len)  # 60 s @ 10 Hz
        self._latest       = None
        self._latest_event = None
        self._lock         = threading.Lock()
        self._running      = False
        self._thread       = None
        self._hb_thread    = None

        self._ser = serial.Serial(port, baud, timeout=0.5)
        self.commander = CommandSender(self._ser)

        self._error_count = 0
        self._good_count  = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop,
                                        daemon=True, name="serial_reader")
        self._thread.start()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                           daemon=True, name="heartbeat")
        self._hb_thread.start()
        print(f"[Logger] started on {self.port} @ {self.baud}")
        print(f"[Logger] session: {self.session_id}")
        print(f"[Logger] CSV:     {self._csv.telem_path}")
        print(f"[Logger] Events:  {self._csv.event_path}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._csv.close()
        self._ser.close()
        print(f"[Logger] stopped. {self._csv.rows_written} rows written.")

    def get_latest(self) -> dict | None:
        with self._lock:
            return self._latest

    def get_latest_event(self) -> dict | None:
        with self._lock:
            e = self._latest_event
            self._latest_event = None  # consume it
            return e

    def get_history(self) -> list:
        with self._lock:
            return list(self._history)

    @property
    def stats(self) -> dict:
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
                print(f"[Logger] serial error: {e}")
                self._error_count += 1
                time.sleep(0.5)
            except Exception as e:
                print(f"[Logger] unexpected error: {e}")
                self._error_count += 1

    def _heartbeat_loop(self):
        while self._running:
            try:
                self.commander.heartbeat()
            except Exception:
                pass
            time.sleep(HEARTBEAT_INTERVAL_S)


# =========================================================================
# STANDALONE ENTRY POINT
# =========================================================================

def main():
    parser = argparse.ArgumentParser(description="Test Stand Logger")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    parser.add_argument("--test-id", default="",
                        help="Optional test identifier (e.g. 'air_60psi_run3')")
    args = parser.parse_args()

    log = Logger(port=args.port, baud=args.baud, test_id=args.test_id)
    log.start()

    try:
        while True:
            time.sleep(2.0)
            s = log.stats
            row = log.get_latest()
            if row:
                print(f"[{row['state_name']:18s}] "
                      f"P={row['p_line_psi']:6.1f} psi  "
                      f"RPM(a/w/s)={row['rpm_air']:5.0f}/"
                      f"{row['rpm_water']:5.0f}/"
                      f"{row['rpm_alternator']:5.0f}  "
                      f"Palt={row['power_alt_W']:6.1f}W  "
                      f"faults={row['fault_list']}  "
                      f"[{s['rows']} rows, {s['errors']} errs]")
            else:
                print(f"[Logger] waiting for data... "
                      f"({s['errors']} parse errors)")
    except KeyboardInterrupt:
        print("\n[Logger] shutting down...")
        log.stop()


if __name__ == "__main__":
    main()
