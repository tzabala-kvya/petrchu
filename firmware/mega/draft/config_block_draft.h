// ============================================================================
// config_block_draft.h
// ----------------------------------------------------------------------------
// DRAFT refactored configuration block for TestStandFirmware.ino.
// Replaces the existing CONFIGURATION sections (currently lines ~29-249 in
// TestStandFirmware.ino) with a cleaner layout that supports the new
// V-regulation control loop, dispatch logic, and expanded fault taxonomy.
//
// >>> REVIEW: this is a draft — none of these constants are committed.
// >>>         Every section has at least one >>> marker that needs attention
// >>>         before this gets merged into the main sketch.
//
// Layout matches the convention recorded in memory:
//   1. Pin assignments        ( >>> VERIFY against schematic )
//   2. Calibration constants  ( >>> CHECK on bench )
//   3. Operating setpoints    ( >>> DECIDE )
//   4. Fault thresholds       ( >>> DECIDE )
//   5. Gear ratios            ( >>> VERIFY once locked )
//   6. Dispatch & engagement  ( >>> TUNE )
//   7. PID gains              ( >>> TUNE )
//   8. Loop rates             ( unchanged from current firmware )
//   9. EEPROM layout          ( additive — extends current map )
//
// >>> REVIEW: using `static const` to match the existing TestStandFirmware.ino
// >>>         style. If this header gets #included from multiple translation
// >>>         units, switch to `inline constexpr` (C++17) to avoid duplicate
// >>>         definitions. Arduino .ino sketches are single TU so this is fine.
// ============================================================================

#ifndef CONFIG_BLOCK_DRAFT_H
#define CONFIG_BLOCK_DRAFT_H

#include <Arduino.h>

// ============================================================================
// 1. PIN ASSIGNMENTS
// ============================================================================
// Most pins UNCHANGED from current TestStandFirmware.ino. Listed here for
// completeness so config is in one place.
//
// >>> VERIFY every pin against the actual wiring before powering on.

// --- Digital outputs (UNCHANGED) ---
static const uint8_t PIN_COMPRESSOR_CONT  = 22;
static const uint8_t PIN_PUMP_CONT        = 23;
static const uint8_t PIN_DISCH_SOL_AIR    = 24;
static const uint8_t PIN_DISCH_SOL_WATER  = 25;
static const uint8_t PIN_STATUS_LED_GREEN = 13;
static const uint8_t PIN_STATUS_LED_RED   = 12;
static const uint8_t PIN_BUZZER           = 11;
static const uint8_t PIN_VALVE_SERVO_AIR   = 9;   // PWM-capable
static const uint8_t PIN_VALVE_SERVO_WATER = 10;  // PWM-capable

// --- Digital inputs (UNCHANGED) ---
static const uint8_t PIN_ESTOP           = 2;   // INT0 — must be interrupt-capable
static const uint8_t PIN_HALL_RPM_AIR    = 3;   // INT1 — hall at gearbox output (air side)
static const uint8_t PIN_HALL_RPM_WATER  = 18;  // INT5 — hall at gearbox output (water side)
static const uint8_t PIN_HALL_RPM_ALT    = 19;  // INT4 — hall at alternator shaft
static const uint8_t PIN_ARM_SWITCH      = 30;
static const uint8_t PIN_MANUAL_RESET    = 31;

// --- Analog inputs ---
// >>> CHANGED: PIN_PRESSURE_XDUCR renamed to PIN_PRESSURE_P1 (compressor outlet).
// >>> NEW:     PIN_PRESSURE_P2 added (downstream of air servo reg, motor inlet).
static const uint8_t PIN_PRESSURE_P1     = A0;  // upstream of solenoid (was PIN_PRESSURE_XDUCR)
static const uint8_t PIN_CURRENT_12V     = A1;
static const uint8_t PIN_CURRENT_5V      = A2;
static const uint8_t PIN_BUS_VOLTAGE     = A3;
static const uint8_t PIN_ALT_VOLTAGE     = A4;
static const uint8_t PIN_ALT_CURRENT     = A5;  // ACS712 between rectifier and load
static const uint8_t PIN_CURRENT_COMP    = A6;  // SCT-013 clamp on compressor mains
static const uint8_t PIN_CURRENT_PUMP    = A7;  // SCT-013 clamp on pump mains
static const uint8_t PIN_VALVE_ENC_AIR   = A8;  // optional servo angle feedback
static const uint8_t PIN_VALVE_ENC_WATER = A9;  // optional servo angle feedback
static const uint8_t PIN_PRESSURE_P2     = A10; // NEW: motor-inlet pressure (read-only)

// ============================================================================
// 2. CALIBRATION CONSTANTS
// ============================================================================
// >>> CHECK each one against the actual installed component before trusting.

static const float    ADC_REF_V   = 5.0f;
static const uint16_t ADC_COUNTS  = 1023;

// --- Pressure transducers ---
// Assumed standard industrial 0.5–4.5V output.
// >>> MEASURE: actual V at 0 psi for each transducer on bench. If they differ
// >>>          by >50 mV from datasheet, override the per-channel offset here.
static const float PRESSURE_V_MIN = 0.5f;
static const float PRESSURE_V_MAX = 4.5f;

// >>> DECIDE: full-scale range per transducer.
//   P1 must cover vessel pressure (~85 psi P_MAX) with margin → 150 psi sensor.
//   P2 only sees post-manual-reg (≤56.5 psi) — a 100 psi sensor gives better
//   resolution. Different parts OK; flag in BOM that they aren't interchangeable.
static const float P1_FS_PSI = 200.0f;
static const float P2_FS_PSI = 100.0f;  // >>> DECIDE: smaller-range sensor for P2?

// --- Voltage dividers ---
// >>> CALCULATE from actual resistor values, then verify with multimeter.
static const float BUS_DIV_RATIO   = 4.0f;   // 12V bus → ~3V at ADC
static const float ALT_V_DIV_RATIO = 10.0f;  // alt up to 50V → 5V max at ADC

// --- Current sensors ---
// Logic / parasitic side uses ACS712-5A (0.185 V/A — better resolution at low I).
// Load side uses ACS712-20A (0.100 V/A — headroom for transient overshoots).
// >>> VERIFY actual ACS712 variant on each physical channel.
static const float CURRENT_GAIN_12V_BUS = 0.185f;  // ACS712-5A on 12V control rail
static const float CURRENT_GAIN_5V_BUS  = 0.185f;  // ACS712-5A on 5V logic rail
static const float CURRENT_GAIN_ALT     = 0.100f;  // ACS712-20A between rectifier and load

// Clamp CTs on AC mains for compressor and pump (round-trip input-energy side).
// >>> DECIDE clamp model. SCT-013-030 (0.033 V/A) is common; SCT-013-000 needs a
// >>>        burden resistor and different math entirely.
static const float CURRENT_GAIN_COMP = 0.033f;  // >>> PLACEHOLDER
static const float CURRENT_GAIN_PUMP = 0.033f;  // >>> PLACEHOLDER

// Zero-current output voltage.
// >>> MEASURE per sensor with primary disconnected. ACS712 spec is 2.5V ±25mV;
// >>>        AC clamp CTs are AC-coupled and read 0V at zero.
static const float CURRENT_OFFSET_DC_V = 2.5f;   // ACS712 (DC hall)
static const float CURRENT_OFFSET_AC_V = 0.0f;   // SCT-013 (AC clamp)

// --- RPM ---
// >>> COUNT actual magnets installed on each shaft.
// >>> TODO: move these to EEPROM (firmware_conventions.md memory) so they
// >>>       can be set per-rig without reflashing.
static const uint8_t PULSES_PER_REV_AIR   = 2;
static const uint8_t PULSES_PER_REV_WATER = 2;
static const uint8_t PULSES_PER_REV_ALT   = 2;

// ============================================================================
// 3. OPERATING SETPOINTS
// ============================================================================

// --- Target output voltage (PoC target = 12V) ---
// Pi can override at runtime via v_setpoint_cV in the Command struct.
static const float V_SETPOINT_DEFAULT_V = 12.0f;

// --- Pressure targets ---
// The vessel can hold higher pressure than the NRL-8K motor can consume.
// Manual reg drops vessel → motor inlet.
// >>> CONFIRM manual reg is physically adjusted to ≤50 psi output.
//
// >>> DECIDE: vessel charge target. 80 psi gives margin below P_VESSEL_MAX
// >>>         and well above what the manual reg passes through.
//             Start LOW (~40 psi) for first hot tests, work up.
static const float P_VESSEL_CHARGE_TARGET_PSI = 80.0f;

// Vessel hard ceiling — protects tank, fittings, plumbing.
// >>> BLOCKED: Brennan to confirm against actual tank's rated pressure.
// >>>          Set to ~80% of weakest component's burst rating.
static const float P_VESSEL_MAX_PSI = 85.0f;  // >>> PLACEHOLDER

// Motor inlet hard ceiling — protects NRL-8K vanes/seals.
// LOCKED at datasheet value; do not raise.
static const float P_MOTOR_MAX_PSI = 56.5f;

// "System is vented / safe" threshold used in shutdown logic.
static const float P_SAFE_VENT_PSI = 5.0f;

// Sanity range for pressure sensor readings (detects sensor disconnection).
static const float P_SANITY_LO_PSI = -2.0f;
static const float P_SANITY_HI_PSI = 200.0f;

// --- Electrical bus limits ---
// >>> DECIDE based on actual measured draw with everything energized.
static const float V_BUS_MIN     = 11.0f;
static const float I_BUS_MAX_12V = 8.0f;
static const float I_BUS_MAX_5V  = 2.0f;

// ============================================================================
// 4. FAULT THRESHOLDS
// ============================================================================
// >>> DECIDE every value here is a placeholder; measure on bench before trusting.

// --- Overvoltage (asymmetric: closes servos faster than opens) ---
// Hardware crowbar fires at ~16V. Firmware must close servos before that.
static const float    V_OVERVOLTAGE_SOFT_V             = 13.5f;  // sustained
static const float    V_OVERVOLTAGE_HARD_V             = 14.5f;  // instantaneous
static const uint16_t V_OVERVOLTAGE_SOFT_DURATION_MS   = 100;

// --- V droop fault (graceful droop tolerated up to this point) ---
// Triggers only when both sides are saturated and V is still falling — i.e.,
// physics-limited delivery, not a control-loop bug.
static const float    V_DROOP_FAULT_FRAC               = 0.70f;  // V_alt < 70% of setpoint
static const uint16_t V_DROOP_FAULT_DURATION_MS        = 2000;

// --- Overspeed ---
// >>> DECIDE: actual mechanical limit of alternator + drivetrain.
// >>>         BLDC datasheet should give max RPM; bearings may be the real limit.
static const float ALT_RPM_WARN  = 2500.0f;
static const float ALT_RPM_FAULT = 3000.0f;

// --- Leak detection (compound: only fires during S_CHARGE_AIR) ---
static const uint16_t LEAK_CHARGE_GRACE_MS      = 5000;   // ignore first 5s of charge
static const float    LEAK_P_GATE_FRAC          = 0.80f;  // only check P1 < 80% of target
static const float    LEAK_MIN_PSI_PER_S        = 1.0f;   // dP/dt below this = stalled
static const uint16_t LEAK_FAULT_DURATION_MS    = 10000;  // sustained 10s

// --- P2-stuck fault (servo commanded motion but P2 not responding) ---
static const float    P2_STUCK_SERVO_DELTA_DEG    = 20.0f;
static const float    P2_STUCK_PRESSURE_DELTA_PSI = 0.5f;
static const uint16_t P2_STUCK_WINDOW_MS          = 3000;

// --- Pi heartbeat (energized states only) ---
static const uint16_t PI_HEARTBEAT_TIMEOUT_MS = 2000;

// --- Generic fault debounce (single-sample noise rejection) ---
static const uint16_t FAULT_DEBOUNCE_MS = 50;

// ============================================================================
// 5. GEAR RATIOS
// ============================================================================
// All ratios expressed as: upstream_RPM × ratio = downstream_RPM.
// >>> VERIFY against actual installed gearboxes and bevel teeth.

// Air side: NRL-8K motor output → step-up gearbox → bevel pinion → alt bevel.
// Sizing philosophy (memory: hardware_locked.md): place motor's peak-power RPM
// (275) at alt cut-in RPM (~750) so the motor has room to slow under load.
// 275 × 2.7 ≈ 743 ≈ alt cut-in.
static const float GEAR_RATIO_AIR_GBOX = 2.7f;   // motor RPM × this = bevel pinion RPM
                                                  // >>> CONFIRM after gearbox is in hand

// Water side: BLOCKED on Philip's turbine choice.
// >>> BLOCKED: water gear ratio depends on turbine design RPM.
static const float GEAR_RATIO_WATER_GBOX = 1.0f; // >>> PLACEHOLDER

// Bevel ratio: alt-bevel-gear teeth / pinion teeth.
// pinion_RPM (when OWB locked) = alt_RPM × this.
// E.g., 30T alt-bevel + 15T pinion → ratio = 2.0.
// >>> VERIFY teeth count once bevel pair is selected.
static const float BEVEL_TEETH_RATIO = 1.0f;     // >>> PLACEHOLDER

// BLDC Kv for V-regulation predictions and feed-forward.
// QIWO QW86Y2H03L30 — locked.
static const float ALT_KV_RPM_PER_V = 62.0f;

// Estimated rectifier diode forward drop (passive 3φ bridge, both diodes in path).
// >>> MEASURE under load — datasheet typical for 1N5408-class diodes.
static const float RECTIFIER_VDROP_V = 1.4f;

// ============================================================================
// 6. DISPATCH & ENGAGEMENT
// ============================================================================
// Auto-allocator hysteresis and OWB engagement thresholds.
// See memory: control_architecture.md for rationale.

// --- Add-air trigger (water saturated AND V error positive) ---
static const float    DISPATCH_ADD_AIR_VALVE_FRAC   = 0.80f;  // water valve >80% open
static const float    DISPATCH_ADD_AIR_V_ERR_V      = 0.5f;   // AND V error > 0.5V
static const uint16_t DISPATCH_ADD_AIR_DURATION_MS  = 2000;

// --- Drop-air trigger (air no longer needed) ---
static const float    DISPATCH_DROP_AIR_VALVE_FRAC  = 0.10f;  // air valve <10% open
static const uint16_t DISPATCH_DROP_AIR_DURATION_MS = 5000;   // wider band → no flap

// --- OWB engagement detection ---
// Hysteresis: lock at 95% of alt-locked RPM, disengage at 85%.
// Mismatch (>120%) = unsafe to engage, would slam OWB.
static const float ENGAGE_LOCK_FRAC      = 0.95f;
static const float ENGAGE_DISENGAGE_FRAC = 0.85f;
static const float ENGAGE_MISMATCH_FRAC  = 1.20f;

// ============================================================================
// 7. PID GAINS
// ============================================================================
// One PI controller per discharge side. D-term is 0 by default — V_alt is
// noisy enough that derivative kicks the output around without helping.
//
// >>> TUNE: placeholders are conservative (slow, won't oscillate). Tune via
// >>>       hand-tuning: raise Kp until oscillation, back off to ~60%, set
// >>>       Ki ≈ Kp / 4. Re-tune after R_LL bench measurement updates the
// >>>       expected droop slope.

// --- Air side ---
static const float PID_AIR_KP            = 8.0f;    // servo deg per V of error
static const float PID_AIR_KI            = 2.0f;    // servo deg per V·s
static const float PID_AIR_KD            = 0.0f;
static const float PID_AIR_OUT_MIN_DEG   = 15.0f;   // >>> CHECK: NRL-8K min vane-seat angle
static const float PID_AIR_OUT_MAX_DEG   = 170.0f;  // mechanical travel limit
static const float PID_AIR_SLEW_DEG_PER_S = 90.0f;  // opening rate cap

// --- Water side ---
static const float PID_WATER_KP            = 12.0f;
static const float PID_WATER_KI            = 3.0f;
static const float PID_WATER_KD            = 0.0f;
static const float PID_WATER_OUT_MIN_DEG   = 0.0f;  // water valve can fully close while running
static const float PID_WATER_OUT_MAX_DEG   = 170.0f;
static const float PID_WATER_SLEW_DEG_PER_S = 60.0f;

// --- Closure target (used by overvoltage handler and SHUTDOWN entry) ---
static const float SERVO_ANGLE_CLOSED_DEG = 0.0f;

// ============================================================================
// 8. LOOP RATES
// ============================================================================
// UNCHANGED from current TestStandFirmware.ino.

static const uint16_t LOOP_PERIOD_MS = 10;    // 100 Hz main loop
static const uint16_t LOG_PERIOD_MS  = 100;   // 10 Hz telemetry

// ============================================================================
// 9. EEPROM LAYOUT
// ============================================================================
// Boot-time-critical calibration only. Everything else lives in Pi config.
// >>> NEW: reserved addresses for ACS712 zero offsets and per-side PPR
// >>>      (not yet implemented in setup/cal flow — wire up next refactor).

static const int      EEPROM_CAL_MAGIC_ADDR = 0;   // 2 bytes
static const int      EEPROM_CAL_ATM_ADDR   = 2;   // 4 bytes — MS5837 atm offset (float)
static const int      EEPROM_ACS_OFFSET_12V = 6;   // 4 bytes float — NEW
static const int      EEPROM_ACS_OFFSET_5V  = 10;  // 4 bytes float — NEW
static const int      EEPROM_ACS_OFFSET_ALT = 14;  // 4 bytes float — NEW
static const int      EEPROM_PPR_AIR        = 18;  // 1 byte — NEW
static const int      EEPROM_PPR_WATER      = 19;  // 1 byte — NEW
static const int      EEPROM_PPR_ALT        = 20;  // 1 byte — NEW

static const uint16_t EEPROM_CAL_MAGIC = 0xCA1B;

#endif // CONFIG_BLOCK_DRAFT_H
