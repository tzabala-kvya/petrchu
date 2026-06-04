// ============================================================================
// config_block_draft.h
// ----------------------------------------------------------------------------
// DRAFT refactored configuration block for TestStandFirmware.ino.
// Replaces the existing CONFIGURATION sections (currently lines ~29-300 of
// TestStandFirmware.ino) with a layout that reflects the post-pivot
// architecture:
//
//   - Air side regulated via PULSE-RATE-MODULATED SOLENOID, not servo prop
//     valve. Air-side prop valve hardware is gone. Pulse-rate PID structure
//     lifts from mosfet_fusch_pid_uno (error variable switched from psi -> V).
//   - Water side keeps the HSH-Flo 0-10V proportional valve on d9 PWM, with
//     OC-PWM white-wire feedback on d4. Drive + feedback code lifts from
//     prop_valve_test_uno.
//   - Alternator RPM via quadrature encoder (CPR=4) on d2/d3. Old alt hall
//     and air-side hall removed. Water-side hall optional (default OFF).
//   - One ACS712 (load side). 5V-bus current sense removed. One voltage
//     sensor (V_alt). Bus-voltage sensor removed.
//   - 5-switch operator panel + 1 E-stop, no buttons.
//   - 5 status LEDs (R/G/Y/B/W) + piezo.
//   - MS5837 is the -02BA variant; requires setModel(MS5837_02BA) on the
//     Blue Robotics library (does NOT autodetect). Atm offset stored in
//     EEPROM; cal procedure in firmware/mega/ms5837_cal_mega/.
//
// >>> REVIEW markers split into action categories:
// >>>   VERIFY    - check against schematic / hardware before flashing
// >>>   MEASURE   - bench-measure the actual value, override the placeholder
// >>>   DECIDE    - design call, owner Tristan
// >>>   BLOCKED   - waits on Brennan (vessel) or Philip (turbine + bevel teeth)
// >>>   TUNE      - PID/dispatch placeholders, expect iteration
// >>>   PLACEHOLDER - guessed number, replace before relying on it
// ============================================================================

#ifndef CONFIG_BLOCK_DRAFT_H
#define CONFIG_BLOCK_DRAFT_H

#include <Arduino.h>

// ============================================================================
// 1. PIN ASSIGNMENTS
// ============================================================================
// >>> VERIFY every pin against actual wiring before powering on.
//
// Mega external-interrupt pin pool: d2, d3, d18, d19, d20, d21.
// d20/d21 are SDA/SCL (MS5837 I2C), so the usable interrupt pool is
// d2, d3, d18, d19. Allocation below uses all four.

// --- Digital outputs: status LEDs (active HIGH, in series with 220-330 ohm) ---
static const uint8_t PIN_LED_RED    = 52;  // fault / E-stop active
static const uint8_t PIN_LED_GREEN  = 50;  // armed / nominal
static const uint8_t PIN_LED_YELLOW = 48;  // warning (approaching limits, valve in transit)
static const uint8_t PIN_LED_BLUE   = 46;  // discharge active
static const uint8_t PIN_LED_WHITE  = 44;  // charge active (pump or compressor running)

// --- Digital output: piezo buzzer ---
static const uint8_t PIN_PIEZO      = 42;  // fault tone, brief beep on fault edge

// --- Digital outputs: MOSFET-driven loads ---
// MOSFET 1 (d45) -> 12V relay coil -> compressor contactor (AC mains).
// MOSFET 2 (d43) -> 12V relay coil -> pump contactor (AC mains).
// MOSFET 3 (d41) -> air pulse-rate solenoid (PID-modulated, <= MAX_PULSE_HZ).
// MOSFET 4 (d39) -> air on/off arm solenoid (open during DISCHARGE_AIR/BOTH).
// All four follow the low-side N-channel pattern from mosfet_test_uno.
static const uint8_t PIN_MOSFET_COMP_CONT  = 45;  // MOSFET 1
static const uint8_t PIN_MOSFET_PUMP_CONT  = 43;  // MOSFET 2
static const uint8_t PIN_MOSFET_AIR_PULSE  = 41;  // MOSFET 3 -- pulse-rate PID drives this
static const uint8_t PIN_MOSFET_AIR_ARM    = 39;  // MOSFET 4 -- on/off arm

// --- Digital output: water proportional valve drive (PWM -> 0-10V converter) ---
// PWM-capable on Mega Timer1. Drives the HSH-Flo green wire via a calibrated
// PWM-to-0-10V converter. Code path lifted from prop_valve_test_uno.
static const uint8_t PIN_WATER_VALVE_PWM   = 9;

// --- Digital inputs: interrupt-capable pins ---
// d2 (INT0): alt encoder channel A
// d3 (INT1): alt encoder channel B (only needed for direction; RPM works on A alone)
// d18 (INT5): E-stop. INPUT_PULLUP, NC contact between pin and GND. Fail-safe.
// d19 (INT4): water-side bevel pinion hall (OPTIONAL; see HAVE_WATER_HALL below).
static const uint8_t PIN_ALT_ENCODER_A     = 2;
static const uint8_t PIN_ALT_ENCODER_B     = 3;
static const uint8_t PIN_ESTOP             = 18;
static const uint8_t PIN_HALL_WATER        = 19;

// --- Digital input: water valve OC-PWM position feedback ---
// White wire from HSH-Flo. External 10k pull-up to 5V (per prop_valve_test_uno).
// Read via pulseIn() in loop -- no interrupt needed.
static const uint8_t PIN_WATER_VALVE_FB    = 4;

// --- Digital inputs: operator-panel switches (all INPUT_PULLUP, closed = LOW) ---
// All five wired identically: pin -> switch -> GND. Fail-safe to "off"
// (cut wire reads HIGH -> disarmed/disabled).
// CHARGE and DISCHARGE are PERMISSIVE: ON means the firmware is *allowed*
// to actuate the corresponding loads when ARM and MODE_* also agree and
// no faults are active. They are not direct triggers.
static const uint8_t PIN_SW_DISCHARGE      = 29;
static const uint8_t PIN_SW_CHARGE         = 31;
static const uint8_t PIN_SW_MODE_WATER     = 33;
static const uint8_t PIN_SW_MODE_AIR       = 35;
static const uint8_t PIN_SW_ARM            = 37;

// --- I2C: MS5837 lower-tank hydrostatic sensor ---
// d20 = SDA, d21 = SCL. Consumes INT3 and INT2 (not available for other use).
// Sensor wired to 3.3V (NOT 5V). Pull-ups: 10k to 3.3V if breakout lacks them.
// Model is MS5837_02BA -- library requires setModel(MS5837_02BA) explicitly.

// --- Analog inputs ---
// >>> CHANGED: this map drops bus voltage, 5V-bus current, alt-shaft hall,
// >>>          and the second ACS712. Pre-pivot pins NOT to repurpose without
// >>>          schematic check.
static const uint8_t PIN_V_ALT             = A9;   // voltage sensor on post-rectifier load bus
static const uint8_t PIN_I_LOAD            = A10;  // ACS712-20A between rectifier and load
static const uint8_t PIN_P1_VESSEL         = A13;  // Fusch 200 psi -- upstream of arm solenoid
static const uint8_t PIN_P2_MOTOR          = A12;  // Fusch 100 psi -- downstream of pulse solenoid
static const uint8_t PIN_I_PUMP            = A14;  // SCT-013 around pump mains
static const uint8_t PIN_I_COMP            = A15;  // SCT-013 around compressor mains
// a8, a11 intentionally unassigned (former V #2 and ACS #2, both dropped).

// --- Compile-time toggles for optional channels ---
// Water hall is the last piece of OWB engagement detection we have. At PoC
// scope, dispatch can run without it -- water-only is the regulated path and
// air can't lock its OWB at 350-500 RPM anyway. Default OFF; flip to true
// when the hall is wired and engagement detection is back in scope.
static const bool HAVE_WATER_HALL          = false;

// ============================================================================
// 2. CALIBRATION CONSTANTS
// ============================================================================
// >>> CHECK each one on bench before trusting. Numbers carried over from the
// >>> validated test sketches noted in each subsection.

// --- ADC ---
static const float    ADC_REF_V   = 5.0f;
static const uint16_t ADC_COUNTS  = 1023;

// --- Fusch pressure transducers (from pressure_fusch_100psi_test_uno + 200) ---
// Both are 3-wire ratiometric 0.5-4.5 V.
static const float PRESSURE_V_MIN  = 0.5f;
static const float PRESSURE_V_MAX  = 4.5f;
static const float P1_FS_PSI       = 200.0f;   // vessel-side sensor
static const float P2_FS_PSI       = 100.0f;   // motor-inlet sensor
static const float P1_NOISE_FLOOR  = 0.20f;    // psi -- clamp tiny readings to 0
static const float P2_NOISE_FLOOR  = 0.10f;
// NOTE: vented-static venting effect is a real flow-dynamic artifact, not a
// calibration issue. With the motor connected and drawing flow, P2 reads true
// static pressure at the motor inlet (which is the control variable). See
// thread notes 2026-06-04 for details. No correction in firmware.

// --- Voltage divider for V_alt (from voltage_sensor_test_uno) ---
// Alt OC voltage can reach 24 V (free-spin); divider must keep ADC under 5 V.
// >>> CALCULATE from actual resistor values, then verify with multimeter.
static const float ALT_V_DIV_RATIO = 10.0f;   // 24V -> 2.4V at ADC

// --- ACS712 (load side, from acs712_test_uno) ---
// ACS712-20A: 0.100 V/A. ACS712-5A would be 0.185 V/A -- VERIFY part installed.
static const float CURRENT_GAIN_LOAD = 0.100f;
// Zero-current output voltage. ACS712 spec is 2.5 V +/- 25 mV.
// >>> MEASURE on bench with primary disconnected, store in EEPROM addr 6.
static const float CURRENT_OFFSET_DC_V = 2.5f;   // fallback if EEPROM uninitialized

// --- SCT-013 clamp CTs (from sct013_030_test_uno) ---
// >>> DECIDE which SCT-013 model. SCT-013-030 has internal burden -> 0.033 V/A.
// >>> SCT-013-000 needs external burden + different math entirely.
static const float CURRENT_GAIN_PUMP = 0.033f;   // >>> PLACEHOLDER until SCT model confirmed
static const float CURRENT_GAIN_COMP = 0.033f;
static const float CURRENT_OFFSET_AC_V = 0.0f;   // AC clamps read 0 at zero current

// --- Alt encoder (from encoder_test_uno) ---
// QIWO QW86Y2H03L30 shipped with a 5-wire encoder. CPR = 4.
static const uint8_t ALT_ENCODER_CPR = 4;

// --- Water-side hall (from hall_test_uno, only if HAVE_WATER_HALL) ---
// >>> COUNT actual magnets installed on the bevel pinion if/when this is wired.
static const uint8_t PULSES_PER_REV_WATER = 2;

// --- Water valve OC-PWM feedback (from prop_valve_test_uno bench cal) ---
static const float FB_DUTY_CLOSED_PCT = 4.9f;
static const float FB_DUTY_OPEN_PCT   = 93.5f;
static const float VALVE_FULL_SCALE_V = 10.0f;   // converter output at 100% PWM duty

// --- MS5837 (from ms5837_cal_mega) ---
// Sensor model is -02BA. Library does NOT autodetect -- call setModel(MS5837_02BA).
static const float FLUID_DENSITY_KG_M3 = 997.0f;   // freshwater @ ~25 C
static const float GRAVITY_M_S2        = 9.80665f;
static const float ATM_DEFAULT_MBAR    = 1013.25f; // fallback when EEPROM uninitialized

// ============================================================================
// 3. OPERATING SETPOINTS
// ============================================================================

// --- Target output voltage ---
// Single setpoint, Pi-overridable via Command.v_setpoint_cV (centivolts).
// Mega EEPROM (addr 10) holds the boot default; Pi can change at runtime.
// PoC mode-specific guidance (Pi picks based on MODE_* switch state):
//   water-only or both:   8.00 V  (loaded water peak is below the old 10-12 V
//                                  free-flow estimate -- 8 V is honest target)
//   air-only:             4.00 V  (alt @ ~310 RPM, well below cut-in --
//                                  demonstrates loop is alive, not useful power)
// Boot default below is the conservative one: safer than 12 V if Pi hasn't
// connected yet (loop can actually reach 8 V; would saturate forever at 12 V).
static const int16_t V_SETPOINT_DEFAULT_CV = 800;   // 8.00 V

// --- Pressure targets ---
static const float P_VESSEL_CHARGE_TARGET_PSI = 100.0f; // compressor auto-stops at this P1
static const float P_VESSEL_MAX_PSI            = 120.0f;// >>> BLOCKED: Brennan; MUST stay above target
static const float P_MOTOR_MAX_PSI             = 56.5f; // LOCKED -- NRL-8K datasheet
static const float P_SAFE_VENT_PSI             = 5.0f;
static const float P_SANITY_LO_PSI             = -2.0f;
static const float P_SANITY_HI_PSI             = 250.0f;

// Lower-tank depth at which the pump auto-stops (lower tank drained ==
// upper tank full, since same volume). 1 cm above cavitation floor.
static const float P_TANK_CHARGED_DEPTH_CM = 6.0f;

// ============================================================================
// 4. FAULT THRESHOLDS
// ============================================================================
// >>> DECIDE every value here is a placeholder; measure on bench before trusting.

// --- Overvoltage (asymmetric: closes actuators faster than opens) ---
// Hardware crowbar fires at ~16 V. Firmware must shut down before that.
static const float    V_OVERVOLTAGE_SOFT_V            = 13.5f;
static const float    V_OVERVOLTAGE_HARD_V            = 14.5f;
static const uint16_t V_OVERVOLTAGE_SOFT_DURATION_MS  = 100;

// --- V droop fault (graceful droop tolerated, this is the "real failure" line) ---
// Triggers only when both sides are saturated and V is still falling.
static const float    V_DROOP_FAULT_FRAC              = 0.70f;
static const uint16_t V_DROOP_FAULT_DURATION_MS       = 2000;

// --- Overspeed ---
// >>> DECIDE: alt + drivetrain mechanical limit. QIWO datasheet gives BLDC RPM
// >>>         max; bearings likely the real limit.
static const float ALT_RPM_WARN   = 2500.0f;
static const float ALT_RPM_FAULT  = 3000.0f;

// --- Air-side leak (only fires during charge: compressor on, arm solenoid closed) ---
static const uint16_t LEAK_CHARGE_GRACE_MS    = 5000;
static const float    LEAK_P_GATE_FRAC        = 0.80f;
static const float    LEAK_MIN_PSI_PER_S      = 1.0f;
static const uint16_t LEAK_FAULT_DURATION_MS  = 10000;

// --- P2-stuck fault (pulse rate commanded > 0 but P2 not responding) ---
// Reworded from the servo-era version. Applies during DISCHARGE_AIR with
// arm solenoid open and pulse rate non-zero.
static const float    P2_STUCK_MIN_PULSE_HZ          = 1.0f;
static const float    P2_STUCK_PRESSURE_DELTA_PSI    = 0.5f;
static const uint16_t P2_STUCK_WINDOW_MS             = 3000;

// --- Pi heartbeat (energized states only) ---
static const uint16_t PI_HEARTBEAT_TIMEOUT_MS = 2000;

// --- Generic fault debounce ---
static const uint16_t FAULT_DEBOUNCE_MS = 50;

// --- Switch / E-stop debounce ---
static const uint16_t SWITCH_DEBOUNCE_MS = 30;   // matches estop_arm_test_mega

// ============================================================================
// 5. GEAR RATIOS
// ============================================================================
// Simplified vs. pre-pivot draft: NO gearboxes on either side in the PoC
// build (decided to ship without them to meet presentation deadline).
// Both sides drive their bevel pinion directly.

// Air side: NRL-8K motor output -> bevel pinion DIRECT.
// Motor runs 350-500 RPM at operating point. At Kv=62, that's V_oc = 5.6..8.0 V.
// Minus ~1.4 V rectifier drop = 4.2..6.6 V open-circuit; under load: lower.
// This is why air-only target is 4 V, not 12 V -- it's "loop alive" demo.
static const float GEAR_RATIO_AIR_GBOX = 1.0f;

// Water side: turbine -> bevel pinion DIRECT.
// >>> CAVEAT: previous "10-12 V achievable" estimate was free-flow turbine
// >>>         RPM under NO LOAD. Loaded RPM will be lower (turbine extracts
// >>>         torque from the flow). Real ceiling under V-regulated load is
// >>>         likely below 12 V -- consider lowering V_SETPOINT_DEFAULT_CV
// >>>         once bench numbers are in.
static const float GEAR_RATIO_WATER_GBOX = 1.0f;

// Bevel ratio: alt-bevel teeth / pinion teeth. >>> BLOCKED: bevel pair selection.
static const float BEVEL_TEETH_RATIO = 1.0f;     // >>> PLACEHOLDER

// BLDC Kv. LOCKED.
static const float ALT_KV_RPM_PER_V = 62.0f;

// Passive 3-phase bridge forward drop (typical 1N5408-class).
// >>> MEASURE under load.
static const float RECTIFIER_VDROP_V = 1.4f;

// ============================================================================
// 6. DISPATCH & ENGAGEMENT
// ============================================================================
// Dispatch rules use water-valve POSITION FEEDBACK (white-wire pulseIn),
// not commanded position -- the valve takes ~7.5 s to traverse 0-100%, so
// "commanded > 80%" would fire while the valve is still in transit.
// (See prop_valve_dynamics memory.)

// --- Add-air trigger ---
static const float    DISPATCH_ADD_AIR_VALVE_PCT     = 80.0f;  // water FB position
static const float    DISPATCH_ADD_AIR_V_ERR_V       = 0.5f;
static const uint16_t DISPATCH_ADD_AIR_DURATION_MS   = 2000;

// --- Drop-air trigger (wider band to avoid flapping) ---
// "Air not needed" on the pulse-rate side = commanded pulse rate near zero.
static const float    DISPATCH_DROP_AIR_PULSE_HZ     = 0.5f;
static const uint16_t DISPATCH_DROP_AIR_DURATION_MS  = 5000;

// --- OWB engagement detection ---
// DISABLED in PoC scope (HAVE_WATER_HALL = false). If reinstated, compare
// water hall RPM * bevel ratio vs alt RPM with these fractions.
static const float ENGAGE_LOCK_FRAC      = 0.95f;
static const float ENGAGE_DISENGAGE_FRAC = 0.85f;
static const float ENGAGE_MISMATCH_FRAC  = 1.20f;

// ============================================================================
// 7. PID GAINS
// ============================================================================
// Two PI controllers, one per side. Both regulate V_alt. Different actuators:
//   - water: continuous valve position (0..100 %)
//   - air:   discrete pulse rate (0..MAX_PULSE_HZ)
//
// Both lift their structure from validated test sketches:
//   - water gains/anti-windup model: derived for the 7-10 s prop valve
//   - air gains/anti-windup model:   lifted from mosfet_fusch_pid_uno
//     (error swapped from psi to V, output stays Hz)
//
// >>> TUNE: placeholders are conservative. Tune on bench. Hand-tune Kp to
// >>>       oscillation, back off to ~60 %, then set Ki ~ Kp/4. D stays 0.

// --- Water PID (V_alt -> valve %) ---
// Bandwidth ceiling is the actuator: ~10 s full travel -> << 0.016 Hz BW.
// Mandatory anti-windup using FB position (freeze integral while
// |commanded - feedback| > WATER_VALVE_SLEW_BAND_PCT).
static const float    PID_WATER_KP            = 7.0f;    // %/V
static const float    PID_WATER_KI            = 1.5f;    // %/(V*s)
static const float    PID_WATER_KD            = 0.0f;
static const float    PID_WATER_OUT_MIN_PCT   = 0.0f;
static const float    PID_WATER_OUT_MAX_PCT   = 100.0f;
static const float    PID_WATER_INTEGRAL_MAX  = 50.0f;
static const float    PID_WATER_INTEGRAL_MIN  = -50.0f;
static const float    PID_WATER_DEADBAND_V    = 0.1f;
static const uint16_t PID_WATER_UPDATE_MS     = 200;     // 5 Hz update -- well above valve BW
static const float    WATER_VALVE_SLEW_PCT_PER_S = 10.0f; // measured cap
static const float    WATER_VALVE_SLEW_BAND_PCT  = 5.0f;  // anti-windup window

// --- Air pulse-rate PID (V_alt -> pulse Hz) ---
// MAX_PULSE_HZ ceiling protects the solenoid mechanical spec.
// PULSE_WIDTH_MS is operator-set; PID modulates frequency only.
static const float    PID_AIR_KP              = 1.0f;    // Hz/V
static const float    PID_AIR_KI              = 0.2f;    // Hz/(V*s)
static const float    PID_AIR_KD              = 0.0f;
static const float    PID_AIR_INTEGRAL_MAX    = 50.0f;
static const float    PID_AIR_INTEGRAL_MIN    = -50.0f;
static const float    PID_AIR_DEADBAND_V      = 0.1f;
static const float    PID_AIR_MIN_USEFUL_HZ   = 0.25f;   // below this -> just close
static const uint16_t PID_AIR_UPDATE_MS       = 200;     // matches mosfet_fusch_pid_uno

static const float    AIR_MAX_PULSE_HZ        = 5.0f;    // solenoid mechanical spec
static const uint16_t AIR_PULSE_WIDTH_MS      = 200;     // >>> TUNE on bench with motor

// --- Closure targets (used by overvoltage handler and SHUTDOWN entry) ---
static const float    WATER_VALVE_CLOSED_PCT  = 0.0f;
static const float    AIR_PULSE_OFF_HZ        = 0.0f;

// ============================================================================
// 8. LOOP RATES
// ============================================================================
static const uint16_t LOOP_PERIOD_MS = 10;    // 100 Hz main loop
static const uint16_t LOG_PERIOD_MS  = 100;   // 10 Hz telemetry to Pi

// ============================================================================
// 9. EEPROM LAYOUT
// ============================================================================
// Mega scope: boot-time-critical calibration only. Everything else is in Pi
// config (gear ratios, fault thresholds, dispatch constants, session metadata).
// Address map kept compact -- room to grow.

static const int      EEPROM_CAL_MAGIC_ADDR     = 0;   // 2 bytes uint16
static const int      EEPROM_CAL_ATM_ADDR       = 2;   // 4 bytes float -- MS5837 atm offset (mbar)
static const int      EEPROM_ACS_LOAD_OFFSET    = 6;   // 4 bytes float -- ACS712 zero (volts)
static const int      EEPROM_V_SETPOINT_CV      = 10;  // 2 bytes int16  -- V setpoint default (centivolts)
// addr 12 onward reserved for future cal slots (e.g. SCT burden, encoder cal).

static const uint16_t EEPROM_CAL_MAGIC = 0xCA1B;

#endif // CONFIG_BLOCK_DRAFT_H
