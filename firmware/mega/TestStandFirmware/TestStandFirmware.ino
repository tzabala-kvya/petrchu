/**
 * TestStandFirmware.ino
 * -------------------------------------------------------------------------
 * PetrChu test stand — primary controller (Arduino Mega 2560)
 * Supervisor / UI / logger: Raspberry Pi 5 (USB-serial, line-delimited JSON)
 *
 * STAGE A+C+D+E (post-pivot refactor, 2026-06):
 *   Stage A: config + sensor reads + switch reads + safety state machine
 *            + telemetry. LIVE.
 *   Stage C: water V-PID. LIVE — drives setWaterValvePct() in DISCHARGE_WATER
 *            and DISCHARGE_BOTH. Anti-windup via white-wire feedback.
 *   Stage D: air pulse-rate PID. LIVE — drives setAirPulseHz() in
 *            DISCHARGE_AIR and DISCHARGE_BOTH (when dispatch engages air).
 *   Stage E: water-first dispatch in DISCHARGE_BOTH (add-air on water
 *            saturation + V_err sustained; drop-air on low pulse Hz
 *            sustained). Fault taxonomy fills out F_LEAK_AIR, F_P2_STUCK,
 *            F_V_DROOP_SEVERE, F_PUMP_CAVITATION. SCT-013 streams true RMS
 *            in a 100-sample sliding window. LIVE.
 *
 * How the loop runs:
 *   - 100 Hz main loop: read sensors, read switches, check faults, step
 *     state machine, commit outputs, feed watchdog
 *   - 10 Hz telemetry: send JSON snapshot to Pi
 *   - Mega OWNS safety. Pi can request; Mega can refuse, fault, or E-stop.
 *
 * Architecture references (in memory):
 *   - pin_map_poc.md          : authoritative pin assignments
 *   - hardware_locked.md      : air motor, alt, no gearboxes
 *   - control_architecture.md : V-loop, dispatch, fault taxonomy
 *   - reservoirs_topology.md  : air chain (pulse-rate solenoid), water loop
 *   - firmware_conventions.md : config block format, EEPROM scope, Pi split
 *
 * Validated test sketches (lifted patterns):
 *   - estop_arm_test_mega/    : E-stop ISR + switch debounce
 *   - ms5837_cal_mega/        : EEPROM cal, MS5837_02BA setModel
 *   - mosfet_fusch_pid_uno/   : pulse-rate PID structure (used in Stage D)
 *   - prop_valve_test_uno/    : water valve PWM + white-wire pulseIn
 *   - pressure_fusch_*_uno/   : Fusch P1/P2 math
 *   - acs712_test_uno/        : load-side current math
 *   - voltage_sensor_test_uno : V_alt math
 *   - sct013_030_test_uno     : pump/comp AC current math
 *   - encoder_test_uno        : alt RPM via quadrature
 *   - led_test_uno, piezo_test_uno : status indicators
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "MS5837.h"

// ============================================================================
// 1. PIN ASSIGNMENTS  (authoritative copy in firmware/mega/draft/config_block_draft.h)
// ============================================================================
// >>> VERIFY every pin against actual wiring before powering on.

// --- Digital outputs: status LEDs ---
static const uint8_t PIN_LED_RED    = 52;  // fault / E-stop
static const uint8_t PIN_LED_GREEN  = 50;  // armed / nominal
static const uint8_t PIN_LED_YELLOW = 48;  // warning
static const uint8_t PIN_LED_BLUE   = 46;  // discharge active
static const uint8_t PIN_LED_WHITE  = 44;  // charge active

// --- Digital output: piezo ---
static const uint8_t PIN_PIEZO      = 42;

// --- Digital outputs: MOSFETs ---
static const uint8_t PIN_MOSFET_COMP_CONT = 45;  // M1 -> compressor contactor relay
static const uint8_t PIN_MOSFET_PUMP_CONT = 43;  // M2 -> pump contactor relay
static const uint8_t PIN_MOSFET_AIR_PULSE = 41;  // M3 -> air pulse-rate solenoid (PID-modulated)
static const uint8_t PIN_MOSFET_AIR_ARM   = 39;  // M4 -> air on/off arm solenoid

// --- Digital output: water prop valve PWM ---
static const uint8_t PIN_WATER_VALVE_PWM  = 9;   // PWM -> 0-10V converter -> HSH-Flo green

// --- Digital inputs: interrupt-capable ---
static const uint8_t PIN_ALT_ENCODER_A = 2;   // INT0
static const uint8_t PIN_ALT_ENCODER_B = 3;   // INT1 (direction; optional)
static const uint8_t PIN_ESTOP         = 18;  // INT5, INPUT_PULLUP, NC to GND
static const uint8_t PIN_HALL_WATER    = 19;  // INT4 — only used if HAVE_WATER_HALL

// --- Digital input: water valve OC-PWM feedback (external 10k pull-up) ---
static const uint8_t PIN_WATER_VALVE_FB = 4;  // pulseIn() read

// --- Digital inputs: operator panel switches (INPUT_PULLUP, closed = LOW) ---
static const uint8_t PIN_SW_DISCHARGE  = 29;
static const uint8_t PIN_SW_CHARGE     = 31;
static const uint8_t PIN_SW_MODE_WATER = 33;
static const uint8_t PIN_SW_MODE_AIR   = 35;
static const uint8_t PIN_SW_ARM        = 37;

// --- I2C (MS5837-02BA on d20/d21) ---
// d20 = SDA, d21 = SCL. Sensor wired to 3.3V (NOT 5V). 4.7k pull-ups on bus.

// --- Analog inputs ---
static const uint8_t PIN_V_ALT     = A9;
static const uint8_t PIN_I_LOAD    = A10;
static const uint8_t PIN_P1_VESSEL = A13;
static const uint8_t PIN_P2_MOTOR  = A12;
static const uint8_t PIN_I_PUMP    = A14;
static const uint8_t PIN_I_COMP    = A15;
// a8, a11 unassigned (former V #2 and ACS #2 — dropped from PoC build).

// --- Optional channel toggles ---
static const bool HAVE_WATER_HALL = false;   // water-side bevel pinion hall
// HAVE_DEPTH_SENSOR: set false on an air-only bench where the MS5837 isn't
// wired. When false: skip the I2C read (no F_I2C_TIMEOUT), drop the depth
// requirement from selftest (no F_SELFTEST from a missing sensor), and disable
// the depth-dependent pump interlocks (cavitation fault + tank-full auto-stop).
static const bool HAVE_DEPTH_SENSOR = false;

// ============================================================================
// 2. CALIBRATION CONSTANTS
// ============================================================================

// --- ADC ---
static const float    ADC_REF_V  = 5.0f;
static const uint16_t ADC_COUNTS = 1023;

// --- Fusch pressure transducers ---
static const float PRESSURE_V_MIN = 0.5f;
static const float PRESSURE_V_MAX = 4.5f;
static const float P1_FS_PSI      = 200.0f;
static const float P2_FS_PSI      = 100.0f;
static const float P1_NOISE_FLOOR = 0.20f;
static const float P2_NOISE_FLOOR = 0.10f;

// --- V_alt voltage divider ---
// >>> CALCULATE from actual resistor values, verify with multimeter.
static const float ALT_V_DIV_RATIO = 10.0f;

// --- ACS712 (load side, 20A part: 0.100 V/A) ---
static const float CURRENT_GAIN_LOAD   = 0.100f;
static const float CURRENT_OFFSET_DC_V = 2.5f;   // fallback if EEPROM uninitialized

// --- SCT-013 (placeholder, depends on model) ---
static const float CURRENT_GAIN_PUMP   = 0.033f;
static const float CURRENT_GAIN_COMP   = 0.033f;
static const float CURRENT_OFFSET_AC_V = 0.0f;

// --- SCT-013 noise floor ---
// RMS of a zero-mean noisy signal is never 0; the SCT reads ~0.7 A with the
// load OFF. Deadband below this floor so the phantom current stays out of the
// transmitted reading (and therefore out of the Pi's power/efficiency integral).
// Power itself (V*I*PF) is computed Pi-side; the Mega only cleans the current.
// >>> RE-MEASURE per channel after final wiring/shielding.
static const float SCT_NOISE_FLOOR_A = 1.0f;

// --- Alt encoder ---
static const uint8_t ALT_ENCODER_CPR = 4;

// --- Water-side hall (only used if HAVE_WATER_HALL) ---
static const uint8_t PULSES_PER_REV_WATER = 2;

// --- Water valve OC-PWM feedback (bench cal) ---
static const float FB_DUTY_CLOSED_PCT = 4.9f;
static const float FB_DUTY_OPEN_PCT   = 93.5f;
static const float VALVE_FULL_SCALE_V = 10.0f;

// --- MS5837 ---
static const float FLUID_DENSITY_KG_M3 = 997.0f;
static const float GRAVITY_M_S2        = 9.80665f;
static const float ATM_DEFAULT_MBAR    = 1013.25f;

// ============================================================================
// 3. OPERATING SETPOINTS
// ============================================================================

// V setpoint -- mode-dependent, Pi-overridable. Boot default in EEPROM addr 10.
// PoC: water/both = 8.00 V, air-only = 4.00 V. Default 8.00 V (safer than 12).
static const int16_t V_SETPOINT_DEFAULT_CV = 800;

// Pressure targets
static const float P_VESSEL_CHARGE_TARGET_PSI = 100.0f;  // compressor auto-stops here
static const float P_VESSEL_MAX_PSI           = 120.0f;  // >>> BLOCKED: Brennan (must stay > target)
static const float P_MOTOR_MAX_PSI            = 56.5f;   // LOCKED NRL-8K
static const float P_SAFE_VENT_PSI            = 5.0f;
static const float P_SANITY_LO_PSI            = -2.0f;
static const float P_SANITY_HI_PSI            = 250.0f;

// Lower-tank depth at which the pump auto-stops during water charging.
// Lower tank is the same size as upper; when lower drains to ~6 cm, upper
// is full. Sits 1 cm above the cavitation fault floor for graceful stop.
static const float P_TANK_CHARGED_DEPTH_CM = 6.0f;

// ============================================================================
// 4. FAULT THRESHOLDS
// ============================================================================

static const float    V_OVERVOLTAGE_SOFT_V           = 13.5f;
static const float    V_OVERVOLTAGE_HARD_V           = 14.5f;
static const uint16_t V_OVERVOLTAGE_SOFT_DURATION_MS = 100;

static const float    V_DROOP_FAULT_FRAC             = 0.70f;
static const uint16_t V_DROOP_FAULT_DURATION_MS      = 2000;

static const float    ALT_RPM_WARN  = 2500.0f;
static const float    ALT_RPM_FAULT = 3000.0f;

static const uint16_t LEAK_CHARGE_GRACE_MS   = 5000;
static const float    LEAK_P_GATE_FRAC       = 0.80f;
static const float    LEAK_MIN_PSI_PER_S     = 1.0f;
static const uint16_t LEAK_FAULT_DURATION_MS = 10000;

static const float    P2_STUCK_MIN_PULSE_HZ       = 1.0f;
static const float    P2_STUCK_PRESSURE_DELTA_PSI = 0.5f;
static const uint16_t P2_STUCK_WINDOW_MS          = 3000;

static const uint16_t PI_HEARTBEAT_TIMEOUT_MS = 2000;
static const uint16_t FAULT_DEBOUNCE_MS       = 50;
static const uint16_t SWITCH_DEBOUNCE_MS      = 30;

// F_PUMP_CAVITATION: depth_cm below this with pump commanded = cavitation risk.
// >>> DECIDE on actual NPSH floor for the installed pump; 5 cm is a placeholder.
static const float P_CAVITATION_DEPTH_CM_MIN = 5.0f;

// "Saturated" threshold (PID output near max) used by V-droop fault check.
static const float WATER_SAT_FRAC = 0.98f;
static const float AIR_SAT_FRAC   = 0.98f;

// ============================================================================
// 5. GEAR RATIOS (no gearboxes in PoC; bevel ratio TBD)
// ============================================================================
static const float GEAR_RATIO_AIR_GBOX   = 1.0f;   // no air gearbox
static const float GEAR_RATIO_WATER_GBOX = 1.0f;   // no water gearbox
static const float BEVEL_TEETH_RATIO     = 1.0f;   // >>> PLACEHOLDER
static const float ALT_KV_RPM_PER_V      = 62.0f;  // LOCKED QIWO
static const float RECTIFIER_VDROP_V     = 1.4f;

// ============================================================================
// 6. DISPATCH & ENGAGEMENT
// ============================================================================
static const float    DISPATCH_ADD_AIR_VALVE_PCT    = 80.0f;  // water FB position
static const float    DISPATCH_ADD_AIR_V_ERR_V      = 0.5f;
static const uint16_t DISPATCH_ADD_AIR_DURATION_MS  = 2000;
static const float    DISPATCH_DROP_AIR_PULSE_HZ    = 0.5f;
static const uint16_t DISPATCH_DROP_AIR_DURATION_MS = 5000;

// Engagement detection -- DISABLED while HAVE_WATER_HALL = false.
static const float ENGAGE_LOCK_FRAC      = 0.95f;
static const float ENGAGE_DISENGAGE_FRAC = 0.85f;
static const float ENGAGE_MISMATCH_FRAC  = 1.20f;

// ============================================================================
// 7. PID GAINS (PIDs not yet active in Stage A — values for Stage C/D)
// ============================================================================

// Water PID (V_alt -> valve %)
// Decent first-tune. Anti-windup + valve slew limit keep this stable even
// when output saturates. Aggressive enough to actually move the valve on
// step loads; not so aggressive it'll oscillate against the 10 s actuator.
static const float    PID_WATER_KP            = 15.0f;   // %/V
static const float    PID_WATER_KI            = 3.0f;    // %/(V*s)
static const float    PID_WATER_KD            = 0.0f;
static const float    PID_WATER_OUT_MIN_PCT   = 0.0f;
static const float    PID_WATER_OUT_MAX_PCT   = 100.0f;
static const float    PID_WATER_INTEGRAL_MAX  =  50.0f;
static const float    PID_WATER_INTEGRAL_MIN  = -50.0f;
static const float    PID_WATER_DEADBAND_V    = 0.1f;
static const uint16_t PID_WATER_UPDATE_MS     = 200;
static const float    WATER_VALVE_SLEW_PCT_PER_S = 10.0f;
static const float    WATER_VALVE_SLEW_BAND_PCT  = 5.0f;

// Air pulse-rate PID (V_alt -> pulse Hz)
// Decent first-tune. At KP=1.5, a 4 V error saturates the output (6 Hz clipped
// to AIR_MAX_PULSE_HZ=5). KI builds out steady-state offset over a few seconds.
static const float    PID_AIR_KP              = 1.5f;   // Hz/V
static const float    PID_AIR_KI              = 0.3f;   // Hz/(V*s)
static const float    PID_AIR_KD              = 0.0f;
static const float    PID_AIR_INTEGRAL_MAX    =  50.0f;
static const float    PID_AIR_INTEGRAL_MIN    = -50.0f;
static const float    PID_AIR_DEADBAND_V      = 0.1f;
static const float    PID_AIR_MIN_USEFUL_HZ   = 0.25f;
static const uint16_t PID_AIR_UPDATE_MS       = 200;
static const float    AIR_MAX_PULSE_HZ        = 5.0f;
static const uint16_t AIR_PULSE_WIDTH_MS      = 200;

// Closure targets
static const float WATER_VALVE_CLOSED_PCT = 0.0f;
static const float AIR_PULSE_OFF_HZ       = 0.0f;

// ============================================================================
// 8. LOOP RATES
// ============================================================================
static const uint16_t LOOP_PERIOD_MS = 10;    // 100 Hz
static const uint16_t LOG_PERIOD_MS  = 100;   // 10 Hz telemetry

// ============================================================================
// 9. EEPROM LAYOUT
// ============================================================================
static const int      EEPROM_CAL_MAGIC_ADDR  = 0;   // 2 bytes uint16
static const int      EEPROM_CAL_ATM_ADDR    = 2;   // 4 bytes float (mbar)
static const int      EEPROM_ACS_LOAD_OFFSET = 6;   // 4 bytes float (volts)
static const int      EEPROM_V_SETPOINT_CV   = 10;  // 2 bytes int16 (centivolts)
static const uint16_t EEPROM_CAL_MAGIC       = 0xCA1B;

// ============================================================================
// 10. ENUMS
// ============================================================================

enum State : uint8_t {
    S_BOOT_SELFTEST,
    S_OFF,              // master power-on, not armed; safe to walk away
    S_ARMED_IDLE,       // ARM on, no mode selected, no charge/discharge
    S_CHARGE_AIR,       // ARM + MODE_AIR + CHARGE -> compressor running
    S_CHARGE_WATER,     // ARM + MODE_WATER + CHARGE -> pump running
    S_CHARGE_BOTH,      // ARM + MODE_AIR + MODE_WATER + CHARGE -> both
    S_DISCHARGE_AIR,    // ARM + MODE_AIR + DISCHARGE -> air arm open, pulse PID
    S_DISCHARGE_WATER,  // ARM + MODE_WATER + DISCHARGE -> water valve PID
    S_DISCHARGE_BOTH,   // ARM + MODE_AIR + MODE_WATER + DISCHARGE -> dispatch
    S_SHUTDOWN,         // graceful: close valves, wait for V to drop
    S_FAULT,            // latched fault, cycle ARM to clear
    S_ESTOP,            // E-stop active, hardware also cuts
    NUM_STATES
};

// Fault flags
enum FaultBits : uint16_t {
    F_ESTOP           = 1 << 0,
    F_OVERVOLTAGE     = 1 << 1,
    F_OVERSPEED       = 1 << 2,
    F_LEAK_AIR        = 1 << 3,
    F_P2_STUCK        = 1 << 4,
    F_V_DROOP_SEVERE  = 1 << 5,
    F_I2C_TIMEOUT     = 1 << 6,
    F_SENSOR_RANGE    = 1 << 7,
    F_PI_LOST         = 1 << 8,
    F_WATCHDOG        = 1 << 9,
    F_SELFTEST        = 1 << 10,
    F_OVERPRESSURE    = 1 << 11,
    F_CAL_FAIL        = 1 << 12,
    F_PUMP_CAVITATION = 1 << 13,
};

// ============================================================================
// 11. DATA STRUCTURES
// ============================================================================

// Discrete inputs (E-stop + 5 panel switches)
struct Inputs {
    bool estop_asserted;   // d18, latched by ISR
    bool arm;              // d37, LOW = armed
    bool mode_air;         // d35
    bool mode_water;       // d33
    bool charge;           // d31 permissive
    bool discharge;        // d29 permissive
};

// Sensor readings in engineering units
struct Sensors {
    // Pressure (Fusch)
    float p1_vessel_psi;     // a13, upstream of arm solenoid
    float p2_motor_psi;      // a12, motor inlet (PID feedback channel for air)
    // Water-tank hydrostatic
    float p_hydro_mbar;      // raw MS5837 pressure
    float temp_water_C;
    float depth_cm;          // p_hydro - atm_offset, converted
    // Alt RPM (encoder)
    float rpm_alt;
    // Water bevel pinion RPM (only if HAVE_WATER_HALL)
    float rpm_water;
    // Electrical (output side)
    float v_alt_V;           // a9 -- the controlled variable
    float i_load_A;          // a10
    // Electrical (input side, for efficiency calc). Current only -- the Pi
    // derives power (V*I*PF) from these; see teststand_logger.py.
    float i_pump_A;          // a14, SCT-013 around pump mains
    float i_comp_A;          // a15, SCT-013 around compressor mains
    // Water valve position feedback (white-wire OC PWM, %)
    float water_valve_fb_pct;
    // Diagnostics
    uint8_t i2c_fail_count;
};

// Pi command snapshot
struct Command {
    int16_t  v_setpoint_cv;       // V setpoint override, centivolts
    bool     manual_override;     // if true, Pi controls actuators directly
    float    water_valve_cmd_pct; // manual override target (if manual_override)
    float    air_pulse_cmd_hz;    // manual override target (if manual_override)
    uint32_t seq;                 // sequence number for deduplication
};

// ============================================================================
// 12. GLOBALS
// ============================================================================

static State    g_state       = S_BOOT_SELFTEST;
static uint16_t g_fault_flags = 0;
static Inputs   g_in  = {};
static Sensors  g_sen = {};
static Command  g_cmd = {};

static uint32_t g_t_last_loop   = 0;
static uint32_t g_t_last_log    = 0;
static uint32_t g_t_last_pi_msg = 0;

// E-stop ISR latch (mirrors estop_arm_test_mega pattern)
static volatile bool          g_estop_isr_raw   = false;
static volatile unsigned long g_estop_isr_ms    = 0;
static volatile unsigned long g_estop_isr_count = 0;

// Alt encoder ISR counter (CPR=4)
static volatile uint32_t g_enc_count = 0;

// Optional water-side hall counter
static volatile uint32_t g_hall_water_count = 0;

// Filtered RPM (low-pass)
static float g_rpm_alt_f   = 0.0f;
static float g_rpm_water_f = 0.0f;

// MS5837 cal state
static float g_p_atm_mbar = ATM_DEFAULT_MBAR;
static bool  g_cal_valid  = false;

// ACS712 load-side zero offset (loaded from EEPROM at boot, fallback to default)
static float g_acs_load_offset_v = CURRENT_OFFSET_DC_V;

// V setpoint (live value used by future PIDs)
static int16_t g_v_setpoint_cv = V_SETPOINT_DEFAULT_CV;

// Actuator commanded values (written by Stage C/D PIDs)
static float g_water_valve_cmd_pct = 0.0f;
static float g_air_pulse_cmd_hz    = 0.0f;

// Stage C: water V-PID state
static unsigned long g_water_pid_last_ms   = 0;
static float         g_water_pid_integral  = 0.0f;
static bool          g_water_pid_first_run = true;
static float         g_water_pid_output_pct = 0.0f;

// Stage D: air pulse-rate PID state
static unsigned long g_air_pid_last_ms   = 0;
static float         g_air_pid_integral  = 0.0f;
static bool          g_air_pid_first_run = true;
static float         g_air_pid_output_hz = 0.0f;

// Stage E: dispatch state (S_DISCHARGE_BOTH only)
static bool          g_dispatch_air_engaged = false;
static unsigned long g_dispatch_add_air_t0  = 0;   // 0 = condition not currently true
static unsigned long g_dispatch_drop_air_t0 = 0;

// Stage E: leak-detection state (S_CHARGE_AIR / S_CHARGE_BOTH only)
static unsigned long g_charge_start_ms     = 0;
static unsigned long g_leak_fault_t0       = 0;
static float         g_leak_p1_window_start_psi = 0.0f;
static unsigned long g_leak_p1_window_start_ms  = 0;

// Stage E: P2-stuck detection state (air discharge only)
static unsigned long g_p2_stuck_t0         = 0;
static float         g_p2_stuck_p_at_t0    = 0.0f;

// Stage E: V-droop fault state (any discharge)
static unsigned long g_v_droop_t0          = 0;

// Stage E: SCT-013 streaming RMS state. 100 samples at 100 Hz = 1 s window
// (~60 mains cycles). Auto-detects DC bias from window mean.
struct RmsState {
    int16_t samples[100];
    int32_t sum_raw;
    uint8_t idx;
    bool    warmed_up;
};
static RmsState g_rms_pump = {};
static RmsState g_rms_comp = {};

// Per-switch debouncer state. Defined here (before the first function) so the
// Arduino auto-generated prototype for debounce_switch() can see the type.
struct SwDebounce {
    uint8_t       last_raw;
    uint8_t       stable;
    unsigned long edge_ms;
};

// Air pulser timing (lifted from mosfet_fusch_pid_uno)
static unsigned long g_air_period_ms       = 0;
static unsigned long g_air_on_ms           = 0;
static unsigned long g_air_period_start_ms = 0;
static bool          g_air_pulser_on       = false;

// Serial RX buffer
static const uint8_t RX_BUF_SIZE = 160;
static char    g_rx_buf[RX_BUF_SIZE];
static uint8_t g_rx_len = 0;

// Output state (for telemetry / debug)
static uint8_t g_outputs_bitmap = 0;

// ============================================================================
// 13. ISRs (keep tiny -- flag + counter only)
// ============================================================================

// E-stop: capture raw level + timestamp; debounce in loop.
void isr_estop() {
    g_estop_isr_raw    = (digitalRead(PIN_ESTOP) == HIGH);
    g_estop_isr_ms     = millis();
    g_estop_isr_count++;
}

// Alt encoder channel A. CPR=4; for direction we'd also read B, but RPM works on A alone.
void isr_alt_encoder() {
    g_enc_count++;
}

// Water-side bevel pinion hall (only attached if HAVE_WATER_HALL).
void isr_hall_water() {
    g_hall_water_count++;
}

// ============================================================================
// 14. LOW-LEVEL HELPERS
// ============================================================================

static inline void write_pin(uint8_t pin, bool on) {
    digitalWrite(pin, on ? HIGH : LOW);
}

// Emergency stop: drive every actuator pin to safe state immediately.
// Called from E-stop, FAULT, SHUTDOWN, and at boot.
void immediate_cutoff_all() {
    write_pin(PIN_MOSFET_COMP_CONT, false);
    write_pin(PIN_MOSFET_PUMP_CONT, false);
    write_pin(PIN_MOSFET_AIR_PULSE, false);
    write_pin(PIN_MOSFET_AIR_ARM,   false);
    analogWrite(PIN_WATER_VALVE_PWM, 0);   // close water valve (PWM 0 -> 0V command)
    // Stop the pulser so servicePulser() doesn't re-energize d41 on next tick.
    g_air_period_ms       = 0;
    g_air_on_ms           = 0;
    g_air_pulser_on       = false;
    g_water_valve_cmd_pct = 0.0f;
    g_air_pulse_cmd_hz    = 0.0f;
}

// Median-of-3: rejects single-sample ADC spikes from EMI.
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static uint16_t analog_median3(uint8_t pin) {
    return median3(analogRead(pin), analogRead(pin), analogRead(pin));
}

static float counts_to_volts(uint16_t c) {
    return (c * ADC_REF_V) / (float)ADC_COUNTS;
}

// Convert Fusch ADC reading to psi using per-sensor FS + noise floor.
// (Lifted from pressure_fusch_*_uno math.)
static float raw_to_psi_fusch(uint16_t c, float fs_psi, float noise_floor) {
    float v_pin = counts_to_volts(c);
    float v_span = PRESSURE_V_MAX - PRESSURE_V_MIN;
    float psi    = (v_pin - PRESSURE_V_MIN) * (fs_psi / v_span);
    if (psi > -noise_floor && psi < noise_floor) psi = 0.0f;
    return psi;
}

// ACS712 (DC hall) reading -> amps. Uses EEPROM-loaded zero offset.
static float raw_to_amps_dc(uint16_t c, float gain, float offset_v) {
    return (counts_to_volts(c) - offset_v) / gain;
}

// SCT-013 (AC clamp) reading -> amps. AC-coupled; offset = 0.
// >>> NOTE: this is a placeholder rectified-peak read. Proper RMS needs
// >>>       windowed sampling; lift from sct013_030_test_uno in Stage E.
static float raw_to_amps_ac(uint16_t c, float gain) {
    float v = counts_to_volts(c);
    return v / gain;   // crude; replace with RMS calc in Stage E
}

// Low-pass filter
static float lowpass(float prev, float x, float a) {
    return prev + a * (x - prev);
}

// Water valve OC-PWM feedback -> position % (from prop_valve_test_uno cal).
// pulseIn() blocks up to timeout; called once per main loop.
static float read_water_valve_fb_pct() {
    const unsigned long TIMEOUT_US = 50000UL;   // 50 ms (covers down to ~20 Hz)
    unsigned long high_us = pulseIn(PIN_WATER_VALVE_FB, HIGH, TIMEOUT_US);
    if (high_us == 0) return -1.0f;             // no pulse / valve absent
    unsigned long low_us  = pulseIn(PIN_WATER_VALVE_FB, LOW, TIMEOUT_US);
    if (low_us  == 0) return -1.0f;
    unsigned long period = high_us + low_us;
    if (period == 0) return -1.0f;
    float duty_pct = 100.0f * (float)high_us / (float)period;
    float span = FB_DUTY_OPEN_PCT - FB_DUTY_CLOSED_PCT;
    if (span <= 0.0f) return -1.0f;
    float pos = (duty_pct - FB_DUTY_CLOSED_PCT) * 100.0f / span;
    if (pos < 0.0f)   pos = 0.0f;
    if (pos > 100.0f) pos = 100.0f;
    return pos;
}

// ============================================================================
// 15. ACTUATOR LAYER (write-only; PIDs in Stage C/D produce the values)
// ============================================================================

// Water valve: 0..100 % -> analogWrite duty 0..255.
// The PWM-to-0-10V converter linearizes for us; valve characteristic non-linearity
// is a control-side concern (see hardware_locked.md notes on ball-valve Cv).
static void setWaterValvePct(float pct) {
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    g_water_valve_cmd_pct = pct;
    uint8_t duty = (uint8_t)(pct * 2.55f + 0.5f);
    analogWrite(PIN_WATER_VALVE_PWM, duty);
}

// Air pulse rate setter. Recomputes period + on_ms; pulser drives the gate.
// Lifted from mosfet_fusch_pid_uno::applyRate().
static void setAirPulseHz(float hz) {
    if (hz < 0.0f) hz = 0.0f;
    if (hz > AIR_MAX_PULSE_HZ) hz = AIR_MAX_PULSE_HZ;
    g_air_pulse_cmd_hz = hz;
    if (hz <= 0.0f) {
        g_air_period_ms = 0;
        g_air_on_ms     = 0;
        write_pin(PIN_MOSFET_AIR_PULSE, false);
        g_air_pulser_on = false;
        return;
    }
    bool was_zero = (g_air_period_ms == 0);
    g_air_period_ms = (unsigned long)(1000.0f / hz + 0.5f);
    g_air_on_ms     = (AIR_PULSE_WIDTH_MS >= g_air_period_ms)
                      ? g_air_period_ms : (unsigned long)AIR_PULSE_WIDTH_MS;
    if (was_zero) g_air_period_start_ms = millis();
}

// Pulser service: called every loop iteration; drives the MOSFET gate based on
// (period, on_ms, period_start). Single-acting: opens for on_ms, closes the rest
// of each period. Lifted from mosfet_fusch_pid_uno::servicePulser().
static void serviceAirPulser() {
    if (g_air_period_ms == 0) {
        if (g_air_pulser_on) {
            write_pin(PIN_MOSFET_AIR_PULSE, false);
            g_air_pulser_on = false;
        }
        return;
    }
    if (g_air_on_ms >= g_air_period_ms) {
        if (!g_air_pulser_on) {
            write_pin(PIN_MOSFET_AIR_PULSE, true);
            g_air_pulser_on = true;
        }
        return;
    }
    unsigned long elapsed = millis() - g_air_period_start_ms;
    if (elapsed >= g_air_period_ms) {
        g_air_period_start_ms += g_air_period_ms;
        elapsed = millis() - g_air_period_start_ms;
    }
    bool want_on = (elapsed < g_air_on_ms);
    if (want_on != g_air_pulser_on) {
        write_pin(PIN_MOSFET_AIR_PULSE, want_on);
        g_air_pulser_on = want_on;
    }
}

static void setAirArmSolenoid(bool open) { write_pin(PIN_MOSFET_AIR_ARM,   open); }
static void setCompressor   (bool on)    { write_pin(PIN_MOSFET_COMP_CONT, on);   }
static void setPump         (bool on)    { write_pin(PIN_MOSFET_PUMP_CONT, on);   }

// ============================================================================
// 16. MS5837 DRIVER (model = -02BA, must call setModel explicitly)
// ============================================================================

static MS5837 g_ms5837;
static bool   g_ms5837_ok = false;

bool ms5837_init() {
    if (!g_ms5837.init()) return false;
    g_ms5837.setModel(MS5837::MS5837_02BA);   // library does NOT autodetect
    g_ms5837.setFluidDensity(FLUID_DENSITY_KG_M3);
    g_ms5837_ok = true;
    return true;
}

bool ms5837_read(float &mbar, float &temp_c) {
    if (!g_ms5837_ok) return false;
    g_ms5837.read();
    mbar  = g_ms5837.pressure();
    temp_c = g_ms5837.temperature();
    if (mbar < 0.0f || mbar > 2000.0f) return false;   // sanity
    return true;
}

// Depth (cm) from raw pressure, using EEPROM atm offset (or fallback).
static float depth_cm_calibrated(float p_mbar) {
    float atm = g_cal_valid ? g_p_atm_mbar : ATM_DEFAULT_MBAR;
    float head_pa = (p_mbar - atm) * 100.0f;             // mbar -> Pa
    return head_pa / (FLUID_DENSITY_KG_M3 * GRAVITY_M_S2) * 100.0f;
}

// ============================================================================
// 17. EEPROM (atm offset, ACS load zero, V setpoint default)
// ============================================================================

static bool eeprom_has_valid_cal() {
    uint16_t magic = 0;
    EEPROM.get(EEPROM_CAL_MAGIC_ADDR, magic);
    return (magic == EEPROM_CAL_MAGIC);
}

static void eeprom_load_all() {
    if (!eeprom_has_valid_cal()) return;
    float atm;
    EEPROM.get(EEPROM_CAL_ATM_ADDR, atm);
    if (atm >= 300.0f && atm <= 1200.0f) {
        g_p_atm_mbar = atm;
        g_cal_valid  = true;
    }
    float acs;
    EEPROM.get(EEPROM_ACS_LOAD_OFFSET, acs);
    if (acs >= 2.0f && acs <= 3.0f) {            // ACS712 spec 2.5 +/- 25 mV
        g_acs_load_offset_v = acs;
    }
    int16_t vsp;
    EEPROM.get(EEPROM_V_SETPOINT_CV, vsp);
    if (vsp >= 100 && vsp <= 2500) {
        g_v_setpoint_cv = vsp;
    }
}

// Pi can update V setpoint at runtime; persist so next boot picks it up.
static void eeprom_write_v_setpoint(int16_t cv) {
    EEPROM.put(EEPROM_V_SETPOINT_CV, cv);
    EEPROM.put(EEPROM_CAL_MAGIC_ADDR, EEPROM_CAL_MAGIC);   // ensure magic is set
}

// ============================================================================
// 18. SENSOR SAMPLING (100 Hz)
// ============================================================================

void sample_sensors() {
    // --- Pressure (Fusch P1 + P2) ---
    g_sen.p1_vessel_psi = raw_to_psi_fusch(analog_median3(PIN_P1_VESSEL), P1_FS_PSI, P1_NOISE_FLOOR);
    g_sen.p2_motor_psi  = raw_to_psi_fusch(analog_median3(PIN_P2_MOTOR),  P2_FS_PSI, P2_NOISE_FLOOR);

    // --- V_alt (the controlled variable) ---
    g_sen.v_alt_V = counts_to_volts(analog_median3(PIN_V_ALT)) * ALT_V_DIV_RATIO;

    // --- I_load (ACS712-20A, post-rectifier) ---
    g_sen.i_load_A = raw_to_amps_dc(analog_median3(PIN_I_LOAD), CURRENT_GAIN_LOAD, g_acs_load_offset_v);
    if (g_sen.i_load_A < 0.0f) g_sen.i_load_A = 0.0f;   // unidirectional

    // --- I_pump, I_comp (SCT-013, AC clamps, input-energy side) ---
    // Stage E: streaming RMS. Single sample per 100 Hz loop iteration into a
    // 100-sample (1 second) window; RMS computed over the window each pass.
    update_rms(g_rms_pump, PIN_I_PUMP);
    update_rms(g_rms_comp, PIN_I_COMP);
    float i_pump = rms_to_amps(g_rms_pump, CURRENT_GAIN_PUMP);
    float i_comp = rms_to_amps(g_rms_comp, CURRENT_GAIN_COMP);
    // Deadband the RMS noise floor so an OFF load reads true zero (keeps the
    // phantom out of the transmitted current and the Pi's energy integral).
    if (i_pump < SCT_NOISE_FLOOR_A) i_pump = 0.0f;
    if (i_comp < SCT_NOISE_FLOOR_A) i_comp = 0.0f;
    g_sen.i_pump_A = i_pump;
    g_sen.i_comp_A = i_comp;

    // --- MS5837 (hydrostatic, depth) ---
    // Skipped entirely on an air-only bench (HAVE_DEPTH_SENSOR=false) so a
    // disconnected sensor can't climb i2c_fail_count into F_I2C_TIMEOUT.
    if (HAVE_DEPTH_SENSOR) {
        float mbar, temp_c;
        if (ms5837_read(mbar, temp_c)) {
            g_sen.p_hydro_mbar    = mbar;
            g_sen.temp_water_C    = temp_c;
            g_sen.depth_cm        = depth_cm_calibrated(mbar);
            g_sen.i2c_fail_count  = 0;
        } else if (g_sen.i2c_fail_count < 255) {
            g_sen.i2c_fail_count++;
        }
    }

    // --- Alt RPM (encoder, CPR=4) ---
    noInterrupts();
    uint32_t enc = g_enc_count;   g_enc_count = 0;
    uint32_t hw  = g_hall_water_count; g_hall_water_count = 0;
    interrupts();
    float rpm_alt_inst = ((float)enc * 60000.0f) / ((float)ALT_ENCODER_CPR * LOOP_PERIOD_MS);
    g_rpm_alt_f = lowpass(g_rpm_alt_f, rpm_alt_inst, 0.3f);
    g_sen.rpm_alt = g_rpm_alt_f;

    if (HAVE_WATER_HALL) {
        float rpm_w_inst = ((float)hw * 60000.0f) / ((float)PULSES_PER_REV_WATER * LOOP_PERIOD_MS);
        g_rpm_water_f = lowpass(g_rpm_water_f, rpm_w_inst, 0.3f);
        g_sen.rpm_water = g_rpm_water_f;
    } else {
        g_sen.rpm_water = 0.0f;
    }

    // --- Water valve feedback (only meaningful when valve is wired) ---
    g_sen.water_valve_fb_pct = read_water_valve_fb_pct();   // -1 if absent
}

// ============================================================================
// 19. DISCRETE INPUT READ (E-stop debounced from ISR, switches polled)
// ============================================================================

// Per-switch debouncer state (struct SwDebounce defined up in the types section).
static SwDebounce g_db_arm = {HIGH, HIGH, 0};
static SwDebounce g_db_mode_air = {HIGH, HIGH, 0};
static SwDebounce g_db_mode_water = {HIGH, HIGH, 0};
static SwDebounce g_db_charge = {HIGH, HIGH, 0};
static SwDebounce g_db_discharge = {HIGH, HIGH, 0};
static SwDebounce g_db_estop = {HIGH, HIGH, 0};

// Returns the debounced raw level (HIGH or LOW).
static uint8_t debounce_switch(uint8_t pin, SwDebounce &s, unsigned long now) {
    uint8_t raw = digitalRead(pin);
    if (raw != s.last_raw) {
        s.last_raw = raw;
        s.edge_ms  = now;
    } else if ((now - s.edge_ms) >= SWITCH_DEBOUNCE_MS) {
        s.stable = raw;
    }
    return s.stable;
}

void read_discrete_inputs() {
    unsigned long now = millis();

    // Switches: closed = LOW = ON.
    g_in.arm        = (debounce_switch(PIN_SW_ARM,        g_db_arm,        now) == LOW);
    g_in.mode_air   = (debounce_switch(PIN_SW_MODE_AIR,   g_db_mode_air,   now) == LOW);
    g_in.mode_water = (debounce_switch(PIN_SW_MODE_WATER, g_db_mode_water, now) == LOW);
    g_in.charge     = (debounce_switch(PIN_SW_CHARGE,     g_db_charge,     now) == LOW);
    g_in.discharge  = (debounce_switch(PIN_SW_DISCHARGE,  g_db_discharge,  now) == LOW);

    // E-stop: ISR latches raw level + timestamp; debounce in loop.
    noInterrupts();
    bool          raw_es  = g_estop_isr_raw;
    unsigned long es_edge = g_estop_isr_ms;
    interrupts();
    static bool          es_stable    = false;
    static unsigned long es_last_edge = 0;
    if (raw_es != es_stable) {
        if (es_edge != es_last_edge) es_last_edge = es_edge;
        if ((now - es_last_edge) >= SWITCH_DEBOUNCE_MS) {
            es_stable = raw_es;
        }
    }
    // E-stop ACTIVE when pin is HIGH (NC contact open or wire cut -- fail-safe).
    g_in.estop_asserted = es_stable;
}

// ============================================================================
// 20. FAULT CHECKING
// ============================================================================

void check_fault_conditions() {
    uint16_t tr = 0;

    if (g_in.estop_asserted)                                          tr |= F_ESTOP;
    if (g_sen.v_alt_V > V_OVERVOLTAGE_HARD_V)                         tr |= F_OVERVOLTAGE;
    if (g_sen.rpm_alt > ALT_RPM_FAULT)                                tr |= F_OVERSPEED;
    if (g_sen.p1_vessel_psi > P_VESSEL_MAX_PSI)                       tr |= F_OVERPRESSURE;
    if (g_sen.p1_vessel_psi < P_SANITY_LO_PSI ||
        g_sen.p1_vessel_psi > P_SANITY_HI_PSI)                        tr |= F_SENSOR_RANGE;
    if (g_sen.i2c_fail_count > 3)                                     tr |= F_I2C_TIMEOUT;

    bool energized = (g_state == S_CHARGE_AIR  || g_state == S_CHARGE_WATER  ||
                      g_state == S_CHARGE_BOTH || g_state == S_DISCHARGE_AIR ||
                      g_state == S_DISCHARGE_WATER || g_state == S_DISCHARGE_BOTH);
    // Pi heartbeat: only relevant if Pi has ever spoken AND we're energized.
    if (energized && g_t_last_pi_msg != 0 &&
        (millis() - g_t_last_pi_msg) > PI_HEARTBEAT_TIMEOUT_MS)       tr |= F_PI_LOST;

    // Sustained-soft overvoltage fault
    static unsigned long ov_soft_t0 = 0;
    if (g_sen.v_alt_V > V_OVERVOLTAGE_SOFT_V) {
        if (ov_soft_t0 == 0) ov_soft_t0 = millis();
        if ((millis() - ov_soft_t0) >= V_OVERVOLTAGE_SOFT_DURATION_MS) tr |= F_OVERVOLTAGE;
    } else {
        ov_soft_t0 = 0;
    }

    // --- F_PUMP_CAVITATION: pump commanded + lower-tank depth below NPSH ---
    const bool pump_on = (g_state == S_CHARGE_WATER || g_state == S_CHARGE_BOTH);
    if (HAVE_DEPTH_SENSOR && pump_on && g_sen.depth_cm < P_CAVITATION_DEPTH_CM_MIN) {
        tr |= F_PUMP_CAVITATION;
    }

    // --- F_LEAK_AIR: P1 not rising during charge (compound condition) ---
    // (charge_age > grace) AND (P1 < gate * target) AND (dP/dt < min) sustained.
    // Windowed dP/dt: sample P1 every 500 ms.
    const bool charging_air = (g_state == S_CHARGE_AIR || g_state == S_CHARGE_BOTH);
    if (charging_air) {
        const unsigned long now_ms = millis();
        if (g_charge_start_ms == 0) g_charge_start_ms = now_ms;
        if (g_leak_p1_window_start_ms == 0) {
            g_leak_p1_window_start_ms  = now_ms;
            g_leak_p1_window_start_psi = g_sen.p1_vessel_psi;
        }
        const unsigned long window_ms = now_ms - g_leak_p1_window_start_ms;
        if (window_ms >= 500) {
            const float dpdt = (g_sen.p1_vessel_psi - g_leak_p1_window_start_psi)
                             / (window_ms * 0.001f);
            const float gate_psi = LEAK_P_GATE_FRAC * P_VESSEL_CHARGE_TARGET_PSI;
            const unsigned long charge_age = now_ms - g_charge_start_ms;
            const bool stalled = (charge_age > LEAK_CHARGE_GRACE_MS) &&
                                 (g_sen.p1_vessel_psi < gate_psi) &&
                                 (dpdt < LEAK_MIN_PSI_PER_S);
            if (stalled) {
                if (g_leak_fault_t0 == 0) g_leak_fault_t0 = now_ms;
                if ((now_ms - g_leak_fault_t0) >= LEAK_FAULT_DURATION_MS) tr |= F_LEAK_AIR;
            } else {
                g_leak_fault_t0 = 0;
            }
            g_leak_p1_window_start_ms  = now_ms;
            g_leak_p1_window_start_psi = g_sen.p1_vessel_psi;
        }
    } else {
        g_charge_start_ms          = 0;
        g_leak_fault_t0            = 0;
        g_leak_p1_window_start_ms  = 0;
    }

    // --- F_P2_STUCK: air pulsing for window, P2 didn't move ---
    const bool air_pulsing = (g_air_pulse_cmd_hz >= P2_STUCK_MIN_PULSE_HZ) &&
                             ((g_state == S_DISCHARGE_AIR) ||
                              (g_state == S_DISCHARGE_BOTH && g_dispatch_air_engaged));
    if (air_pulsing) {
        const unsigned long now_ms = millis();
        if (g_p2_stuck_t0 == 0) {
            g_p2_stuck_t0      = now_ms;
            g_p2_stuck_p_at_t0 = g_sen.p2_motor_psi;
        }
        if ((now_ms - g_p2_stuck_t0) >= P2_STUCK_WINDOW_MS) {
            const float delta = fabs(g_sen.p2_motor_psi - g_p2_stuck_p_at_t0);
            if (delta < P2_STUCK_PRESSURE_DELTA_PSI) tr |= F_P2_STUCK;
            // Restart window for next check
            g_p2_stuck_t0      = now_ms;
            g_p2_stuck_p_at_t0 = g_sen.p2_motor_psi;
        }
    } else {
        g_p2_stuck_t0 = 0;
    }

    // --- F_V_DROOP_SEVERE: V below 70% setpoint, control authority exhausted ---
    const bool discharging = (g_state == S_DISCHARGE_AIR  ||
                              g_state == S_DISCHARGE_WATER ||
                              g_state == S_DISCHARGE_BOTH);
    if (discharging) {
        const float setpoint_v = (float)g_v_setpoint_cv * 0.01f;
        const float droop_v    = setpoint_v * V_DROOP_FAULT_FRAC;
        // "Saturated" means the active actuator(s) are at the rail.
        const bool water_sat = (g_water_pid_output_pct >= PID_WATER_OUT_MAX_PCT * WATER_SAT_FRAC);
        const bool air_sat   = (g_air_pid_output_hz   >= AIR_MAX_PULSE_HZ      * AIR_SAT_FRAC);
        bool saturated = false;
        switch (g_state) {
            case S_DISCHARGE_WATER: saturated = water_sat; break;
            case S_DISCHARGE_AIR:   saturated = air_sat;   break;
            case S_DISCHARGE_BOTH:  saturated = water_sat && (!g_dispatch_air_engaged || air_sat); break;
            default: break;
        }
        if (g_sen.v_alt_V < droop_v && saturated) {
            if (g_v_droop_t0 == 0) g_v_droop_t0 = millis();
            if ((millis() - g_v_droop_t0) >= V_DROOP_FAULT_DURATION_MS) tr |= F_V_DROOP_SEVERE;
        } else {
            g_v_droop_t0 = 0;
        }
    } else {
        g_v_droop_t0 = 0;
    }

    // Generic debounce
    static uint32_t ts = 0;
    if (tr) {
        if (!ts) ts = millis();
        if ((millis() - ts) >= FAULT_DEBOUNCE_MS) g_fault_flags |= tr;
    } else {
        ts = 0;
    }
}

// Cycle ARM (off then on) is the reset gesture -- no dedicated reset input.
// Edge-detected here: if ARM transitions LOW->HIGH->LOW with faults present
// and root cause cleared, clear the latched faults.
static bool g_arm_prev_state = false;
static uint8_t g_arm_cycle_phase = 0;  // 0=idle, 1=saw_off
void check_arm_reset_gesture() {
    if (!g_arm_prev_state && g_in.arm) {
        // off -> on
        if (g_arm_cycle_phase == 1) {
            // Clear faults whose root cause is gone
            uint16_t still = 0;
            if (g_in.estop_asserted)                       still |= F_ESTOP;
            if (g_sen.v_alt_V > V_OVERVOLTAGE_HARD_V)      still |= F_OVERVOLTAGE;
            if (g_sen.p1_vessel_psi > P_VESSEL_MAX_PSI)    still |= F_OVERPRESSURE;
            if (still == 0) g_fault_flags = 0;
        }
        g_arm_cycle_phase = 0;
    } else if (g_arm_prev_state && !g_in.arm) {
        // on -> off
        g_arm_cycle_phase = 1;
    }
    g_arm_prev_state = g_in.arm;
}

// ============================================================================
// 21. STATE MACHINE
// ============================================================================
// Switch-driven mode selection. Switches are PERMISSIVE: CHARGE/DISCHARGE
// only fire when ARM + correct MODE_* is also on and no faults are active.

State next_state(State cur) {
    if (g_in.estop_asserted) return S_ESTOP;

    if (g_fault_flags && cur != S_ESTOP) return S_FAULT;

    if (!g_in.arm) {
        // Not armed -- only OFF, FAULT, ESTOP, SHUTDOWN are valid
        if (cur != S_OFF && cur != S_FAULT && cur != S_ESTOP && cur != S_SHUTDOWN)
            return S_OFF;
        return cur;
    }

    // Armed. Decide based on MODE + CHARGE/DISCHARGE.
    bool air = g_in.mode_air;
    bool wat = g_in.mode_water;
    bool ch  = g_in.charge;
    bool dis = g_in.discharge;

    // CHARGE and DISCHARGE both on simultaneously = configuration error.
    // Fall back to idle (firmware refuses ambiguous request).
    if (ch && dis) {
        return S_ARMED_IDLE;
    }

    if (ch) {
        if (air && wat) return S_CHARGE_BOTH;
        if (air)        return S_CHARGE_AIR;
        if (wat)        return S_CHARGE_WATER;
        return S_ARMED_IDLE;  // CHARGE on but no MODE selected
    }

    if (dis) {
        if (air && wat) return S_DISCHARGE_BOTH;
        if (air)        return S_DISCHARGE_AIR;
        if (wat)        return S_DISCHARGE_WATER;
        return S_ARMED_IDLE;
    }

    // ARM on, no CHARGE or DISCHARGE -- idle
    return S_ARMED_IDLE;
}

// Per-state entry actions (LEDs, latched outputs).
void on_enter(State s) {
    // Default LED state per entry; specific cases override below.
    write_pin(PIN_LED_RED,    false);
    write_pin(PIN_LED_GREEN,  false);
    write_pin(PIN_LED_YELLOW, false);
    write_pin(PIN_LED_BLUE,   false);
    write_pin(PIN_LED_WHITE,  false);
    write_pin(PIN_PIEZO,      false);

    switch (s) {
    case S_BOOT_SELFTEST:
        immediate_cutoff_all();
        write_pin(PIN_LED_YELLOW, true);
        break;
    case S_OFF:
        immediate_cutoff_all();
        break;
    case S_ARMED_IDLE:
        immediate_cutoff_all();
        write_pin(PIN_LED_GREEN, true);
        break;
    case S_CHARGE_AIR:
    case S_CHARGE_WATER:
    case S_CHARGE_BOTH:
        write_pin(PIN_LED_GREEN, true);
        write_pin(PIN_LED_WHITE, true);
        break;
    case S_DISCHARGE_AIR:
    case S_DISCHARGE_WATER:
    case S_DISCHARGE_BOTH:
        write_pin(PIN_LED_GREEN, true);
        write_pin(PIN_LED_BLUE,  true);
        break;
    case S_SHUTDOWN:
        immediate_cutoff_all();
        write_pin(PIN_LED_YELLOW, true);
        break;
    case S_FAULT:
        immediate_cutoff_all();
        write_pin(PIN_LED_RED, true);
        write_pin(PIN_PIEZO,   true);
        break;
    case S_ESTOP:
        immediate_cutoff_all();
        write_pin(PIN_LED_RED, true);
        write_pin(PIN_PIEZO,   true);
        break;
    default:
        break;
    }
}

// ============================================================================
// 21b. WATER V-PID  (Stage C)
// ============================================================================
// Closed-loop PI on V_alt -> water valve %. Slow loop: 5 Hz update, well
// below the valve's ~7-10 s full-travel transit. Anti-windup uses the
// white-wire feedback to detect "valve in transit" and freezes the integrator
// during it. D-term stays 0 (constant-rate actuator makes D useless).
//
// Active in S_DISCHARGE_WATER and S_DISCHARGE_BOTH (unless Pi has set the
// manual-override flag). Inactive states reset the integrator and first-run
// flag so re-entry starts cleanly.

void servicePidWater(bool active) {
    const unsigned long now = millis();

    if (!active) {
        g_water_pid_integral   = 0.0f;
        g_water_pid_first_run  = true;
        g_water_pid_last_ms    = now;
        g_water_pid_output_pct = 0.0f;
        return;
    }

    if (g_water_pid_first_run) {
        g_water_pid_integral   = 0.0f;
        g_water_pid_output_pct = 0.0f;   // start closed; PID opens it as needed
        g_water_pid_first_run  = false;
        g_water_pid_last_ms    = now;
        return;
    }

    // Rate-limited to PID_WATER_UPDATE_MS (200 ms = 5 Hz)
    if ((now - g_water_pid_last_ms) < PID_WATER_UPDATE_MS) return;
    const float dt = (now - g_water_pid_last_ms) / 1000.0f;
    g_water_pid_last_ms = now;
    if (dt <= 0.0f) return;

    // Setpoint comes from g_v_setpoint_cv (Pi-overridable, EEPROM-backed).
    const float setpoint_v = (float)g_v_setpoint_cv * 0.01f;
    float error_v = setpoint_v - g_sen.v_alt_V;

    // Deadband: small errors don't move the valve, prevents micro-jitter
    if (fabs(error_v) < PID_WATER_DEADBAND_V) error_v = 0.0f;

    // Anti-windup: if valve commanded position differs from feedback by more
    // than the slew band, the valve is in transit -- freeze the integrator
    // so V_err doesn't accumulate during the 7-10 s ramp. Skip the check if
    // feedback is unavailable (returns -1 from read_water_valve_fb_pct).
    bool freeze_for_transit = false;
    if (g_sen.water_valve_fb_pct >= 0.0f) {
        const float delta = fabs(g_water_valve_cmd_pct - g_sen.water_valve_fb_pct);
        if (delta > WATER_VALVE_SLEW_BAND_PCT) freeze_for_transit = true;
    }

    // Tentative integral with hard clamp
    float candidate_i = g_water_pid_integral + error_v * dt;
    if (candidate_i > PID_WATER_INTEGRAL_MAX) candidate_i = PID_WATER_INTEGRAL_MAX;
    if (candidate_i < PID_WATER_INTEGRAL_MIN) candidate_i = PID_WATER_INTEGRAL_MIN;

    // PID output (D-term stays 0)
    const float unsat_cmd = PID_WATER_KP * error_v + PID_WATER_KI * candidate_i;

    // Conditional integration: if output would saturate AND error is driving
    // it further into saturation, freeze. (Mirror of mosfet_fusch_pid_uno.)
    const bool saturating =
        (unsat_cmd > PID_WATER_OUT_MAX_PCT && error_v > 0.0f) ||
        (unsat_cmd < PID_WATER_OUT_MIN_PCT && error_v < 0.0f);

    if (!freeze_for_transit && !saturating) {
        g_water_pid_integral = candidate_i;
    }

    // Final command using the (possibly not-updated) integral
    float cmd = PID_WATER_KP * error_v + PID_WATER_KI * g_water_pid_integral;
    if (cmd < PID_WATER_OUT_MIN_PCT) cmd = PID_WATER_OUT_MIN_PCT;
    if (cmd > PID_WATER_OUT_MAX_PCT) cmd = PID_WATER_OUT_MAX_PCT;
    g_water_pid_output_pct = cmd;
}

// ============================================================================
// 21d. DISPATCH  (Stage E)
// ============================================================================
// In S_DISCHARGE_BOTH, water is primary; air joins only when water saturates
// AND V_err is still positive for a sustained window. Air disengages when its
// pulse rate falls below DISPATCH_DROP_AIR_PULSE_HZ for a wider window
// (asymmetric: faster to add, slower to drop, prevents flapping).
//
// Uses water valve FEEDBACK position when available -- at 7-10 s transit time,
// "commanded > 80%" can be true while the valve is still in motion and isn't
// actually delivering. Falls back to commanded position only when feedback is
// absent.

void serviceDispatch() {
    if (g_state != S_DISCHARGE_BOTH) {
        g_dispatch_air_engaged = false;
        g_dispatch_add_air_t0  = 0;
        g_dispatch_drop_air_t0 = 0;
        return;
    }

    const unsigned long now = millis();
    const float setpoint_v = (float)g_v_setpoint_cv * 0.01f;
    const float v_err = setpoint_v - g_sen.v_alt_V;

    if (!g_dispatch_air_engaged) {
        // Add-air check
        const bool fb_ok = (g_sen.water_valve_fb_pct >= 0.0f);
        const float effective_pos = fb_ok ? g_sen.water_valve_fb_pct
                                          : g_water_valve_cmd_pct;
        const bool water_saturated = (effective_pos > DISPATCH_ADD_AIR_VALVE_PCT);
        const bool v_too_low = (v_err > DISPATCH_ADD_AIR_V_ERR_V);
        if (water_saturated && v_too_low) {
            if (g_dispatch_add_air_t0 == 0) g_dispatch_add_air_t0 = now;
            if ((now - g_dispatch_add_air_t0) >= DISPATCH_ADD_AIR_DURATION_MS) {
                g_dispatch_air_engaged = true;
                g_dispatch_add_air_t0  = 0;
            }
        } else {
            g_dispatch_add_air_t0 = 0;
        }
    } else {
        // Drop-air check (commanded pulse Hz instead of feedback -- air pulse
        // updates at 5 Hz so command is effectively current)
        if (g_air_pid_output_hz < DISPATCH_DROP_AIR_PULSE_HZ) {
            if (g_dispatch_drop_air_t0 == 0) g_dispatch_drop_air_t0 = now;
            if ((now - g_dispatch_drop_air_t0) >= DISPATCH_DROP_AIR_DURATION_MS) {
                g_dispatch_air_engaged = false;
                g_dispatch_drop_air_t0 = 0;
            }
        } else {
            g_dispatch_drop_air_t0 = 0;
        }
    }
}

// ============================================================================
// 21e. SCT-013 STREAMING RMS  (Stage E)
// ============================================================================
// Updates a circular buffer of 100 raw ADC samples per channel and computes
// true RMS over the window each time current is needed. Auto-removes DC bias
// (whatever bias circuit is on the SCT-013 burden) by subtracting the running
// mean.
//
// Called once per 100 Hz loop iteration from sample_sensors().

static void update_rms(RmsState &r, uint8_t pin) {
    const int16_t raw = (int16_t)analogRead(pin);
    r.sum_raw -= r.samples[r.idx];
    r.samples[r.idx] = raw;
    r.sum_raw += raw;
    r.idx++;
    if (r.idx >= 100) { r.idx = 0; r.warmed_up = true; }
}

static float rms_to_amps(const RmsState &r, float gain) {
    if (!r.warmed_up) return 0.0f;
    const int16_t mean = (int16_t)(r.sum_raw / 100);
    int32_t sum_sq = 0;
    for (uint8_t i = 0; i < 100; i++) {
        int32_t dev = (int32_t)r.samples[i] - (int32_t)mean;
        sum_sq += dev * dev;
    }
    const float rms_counts = sqrtf((float)sum_sq / 100.0f);
    const float rms_volts  = rms_counts * ADC_REF_V / (float)ADC_COUNTS;
    return rms_volts / gain;
}

// ============================================================================
// 21c. AIR PULSE-RATE PID  (Stage D)
// ============================================================================
// Closed-loop PI on V_alt -> air pulse Hz. Structure lifted from
// mosfet_fusch_pid_uno; error variable swapped from psi to V_alt. Output
// is pulse frequency clamped to [0, AIR_MAX_PULSE_HZ]. Below
// PID_AIR_MIN_USEFUL_HZ the controller forces 0 (don't dribble pulses).
//
// Active in S_DISCHARGE_AIR and S_DISCHARGE_BOTH (unless manual override).
// In S_DISCHARGE_BOTH both PIDs run in parallel without dispatch awareness;
// Stage E adds the water-saturated-then-air-joins logic on top.

void servicePidAir(bool active) {
    const unsigned long now = millis();

    if (!active) {
        g_air_pid_integral   = 0.0f;
        g_air_pid_first_run  = true;
        g_air_pid_last_ms    = now;
        g_air_pid_output_hz  = 0.0f;
        return;
    }

    if (g_air_pid_first_run) {
        g_air_pid_integral   = 0.0f;
        g_air_pid_output_hz  = 0.0f;
        g_air_pid_first_run  = false;
        g_air_pid_last_ms    = now;
        return;
    }

    if ((now - g_air_pid_last_ms) < PID_AIR_UPDATE_MS) return;
    const float dt = (now - g_air_pid_last_ms) / 1000.0f;
    g_air_pid_last_ms = now;
    if (dt <= 0.0f) return;

    const float setpoint_v = (float)g_v_setpoint_cv * 0.01f;
    float error_v = setpoint_v - g_sen.v_alt_V;

    if (fabs(error_v) < PID_AIR_DEADBAND_V) error_v = 0.0f;

    // Tentative integral with hard clamp
    float candidate_i = g_air_pid_integral + error_v * dt;
    if (candidate_i > PID_AIR_INTEGRAL_MAX) candidate_i = PID_AIR_INTEGRAL_MAX;
    if (candidate_i < PID_AIR_INTEGRAL_MIN) candidate_i = PID_AIR_INTEGRAL_MIN;

    const float unsat_cmd = PID_AIR_KP * error_v + PID_AIR_KI * candidate_i;

    // Conditional integration -- mosfet_fusch_pid_uno pattern
    const bool saturating =
        (unsat_cmd > AIR_MAX_PULSE_HZ && error_v > 0.0f) ||
        (unsat_cmd < 0.0f             && error_v < 0.0f);

    if (!saturating) {
        g_air_pid_integral = candidate_i;
    }

    // Final output
    float cmd = PID_AIR_KP * error_v + PID_AIR_KI * g_air_pid_integral;
    if (cmd < 0.0f)             cmd = 0.0f;
    if (cmd > AIR_MAX_PULSE_HZ) cmd = AIR_MAX_PULSE_HZ;
    if (cmd < PID_AIR_MIN_USEFUL_HZ) cmd = 0.0f;   // don't dribble
    g_air_pid_output_hz = cmd;
}

// ============================================================================
// 22. OUTPUT COMMIT (single place actuator pins are written each loop)
// ============================================================================
// PID outputs are computed in servicePidWater() and servicePidAir();
// commit_outputs reads those values + state + interlocks and writes pins.

void commit_outputs() {
    // Default: everything off
    bool comp = false, pump = false, air_arm = false;
    float water_pct = 0.0f;
    float air_hz    = 0.0f;

    switch (g_state) {
    case S_CHARGE_AIR:    comp = true;                            break;
    case S_CHARGE_WATER:  pump = true;                            break;
    case S_CHARGE_BOTH:   comp = true; pump = true;               break;

    case S_DISCHARGE_AIR:
        air_arm = true;
        air_hz  = g_air_pid_output_hz;
        break;

    case S_DISCHARGE_WATER:
        water_pct = g_water_pid_output_pct;
        break;

    case S_DISCHARGE_BOTH:
        water_pct = g_water_pid_output_pct;
        // Dispatch: air arm + pulse only when serviceDispatch() decides air is needed.
        air_arm   = g_dispatch_air_engaged;
        air_hz    = g_dispatch_air_engaged ? g_air_pid_output_hz : 0.0f;
        break;

    default:
        // OFF / IDLE / SHUTDOWN / FAULT / ESTOP / BOOT -- all off
        break;
    }

    // --- Hard interlocks (these override everything) ---

    // Overpressure: kill charging immediately (safety, not a target)
    if (g_sen.p1_vessel_psi > P_VESSEL_MAX_PSI) {
        comp = false;
        pump = false;
    }

    // Charge-complete auto-stops. These are NOT faults -- the system has
    // simply reached the desired charge level. Operator should flip CHARGE
    // off when this happens; if they don't, the comp/pump just stays idle
    // and resumes if the level drops back below threshold (no hysteresis).
    if (g_sen.p1_vessel_psi >= P_VESSEL_CHARGE_TARGET_PSI) {
        comp = false;  // air vessel reached target
    }
    if (HAVE_DEPTH_SENSOR && g_sen.depth_cm <= P_TANK_CHARGED_DEPTH_CM) {
        pump = false;  // lower tank drained = upper tank full
    }

    // Soft overvoltage: drive V-regulating actuators closed fast
    if (g_sen.v_alt_V > V_OVERVOLTAGE_SOFT_V) {
        water_pct = 0.0f;
        air_hz    = 0.0f;
    }

    // ARM off: kill everything (defensive -- state machine should already be OFF)
    if (!g_in.arm) {
        comp = false; pump = false; air_arm = false;
        water_pct = 0.0f; air_hz = 0.0f;
    }

    // --- Pi manual override (only honored in normal energized states) ---
    if (g_cmd.manual_override &&
        (g_state == S_DISCHARGE_AIR || g_state == S_DISCHARGE_WATER || g_state == S_DISCHARGE_BOTH)) {
        water_pct = g_cmd.water_valve_cmd_pct;
        air_hz    = g_cmd.air_pulse_cmd_hz;
    }

    // --- Write outputs ---
    setCompressor(comp);
    setPump(pump);
    setAirArmSolenoid(air_arm);
    setWaterValvePct(water_pct);
    setAirPulseHz(air_hz);

    // Bitmap for telemetry
    uint8_t b = 0;
    if (comp)    b |= (1 << 0);
    if (pump)    b |= (1 << 1);
    if (air_arm) b |= (1 << 2);
    if (water_pct > 0.5f) b |= (1 << 3);
    if (air_hz    > 0.01f) b |= (1 << 4);
    g_outputs_bitmap = b;
}

// ============================================================================
// 23. SERIAL PROTOCOL (line-delimited JSON, both directions)
// ============================================================================
// Pi commands: {"seq":N,"vsp":CV,"man":0/1,"wvp":PCT,"aph":HZ}
//   seq: monotonic sequence number (dedup)
//   vsp: V setpoint in centivolts (e.g. 800 = 8.00 V)
//   man: 1 = Pi manually controls water/air actuators (overrides PIDs)
//   wvp: water valve % (only used if man=1)
//   aph: air pulse Hz (only used if man=1)
// Telemetry every 100 ms.

static bool jnum(const char *src, const char *key, long &out) {
    const char *p = strstr(src, key); if (!p) return false;
    p = strchr(p, ':'); if (!p) return false; p++;
    while (*p == ' ') p++;
    out = atol(p);
    return true;
}

void handle_cmd(const char *line) {
    long v;
    if (jnum(line, "\"seq\"", v)) {
        if ((uint32_t)v <= g_cmd.seq) return;   // stale/duplicate
        g_cmd.seq = (uint32_t)v;
    }
    if (jnum(line, "\"vsp\"", v)) {
        if (v >= 100 && v <= 2500) {
            g_cmd.v_setpoint_cv = (int16_t)v;
            g_v_setpoint_cv = (int16_t)v;
            eeprom_write_v_setpoint((int16_t)v);
        }
    }
    if (jnum(line, "\"man\"", v)) g_cmd.manual_override = (v != 0);
    if (jnum(line, "\"wvp\"", v)) g_cmd.water_valve_cmd_pct = (float)v;
    if (jnum(line, "\"aph\"", v)) g_cmd.air_pulse_cmd_hz    = (float)v * 0.01f;  // cHz -> Hz

    g_t_last_pi_msg = millis();
}

void process_serial_rx() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_rx_len > 0) {
                g_rx_buf[g_rx_len] = 0;
                handle_cmd(g_rx_buf);
                g_rx_len = 0;
            }
        } else if (g_rx_len < RX_BUF_SIZE - 1) {
            g_rx_buf[g_rx_len++] = c;
        } else {
            g_rx_len = 0;   // overflow -- drop the line
        }
    }
}

void send_telemetry() {
    char buf[320];
    snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"st\":%u,\"f\":%u,\"o\":%u,"
        "\"va\":%ld,\"il\":%ld,"
        "\"p1\":%ld,\"p2\":%ld,"
        "\"ph\":%ld,\"tw\":%ld,\"d\":%ld,"
        "\"ra\":%ld,\"rw\":%ld,"
        "\"ip\":%ld,\"ic\":%ld,"
        "\"wvp\":%ld,\"wfb\":%ld,\"aph\":%ld,"
        "\"es\":%u,\"arm\":%u,\"ma\":%u,\"mw\":%u,\"ch\":%u,\"di\":%u,"
        "\"vsp\":%d}",
        (unsigned long)millis(), (unsigned)g_state, (unsigned)g_fault_flags, (unsigned)g_outputs_bitmap,
        (long)(g_sen.v_alt_V * 100), (long)(g_sen.i_load_A * 1000),
        (long)(g_sen.p1_vessel_psi * 100), (long)(g_sen.p2_motor_psi * 100),
        (long)(g_sen.p_hydro_mbar * 10), (long)(g_sen.temp_water_C * 10), (long)(g_sen.depth_cm * 10),
        (long)(g_sen.rpm_alt * 10), (long)(g_sen.rpm_water * 10),
        (long)(g_sen.i_pump_A * 1000), (long)(g_sen.i_comp_A * 1000),
        (long)(g_water_valve_cmd_pct * 10), (long)(g_sen.water_valve_fb_pct * 10), (long)(g_air_pulse_cmd_hz * 100),
        (unsigned)(g_in.estop_asserted ? 1 : 0),
        (unsigned)(g_in.arm ? 1 : 0),
        (unsigned)(g_in.mode_air ? 1 : 0), (unsigned)(g_in.mode_water ? 1 : 0),
        (unsigned)(g_in.charge ? 1 : 0), (unsigned)(g_in.discharge ? 1 : 0),
        (int)g_v_setpoint_cv
    );
    Serial.println(buf);
}

// ============================================================================
// 24. SELF-TEST (runs once at boot)
// ============================================================================

bool run_selftest() {
    sample_sensors();
    read_discrete_inputs();

    if (g_in.estop_asserted) return false;
    if (g_sen.p1_vessel_psi < P_SANITY_LO_PSI ||
        g_sen.p1_vessel_psi > P_SANITY_HI_PSI) return false;

    // Depth sensor is part of selftest only when it's supposed to be present.
    if (HAVE_DEPTH_SENSOR) {
        float m, t;
        if (!ms5837_read(m, t)) return false;
    }

    // Visual ack: green LED blink x3
    for (uint8_t i = 0; i < 3; i++) {
        write_pin(PIN_LED_GREEN, true);  delay(80);
        write_pin(PIN_LED_GREEN, false); delay(80);
    }
    return true;
}

// ============================================================================
// 25. SETUP
// ============================================================================

void setup() {
    // Output pins
    uint8_t outs[] = {
        PIN_LED_RED, PIN_LED_GREEN, PIN_LED_YELLOW, PIN_LED_BLUE, PIN_LED_WHITE,
        PIN_PIEZO,
        PIN_MOSFET_COMP_CONT, PIN_MOSFET_PUMP_CONT,
        PIN_MOSFET_AIR_PULSE, PIN_MOSFET_AIR_ARM,
        PIN_WATER_VALVE_PWM
    };
    for (uint8_t i = 0; i < sizeof(outs); i++) {
        pinMode(outs[i], OUTPUT);
    }
    immediate_cutoff_all();

    // Input pins (INPUT_PULLUP everywhere -- fail-safe)
    pinMode(PIN_ESTOP,         INPUT_PULLUP);
    pinMode(PIN_SW_ARM,        INPUT_PULLUP);
    pinMode(PIN_SW_MODE_AIR,   INPUT_PULLUP);
    pinMode(PIN_SW_MODE_WATER, INPUT_PULLUP);
    pinMode(PIN_SW_CHARGE,     INPUT_PULLUP);
    pinMode(PIN_SW_DISCHARGE,  INPUT_PULLUP);
    pinMode(PIN_ALT_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ALT_ENCODER_B, INPUT_PULLUP);
    pinMode(PIN_WATER_VALVE_FB, INPUT);   // external 10k pull-up on the wire

    if (HAVE_WATER_HALL) {
        pinMode(PIN_HALL_WATER, INPUT_PULLUP);
    }

    // Seed E-stop ISR state from current pin level so boot doesn't latch a phantom press
    g_estop_isr_raw = (digitalRead(PIN_ESTOP) == HIGH);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP),         isr_estop,        CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ALT_ENCODER_A), isr_alt_encoder,  RISING);
    if (HAVE_WATER_HALL) {
        attachInterrupt(digitalPinToInterrupt(PIN_HALL_WATER), isr_hall_water, RISING);
    }

    // Serial + I2C
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);

    // EEPROM load (atm offset, ACS zero, V setpoint default)
    eeprom_load_all();

    // MS5837 init (must call setModel_02BA -- done inside ms5837_init).
    // Skipped on an air-only bench so a missing sensor doesn't stall boot.
    if (HAVE_DEPTH_SENSOR) ms5837_init();

    // Self-test (BEFORE wdt_enable -- selftest's visual-ack blink takes
    // ~480 ms when it passes, and we don't want a 250 ms watchdog killing
    // the AVR mid-blink. Watchdog exists to catch main-loop hangs, not to
    // bound boot time.)
    g_state = S_BOOT_SELFTEST;
    on_enter(g_state);
    if (!run_selftest()) {
        g_fault_flags |= F_SELFTEST;
        g_state = S_FAULT;
        on_enter(g_state);
    } else {
        g_state = S_OFF;
        on_enter(g_state);
    }

    // Watchdog arms only after selftest completes.
    wdt_enable(WDTO_250MS);

    g_t_last_loop   = millis();
    g_t_last_log    = millis();
    g_t_last_pi_msg = 0;   // Pi hasn't spoken yet
}

// ============================================================================
// 26. MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = millis();

    // 100 Hz control loop
    if ((now - g_t_last_loop) >= LOOP_PERIOD_MS) {
        g_t_last_loop = now;

        sample_sensors();
        read_discrete_inputs();
        process_serial_rx();

        check_fault_conditions();
        check_arm_reset_gesture();

        State next = next_state(g_state);
        if (next != g_state) {
            g_state = next;
            on_enter(g_state);
        }

        // Stage E dispatch decides whether air joins in S_DISCHARGE_BOTH.
        serviceDispatch();

        // PID computation. Air PID only runs in S_DISCHARGE_BOTH when dispatch
        // has engaged it; otherwise air PID resets (so re-engagement starts
        // from clean integrator).
        const bool water_pid_active =
            !g_cmd.manual_override &&
            (g_state == S_DISCHARGE_WATER || g_state == S_DISCHARGE_BOTH);
        const bool air_pid_active =
            !g_cmd.manual_override &&
            ((g_state == S_DISCHARGE_AIR) ||
             (g_state == S_DISCHARGE_BOTH && g_dispatch_air_engaged));
        servicePidWater(water_pid_active);
        servicePidAir(air_pid_active);

        commit_outputs();

        wdt_reset();
    }

    // Air pulser must run every iteration, not just on 100 Hz tick, because
    // pulse on/off transitions can be inside a 10 ms window when Hz is high.
    serviceAirPulser();

    // 10 Hz telemetry
    if ((now - g_t_last_log) >= LOG_PERIOD_MS) {
        g_t_last_log = now;
        send_telemetry();
    }
}
