/**
 * TestStandFirmware.ino
 * -------------------------------------------------------------------------
 * Water / Air Impulse-Wheel + Pneumatic-Motor Test Stand
 * Primary controller: Arduino Mega 2560
 * Supervisor / UI / logger: Raspberry Pi 5 (USB-serial, line-delimited JSON)
 *
 * How it works (big picture):
 *   - The Mega runs a loop 100 times per second (100 Hz)
 *   - Every loop: read all sensors, check for faults, step the state machine,
 *     write outputs, and feed the watchdog
 *   - Every 10th loop (10 Hz): send a JSON telemetry packet to the Pi
 *   - The Pi runs the operator UI, logs everything to CSV, and sends
 *     commands back (like "start charging" or "discharge now")
 *   - The Mega OWNS safety — the Pi can request stuff but the Mega
 *     can refuse, go to FAULT, or trip E-STOP at any time
 *
 * Rev 1 — added calibration flow, 3 RPM sensors, clamp CTs,
 *         alternator V/I, variable valve position, expanded telemetry
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>   // hardware watchdog — resets the Mega if our loop ever freezes
#include <EEPROM.h>    // saves calibration offset so it survives power cycles
#include "MS5837.h"    // BlueRobotics MS5837 library for hydrostatic pressure sensor

// ========================================================================
// 1. CONFIGURATION — this is where most of my changes will happen
//    Everything that might change per build/test goes here at the top
//    so I don't have to hunt through 600 lines to tweak a number.
// ========================================================================

// --- Timing ---
// These control how fast things run. Probably fine as-is unless I notice
// the loop is too slow (check by toggling a pin and measuring on a scope).
static const uint16_t LOOP_PERIOD_MS          = 10;    // 100 Hz main loop
static const uint16_t LOG_PERIOD_MS           = 100;   // 10 Hz telemetry to Pi
static const uint16_t FAULT_DEBOUNCE_MS       = 50;    // fault must persist 50ms before tripping
static const uint16_t PI_HEARTBEAT_TIMEOUT_MS = 2000;  // if Pi goes silent for 2s during a live test, shut down

// ========================================================================
// PIN ASSIGNMENTS
// ========================================================================
// >>> VERIFY: every single pin here against the actual wiring before
// >>> powering on for the first time. Mismatched pins = fried MOSFET
// >>> or worse, a solenoid opening when it shouldn't.

// --- Digital outputs (MOSFET gate drivers) ---
// Each MOSFET drives a contactor coil or solenoid.
// HIGH = MOSFET on = contactor/solenoid energized.
// >>> DECIDE: are these active-HIGH or active-LOW for my MOSFET drivers?
// >>>         If active-LOW, I need to flip the logic in write_pin().
static const uint8_t PIN_COMPRESSOR_CONT  = 22;  // MOSFET → contactor coil → compressor runs → charges air vessel
static const uint8_t PIN_PUMP_CONT        = 23;  // MOSFET → contactor coil → pump runs → charges water side
static const uint8_t PIN_DISCH_SOL_AIR    = 24;  // solenoid valve — opens air discharge to wheel
static const uint8_t PIN_DISCH_SOL_WATER  = 25;  // solenoid valve — opens water discharge to wheel
static const uint8_t PIN_STATUS_LED_GREEN = 13;  // onboard LED works, easy for testing
static const uint8_t PIN_STATUS_LED_RED   = 12;
static const uint8_t PIN_BUZZER           = 11;  // audible alarm on fault/estop

// --- Servo / variable valve PWM ---
// These drive servos (or stepper drivers) that control variable flow valves.
// >>> DECIDE: am I using actual hobby servos? or stepper + driver?
// >>>         If steppers, I'll need step/dir pins instead of PWM.
// >>>         The Servo library uses these as PWM outputs.
static const uint8_t PIN_VALVE_SERVO_AIR    = 9;   // needs to be PWM-capable pin
static const uint8_t PIN_VALVE_SERVO_WATER  = 10;  // needs to be PWM-capable pin

// --- Digital inputs ---
// All wired with INPUT_PULLUP — the switch connects the pin to GND.
// So: switch pressed = pin reads LOW = true.
// >>> VERIFY: E-stop MUST be on an interrupt pin (only 2,3,18,19,20,21 on Mega)
// >>> VERIFY: all 3 hall sensors MUST be on interrupt pins too
static const uint8_t PIN_ESTOP           = 2;   // INT0 — E-stop, active-LOW, highest priority
static const uint8_t PIN_HALL_RPM_AIR    = 3;   // INT1 — hall sensor on air-side wheel
static const uint8_t PIN_HALL_RPM_WATER  = 18;  // INT5 — hall sensor on water-side wheel
static const uint8_t PIN_HALL_RPM_ALT    = 19;  // INT4 — hall sensor on alternator shaft
// These two DON'T need interrupts — we just poll them each loop.
// Moved arm switch off pin 3 to free up the interrupt for hall air.
static const uint8_t PIN_ARM_SWITCH      = 30;  // key switch / arming toggle
static const uint8_t PIN_MANUAL_RESET    = 31;  // pushbutton to clear faults

// --- Analog inputs ---
// All read 0–5V, mapped to 0–1023 by the Mega's 10-bit ADC.
// >>> VERIFY: each analog pin matches the physical wire to that sensor.
static const uint8_t PIN_PRESSURE_XDUCR  = A0;  // main line pressure transducer
static const uint8_t PIN_CURRENT_12V     = A1;  // inline current sensor on 12V bus (ACS712 or similar)
static const uint8_t PIN_CURRENT_5V      = A2;  // inline current sensor on 5V logic rail
static const uint8_t PIN_BUS_VOLTAGE     = A3;  // 12V bus through a voltage divider
static const uint8_t PIN_ALT_VOLTAGE     = A4;  // alternator output through a voltage divider
static const uint8_t PIN_ALT_CURRENT     = A5;  // alternator output current sensor
static const uint8_t PIN_CURRENT_COMP    = A6;  // clamp-on CT around compressor wire
static const uint8_t PIN_CURRENT_PUMP    = A7;  // clamp-on CT around pump wire
static const uint8_t PIN_VALVE_ENC_AIR   = A8;  // encoder/pot feedback from air valve (if wired)
static const uint8_t PIN_VALVE_ENC_WATER = A9;  // encoder/pot feedback from water valve (if wired)

// --- I2C address ---
// MS5837-02BA pressure sensor. Only one device on the I2C bus right now.
// SDA = pin 20, SCL = pin 21 (hardwired on Mega, can't change these).
// >>> IMPORTANT: MS5837-02BA is a 3.3V device. Mega is 5V.
// >>> I NEED a level shifter between them or the sensor will fry.

// ========================================================================
// SAFETY THRESHOLDS
// ========================================================================
// These are the numbers that decide when the system trips a fault.
// Getting these wrong = either nuisance trips (too tight) or actual
// danger (too loose). Measure on the bench before trusting these.

// >>> DECIDE: what's the actual max safe pressure for my tank/plumbing?
// >>>         120 psi is a guess. Check the rating of the weakest component
// >>>         in the pressure path (tank, fittings, hose, valves).
// >>>         Set this to ~80% of the burst/rated pressure.
static const float P_MAX_PSI          = 120.0f;

// >>> DECIDE: what pressure do I want to charge to before discharging?
// >>>         This depends on my test plan. Start low (like 40 psi) for
// >>>         first tests and work up.
static const float P_CHARGE_TARGET    = 100.0f;

// >>> This is the pressure below which we consider the system "vented" / safe.
// >>> 5 psi is basically atmospheric + noise. Probably fine.
static const float P_SAFE_VENT        = 5.0f;

// Sanity range — if the pressure sensor reads outside this range,
// something is physically wrong (sensor disconnected, wiring fault, etc.)
// >>> CHECK: does my pressure transducer output 0.5V at 0 psi?
// >>>        If so, a reading of -2 psi means the ADC is reading below 0.5V
// >>>        which would indicate a wiring issue.
static const float P_SANITY_LO        = -2.0f;
static const float P_SANITY_HI        = 150.0f;

// >>> DECIDE: what's the actual minimum voltage my system works at?
// >>>         11V is typical for a 12V lead-acid under load.
// >>>         If I'm using a bench supply, maybe 11.5V is safer.
static const float V_MIN_BUS          = 11.0f;

// >>> DECIDE: what's the max current my 12V rail should ever draw?
// >>>         Add up all the solenoid coil currents + contactor + margin.
// >>>         8A is a guess — need to measure actual draw with everything on.
static const float I_MAX_12V          = 8.0f;

// >>> This is for the 5V logic rail. If logic current exceeds 2A,
// >>> something is very wrong (short circuit on a sensor, etc.)
static const float I_MAX_5V           = 2.0f;

// ========================================================================
// SENSOR SCALING
// ========================================================================
// These convert raw ADC counts (0–1023) into engineering units.
// Every sensor has different scaling — check the datasheet for each one.

static const float ADC_REF_V          = 5.0f;   // Mega ADC reference voltage (don't change)
static const uint16_t ADC_COUNTS      = 1023;   // 10-bit ADC max (don't change)

// --- Pressure transducer ---
// Most industrial pressure transducers output 0.5V at zero and 4.5V at full scale.
// >>> CHECK: look at my specific transducer's datasheet.
// >>>        Hook it up, measure voltage at 0 psi with a multimeter.
// >>>        That's my actual V_MIN. Adjust if it's not exactly 0.5V.
static const float PRESSURE_V_MIN     = 0.5f;   // >>> MEASURE THIS on bench
static const float PRESSURE_V_MAX     = 4.5f;   // >>> MEASURE THIS on bench (or from datasheet)
// >>> DECIDE: what's the full-scale range of my pressure transducer?
// >>>         Common values: 100, 150, 200, 300 psi. Check the part number.
static const float PRESSURE_FS_PSI    = 150.0f;

// --- Voltage dividers ---
// The 12V bus can't go directly into the Mega's 5V ADC input, so I use a
// resistor divider. The ratio is (R1+R2)/R2.
// Example: 10k + 3.3k divider → ratio = 4.03 → rounds to 4.0
// >>> CALCULATE: based on the actual resistors I'm using.
// >>>           Measure V_bus with a multimeter and compare to what the
// >>>           code says. Adjust ratio until they match.
static const float BUS_DIV_RATIO      = 4.0f;   // >>> CALCULATE from my actual resistor values

// The alternator can output higher voltage than 12V depending on RPM/load.
// >>> DECIDE: what's the max alternator voltage I expect?
// >>>         My divider needs to bring that down to under 5V.
// >>>         If max alt V = 50V and I want 5V max at ADC:
// >>>         ratio = 50/5 = 10. Use 9k + 1k divider = ratio 10.
static const float ALT_V_DIV_RATIO    = 10.0f;  // >>> CALCULATE from my actual resistor values

// --- Current sensors ---
// ACS712 outputs 2.5V at 0A and changes by "sensitivity" volts per amp.
// ACS712-5A:  sensitivity = 0.185 V/A
// ACS712-20A: sensitivity = 0.100 V/A
// ACS712-30A: sensitivity = 0.066 V/A
// >>> DECIDE: which ACS712 variant am I using for each sensor?
// >>>         Probably 5A for logic rail, maybe 20A for 12V bus.
static const float CURRENT_GAIN_12V   = 0.185f;  // >>> CHECK: ACS712 variant for 12V bus
static const float CURRENT_GAIN_5V    = 0.185f;  // >>> CHECK: ACS712 variant for 5V rail
static const float CURRENT_GAIN_ALT   = 0.185f;  // >>> CHECK: what sensor am I using for alternator output?
// Clamp-on CTs have wildly different sensitivities depending on the model.
// SCT-013-000: outputs current (needs burden resistor), very different math
// SCT-013-030: outputs voltage, ~1V at 30A → gain ≈ 0.033 V/A
// Hall-effect clamp: check specific datasheet
// >>> DECIDE: what exact clamp sensors am I using? This number is critical.
static const float CURRENT_GAIN_COMP  = 0.050f;  // >>> PLACEHOLDER — get from clamp CT datasheet
static const float CURRENT_GAIN_PUMP  = 0.050f;  // >>> PLACEHOLDER — get from clamp CT datasheet

// ACS712 zero-current output voltage (should be Vcc/2 = 2.5V)
// >>> MEASURE: with nothing connected, what voltage does each current sensor output?
// >>>          It's supposed to be 2.5V but can drift ±0.025V. Measure and adjust.
static const float CURRENT_OFFSET_V   = 2.5f;    // >>> MEASURE on bench for each sensor

// Clamp CTs might have a different offset.
// AC clamp CTs: offset is 0V (AC-coupled, centered on zero)
// DC hall clamps: usually Vcc/2 like ACS712
// >>> DECIDE: what type of clamp sensor am I using? AC or DC?
// >>>         AC type: set this to 0.0f and use RMS calculation instead
// >>>         DC type: set this to 2.5f (same as ACS712)
static const float CLAMP_OFFSET_V     = 2.5f;    // >>> DEPENDS on clamp type — see above

// --- RPM ---
// How many magnet pulses per revolution of each shaft.
// Depends on how many magnets are on my hall sensor wheel/ring.
// 1 magnet = 1 pulse/rev, 2 magnets = 2 pulses/rev, etc.
// >>> COUNT: how many magnets did I put on each shaft?
static const uint8_t PULSES_PER_REV_AIR   = 2;   // >>> VERIFY: magnets on air wheel
static const uint8_t PULSES_PER_REV_WATER = 2;   // >>> VERIFY: magnets on water wheel
static const uint8_t PULSES_PER_REV_ALT   = 2;   // >>> VERIFY: magnets on alternator shaft

// --- Valve servo ---
// Standard hobby servo pulse range. Most servos work with 544–2400 µs.
// Some cheaper ones only go 1000–2000 µs. If the servo buzzes or
// doesn't reach full travel, narrow these.
// >>> CHECK: test each servo manually first. Send 544µs and 2400µs
// >>>        and see if it actually hits 0° and 180° without straining.
static const int VALVE_SERVO_MIN_US = 544;    // pulse width for 0°
static const int VALVE_SERVO_MAX_US = 2400;   // pulse width for 180°

// Set these to true if I actually wire encoder/potentiometer feedback
// on the valve servos. If false, the code still works — it just won't
// know the actual position (only the commanded position).
// >>> DECIDE: am I adding encoders to the valve servos?
static const bool VALVE_ENCODER_PRESENT_AIR   = false;
static const bool VALVE_ENCODER_PRESENT_WATER = false;

// --- MS5837 calibration storage ---
// These EEPROM addresses store the atmospheric pressure offset so it
// survives power cycles. The magic number 0xCA1B is just a tag that
// says "yes, there's valid calibration data here" vs uninitialized memory.
// Don't change these unless I run out of EEPROM space (which I won't).
static const int EEPROM_CAL_MAGIC_ADDR = 0;
static const int EEPROM_CAL_ATM_ADDR   = 2;
static const uint16_t EEPROM_CAL_MAGIC = 0xCA1B;

// ========================================================================
// 2. STATE MACHINE STATES
// ========================================================================
// These are all the modes the system can be in.
// The order matters — state codes get sent to the Pi as numbers (0, 1, 2...)
// so if I rearrange these, the Pi side needs to match.

enum State : uint8_t {
    S_BOOT_SELFTEST,    // 0  — checks sensors, voltage, E-stop on power-up
    S_CAL_PROMPT,       // 1  — waiting for user to start or skip calibration
    S_CAL_ZEROING,      // 2  — reading atmospheric pressure (sensor must be in air)
    S_CAL_REPLACE,      // 3  — waiting for user to put sensor back in water
    S_OFF,              // 4  — system idle, everything off, safe to walk away
    S_ARM,              // 5  — arm switch on, contactor engaged, ready for commands
    S_CHARGE,           // 6  — actively pressurizing (compressor/pump running)
    S_READY,            // 7  — charged to target, waiting for discharge command
    S_DISCHARGE_AIR,    // 8  — air valve open, wheel spinning
    S_DISCHARGE_WATER,  // 9  — water valve open, wheel spinning
    S_DISCHARGE_BOTH,   // 10 — both valves open
    S_MANUAL_DIAG,      // 11 — manual control for testing individual outputs
    S_SHUTDOWN,         // 12 — venting pressure, spinning down, returning to safe
    S_FAULT,            // 13 — something bad happened, all outputs off, buzzer on
    S_ESTOP,            // 14 — E-stop pressed, hardware relay also cuts power
    NUM_STATES
};

// Commands the Pi can send us
enum ReqMode : uint8_t {
    REQ_NONE = 0,
    REQ_ARM, REQ_CHARGE, REQ_DISARM,
    REQ_DISCHARGE_AIR, REQ_DISCHARGE_WATER, REQ_DISCHARGE_BOTH,
    REQ_STOP, REQ_ABORT,
    REQ_MANUAL, REQ_EXIT_MANUAL,
    REQ_CAL_ZERO, REQ_CAL_DONE, REQ_CAL_SKIP,
};

// Fault flags — each bit represents one type of problem.
// Multiple faults can be active at the same time (that's why it's a bitfield).
enum FaultBits : uint16_t {
    F_ESTOP         = 1 << 0,   // E-stop button pressed
    F_OVERPRESSURE  = 1 << 1,   // line pressure exceeded P_MAX_PSI
    F_UNDERVOLTAGE  = 1 << 2,   // bus voltage dropped below V_MIN_BUS
    F_OVERCURRENT   = 1 << 3,   // current exceeded I_MAX on either rail
    F_I2C_TIMEOUT   = 1 << 4,   // MS5837 stopped responding
    F_SENSOR_RANGE  = 1 << 5,   // pressure reading outside sanity bounds
    F_PI_LOST       = 1 << 6,   // Pi stopped sending heartbeats during active test
    F_WATCHDOG      = 1 << 7,   // (set by hardware if loop freezes > 250ms)
    F_SELFTEST      = 1 << 8,   // self-test on boot failed
    F_ILLEGAL_TRANS = 1 << 9,   // state machine tried to make an invalid jump
    F_CAL_FAIL      = 1 << 10,  // calibration reading was out of range
};

// ========================================================================
// 3. DATA STRUCTURES
// ========================================================================

// What we read from physical switches
struct Inputs {
    bool estop_asserted;        // true = E-stop pressed (BAD)
    bool arm_switch_on;         // true = arm key turned on
    bool manual_reset_pressed;  // true = operator pressing reset button
};

// What we read from all sensors (converted to engineering units)
struct Sensors {
    float p_line_psi;       // main line pressure
    float p_hydro_mbar;     // MS5837 absolute pressure (raw, before level calc)
    float temp_water_C;     // MS5837 temperature (free bonus from the sensor)
    float rpm_air;          // air-side wheel speed
    float rpm_water;        // water-side wheel speed
    float rpm_alt;          // alternator shaft speed
    float i_12v_A;          // 12V bus current draw
    float i_5v_A;           // 5V logic current draw
    float v_bus_V;          // 12V supply voltage
    float i_comp_A;         // compressor motor current (clamp CT)
    float i_pump_A;         // pump motor current (clamp CT)
    float v_alt_V;          // alternator output voltage
    float i_alt_A;          // alternator output current
    uint8_t i2c_fail_count; // how many consecutive I2C read failures
};

// Commands from the Pi (parsed from JSON)
struct Command {
    uint8_t  requested_mode;      // what the Pi wants us to do
    bool     manual_reset;        // Pi requesting fault clear
    bool     charge_air;          // include air side in charge?
    bool     charge_water;        // include water side in charge?
    uint8_t  manual_io_mask;      // which outputs to control in MANUAL mode
    uint8_t  manual_io_value;     // on/off values for those outputs
    int16_t  valve_pos_air_cmd;   // servo angle for air variable valve (-1 = no change)
    int16_t  valve_pos_water_cmd; // servo angle for water variable valve (-1 = no change)
    uint32_t seq;                 // sequence number — prevents processing stale/duplicate cmds
};

// ========================================================================
// 4. GLOBALS
// ========================================================================
// Prefixed with g_ so I can tell at a glance "this is global state" vs
// a local variable in a function.

static State    g_state       = S_BOOT_SELFTEST;  // start here on power-up
static uint16_t g_fault_flags = 0;                // no faults initially
static Inputs   g_in  = {};   // zero-initialized — all false
static Sensors  g_sen = {};   // zero-initialized — all 0.0
static Command  g_cmd = {};   // zero-initialized — REQ_NONE

// Timing — tracks when we last did each periodic task
static uint32_t g_t_last_loop = 0;
static uint32_t g_t_last_log  = 0;
static uint32_t g_t_last_pi_msg = 0;  // last time Pi sent us anything

// What outputs are currently being driven (bitmap)
static uint8_t g_outputs_bitmap = 0;

// Hall sensor pulse counters — incremented in ISRs, read in main loop.
// volatile because they're modified in interrupts.
static volatile uint16_t g_hall_cnt_air = 0;
static volatile uint16_t g_hall_cnt_water = 0;
static volatile uint16_t g_hall_cnt_alt = 0;

// Filtered RPM values (low-pass filtered to reduce noise)
static float g_rpm_air_f = 0;
static float g_rpm_water_f = 0;
static float g_rpm_alt_f = 0;

// Valve position tracking
static int16_t g_valve_pos_air_cmd = 0;    // last commanded angle
static int16_t g_valve_pos_water_cmd = 0;
static int16_t g_valve_pos_air_act = -1;   // actual measured angle (-1 = no encoder)
static int16_t g_valve_pos_water_act = -1;

// MS5837 calibration state
static float   g_p_atm_mbar      = 1013.25f;  // atmospheric offset (default: standard atmosphere)
static bool    g_cal_valid        = false;      // have we calibrated yet?
static bool    g_cal_skipped      = false;      // did the user skip calibration?
static uint8_t g_cal_sample_count = 0;          // how many cal readings we've taken so far
static float   g_cal_accumulator  = 0.0f;       // running sum of cal readings
static const uint8_t CAL_NUM_SAMPLES = 50;      // average 50 readings (~5 seconds at 10ms/loop)

// Serial receive buffer for Pi commands
static const uint8_t RX_BUF_SIZE = 160;
static char    g_rx_buf[RX_BUF_SIZE];
static uint8_t g_rx_len = 0;

// ========================================================================
// 5. INTERRUPT SERVICE ROUTINES (ISRs)
// ========================================================================
// These fire instantly when a hall sensor pulse or E-stop event happens.
// Keep them TINY — no Serial prints, no float math, no delays.
// Just increment a counter or set a flag and get out.

void isr_hall_air()   { g_hall_cnt_air++;   }
void isr_hall_water() { g_hall_cnt_water++; }
void isr_hall_alt()   { g_hall_cnt_alt++;   }

// E-stop ISR — the hardware relay also physically cuts actuator power,
// but we latch the fault in software too so the state machine knows about it.
void isr_estop() {
    immediate_cutoff_all();      // kill all outputs NOW
    g_fault_flags |= F_ESTOP;   // latch the fault
}

// ========================================================================
// 6. LOW-LEVEL HELPERS
// ========================================================================

static inline void write_pin(uint8_t pin, bool on) {
    // >>> NOTE: if my MOSFETs are active-LOW (some driver boards are),
    // >>>       change HIGH/LOW here and everything else stays the same.
    digitalWrite(pin, on ? HIGH : LOW);
}

// Emergency shutdown — called from E-stop ISR and whenever we enter FAULT.
// This is the "oh shit" function. Every actuator goes off immediately.
void immediate_cutoff_all() {
    digitalWrite(PIN_COMPRESSOR_CONT, LOW);
    digitalWrite(PIN_PUMP_CONT, LOW);
    digitalWrite(PIN_DISCH_SOL_AIR, LOW);
    digitalWrite(PIN_DISCH_SOL_WATER, LOW);
    g_outputs_bitmap = 0;
}

// Takes 3 values, returns the middle one. Used to reject single-sample
// ADC spikes (EMI from solenoids loves to corrupt analog reads).
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a>b) { uint16_t t=a; a=b; b=t; }
    if (b>c) { uint16_t t=b; b=c; c=t; }
    if (a>b) { uint16_t t=a; a=b; b=t; }
    return b;
}

// Read an analog pin 3 times and take the median
static uint16_t analog_median3(uint8_t pin) {
    return median3(analogRead(pin), analogRead(pin), analogRead(pin));
}

// Convert raw ADC counts (0–1023) to voltage (0–5V)
static float counts_to_volts(uint16_t c) {
    return (c * ADC_REF_V) / (float)ADC_COUNTS;
}

// Convert raw ADC counts from pressure transducer to psi
// Uses the linear map: V_MIN=0psi, V_MAX=full scale
static float raw_to_psi(uint16_t c) {
    float v = counts_to_volts(c);
    return ((v - PRESSURE_V_MIN) / (PRESSURE_V_MAX - PRESSURE_V_MIN)) * PRESSURE_FS_PSI;
}

// Convert raw ADC from a current sensor to amps
// offset = voltage at 0A (usually 2.5V for ACS712)
// gain = volts per amp (from datasheet)
static float raw_to_amps(uint16_t c, float gain, float off) {
    return (counts_to_volts(c) - off) / gain;
}

// Simple low-pass filter: smooths a noisy signal over time.
// alpha = 0.3 means 30% new reading, 70% old value.
// Lower alpha = smoother but slower to respond.
// >>> DECIDE: 0.3 works for RPM. If readings are too jumpy, lower it (try 0.15).
// >>>         If readings are too slow to react, raise it (try 0.5).
static float lowpass(float prev, float x, float a) {
    return prev + a * (x - prev);
}

// ========================================================================
// 7. MS5837-02BA DRIVER (BlueRobotics library)
// ========================================================================
// Uses the BlueRobotics MS5837 library.
// Install via Arduino IDE: Library Manager → search "MS5837"
// >>> IMPORTANT: MS5837-02BA is a 3.3V device. Mega is 5V.
// >>> I NEED a level shifter between them or the sensor will fry.

static MS5837 g_ms5837;
static bool g_ms5837_ok = false;

bool ms5837_init() {
    if (!g_ms5837.init()) return false;
    g_ms5837.setModel(MS5837::MS5837_02BA);
    g_ms5837.setFluidDensity(997);  // freshwater kg/m^3 (use 1029 for seawater)
    g_ms5837_ok = true;
    return true;
}

bool ms5837_read(float &mbar, float &temp) {
    if (!g_ms5837_ok) return false;
    g_ms5837.read();
    mbar = g_ms5837.pressure();     // returns mbar
    temp = g_ms5837.temperature();  // returns degrees C
    // Sanity check — if read failed the values will be wildly off
    if (mbar < 0.0f || mbar > 2000.0f) return false;
    return true;
}

// ========================================================================
// 8. EEPROM — saves calibration so it survives power cycles
// ========================================================================

void cal_save(float p) {
    EEPROM.put(EEPROM_CAL_MAGIC_ADDR, EEPROM_CAL_MAGIC);
    EEPROM.put(EEPROM_CAL_ATM_ADDR, p);
}

bool cal_load(float &p) {
    uint16_t m;
    EEPROM.get(EEPROM_CAL_MAGIC_ADDR, m);
    if (m != EEPROM_CAL_MAGIC) return false;  // no valid cal data stored
    EEPROM.get(EEPROM_CAL_ATM_ADDR, p);
    // Sanity: MS5837-02BA physical range is 300–1200 mbar
    return (p >= 300.0f && p <= 1200.0f);
}

// ========================================================================
// 9. SENSOR SAMPLING — runs every loop (100 Hz)
// ========================================================================

void sample_sensors() {
    // --- Analog sensors (median of 3 to reject EMI spikes) ---
    g_sen.p_line_psi = raw_to_psi(analog_median3(PIN_PRESSURE_XDUCR));
    g_sen.i_12v_A    = raw_to_amps(analog_median3(PIN_CURRENT_12V),  CURRENT_GAIN_12V, CURRENT_OFFSET_V);
    g_sen.i_5v_A     = raw_to_amps(analog_median3(PIN_CURRENT_5V),   CURRENT_GAIN_5V,  CURRENT_OFFSET_V);
    g_sen.v_bus_V    = counts_to_volts(analogRead(PIN_BUS_VOLTAGE)) * BUS_DIV_RATIO;
    g_sen.v_alt_V    = counts_to_volts(analog_median3(PIN_ALT_VOLTAGE)) * ALT_V_DIV_RATIO;
    g_sen.i_alt_A    = raw_to_amps(analog_median3(PIN_ALT_CURRENT),  CURRENT_GAIN_ALT, CURRENT_OFFSET_V);

    // Clamp CTs — clamped to zero because negative current doesn't make sense
    // for a unidirectional load like a motor
    g_sen.i_comp_A = max(0.0f, raw_to_amps(analog_median3(PIN_CURRENT_COMP), CURRENT_GAIN_COMP, CLAMP_OFFSET_V));
    g_sen.i_pump_A = max(0.0f, raw_to_amps(analog_median3(PIN_CURRENT_PUMP), CURRENT_GAIN_PUMP, CLAMP_OFFSET_V));
    if (g_sen.i_alt_A < 0) g_sen.i_alt_A = 0;  // alternator shouldn't read negative either

    // --- I2C hydrostatic pressure + temperature ---
    float mbar, temp;
    if (ms5837_read(mbar, temp)) {
        g_sen.p_hydro_mbar = mbar;
        g_sen.temp_water_C = temp;
        g_sen.i2c_fail_count = 0;
    } else if (g_sen.i2c_fail_count < 255) {
        g_sen.i2c_fail_count++;
        // If this keeps failing, check:
        // 1. Level shifter wired correctly?
        // 2. Pullup resistors on SDA/SCL? (4.7k to 3.3V)
        // 3. Wire.h sometimes gets stuck — might need to power cycle
    }

    // --- RPM from hall sensors ---
    // Disable interrupts briefly so we get a clean snapshot of the counters
    noInterrupts();
    uint16_t pa = g_hall_cnt_air;   g_hall_cnt_air   = 0;
    uint16_t pw = g_hall_cnt_water; g_hall_cnt_water = 0;
    uint16_t ps = g_hall_cnt_alt;   g_hall_cnt_alt   = 0;
    interrupts();

    // Convert pulse count over LOOP_PERIOD_MS to RPM:
    // RPM = (pulses × 60000) / (pulses_per_rev × loop_period_ms)
    float rpm_a = ((float)pa * 60000.0f) / ((float)PULSES_PER_REV_AIR   * LOOP_PERIOD_MS);
    float rpm_w = ((float)pw * 60000.0f) / ((float)PULSES_PER_REV_WATER * LOOP_PERIOD_MS);
    float rpm_s = ((float)ps * 60000.0f) / ((float)PULSES_PER_REV_ALT   * LOOP_PERIOD_MS);

    // Low-pass filter to smooth out the readings
    g_rpm_air_f   = lowpass(g_rpm_air_f,   rpm_a, 0.3f);
    g_rpm_water_f = lowpass(g_rpm_water_f, rpm_w, 0.3f);
    g_rpm_alt_f   = lowpass(g_rpm_alt_f,   rpm_s, 0.3f);

    g_sen.rpm_air   = g_rpm_air_f;
    g_sen.rpm_water = g_rpm_water_f;
    g_sen.rpm_alt   = g_rpm_alt_f;

    // --- Valve encoder readback (only if physically wired) ---
    if (VALVE_ENCODER_PRESENT_AIR) {
        g_valve_pos_air_act = map(analogRead(PIN_VALVE_ENC_AIR), 0, 1023, 0, 180);
    }
    if (VALVE_ENCODER_PRESENT_WATER) {
        g_valve_pos_water_act = map(analogRead(PIN_VALVE_ENC_WATER), 0, 1023, 0, 180);
    }
}

void read_discrete_inputs() {
    // All wired with pullups — pressed = LOW = true
    g_in.estop_asserted       = (digitalRead(PIN_ESTOP)        == LOW);
    g_in.arm_switch_on        = (digitalRead(PIN_ARM_SWITCH)   == LOW);
    g_in.manual_reset_pressed = (digitalRead(PIN_MANUAL_RESET) == LOW);
}

// ========================================================================
// 10. FAULT CHECKING
// ========================================================================
// Runs every loop. Checks every sensor against its threshold.
// Uses debouncing so a single glitch doesn't trip a fault.

void check_fault_conditions() {
    uint16_t tr = 0;  // transient fault flags (might clear next loop)

    if (g_in.estop_asserted)                                   tr |= F_ESTOP;
    if (g_sen.p_line_psi > P_MAX_PSI)                          tr |= F_OVERPRESSURE;
    if (g_sen.v_bus_V    < V_MIN_BUS)                          tr |= F_UNDERVOLTAGE;
    if (g_sen.i_12v_A > I_MAX_12V || g_sen.i_5v_A > I_MAX_5V) tr |= F_OVERCURRENT;
    if (g_sen.p_line_psi < P_SANITY_LO ||
        g_sen.p_line_psi > P_SANITY_HI)                       tr |= F_SENSOR_RANGE;
    if (g_sen.i2c_fail_count > 3)                              tr |= F_I2C_TIMEOUT;

    // Only check Pi heartbeat during energized states — don't care during OFF/ARM
    bool energized = (g_state >= S_CHARGE && g_state <= S_DISCHARGE_BOTH);
    if (energized && (millis() - g_t_last_pi_msg) > PI_HEARTBEAT_TIMEOUT_MS)
        tr |= F_PI_LOST;

    // Debounce: fault must be present for FAULT_DEBOUNCE_MS before it sticks.
    // This prevents a single noisy ADC reading from shutting everything down.
    static uint32_t ts = 0;
    if (tr) {
        if (!ts) ts = millis();                        // start the timer
        if ((millis() - ts) >= FAULT_DEBOUNCE_MS)
            g_fault_flags |= tr;                       // timer expired, latch the faults
    } else {
        ts = 0;                                         // no faults, reset timer
    }
}

// Called when operator presses manual reset button
void clear_faults_if_allowed() {
    if (!g_in.manual_reset_pressed) return;
    // Can only clear if the root cause is actually gone
    uint16_t still = 0;
    if (g_in.estop_asserted)          still |= F_ESTOP;
    if (g_sen.p_line_psi > P_MAX_PSI) still |= F_OVERPRESSURE;
    if (g_sen.v_bus_V    < V_MIN_BUS) still |= F_UNDERVOLTAGE;
    if (still == 0) g_fault_flags = 0;  // all clear
    // If still != 0, faults stay latched — operator needs to fix the problem first
}

// ========================================================================
// 11. STATE MACHINE — the brain of the whole system
// ========================================================================
// next_state() decides what state we should be in based on current state,
// sensor readings, and commands from the Pi.
// on_enter() runs once when we transition INTO a new state.
// run_state_logic() runs every loop while we're IN a state.

State next_state(State cur) {
    // E-STOP always wins, from any state, no questions asked
    if (g_in.estop_asserted) return S_ESTOP;

    // Any fault forces FAULT (except during calibration — let cal finish or skip)
    if (g_fault_flags && cur != S_ESTOP &&
        cur != S_CAL_PROMPT && cur != S_CAL_ZEROING && cur != S_CAL_REPLACE)
        return S_FAULT;

    switch (cur) {

    // Boot → always go to calibration prompt
    case S_BOOT_SELFTEST:
        return S_CAL_PROMPT;

    // Calibration: wait for user to tell us what to do
    case S_CAL_PROMPT:
        if (g_cmd.requested_mode == REQ_CAL_ZERO) return S_CAL_ZEROING;
        if (g_cmd.requested_mode == REQ_CAL_SKIP) {
            g_cal_skipped = true;
            // Try loading a previously saved calibration from EEPROM
            float stored;
            if (cal_load(stored)) { g_p_atm_mbar = stored; g_cal_valid = true; }
            // If no stored cal, we use the default 1013.25 — water level will be approximate
            return S_OFF;
        }
        break;

    // Accumulating atmospheric readings — transition when we have enough
    case S_CAL_ZEROING:
        if (g_cal_sample_count >= CAL_NUM_SAMPLES) {
            g_p_atm_mbar = g_cal_accumulator / (float)CAL_NUM_SAMPLES;
            if (g_p_atm_mbar >= 300.0f && g_p_atm_mbar <= 1200.0f) {
                g_cal_valid = true;
                cal_save(g_p_atm_mbar);  // save to EEPROM for next power cycle
                return S_CAL_REPLACE;
            } else {
                g_fault_flags |= F_CAL_FAIL;  // reading was way off — sensor problem?
                return S_FAULT;
            }
        }
        break;

    // User puts sensor back in water, confirms
    case S_CAL_REPLACE:
        if (g_cmd.requested_mode == REQ_CAL_DONE ||
            g_cmd.requested_mode == REQ_CAL_SKIP)
            return S_OFF;
        break;

    // --- Normal operation ---

    case S_OFF:
        // Can only arm if the arm switch is physically on
        if (g_cmd.requested_mode == REQ_ARM && g_in.arm_switch_on)
            return S_ARM;
        break;

    case S_ARM:
        if (!g_in.arm_switch_on) return S_OFF;  // arm switch turned off → go safe
        if (g_cmd.requested_mode == REQ_CHARGE)  return S_CHARGE;
        if (g_cmd.requested_mode == REQ_MANUAL)  return S_MANUAL_DIAG;
        break;

    case S_CHARGE:
        // Auto-transition to READY when pressure hits target
        if (g_sen.p_line_psi >= P_CHARGE_TARGET) return S_READY;
        if (g_cmd.requested_mode == REQ_ABORT)   return S_ARM;
        break;

    case S_READY:
        if (g_cmd.requested_mode == REQ_DISCHARGE_AIR)   return S_DISCHARGE_AIR;
        if (g_cmd.requested_mode == REQ_DISCHARGE_WATER) return S_DISCHARGE_WATER;
        if (g_cmd.requested_mode == REQ_DISCHARGE_BOTH)  return S_DISCHARGE_BOTH;
        if (g_cmd.requested_mode == REQ_DISARM)          return S_ARM;
        break;

    // Discharge states — keep going until operator stops or pressure is spent
    case S_DISCHARGE_AIR:
    case S_DISCHARGE_WATER:
    case S_DISCHARGE_BOTH:
        if (g_cmd.requested_mode == REQ_STOP || g_sen.p_line_psi < P_SAFE_VENT)
            return S_SHUTDOWN;
        break;

    case S_MANUAL_DIAG:
        if (g_cmd.requested_mode == REQ_EXIT_MANUAL) return S_ARM;
        break;

    // Venting — wait until pressure is safely low
    case S_SHUTDOWN:
        if (g_sen.p_line_psi < P_SAFE_VENT) return S_OFF;
        break;

    // Fault — can only leave after operator presses reset AND root cause is gone
    case S_FAULT:
        clear_faults_if_allowed();
        if (g_fault_flags == 0) return S_OFF;
        break;

    // E-stop — extra cautious: must release E-stop AND press reset,
    // and even then we go to FAULT first (not directly to OFF)
    case S_ESTOP:
        if (!g_in.estop_asserted && g_in.manual_reset_pressed)
            return S_FAULT;  // goes through FAULT for inspection before OFF
        break;

    default: break;
    }

    return cur;  // no transition — stay in current state
}

// Runs ONCE when entering a new state
void on_enter(State s) {
    switch (s) {
    case S_BOOT_SELFTEST:
        immediate_cutoff_all();
        break;
    case S_CAL_PROMPT:
        immediate_cutoff_all();
        write_pin(PIN_STATUS_LED_GREEN, true);
        write_pin(PIN_STATUS_LED_RED, true);   // both LEDs = "pay attention"
        break;
    case S_CAL_ZEROING:
        g_cal_sample_count = 0;
        g_cal_accumulator = 0.0f;
        break;
    case S_CAL_REPLACE:
        write_pin(PIN_STATUS_LED_RED, false);  // green stays on = "good to go"
        break;
    case S_OFF:
        immediate_cutoff_all();
        write_pin(PIN_STATUS_LED_GREEN, false);
        write_pin(PIN_STATUS_LED_RED, false);
        write_pin(PIN_BUZZER, false);
        break;
    case S_ARM:
        immediate_cutoff_all();
        write_pin(PIN_STATUS_LED_GREEN, true);  // green = armed, ready for commands
        break;
    case S_FAULT:
    case S_ESTOP:
        immediate_cutoff_all();
        write_pin(PIN_STATUS_LED_RED, true);
        write_pin(PIN_BUZZER, true);  // loud annoyance until you fix the problem
        break;
    default:
        // CHARGE, READY, DISCHARGE_*, MANUAL, SHUTDOWN —
        // outputs are handled in commit_outputs(), not here
        break;
    }
}

// Runs EVERY LOOP while in a given state (for continuous tasks)
void run_state_logic(State s) {
    if (s == S_CAL_ZEROING) {
        // Keep accumulating atmospheric pressure readings
        if (g_sen.i2c_fail_count == 0 && g_cal_sample_count < CAL_NUM_SAMPLES) {
            g_cal_accumulator += g_sen.p_hydro_mbar;
            g_cal_sample_count++;
        }
    }
    // Could add more per-state logic here later, like:
    // - charge rate monitoring
    // - discharge energy integration
    // - RPM limiting
}

// ========================================================================
// 12. OUTPUT COMMIT — THE ONLY FUNCTION THAT TOUCHES ACTUATOR PINS
// ========================================================================
// This is a safety-critical design choice: no other function writes to
// valve/solenoid/contactor pins. Everything goes through here.
// That means interlocks are enforced in ONE place, not scattered around.

void commit_outputs() {
    uint8_t out = 0;

    // Bit positions in the output bitmap — must match what the Pi expects
    #define BIT_COMP (1<<0)  // compressor contactor (charges air vessel)
    #define BIT_PUMP (1<<1)  // pump contactor (charges water side)
    #define BIT_DSA  (1<<2)  // discharge solenoid air (releases air to wheel)
    #define BIT_DSW  (1<<3)  // discharge solenoid water (releases water to wheel)

    // Set outputs based on current state
    switch (g_state) {
    case S_ARM:
        out = 0;  // armed but nothing running yet
        break;
    case S_CHARGE:
        if (g_cmd.charge_air)   out |= BIT_COMP;  // run compressor
        if (g_cmd.charge_water) out |= BIT_PUMP;   // run pump
        break;
    case S_READY:
        out = 0;  // charged, everything off, holding pressure
        break;
    case S_DISCHARGE_AIR:
        out = BIT_DSA;
        break;
    case S_DISCHARGE_WATER:
        out = BIT_DSW;
        break;
    case S_DISCHARGE_BOTH:
        out = BIT_DSA | BIT_DSW;
        break;
    case S_MANUAL_DIAG:
        // In manual mode, the Pi directly controls which outputs are on.
        // This is for bench testing individual outputs.
        out = (g_cmd.manual_io_value & g_cmd.manual_io_mask);
        break;
    default:
        out = 0;  // OFF, SHUTDOWN, FAULT, ESTOP, CAL states — everything off
        break;
    }

    // ==========================================
    // HARD INTERLOCKS — these override EVERYTHING
    // ==========================================

    // Overpressure: immediately stop charging (compressor/pump off)
    // The mechanical pressure relief valve handles the actual venting.
    if (g_sen.p_line_psi > P_MAX_PSI) {
        out &= ~(BIT_COMP | BIT_PUMP);
    }

    // Arm switch off: kill everything
    // Exception: MANUAL mode ignores arm switch (for bench testing)
    if (!g_in.arm_switch_on && g_state != S_MANUAL_DIAG) {
        out = 0;
    }

    // MUTUAL EXCLUSION: never charge and discharge the same fluid at the same time.
    // This prevents pressurizing against an open discharge (could overspeed the wheel
    // or create a water hammer situation).
    if (out & BIT_DSA) out &= ~BIT_COMP;  // if discharging air, don't run compressor
    if (out & BIT_DSW) out &= ~BIT_PUMP;  // if discharging water, don't run pump

    // ==========================================
    // Write to actual pins
    // ==========================================
    write_pin(PIN_COMPRESSOR_CONT, out & BIT_COMP);
    write_pin(PIN_PUMP_CONT,       out & BIT_PUMP);
    write_pin(PIN_DISCH_SOL_AIR,   out & BIT_DSA);
    write_pin(PIN_DISCH_SOL_WATER, out & BIT_DSW);

    g_outputs_bitmap = out;  // save for telemetry
}

// ========================================================================
// 13. SERIAL PROTOCOL — talking to the Pi
// ========================================================================
// The Pi sends JSON commands, we send JSON telemetry back.
// All over USB-serial at 115200 baud.
//
// This is a bare-minimum JSON parser that just looks for specific keys.
// >>> TODO: replace with ArduinoJson library for robustness.
// >>>       This hand-rolled version works but will break if the Pi
// >>>       sends malformed JSON or unexpected field ordering.

// Convert a mode string like "CHARGE" to the enum value
static ReqMode parse_mode(const char *s) {
    if (!strcmp(s, "ARM"))             return REQ_ARM;
    if (!strcmp(s, "CHARGE"))          return REQ_CHARGE;
    if (!strcmp(s, "DISARM"))          return REQ_DISARM;
    if (!strcmp(s, "DISCHARGE_AIR"))   return REQ_DISCHARGE_AIR;
    if (!strcmp(s, "DISCHARGE_WATER")) return REQ_DISCHARGE_WATER;
    if (!strcmp(s, "DISCHARGE_BOTH"))  return REQ_DISCHARGE_BOTH;
    if (!strcmp(s, "STOP"))            return REQ_STOP;
    if (!strcmp(s, "ABORT"))           return REQ_ABORT;
    if (!strcmp(s, "MANUAL"))          return REQ_MANUAL;
    if (!strcmp(s, "EXIT_MANUAL"))     return REQ_EXIT_MANUAL;
    if (!strcmp(s, "CAL_ZERO"))        return REQ_CAL_ZERO;
    if (!strcmp(s, "CAL_DONE"))        return REQ_CAL_DONE;
    if (!strcmp(s, "CAL_SKIP"))        return REQ_CAL_SKIP;
    return REQ_NONE;
}

// Ugly but functional: find a string value for a given key in JSON
static bool jstr(const char *src, const char *key, char *out, uint8_t sz) {
    const char *p = strstr(src, key); if (!p) return false;
    p = strchr(p, ':'); if (!p) return false;
    p = strchr(p, '"'); if (!p) return false; p++;
    uint8_t i = 0;
    while (*p && *p != '"' && i < sz-1) out[i++] = *p++;
    out[i] = 0;
    return true;
}

// Find a number value for a given key in JSON
static bool jnum(const char *src, const char *key, long &out) {
    const char *p = strstr(src, key); if (!p) return false;
    p = strchr(p, ':'); if (!p) return false; p++;
    while (*p == ' ') p++;
    out = atol(p);
    return true;
}

// Process one complete JSON line from the Pi
void handle_cmd(const char *line) {
    char mb[24] = {};
    long v;

    // Check sequence number — ignore stale/duplicate commands
    jnum(line, "\"seq\"", v);
    if (v > 0 && (uint32_t)v <= g_cmd.seq) return;
    g_cmd.seq = (uint32_t)v;

    // Parse fields
    if (jstr(line, "\"mode\"", mb, sizeof(mb)))   g_cmd.requested_mode      = parse_mode(mb);
    if (jnum(line, "\"manual_mask\"", v))          g_cmd.manual_io_mask      = (uint8_t)v;
    if (jnum(line, "\"manual_val\"", v))           g_cmd.manual_io_value     = (uint8_t)v;
    if (jnum(line, "\"reset\"", v) && v)           g_cmd.manual_reset        = true;
    if (jnum(line, "\"charge_air\"", v))           g_cmd.charge_air          = (v != 0);
    if (jnum(line, "\"charge_water\"", v))         g_cmd.charge_water        = (v != 0);
    if (jnum(line, "\"vpa\"", v))                  g_cmd.valve_pos_air_cmd   = (int16_t)v;
    if (jnum(line, "\"vpw\"", v))                  g_cmd.valve_pos_water_cmd = (int16_t)v;

    g_t_last_pi_msg = millis();  // Pi is alive
}

// Read incoming serial bytes, assemble into lines, process when complete
void process_serial_rx() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_rx_len > 0) {
                g_rx_buf[g_rx_len] = 0;   // null-terminate the string
                handle_cmd(g_rx_buf);
                g_rx_len = 0;
            }
        } else if (g_rx_len < RX_BUF_SIZE - 1) {
            g_rx_buf[g_rx_len++] = c;
        } else {
            g_rx_len = 0;  // buffer overflow — drop the line
        }
    }
}

// Send telemetry to the Pi (called at 10 Hz)
// Values are scaled to integers to avoid AVR float-to-string overhead.
// The Pi unscales them: p/100 = psi, i12/1000 = amps, etc.
void send_telemetry() {
    char buf[300];
    snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"st\":%u,"           // timestamp_ms, state
        "\"p\":%ld,\"ph\":%ld,\"tw\":%ld,"  // pressure×100, hydro×10, temp×10
        "\"ra\":%ld,\"rw\":%ld,\"rs\":%ld," // rpm_air×10, rpm_water×10, rpm_alt×10
        "\"i12\":%ld,\"i5\":%ld,\"vb\":%ld," // current×1000, current×1000, voltage×100
        "\"ic\":%ld,\"ip\":%ld,"            // compressor_i×1000, pump_i×1000
        "\"va\":%ld,\"ia\":%ld,"            // alt_v×100, alt_i×1000
        "\"vpa\":%d,\"vpw\":%d,\"vpaa\":%d,\"vpwa\":%d," // valve positions
        "\"es\":%u,\"arm\":%u,\"mr\":%u,"   // estop, arm, manual_reset (0/1)
        "\"f\":%u,\"o\":%u,"               // fault_flags, output_bitmap
        "\"cv\":%u,\"cp\":%ld}",            // cal_valid (0/1), cal_p_atm×10
        // Values:
        (unsigned long)millis(), (unsigned)g_state,
        (long)(g_sen.p_line_psi * 100),
        (long)(g_sen.p_hydro_mbar * 10),
        (long)(g_sen.temp_water_C * 10),
        (long)(g_sen.rpm_air * 10),
        (long)(g_sen.rpm_water * 10),
        (long)(g_sen.rpm_alt * 10),
        (long)(g_sen.i_12v_A * 1000),
        (long)(g_sen.i_5v_A * 1000),
        (long)(g_sen.v_bus_V * 100),
        (long)(g_sen.i_comp_A * 1000),
        (long)(g_sen.i_pump_A * 1000),
        (long)(g_sen.v_alt_V * 100),
        (long)(g_sen.i_alt_A * 1000),
        (int)g_valve_pos_air_cmd,
        (int)g_valve_pos_water_cmd,
        (int)g_valve_pos_air_act,
        (int)g_valve_pos_water_act,
        (unsigned)(g_in.estop_asserted ? 1 : 0),
        (unsigned)(g_in.arm_switch_on ? 1 : 0),
        (unsigned)(g_in.manual_reset_pressed ? 1 : 0),
        (unsigned)g_fault_flags,
        (unsigned)g_outputs_bitmap,
        (unsigned)(g_cal_valid ? 1 : 0),
        (long)(g_p_atm_mbar * 10)
    );
    Serial.println(buf);
}

// ========================================================================
// 14. SELF-TEST — runs once on power-up
// ========================================================================
// Checks that the basics are working before letting the operator do anything.
// If any check fails, the system goes to FAULT and won't proceed until fixed.

bool run_selftest() {
    sample_sensors();
    read_discrete_inputs();

    // Can't self-test if E-stop is pressed
    if (g_in.estop_asserted) return false;

    // Bus voltage should be in a reasonable range
    // >>> CHECK: what's my actual supply voltage? Adjust 14.0 upper limit if needed.
    if (g_sen.v_bus_V < V_MIN_BUS || g_sen.v_bus_V > 14.0f) return false;

    // Pressure should read something reasonable at ambient (not -50 or 9999)
    if (g_sen.p_line_psi < P_SANITY_LO || g_sen.p_line_psi > P_SANITY_HI) return false;

    // MS5837 should respond to I2C
    float m, t;
    if (!ms5837_read(m, t)) return false;

    // Flash green LED 3 times as visual confirmation self-test passed.
    // This is the ONLY place delay() is used — it's fine here because
    // we're not in the main loop yet.
    for (uint8_t i = 0; i < 3; i++) {
        write_pin(PIN_STATUS_LED_GREEN, true);  delay(80);
        write_pin(PIN_STATUS_LED_GREEN, false); delay(80);
    }
    return true;
}

// ========================================================================
// 15. SETUP — runs once on power-up
// ========================================================================

void setup() {
    // --- Configure all output pins ---
    uint8_t outs[] = {
        PIN_COMPRESSOR_CONT, PIN_PUMP_CONT,
        PIN_DISCH_SOL_AIR, PIN_DISCH_SOL_WATER,
        PIN_STATUS_LED_GREEN, PIN_STATUS_LED_RED, PIN_BUZZER,
        PIN_VALVE_SERVO_AIR, PIN_VALVE_SERVO_WATER
    };
    for (uint8_t i = 0; i < sizeof(outs); i++) {
        pinMode(outs[i], OUTPUT);
    }
    immediate_cutoff_all();  // start with EVERYTHING off

    // --- Configure input pins with internal pullups ---
    // INPUT_PULLUP means the pin reads HIGH normally and LOW when the
    // switch connects it to GND. This is the standard way to wire buttons.
    pinMode(PIN_ESTOP,          INPUT_PULLUP);
    pinMode(PIN_HALL_RPM_AIR,   INPUT_PULLUP);
    pinMode(PIN_HALL_RPM_WATER, INPUT_PULLUP);
    pinMode(PIN_HALL_RPM_ALT,   INPUT_PULLUP);
    pinMode(PIN_ARM_SWITCH,     INPUT_PULLUP);
    pinMode(PIN_MANUAL_RESET,   INPUT_PULLUP);

    // --- Attach interrupts ---
    // FALLING = trigger when pin goes HIGH→LOW (switch closed)
    // RISING  = trigger when pin goes LOW→HIGH (hall magnet passes)
    // >>> VERIFY: check if my hall sensors trigger on RISING or FALLING
    // >>>         Depends on the specific hall sensor and whether it's
    // >>>         open-drain or push-pull output. Test with a scope.
    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP),         isr_estop,      FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_RPM_AIR),  isr_hall_air,   RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_RPM_WATER),isr_hall_water, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_RPM_ALT),  isr_hall_alt,   RISING);

    // --- Serial and I2C ---
    Serial.begin(115200);       // USB to Pi
    Wire.begin();               // I2C bus for MS5837
    Wire.setClock(100000);      // 100 kHz I2C (standard mode, most reliable)

    // --- Watchdog timer ---
    // If the main loop ever takes more than 250ms (frozen code, infinite loop,
    // crash), the watchdog automatically resets the Mega. Safety net.
    wdt_enable(WDTO_250MS);

    // --- Initialize MS5837 ---
    ms5837_init();

    // --- Self-test ---
    g_state = S_BOOT_SELFTEST;
    on_enter(g_state);

    if (!run_selftest()) {
        g_fault_flags |= F_SELFTEST;
        g_state = S_FAULT;
        on_enter(g_state);
    } else {
        g_state = S_CAL_PROMPT;
        on_enter(g_state);
    }

    g_t_last_loop = g_t_last_log = g_t_last_pi_msg = millis();
}

// ========================================================================
// 16. MAIN LOOP — runs forever after setup()
// ========================================================================
// This is the superloop. It runs as fast as it can, but only executes the
// control logic every LOOP_PERIOD_MS (10ms = 100Hz).
// Telemetry is sent at a slower rate (LOG_PERIOD_MS = 100ms = 10Hz).

void loop() {
    uint32_t now = millis();

    // --- 100 Hz control loop ---
    if ((now - g_t_last_loop) >= LOOP_PERIOD_MS) {
        g_t_last_loop = now;

        // 1. Read everything
        sample_sensors();
        read_discrete_inputs();
        process_serial_rx();

        // 2. Check for faults (skip during calibration to avoid nuisance trips)
        if (g_state != S_CAL_PROMPT && g_state != S_CAL_ZEROING && g_state != S_CAL_REPLACE)
            check_fault_conditions();

        // 3. Step the state machine
        State next = next_state(g_state);
        if (next != g_state) {
            g_state = next;
            on_enter(g_state);
        }

        // 4. Run per-state logic (like calibration accumulation)
        run_state_logic(g_state);

        // 5. Write outputs (only in operational states, not during cal)
        if (g_state >= S_OFF)
            commit_outputs();

        // 6. Clear the command so it doesn't re-trigger next loop
        g_cmd.requested_mode = REQ_NONE;

        // 7. Feed the watchdog — "yes I'm still alive"
        wdt_reset();
    }

    // --- 10 Hz telemetry ---
    if ((now - g_t_last_log) >= LOG_PERIOD_MS) {
        g_t_last_log = now;
        send_telemetry();
    }
}
