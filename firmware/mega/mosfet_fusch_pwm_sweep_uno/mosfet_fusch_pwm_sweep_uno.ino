// ============================================================================
// PetrChu — MOSFET pulse-rate driver + dual Fusch logger (Arduino Uno)
// ----------------------------------------------------------------------------
// Bench sketch for characterising a SOLENOID valve by firing distinct on/off
// pulses at a configurable rate while logging both Fusch pressure transducers.
// No button — fully automatic, configured by the constants in the USER CONFIG
// block below.
//
// WHY NOT PWM — typical analogWrite() PWM (~490 Hz on D9) is way faster than
// a mechanical solenoid can follow. The armature can't track each cycle, so
// instead of opening and closing it just chatters somewhere in between. For a
// valve rated N actuations per second, the meaningful variable is "pulses per
// second" (with an explicit pulse width per pulse), not analog PWM duty.
//
//   flow_rate ~= PULSE_HZ * volume_per_pulse
//
// Two run modes, selected by PULSE_MODE:
//
//   PULSE_MODE_CONSTANT — hold PULSE_HZ_CONST forever.
//   PULSE_MODE_SWEEP    — step from SWEEP_START_HZ to SWEEP_END_HZ in
//                         SWEEP_STEP_HZ increments, holding each step for
//                         SWEEP_DWELL_MS. When the end is reached, either
//                         wrap to SWEEP_START_HZ or ping-pong, per
//                         SWEEP_PINGPONG.
//
// PULSE_DUTY_PCT (0..100 %) sets how long each pulse stays OPEN within its
// period — independent of mode. On time = (1000 / PULSE_HZ) * duty.
// Example: PULSE_HZ=2, PULSE_DUTY_PCT=25 -> 125 ms open, 375 ms closed,
// repeating twice per second.
//
// SAFETY — PULSE_HZ is clamped to MAX_PULSE_HZ in firmware so a config
// mistake can't command the solenoid above its mechanical spec.
//
// Each loop pass samples Fusch 100 psi (A0) and Fusch 200 psi (A1), averages
// each over SAMPLE_WINDOW_MS, and emits one CSV row per window:
//
//   # format: t_ms, pulse_hz, duty_pct, on_frac, v100, psi100, v200, psi200, samples
//
//   on_frac = fraction of samples in the window where the valve was driven
//   open. With many samples per period this converges to PULSE_DUTY_PCT/100
//   and is a sanity check on the pulser.
//
// MOSFET WIRING (low-side N-channel switch — same as the combo test):
//
//                              +V_LOAD (external supply +)
//                                  |
//                                 [SOLENOID]   FLYBACK DIODE REQUIRED
//                                  |           (1N4007 across coil,
//                                  |            cathode -> +V)
//                                  | drain
//                                gate node
//                                  |
//   Uno D9 ---[220 ohm]------+-----+ G   (N-channel logic-level MOSFET)
//                            |           |
//                       [10 kOhm]        |
//                            |           |
//                           GND        source
//                                        |
//                                       GND  (common with Uno GND AND load
//                                             supply GND)
//
//   - Common GND between Uno and load supply is mandatory.
//   - Logic-level part required (IRLZ44N / IRLB8721 / AOI518).
//   - A solenoid is inductive. The flyback diode is non-optional: every
//     turn-off generates a kickback spike that will punch through the MOSFET
//     without a freewheel path.
//   - The gate pin does NOT need to be PWM-capable any more — pulsing is done
//     in software via digitalWrite. D9 is kept for wiring continuity with the
//     other bench sketches, but any digital pin will work.
//
// FUSCH WIRING (both sensors are 3-wire ratiometric, 0.5 .. 4.5 V):
//
//   100 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A0
//   200 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A1
//
// PIN MAP SUMMARY:
//   D9  -> MOSFET gate (digital, via 220 R + 10 k pull-down)
//   D13 -> onboard LED (mirrors valve state)
//   A0  -> Fusch 100 psi signal
//   A1  -> Fusch 200 psi signal
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
// MOSFET
const uint8_t  PIN_MOSFET     = 9;
const uint8_t  PIN_LED_MIRROR = LED_BUILTIN;

// Safety — hard ceiling on commanded pulse rate. Solenoid spec is 5 Hz max,
// so the firmware will clamp any larger PULSE_HZ value to this number.
const float    MAX_PULSE_HZ   = 5.0f;

// Pulse shape — applied in both run modes
const uint8_t  PULSE_DUTY_PCT = 50;         // 0..100 % open per period

// Run mode
#define PULSE_MODE_CONSTANT  0
#define PULSE_MODE_SWEEP     1
const uint8_t  PULSE_MODE     = PULSE_MODE_CONSTANT;

// --- Constant mode (used when PULSE_MODE == PULSE_MODE_CONSTANT) ---
const float    PULSE_HZ_CONST = 2.0f;       // pulses per second

// --- Sweep mode (used when PULSE_MODE == PULSE_MODE_SWEEP) ---
const float    SWEEP_START_HZ = 1.0f;
const float    SWEEP_END_HZ   = 5.0f;
const float    SWEEP_STEP_HZ  = 1.0f;
const unsigned long SWEEP_DWELL_MS = 3000;  // hold each rate this long
const bool     SWEEP_PINGPONG = true;       // true: up then back down, repeat
                                            // false: up, wrap to start, repeat

// ADC
const float    VREF_VOLTS = 5.0f;
const uint16_t ADC_COUNTS = 1024;

// Fusch 100 psi (on A0)
const uint8_t  PIN_FUSCH_100   = A0;
const float    FS_PSI_100      = 100.0f;
const float    V_AT_0PSI_100   = 0.5f;
const float    V_AT_FS_PSI_100 = 4.5f;
const float    NOISE_FLOOR_100 = 0.10f;

// Fusch 200 psi (on A1)
const uint8_t  PIN_FUSCH_200   = A1;
const float    FS_PSI_200      = 200.0f;
const float    V_AT_0PSI_200   = 0.5f;
const float    V_AT_FS_PSI_200 = 4.5f;
const float    NOISE_FLOOR_200 = 0.20f;

// Sample windowing — one CSV row per window
const unsigned long SAMPLE_WINDOW_MS = 200; // ~5 Hz logging

// ---------- STATE ----------
static float         g_pulse_hz        = 0.0f;  // current commanded rate
static unsigned long g_period_ms       = 1000;  // derived from g_pulse_hz
static unsigned long g_on_ms           = 0;     // derived from period & duty
static unsigned long g_period_start_ms = 0;     // start of current pulse cycle
static bool          g_valve_on        = false; // last applied valve state

static int8_t        g_step_dir        = +1;    // sweep direction
static unsigned long g_step_start_ms   = 0;

static unsigned long g_window_start_ms = 0;
static unsigned long g_sum_100         = 0;
static unsigned long g_sum_200         = 0;
static unsigned long g_n_samples       = 0;
static unsigned long g_n_samples_on    = 0;     // for on_frac in CSV

// ---------- HELPERS ----------
static float clampHz(float hz) {
  if (hz < 0.0f)          return 0.0f;
  if (hz > MAX_PULSE_HZ)  return MAX_PULSE_HZ;
  return hz;
}

// Recompute period_ms and on_ms from the current commanded rate and duty.
// pulse_hz == 0 means "valve held closed" (period -> infinite, on -> 0).
static void recomputeTiming() {
  uint8_t duty = (PULSE_DUTY_PCT > 100) ? 100 : PULSE_DUTY_PCT;
  if (g_pulse_hz <= 0.0f) {
    g_period_ms = 0;
    g_on_ms     = 0;
  } else {
    g_period_ms = (unsigned long)(1000.0f / g_pulse_hz + 0.5f);
    g_on_ms     = (g_period_ms * (unsigned long)duty) / 100UL;
  }
}

static void setValve(bool on) {
  if (on == g_valve_on) return;
  g_valve_on = on;
  digitalWrite(PIN_MOSFET,     on ? HIGH : LOW);
  digitalWrite(PIN_LED_MIRROR, on ? HIGH : LOW);
}

static void announceRate(float hz) {
  Serial.print(F("# event: pulse_hz -> "));
  Serial.print(hz, 2);
  Serial.print(F(" Hz, period="));
  Serial.print(g_period_ms);
  Serial.print(F(" ms, on="));
  Serial.print(g_on_ms);
  Serial.print(F(" ms  @ t_ms="));
  Serial.println(millis());
}

static void applyRate(float hz) {
  g_pulse_hz = clampHz(hz);
  recomputeTiming();
  // Restart the period cleanly so the new on-time begins at the next pulse.
  g_period_start_ms = millis();
  setValve(g_pulse_hz > 0.0f && g_on_ms > 0);
  announceRate(g_pulse_hz);
}

// Advance to the next sweep step. Handles wrap and ping-pong, and clamps to
// [SWEEP_START_HZ, SWEEP_END_HZ] so SWEEP_STEP_HZ cannot overshoot.
static void advanceSweep() {
  float next = g_pulse_hz + (float)g_step_dir * SWEEP_STEP_HZ;

  if (SWEEP_PINGPONG) {
    if (next > SWEEP_END_HZ) {
      next = SWEEP_END_HZ;
      g_step_dir = -1;
    } else if (next < SWEEP_START_HZ) {
      next = SWEEP_START_HZ;
      g_step_dir = +1;
    }
  } else {
    if (next > SWEEP_END_HZ) next = SWEEP_START_HZ;
  }

  applyRate(next);
}

// Software pulser: decide whether the valve should be open right now based on
// where we are in the current period.
static void servicePulser() {
  if (g_period_ms == 0) {
    setValve(false);
    return;
  }
  const unsigned long elapsed = millis() - g_period_start_ms;
  if (elapsed >= g_period_ms) {
    g_period_start_ms += g_period_ms;  // start next period
    setValve(g_on_ms > 0);
    return;
  }
  setValve(elapsed < g_on_ms);
}

static float countsToVolts(unsigned long sum, unsigned long n) {
  if (n == 0) return 0.0f;
  const float voltsPerCount = VREF_VOLTS / (float)ADC_COUNTS;
  return ((float)sum / (float)n) * voltsPerCount;
}

static float voltsToPsi(float v_pin,
                        float v_at_0, float v_at_fs, float fs_psi,
                        float noise_floor) {
  const float v_span    = v_at_fs - v_at_0;
  const float psi_per_v = fs_psi / v_span;
  float psi = (v_pin - v_at_0) * psi_per_v;
  if (psi > -noise_floor && psi < noise_floor) psi = 0.0f;
  return psi;
}

static void emitCsvRow() {
  const float v100   = countsToVolts(g_sum_100, g_n_samples);
  const float v200   = countsToVolts(g_sum_200, g_n_samples);
  const float psi100 = voltsToPsi(v100, V_AT_0PSI_100, V_AT_FS_PSI_100,
                                  FS_PSI_100, NOISE_FLOOR_100);
  const float psi200 = voltsToPsi(v200, V_AT_0PSI_200, V_AT_FS_PSI_200,
                                  FS_PSI_200, NOISE_FLOOR_200);
  const float on_frac = (g_n_samples > 0)
                          ? (float)g_n_samples_on / (float)g_n_samples
                          : 0.0f;

  Serial.print(millis());        Serial.print(',');
  Serial.print(g_pulse_hz, 2);   Serial.print(',');
  Serial.print(PULSE_DUTY_PCT);  Serial.print(',');
  Serial.print(on_frac, 3);      Serial.print(',');
  Serial.print(v100, 4);         Serial.print(',');
  Serial.print(psi100, 2);       Serial.print(',');
  Serial.print(v200, 4);         Serial.print(',');
  Serial.print(psi200, 2);       Serial.print(',');
  Serial.println(g_n_samples);
}

static void resetWindow() {
  g_window_start_ms = millis();
  g_sum_100      = 0;
  g_sum_200      = 0;
  g_n_samples    = 0;
  g_n_samples_on = 0;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);
  pinMode(PIN_LED_MIRROR, OUTPUT);
  digitalWrite(PIN_LED_MIRROR, LOW);

  pinMode(PIN_FUSCH_100, INPUT);
  pinMode(PIN_FUSCH_200, INPUT);
  analogRead(PIN_FUSCH_100);   // discard first reading after mux switch
  analogRead(PIN_FUSCH_200);

  Serial.println(F("# mosfet pulse-rate driver + dual fusch logger ready"));
  Serial.print  (F("# mosfet gate=D"));   Serial.println(PIN_MOSFET);
  Serial.print  (F("# fusch_100=A"));     Serial.print(PIN_FUSCH_100 - A0);
  Serial.print  (F(" (FS="));             Serial.print(FS_PSI_100, 0);
  Serial.print  (F(" psi), fusch_200=A"));Serial.print(PIN_FUSCH_200 - A0);
  Serial.print  (F(" (FS="));             Serial.print(FS_PSI_200, 0);
  Serial.println(F(" psi)"));
  Serial.print  (F("# max_pulse_hz="));   Serial.print(MAX_PULSE_HZ, 2);
  Serial.print  (F(" Hz, pulse_duty="));  Serial.print(PULSE_DUTY_PCT);
  Serial.println(F(" %"));

  if (PULSE_MODE == PULSE_MODE_CONSTANT) {
    Serial.print  (F("# mode=CONSTANT, pulse_hz="));
    Serial.print  (clampHz(PULSE_HZ_CONST), 2);
    Serial.println(F(" Hz"));
    applyRate(PULSE_HZ_CONST);
  } else {
    Serial.print  (F("# mode=SWEEP, "));
    Serial.print  (SWEEP_START_HZ, 2);  Serial.print(F(" Hz -> "));
    Serial.print  (SWEEP_END_HZ, 2);    Serial.print(F(" Hz step "));
    Serial.print  (SWEEP_STEP_HZ, 2);   Serial.print(F(" Hz, dwell "));
    Serial.print  (SWEEP_DWELL_MS);     Serial.print(F(" ms, "));
    Serial.println(SWEEP_PINGPONG ? F("pingpong") : F("wrap"));
    g_step_dir = +1;
    applyRate(SWEEP_START_HZ);
  }
  Serial.println(F("# format: t_ms, pulse_hz, duty_pct, on_frac, v100, psi100, v200, psi200, samples"));

  g_step_start_ms = millis();
  resetWindow();
}

// ---------- LOOP ----------
void loop() {
  // Drive the valve from the software pulser every iteration.
  servicePulser();

  // One sample per channel per pass; analogRead is ~110 us each.
  g_sum_100 += (unsigned long)analogRead(PIN_FUSCH_100);
  g_sum_200 += (unsigned long)analogRead(PIN_FUSCH_200);
  g_n_samples++;
  if (g_valve_on) g_n_samples_on++;

  if (millis() - g_window_start_ms >= SAMPLE_WINDOW_MS) {
    emitCsvRow();
    resetWindow();
  }

  if (PULSE_MODE == PULSE_MODE_SWEEP &&
      (millis() - g_step_start_ms) >= SWEEP_DWELL_MS) {
    advanceSweep();
    g_step_start_ms = millis();
  }
}
