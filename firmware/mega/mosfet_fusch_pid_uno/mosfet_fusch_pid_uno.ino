// ============================================================================
// PetrChu — MOSFET pulse-rate PID + dual Fusch logger (Arduino Uno)
// ----------------------------------------------------------------------------
// Closed-loop pressure regulation for a SOLENOID FILL valve. The PID reads
// the Fusch 100 psi sensor (mounted DOWNSTREAM of the solenoid), compares to
// PRESSURE_SETPOINT_PSI, and modulates the valve PULSE RATE to track it.
// Pulse width is operator-set and held constant — only frequency is the
// control variable.
//
//   error          = setpoint - measured_psi   (positive => fill more)
//   cmd_hz         = PID output, clamped to [0, MAX_PULSE_HZ]
//   period_ms      = 1000 / pulse_hz
//   on_ms          = min(PULSE_WIDTH_MS, period_ms)
//                    -- if width >= period, valve is held continuously OPEN
//
// SINGLE-ACTING CAVEAT — there is no vent / bleed valve. The controller can
// only INCREASE pressure (more pulses) or stop adding (0 Hz). If pressure
// overshoots, it bleeds down only via downstream consumption (vane motor)
// or leakage. On the bench with the outlet open to atmosphere, the loop
// WILL saturate at MAX_PULSE_HZ and pressure WILL stay near 0 — that is
// expected. Real regulation needs downstream impedance.
//
// SUPPLY = SETPOINT WARNING — with a 60 psi supply line and a 60 psi
// setpoint, the loop has no headroom: even fully open, outlet pressure
// will sit slightly below 60 psi due to drop across the valve. Drop the
// setpoint a few psi below supply if you want closed-loop behaviour
// instead of pinned-open behaviour.
//
// SAFETY — pulse rate is clamped to MAX_PULSE_HZ in firmware so a tuning
// blow-up or config typo cannot command the solenoid above its mechanical
// spec.
//
// CSV (one row per SAMPLE_WINDOW_MS):
//   # format: t_ms, setpoint, psi100, error, cmd_hz, pulse_hz, on_ms,
//   #         on_frac, v100, v200, psi200, samples
//
//   on_frac = observed valve-open fraction in this window. Useful as a
//   sanity check on the pulser: should converge to on_ms / period_ms.
//
// MOSFET WIRING (low-side N-channel switch — flyback diode REQUIRED):
//
//                              +V_SUPPLY (60 psi line solenoid coil supply)
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
//                                       GND  (common with Uno GND AND coil
//                                             supply GND)
//
// FUSCH WIRING (mounted DOWNSTREAM of the solenoid for PID feedback):
//   100 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A0  (controlled)
//   200 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A1  (witness)
//
// PIN MAP SUMMARY:
//   D9  -> MOSFET gate (digital, via 220 R + 10 k pull-down)
//   D13 -> onboard LED (mirrors valve state)
//   A0  -> Fusch 100 psi signal (PID feedback)
//   A1  -> Fusch 200 psi signal (logged only)
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
// MOSFET
const uint8_t  PIN_MOSFET     = 9;
const uint8_t  PIN_LED_MIRROR = LED_BUILTIN;

// Hard ceiling on commanded pulse rate. Matches solenoid spec.
const float    MAX_PULSE_HZ   = 5.0f;

// Pulse width — operator-controlled. Each pulse holds the valve OPEN for
// this many ms. If this is >= the period at the commanded pulse rate, the
// valve is held continuously open (effective duty 100 %).
const unsigned long PULSE_WIDTH_MS = 200;   // can be > 1000

// PID setpoint
const float    PRESSURE_SETPOINT_PSI = 60.0f;

// PID gains. Output is Hz, error is psi.
// Start conservative; bench-tune by jacking up KP first, then add KI to
// remove steady-state offset, KD only if you see oscillation.
float          PID_KP = 0.20f;     // Hz / psi
float          PID_KI = 0.03f;     // Hz / (psi*s)
float          PID_KD = 0.00f;     // Hz / (psi/s)

// PID loop timing
const unsigned long PID_UPDATE_MS  = 200;   // 5 Hz control update

// PID behaviour shaping
const float    PID_DEADBAND_PSI    = 0.5f;  // |err| below this -> err = 0
const float    PID_INTEGRAL_MIN    = -50.0f;
const float    PID_INTEGRAL_MAX    =  50.0f;
const float    PID_MIN_USEFUL_HZ   = 0.25f; // below this, just close

// ADC
const float    VREF_VOLTS = 5.0f;
const uint16_t ADC_COUNTS = 1024;

// Fusch 100 psi (on A0) — PID feedback channel
const uint8_t  PIN_FUSCH_100   = A0;
const float    FS_PSI_100      = 100.0f;
const float    V_AT_0PSI_100   = 0.5f;
const float    V_AT_FS_PSI_100 = 4.5f;
const float    NOISE_FLOOR_100 = 0.10f;

// Fusch 200 psi (on A1) — witness only
const uint8_t  PIN_FUSCH_200   = A1;
const float    FS_PSI_200      = 200.0f;
const float    V_AT_0PSI_200   = 0.5f;
const float    V_AT_FS_PSI_200 = 4.5f;
const float    NOISE_FLOOR_200 = 0.20f;

// Logging window
const unsigned long SAMPLE_WINDOW_MS = 200;

// ---------- STATE ----------
// Pulser
static float         g_pulse_hz        = 0.0f;
static float         g_cmd_hz_raw      = 0.0f;  // last PID output (pre-clamp)
static unsigned long g_period_ms       = 0;
static unsigned long g_on_ms           = 0;
static unsigned long g_period_start_ms = 0;
static bool          g_valve_on        = false;

// PID
static unsigned long g_pid_last_ms     = 0;
static float         g_pid_integral    = 0.0f;
static float         g_pid_prev_psi    = 0.0f;
static bool          g_pid_first_run   = true;

// Logging window
static unsigned long g_window_start_ms = 0;
static unsigned long g_sum_100         = 0;
static unsigned long g_sum_200         = 0;
static unsigned long g_n_samples       = 0;
static unsigned long g_n_samples_on    = 0;
static float         g_last_error_psi  = 0.0f;  // raw err for CSV

// ---------- HELPERS ----------
static float clampHz(float hz) {
  if (hz < 0.0f)         return 0.0f;
  if (hz > MAX_PULSE_HZ) return MAX_PULSE_HZ;
  return hz;
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

static void setValve(bool on) {
  if (on == g_valve_on) return;
  g_valve_on = on;
  digitalWrite(PIN_MOSFET,     on ? HIGH : LOW);
  digitalWrite(PIN_LED_MIRROR, on ? HIGH : LOW);
}

// Recompute period_ms and on_ms from g_pulse_hz and the fixed PULSE_WIDTH_MS.
// If width >= period, valve runs continuously open (on_ms == period_ms).
static void recomputeTiming() {
  if (g_pulse_hz <= 0.0f) {
    g_period_ms = 0;
    g_on_ms     = 0;
    return;
  }
  g_period_ms = (unsigned long)(1000.0f / g_pulse_hz + 0.5f);
  g_on_ms     = (PULSE_WIDTH_MS >= g_period_ms) ? g_period_ms
                                                : PULSE_WIDTH_MS;
}

// Apply a new pulse-rate command. Avoids restarting the period mid-cycle
// (would cause jitter); only restarts when transitioning out of 0 Hz.
static void applyRate(float hz_raw, bool quiet) {
  const bool was_zero = (g_pulse_hz <= 0.0f);
  g_pulse_hz = clampHz(hz_raw);
  recomputeTiming();

  if (g_pulse_hz <= 0.0f) {
    setValve(false);
  } else if (was_zero) {
    g_period_start_ms = millis();   // clean restart from idle
  }

  if (!quiet) {
    Serial.print(F("# event: pulse_hz -> "));
    Serial.print(g_pulse_hz, 2);
    Serial.print(F(" Hz, period="));
    Serial.print(g_period_ms);
    Serial.print(F(" ms, on="));
    Serial.print(g_on_ms);
    Serial.print(F(" ms  @ t_ms="));
    Serial.println(millis());
  }
}

// Software pulser. Recomputes desired valve state each loop pass.
static void servicePulser() {
  if (g_period_ms == 0) {
    setValve(false);
    return;
  }
  if (g_on_ms >= g_period_ms) {
    setValve(true);                 // continuous-open mode
    return;
  }
  unsigned long elapsed = millis() - g_period_start_ms;
  if (elapsed >= g_period_ms) {
    g_period_start_ms += g_period_ms;
    elapsed = millis() - g_period_start_ms;
  }
  setValve(elapsed < g_on_ms);
}

// Take a fresh small-N average from the PID feedback channel. Independent
// of the logging window so the two don't fight over the ADC accumulator.
static float readPidPressurePsi() {
  const uint8_t n = 8;
  unsigned long sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    sum += (unsigned long)analogRead(PIN_FUSCH_100);
  }
  const float v = countsToVolts(sum, n);
  return voltsToPsi(v, V_AT_0PSI_100, V_AT_FS_PSI_100,
                       FS_PSI_100, NOISE_FLOOR_100);
}

// One PID step. Conditional integration: if the unclamped command would
// saturate, freeze the integrator instead of letting it wind up.
static void servicePID() {
  const unsigned long now = millis();
  if (g_pid_last_ms == 0) {
    g_pid_last_ms = now;
    return;
  }
  if (now - g_pid_last_ms < PID_UPDATE_MS) return;

  const float dt = (now - g_pid_last_ms) / 1000.0f;
  g_pid_last_ms = now;
  if (dt <= 0.0f) return;

  const float psi = readPidPressurePsi();

  float error = PRESSURE_SETPOINT_PSI - psi;
  g_last_error_psi = error;
  if (fabs(error) < PID_DEADBAND_PSI) error = 0.0f;

  if (g_pid_first_run) {
    g_pid_prev_psi  = psi;
    g_pid_first_run = false;
  }

  const float dPsi_dt = (psi - g_pid_prev_psi) / dt;
  g_pid_prev_psi = psi;

  // Tentative integral (only committed if it doesn't drive saturation)
  const float candidate_I  = g_pid_integral + error * dt;
  const float clamped_I    = (candidate_I < PID_INTEGRAL_MIN) ? PID_INTEGRAL_MIN
                            : (candidate_I > PID_INTEGRAL_MAX) ? PID_INTEGRAL_MAX
                            : candidate_I;

  const float unsat_cmd = PID_KP * error
                        + PID_KI * clamped_I
                        - PID_KD * dPsi_dt;

  const bool saturating =
        (unsat_cmd > MAX_PULSE_HZ && error > 0.0f) ||
        (unsat_cmd < 0.0f         && error < 0.0f);

  if (!saturating) {
    g_pid_integral = clamped_I;
  }

  g_cmd_hz_raw = unsat_cmd;
  float cmd = clampHz(unsat_cmd);
  if (cmd < PID_MIN_USEFUL_HZ) cmd = 0.0f;

  applyRate(cmd, /*quiet=*/true);
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

  Serial.print(millis());                Serial.print(',');
  Serial.print(PRESSURE_SETPOINT_PSI,2); Serial.print(',');
  Serial.print(psi100, 2);               Serial.print(',');
  Serial.print(g_last_error_psi, 2);     Serial.print(',');
  Serial.print(g_cmd_hz_raw, 2);         Serial.print(',');
  Serial.print(g_pulse_hz, 2);           Serial.print(',');
  Serial.print(g_on_ms);                 Serial.print(',');
  Serial.print(on_frac, 3);              Serial.print(',');
  Serial.print(v100, 4);                 Serial.print(',');
  Serial.print(v200, 4);                 Serial.print(',');
  Serial.print(psi200, 2);               Serial.print(',');
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
  analogRead(PIN_FUSCH_100);
  analogRead(PIN_FUSCH_200);

  Serial.println(F("# mosfet pulse-rate PID + dual fusch logger ready"));
  Serial.print  (F("# mosfet gate=D"));   Serial.println(PIN_MOSFET);
  Serial.print  (F("# fusch_100=A"));     Serial.print(PIN_FUSCH_100 - A0);
  Serial.print  (F(" (PID feedback, FS=")); Serial.print(FS_PSI_100, 0);
  Serial.print  (F(" psi), fusch_200=A"));  Serial.print(PIN_FUSCH_200 - A0);
  Serial.print  (F(" (witness, FS="));      Serial.print(FS_PSI_200, 0);
  Serial.println(F(" psi)"));
  Serial.print  (F("# setpoint="));       Serial.print(PRESSURE_SETPOINT_PSI, 2);
  Serial.print  (F(" psi, max_pulse_hz="));Serial.print(MAX_PULSE_HZ, 2);
  Serial.print  (F(" Hz, pulse_width="));  Serial.print(PULSE_WIDTH_MS);
  Serial.println(F(" ms"));
  Serial.print  (F("# gains: KP="));      Serial.print(PID_KP, 3);
  Serial.print  (F(", KI="));             Serial.print(PID_KI, 3);
  Serial.print  (F(", KD="));             Serial.print(PID_KD, 3);
  Serial.print  (F("; update="));         Serial.print(PID_UPDATE_MS);
  Serial.print  (F(" ms, deadband="));    Serial.print(PID_DEADBAND_PSI, 2);
  Serial.println(F(" psi"));
  Serial.println(F("# format: t_ms, setpoint, psi100, error, cmd_hz, pulse_hz, on_ms, on_frac, v100, v200, psi200, samples"));

  // Start with the valve closed; PID will spin it up on its first update.
  applyRate(0.0f, /*quiet=*/true);
  g_window_start_ms = millis();
  g_pid_last_ms     = 0;
  resetWindow();
}

// ---------- LOOP ----------
void loop() {
  servicePulser();
  servicePID();

  g_sum_100 += (unsigned long)analogRead(PIN_FUSCH_100);
  g_sum_200 += (unsigned long)analogRead(PIN_FUSCH_200);
  g_n_samples++;
  if (g_valve_on) g_n_samples_on++;

  if (millis() - g_window_start_ms >= SAMPLE_WINDOW_MS) {
    emitCsvRow();
    resetWindow();
  }
}
