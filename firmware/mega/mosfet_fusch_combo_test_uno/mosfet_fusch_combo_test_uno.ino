// ============================================================================
// PetrChu — MOSFET + dual Fusch (100 psi / 200 psi) combo test (Arduino Uno)
// ----------------------------------------------------------------------------
// Single bench sketch that ties three pieces together to correlate
// MOSFET switching with pressure response on both Fusch transducers:
//
//   1. A momentary pushbutton TOGGLES a logic-level N-channel MOSFET.
//   2. The Fusch 100 psi sensor is sampled on A0.
//   3. The Fusch 200 psi sensor is sampled on A1.
//
// Both sensors are sampled continuously, averaged over SAMPLE_WINDOW_MS, and
// emitted as one CSV row per window. The MOSFET state at emission time is
// included as a column so the pressure record is self-explanatory after the
// fact:
//
//   # format: t_ms, mosfet, v100, psi100, v200, psi200, samples
//
// MOSFET WIRING (low-side N-channel switch — same as the standalone test):
//
//                              +V_LOAD (external supply +)
//                                  |
//                                 [LOAD]   (with flyback diode if inductive)
//                                  |
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
//   Uno D2 ----+
//              |
//             [BTN]   (momentary pushbutton, normally open)
//              |
//             GND
//
//   - Common GND between Uno and load supply is mandatory.
//   - Logic-level part required (IRLZ44N / IRLB8721 / AOI518). Standard-level
//     parts like IRF540 will not fully enhance at 5 V Vgs.
//
// FUSCH WIRING (both sensors are 3-wire ratiometric, 0.5 .. 4.5 V):
//
//   100 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A0
//   200 psi sensor:   RED -> 5V    BLACK -> GND    SIGNAL -> A1
//
//   - Both sensors share the Uno's 5 V rail and GND. Ratiometric output means
//     supply sag cancels out at the ADC (Uno uses the same 5 V as ADC ref),
//     so this is safe even if the rail droops slightly under MOSFET load.
//   - Signal wire colour varies — check the label, not the colour. Vented to
//     atmosphere a working sensor reads ~0.5 V at its signal pin.
//
// SAFETY (Fusch):
//   - 100 psi sensor: keep below 100 psi. Output saturates above and the
//     bridge can drift permanently after overpressure.
//   - 200 psi sensor: same rule, ceiling at 200 psi.
//   - Output never exceeds 4.5 V, so direct connection to the Uno ADC is safe.
//
// MATH (per sensor):
//   v_pin = raw * VREF / ADC_COUNTS
//   psi   = (v_pin - V_AT_0PSI) * (FS_PSI / V_SPAN)
//
//   psi_per_V is 25.0 for the 100 psi sensor and 50.0 for the 200 psi sensor.
//   We average over a window of N samples per channel, then clamp tiny
//   negative readings to 0 so a vented sensor logs as 0.00, not -0.07.
//
// PIN MAP SUMMARY:
//   D2  -> button (INPUT_PULLUP, active LOW)
//   D9  -> MOSFET gate (via 220 R + 10 k pull-down)
//   D13 -> onboard LED (mirrors MOSFET state)
//   A0  -> Fusch 100 psi signal
//   A1  -> Fusch 200 psi signal
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
// MOSFET / button
const uint8_t  PIN_MOSFET     = 9;
const uint8_t  PIN_BUTTON     = 2;
const uint8_t  PIN_LED_MIRROR = LED_BUILTIN;
const unsigned long DEBOUNCE_MS = 30;

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

// Sample windowing
const unsigned long SAMPLE_WINDOW_MS = 200;   // CSV row every 200 ms (~5 Hz)

// ---------- STATE ----------
static bool          g_mosfet_on        = false;

static uint8_t       g_button_stable    = HIGH;
static uint8_t       g_button_last_raw  = HIGH;
static unsigned long g_button_edge_ms   = 0;

static unsigned long g_window_start_ms  = 0;
static unsigned long g_sum_100          = 0;
static unsigned long g_sum_200          = 0;
static unsigned long g_n_samples        = 0;

// ---------- HELPERS ----------
static void applyMosfet(bool on) {
  digitalWrite(PIN_MOSFET,     on ? HIGH : LOW);
  digitalWrite(PIN_LED_MIRROR, on ? HIGH : LOW);
  Serial.print(F("# event: MOSFET -> "));
  Serial.print(on ? F("ON") : F("OFF"));
  Serial.print(F("  @ t_ms="));
  Serial.println(millis());
}

static void toggleMosfet() {
  g_mosfet_on = !g_mosfet_on;
  applyMosfet(g_mosfet_on);
}

// Debounced press detector — true exactly once per accepted falling edge.
static bool buttonPressed() {
  const uint8_t raw = digitalRead(PIN_BUTTON);
  const unsigned long now = millis();

  if (raw != g_button_last_raw) {
    g_button_last_raw = raw;
    g_button_edge_ms  = now;
    return false;
  }
  if ((now - g_button_edge_ms) < DEBOUNCE_MS) return false;
  if (raw == g_button_stable) return false;

  g_button_stable = raw;
  return (g_button_stable == LOW);
}

// Convert averaged raw ADC counts (sum / n) to volts at the pin.
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

  Serial.print(millis());            Serial.print(',');
  Serial.print(g_mosfet_on ? 1 : 0); Serial.print(',');
  Serial.print(v100, 4);             Serial.print(',');
  Serial.print(psi100, 2);           Serial.print(',');
  Serial.print(v200, 4);             Serial.print(',');
  Serial.print(psi200, 2);           Serial.print(',');
  Serial.println(g_n_samples);
}

static void resetWindow() {
  g_window_start_ms = millis();
  g_sum_100   = 0;
  g_sum_200   = 0;
  g_n_samples = 0;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);
  pinMode(PIN_LED_MIRROR, OUTPUT);
  digitalWrite(PIN_LED_MIRROR, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_FUSCH_100, INPUT);
  pinMode(PIN_FUSCH_200, INPUT);
  analogRead(PIN_FUSCH_100);   // discard first reading after mux switch
  analogRead(PIN_FUSCH_200);

  Serial.println(F("# mosfet + dual fusch combo test ready"));
  Serial.print  (F("# mosfet gate=D"));   Serial.print(PIN_MOSFET);
  Serial.print  (F(", button=D"));        Serial.println(PIN_BUTTON);
  Serial.print  (F("# fusch_100=A"));     Serial.print(PIN_FUSCH_100 - A0);
  Serial.print  (F(" (FS="));             Serial.print(FS_PSI_100, 0);
  Serial.print  (F(" psi), fusch_200=A"));Serial.print(PIN_FUSCH_200 - A0);
  Serial.print  (F(" (FS="));             Serial.print(FS_PSI_200, 0);
  Serial.println(F(" psi)"));
  Serial.println(F("# press button to toggle MOSFET; pressures stream every 200 ms"));
  Serial.println(F("# format: t_ms, mosfet, v100, psi100, v200, psi200, samples"));

  resetWindow();
}

// ---------- LOOP ----------
void loop() {
  // Always responsive: button check runs every iteration.
  if (buttonPressed()) toggleMosfet();

  // One sample from each channel per iteration. analogRead is ~110 us each,
  // so a loop pass is roughly 250 us — debounce stays accurate.
  g_sum_100 += (unsigned long)analogRead(PIN_FUSCH_100);
  g_sum_200 += (unsigned long)analogRead(PIN_FUSCH_200);
  g_n_samples++;

  if (millis() - g_window_start_ms >= SAMPLE_WINDOW_MS) {
    emitCsvRow();
    resetWindow();
  }
}
