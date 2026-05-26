// ============================================================================
// PetrChu — SCT-013-030 AC current sensor test (Arduino Uno)
// ----------------------------------------------------------------------------
// Companion to current_sensor_test_uno/ (the SCT-013-000 version). Use this
// sketch only if the sensor turns out to be the -030 variant, NOT the -000.
//
// How to tell which you have:
//   - The suffix on the casing label is authoritative. "-000" = current
//     output (needs external burden); "-030" = voltage output (internal
//     burden, 1 V RMS at 30 A primary). A label that says "SCT-013-000 30A"
//     is still a -000 — the "30A" is just the recommended max primary.
//   - Quick bench check: with no external burden and the CT clamped around
//     a small known AC load (e.g. 2-5 A), measure ACV across the plug. A
//     -030 will show a sensible voltage proportional to current. A -000
//     will show a wildly high open-circuit voltage — disconnect immediately
//     and use the -000 sketch with a burden resistor instead.
//
// SENSOR SPECS (-030):
//   Primary range : 0-30 A AC (RMS)
//   Output        : 0-1 V RMS, linear (1 V RMS == 30 A primary)
//   Output type   : voltage (internal burden — DO NOT add an external one)
//
// WIRING (3.5mm jack on the sensor; tip + sleeve are the two output leads):
//
//                +5V
//                 |
//                [10k]            <- bias divider top
//                 |
//   SLEEVE ------+                <- bias node, ~2.5V
//                 |
//                [10uF]           <- (optional) bias-decoupling cap
//                 |
//                [10k]            <- bias divider bottom
//                 |
//                GND
//
//   TIP    ------------- A0      <- signal lead direct to Uno A0
//
//   IMPORTANT: TIP and SLEEVE go to DIFFERENT nodes. Do not tie them
//   together — that shorts the sensor's internal burden and you'll
//   read exactly 0 A. A0 is biased to ~2.5V through the CT's own
//   winding; the AC signal then rides on top of that bias.
//
//   ALSO: clamp around ONE conductor only (hot OR neutral). Clamping
//   the whole 2-wire cord makes the opposing currents cancel and the
//   reading sits at zero.
//
// NOTES:
//   - The two 10k resistors bias A0 to ~Vcc/2 so the AC waveform sits in the
//     middle of the ADC range (the ADC cannot read negative voltages).
//   - No burden resistor — the -030 has it built in. Adding one in parallel
//     would lower the effective burden and shrink the output below 1 V/30 A.
//   - At 30 A primary the output is 1 V RMS (~1.41 V peak), so the signal at
//     A0 swings ~1.09 V .. ~3.91 V around the 2.5 V bias — well inside the
//     0-5 V ADC range. The sensor clips internally above ~30 A, so don't
//     trust readings past nameplate.
//   - The 10 uF cap is optional but quiets the bias node.
//
// HOW THE MATH WORKS:
//   We sample A0 fast enough to capture many points per AC cycle, subtract
//   the DC bias, square each sample, then take sqrt(mean) -> ADC RMS in
//   counts. Convert to volts at the pin, then to amps via the sensor's
//   voltage-to-current scale:
//       I_primary_RMS = V_pin_RMS * AMPS_PER_VOLT
//
//   Bias is tracked with a slow IIR (alpha = 1/1024). At ~10 kHz sampling
//   that's a ~1.5 Hz corner — well below 50/60 Hz, so the bias estimate
//   doesn't chase the AC waveform.
//
// CONFIG NOTES:
//   - If zero-current readings are noisy: install the 10 uF cap, lengthen
//     SAMPLE_WINDOW_MS, or raise NOISE_FLOOR_AMPS.
//   - Calibrate by clamping a known load (e.g. a kettle on a known-V outlet)
//     and adjusting AMPS_PER_VOLT until reported amps match a clamp meter.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_CT             = A0;
const float    AMPS_PER_VOLT      = 30.0f;     // SCT-013-030: 1 V RMS == 30 A primary
const float    VREF_VOLTS         = 5.0f;      // ADC reference (Uno = 5.0, 3.3V boards = 3.3)
const uint16_t ADC_COUNTS         = 1024;      // 10-bit ADC

const unsigned long SAMPLE_WINDOW_MS  = 200;   // RMS window — 12 cycles @ 60Hz, 10 @ 50Hz
const float         NOISE_FLOOR_AMPS  = 0.10f; // print 0 below this to hide jitter
const float         BIAS_IIR_ALPHA    = 1.0f / 1024.0f;  // ~1.5 Hz corner @ ~10kHz

// ---------- STATE ----------
float biasCounts = 512.0f;

// ---------- HELPERS ----------
static void seedBias() {
  // Quick average to seed the bias estimate; otherwise the first window's
  // RMS would be poisoned by the IIR's slow approach to the true midpoint.
  double sum = 0.0;
  unsigned long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    sum += analogRead(PIN_CT);
    n++;
  }
  if (n > 0) biasCounts = (float)(sum / n);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_CT, INPUT);
  analogRead(PIN_CT);  // discard first reading (mux settle)
  seedBias();

  Serial.println(F("# sct-013-030 current test ready"));
  Serial.print  (F("# amps_per_volt=")); Serial.print(AMPS_PER_VOLT, 1);
  Serial.print  (F(", window="));        Serial.print(SAMPLE_WINDOW_MS);
  Serial.print  (F("ms, bias_seed="));   Serial.println(biasCounts, 1);
  Serial.println(F("# format: t_ms, bias_counts, irms_a, samples"));
}

// ---------- LOOP ----------
void loop() {
  double sumSq = 0.0;
  unsigned long n = 0;
  const unsigned long deadline = millis() + SAMPLE_WINDOW_MS;

  while ((long)(millis() - deadline) < 0) {
    int raw = analogRead(PIN_CT);
    biasCounts += ((float)raw - biasCounts) * BIAS_IIR_ALPHA;
    float ac = (float)raw - biasCounts;
    sumSq += (double)ac * (double)ac;
    n++;
  }

  float rmsAmps = 0.0f;
  if (n > 0) {
    float rmsCounts = sqrtf((float)(sumSq / n));
    float rmsVolts  = rmsCounts * (VREF_VOLTS / (float)ADC_COUNTS);
    rmsAmps         = rmsVolts * AMPS_PER_VOLT;
    if (rmsAmps < NOISE_FLOOR_AMPS) rmsAmps = 0.0f;
  }

  Serial.print(millis());      Serial.print(',');
  Serial.print(biasCounts, 1); Serial.print(',');
  Serial.print(rmsAmps, 3);    Serial.print(',');
  Serial.println(n);
}
