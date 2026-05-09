// ============================================================================
// PetrChu — SCT-013-000 AC current sensor test (Arduino Uno)
// ----------------------------------------------------------------------------
// SCT-013-000 is a non-invasive split-core current transformer (CT).
// The "-000" variant outputs CURRENT (not voltage), so it needs an external
// burden resistor to develop a measurable voltage. Other variants in the
// SCT-013 family (e.g. -030) include an internal burden and output 1 V RMS
// at full scale — those don't need this circuit.
//
// SENSOR SPECS:
//   Primary range : 0-100 A AC (RMS)
//   Turns ratio   : 2000:1 (so 100 A primary -> 50 mA secondary)
//   Output type   : current (external burden required)
//
// WIRING (3.5mm jack on the sensor; tip + sleeve are the two CT leads):
//
//                +5V
//                 |
//                [10k]            <- bias divider top
//                 |
//   TIP    ---+---o A0 o---+---  SLEEVE
//             |            |
//            [33R]        [10k]   <- bias divider bottom
//             |            |
//             +-----+------+
//                   |
//                 [10uF]           <- bias-decoupling cap (recommended)
//                   |
//                  GND
//
// NOTES:
//   - The two 10k resistors bias A0 to ~Vcc/2 so the AC waveform sits in the
//     middle of the ADC range (the ADC cannot read negative voltages).
//   - The 33R BURDEN converts secondary current into a voltage at A0:
//       V_burden_peak = I_sec_peak * R_burden
//     For 30 A primary RMS: I_sec = 15 mA RMS = 21.2 mA peak,
//     V_burden = 0.70 V peak (~1.4 Vpp) — comfortably inside +/-2.5 V around
//     the bias point.
//   - At full 100 A primary, V_burden_peak ~= 2.33 V — just under the 2.5 V
//     headroom. For a smaller max current, raise R_burden:
//       R_burden = V_headroom / (I_max_primary * sqrt(2) / CT_RATIO)
//     For example, 50 A max -> ~56R; 20 A max -> ~150R.
//   - The 10 uF cap is optional but quiets the bias node; without it expect
//     ~50-200 mA of zero-current jitter.
//
// HOW THE MATH WORKS:
//   We sample A0 fast enough to capture many points per AC cycle, subtract
//   the DC bias, square each sample, then take sqrt(mean) -> ADC RMS in
//   counts. Convert to volts at the pin, then to amps:
//       I_primary_RMS = V_pin_RMS / R_burden * CT_RATIO
//
//   Bias is tracked with a slow IIR (alpha = 1/1024). At ~10 kHz sampling
//   that's a ~1.5 Hz corner — well below 50/60 Hz, so the bias estimate
//   doesn't chase the AC waveform.
//
// CONFIG NOTES:
//   - If zero-current readings are noisy: install the 10 uF cap, lengthen
//     SAMPLE_WINDOW_MS, or raise NOISE_FLOOR_AMPS.
//   - If readings clip / cap out: burden resistor is too large for the
//     current you're measuring. Drop to a smaller value.
//   - Calibrate by clamping a known load (e.g. a kettle on a known-V outlet)
//     and adjusting CT_RATIO until reported amps match a clamp meter.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_CT             = A0;
const float    BURDEN_OHMS        = 33.0f;     // burden resistor across CT leads
const float    CT_RATIO           = 2000.0f;   // SCT-013-000 turns ratio (100A:50mA)
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

  Serial.println(F("# sct-013-000 current test ready"));
  Serial.print  (F("# burden=")); Serial.print(BURDEN_OHMS, 1);
  Serial.print  (F("R, ratio=")); Serial.print(CT_RATIO, 0);
  Serial.print  (F(", window=")); Serial.print(SAMPLE_WINDOW_MS);
  Serial.print  (F("ms, bias_seed=")); Serial.println(biasCounts, 1);
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
    rmsAmps         = rmsVolts / BURDEN_OHMS * CT_RATIO;
    if (rmsAmps < NOISE_FLOOR_AMPS) rmsAmps = 0.0f;
  }

  Serial.print(millis());      Serial.print(',');
  Serial.print(biasCounts, 1); Serial.print(',');
  Serial.print(rmsAmps, 3);    Serial.print(',');
  Serial.println(n);
}
