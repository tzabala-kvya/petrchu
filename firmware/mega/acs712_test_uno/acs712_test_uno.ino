// ============================================================================
// PetrChu — ACS712 current sensor test (Arduino Uno)
// ----------------------------------------------------------------------------
// ACS712 is a Hall-effect-based linear current sensor. Unlike the SCT-013,
// it is GALVANICALLY CONNECTED (the load current passes through the chip's
// internal copper trace), it measures BOTH DC AND AC, and it outputs a
// VOLTAGE — no burden resistor needed. Sensitivity depends on the variant.
//
// VARIANTS:
//   ACS712-05B  : +/- 5  A range,  185 mV/A
//   ACS712-20A  : +/- 20 A range,  100 mV/A
//   ACS712-30A  : +/- 30 A range,   66 mV/A
//
// At zero current the output sits at Vcc/2 (~2.5 V on a 5 V board). Positive
// current pushes it up, negative current pulls it down — so the chip reads
// bidirectional DC as well as AC.
//
// WIRING:
//
//        +5V ----o VCC
//                |
//   [LOAD] --IP+ ACS712 IP-- [SOURCE]      (load current goes through these)
//                |
//        GND ----o GND
//                |
//        A0  <---o OUT
//
//   Most breakout boards have VCC, GND, OUT pins on a header and a screw
//   terminal for the IP+/IP- side. The two IP pins are isolated from VCC/GND
//   (~2.1 kV isolation), so they go in series with the load — DO NOT short
//   them to ground.
//
// NOTES:
//   - Powered from 5 V. The OUT pin swings 0..5 V, so use a 5 V Arduino (Uno,
//     Mega, Nano). On a 3.3 V board you must level-shift or attenuate OUT or
//     you will damage the ADC pin.
//   - The breakout's onboard RC filter (typ. 1 kohm + 1 nF) gives ~1.7 kHz
//     bandwidth — fine for 50/60 Hz mains, marginal for switching loads.
//   - Quiescent offset error is the dominant noise source at low currents.
//     Datasheet specs +/- 25 mV typical at 25 C; that's ~135 mA on the 05B,
//     ~250 mA on the 20A, ~380 mA on the 30A. We auto-zero at startup to
//     remove the static part — the load must be OFF during the first second.
//   - For tiny currents (< ~1 % of full scale) consider a smaller variant.
//     Sensitivity scales linearly: the 05B resolves ~5x finer than the 30A.
//
// HOW THE MATH WORKS:
//   We sample OUT fast, subtract the bias (Vcc/2 plus chip offset), then:
//       DC: report the mean of (sample - bias) -> volts -> amps
//       AC: square each sample, mean, sqrt -> RMS counts -> volts -> amps
//   Both are useful: DC mean tells you net current direction (positive or
//   negative), AC RMS tells you the magnitude of an AC waveform regardless
//   of direction. For pure DC the RMS equals |mean|; for pure AC the mean
//   is ~0 and the RMS is the meaningful value.
//
//   The bias is seeded by averaging 100 ms of samples in setup() — the load
//   MUST be off then. After that we hold the bias static; if you want it to
//   track slow thermal drift, enable BIAS_TRACKING below (only safe for AC
//   loads — DC current would be averaged away).
//
// CONFIG NOTES:
//   - Set SENSITIVITY_V_PER_A to match your variant.
//   - Calibrate by passing a known DC current (bench supply in current-limit)
//     and adjusting SENSITIVITY_V_PER_A until reported amps match.
//   - If zero-current readings have a constant offset: re-run setup() with
//     the load truly disconnected, or bump CAL_WINDOW_MS for a longer seed.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_OUT             = A0;
const float    SENSITIVITY_V_PER_A = 0.100f;   // 0.185=05B, 0.100=20A, 0.066=30A
const float    VREF_VOLTS          = 5.0f;     // ADC reference (Uno = 5.0)
const uint16_t ADC_COUNTS          = 1024;     // 10-bit ADC

const unsigned long SAMPLE_WINDOW_MS = 200;    // RMS/mean window (12 cyc @ 60Hz)
const unsigned long CAL_WINDOW_MS    = 500;    // bias seed duration (load OFF!)
const float         NOISE_FLOOR_AMPS = 0.05f;  // print 0 below this

// Bias tracking: leave OFF for DC, ON for AC-only loads where you want slow
// thermal drift to be removed automatically.
const bool          BIAS_TRACKING    = false;
const float         BIAS_IIR_ALPHA   = 1.0f / 1024.0f;  // ~1.5 Hz @ ~10 kHz

// ---------- STATE ----------
float biasCounts = 512.0f;

// ---------- HELPERS ----------
static void seedBias() {
  // Average a window of samples to capture the chip's true zero-current
  // output (Vcc/2 + offset). The load MUST be off during this — any current
  // flowing now will be subtracted from every subsequent reading.
  double sum = 0.0;
  unsigned long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < CAL_WINDOW_MS) {
    sum += analogRead(PIN_OUT);
    n++;
  }
  if (n > 0) biasCounts = (float)(sum / n);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_OUT, INPUT);
  analogRead(PIN_OUT);  // discard first reading (mux settle)

  Serial.println(F("# acs712 current test"));
  Serial.println(F("# IMPORTANT: load must be OFF during initial bias cal"));
  delay(200);
  seedBias();

  const float biasVolts = biasCounts * (VREF_VOLTS / (float)ADC_COUNTS);
  Serial.print  (F("# sensitivity=")); Serial.print(SENSITIVITY_V_PER_A, 3);
  Serial.print  (F(" V/A, window="));  Serial.print(SAMPLE_WINDOW_MS);
  Serial.print  (F("ms, bias=")); Serial.print(biasCounts, 1);
  Serial.print  (F(" counts (")); Serial.print(biasVolts, 3); Serial.println(F(" V)"));
  Serial.println(F("# format: t_ms, bias_counts, idc_a, irms_a, samples"));
}

// ---------- LOOP ----------
void loop() {
  double sum   = 0.0;
  double sumSq = 0.0;
  unsigned long n = 0;
  const unsigned long deadline = millis() + SAMPLE_WINDOW_MS;

  while ((long)(millis() - deadline) < 0) {
    int raw = analogRead(PIN_OUT);
    if (BIAS_TRACKING) {
      biasCounts += ((float)raw - biasCounts) * BIAS_IIR_ALPHA;
    }
    float ac = (float)raw - biasCounts;
    sum   += (double)ac;
    sumSq += (double)ac * (double)ac;
    n++;
  }

  float dcAmps  = 0.0f;
  float rmsAmps = 0.0f;
  if (n > 0) {
    const float voltsPerCount = VREF_VOLTS / (float)ADC_COUNTS;
    float meanCounts = (float)(sum / n);
    float rmsCounts  = sqrtf((float)(sumSq / n));
    dcAmps  = (meanCounts * voltsPerCount) / SENSITIVITY_V_PER_A;
    rmsAmps = (rmsCounts  * voltsPerCount) / SENSITIVITY_V_PER_A;
    if (fabsf(dcAmps)  < NOISE_FLOOR_AMPS) dcAmps  = 0.0f;
    if (rmsAmps        < NOISE_FLOOR_AMPS) rmsAmps = 0.0f;
  }

  Serial.print(millis());      Serial.print(',');
  Serial.print(biasCounts, 1); Serial.print(',');
  Serial.print(dcAmps, 3);     Serial.print(',');
  Serial.print(rmsAmps, 3);    Serial.print(',');
  Serial.println(n);
}
