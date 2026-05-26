// ============================================================================
// PetrChu — Fusch 200 psi pressure transducer test (Arduino Uno)
// ----------------------------------------------------------------------------
// Same Fusch-style 3-wire ratiometric sensor as the 100 psi version, just
// scaled to a 200 psi full-scale range. Piezoresistive bridge with onboard
// amplifier; output maps linearly to 0.5 V .. 4.5 V.
//
//   At 0   psi (gauge, atmospheric) the output sits at 0.5 V
//   At 200 psi (full scale)         the output sits at 4.5 V
//   Slope                                              0.02 V/psi
//                                                  =  50.0 psi/V
//
// "Ratiometric" means the 0.5 / 4.5 V endpoints scale with the supply rail.
// On a 5.00 V Arduino rail this is harmless; on a sagging rail the endpoints
// shift in proportion, so it cancels out IF you reference the ADC to the same
// 5 V (which the Uno does by default — analogReference(DEFAULT)).
//
// PINOUT (3-wire, typical Deutsch-style or bare lead variants):
//
//   RED   --- VCC  --- Arduino 5V
//   BLACK --- GND  --- Arduino GND
//   BLUE  --- OUT  --- Arduino A0     (signal, 0.5 .. 4.5 V)
//
//   Some units use yellow/green for the signal — check the label, not the
//   wire colour. If unsure, with the sensor powered and vented to atmosphere
//   the signal lead reads ~0.5 V to GND on a DMM.
//
// SAFETY:
//   - 200 psi rated. Burst pressure is typically 3x rated, but DO NOT exceed
//     200 psi during testing — output saturates at 4.5 V and the sensor can
//     drift permanently after overpressure events.
//   - The output is short-circuit protected on most variants, but do not tie
//     the signal lead to VCC or GND while powered.
//   - Powered from 5 V. Output never exceeds 4.5 V so the Uno ADC is safe.
//
// HOW THE MATH WORKS:
//   raw counts -> volts at A0:   v_pin = raw * VREF / ADC_COUNTS
//   v_pin     -> pressure:       psi   = (v_pin - V_AT_0PSI) * (FS_PSI / V_SPAN)
//
//   Using nominal datasheet endpoints:
//     V_AT_0PSI = 0.5 V,  V_FS_PSI = 4.5 V,  V_SPAN = 4.0 V,  FS_PSI = 200
//
//   We average a window of samples to suppress ADC noise. Below the noise
//   floor we clamp to 0 so an unloaded sensor doesn't show -0.5 psi jitter.
//
// CONFIG NOTES:
//   - This sketch uses the NOMINAL 0.5 V offset from the datasheet. If you
//     want to tare against the actual atmospheric reading instead, set
//     ENABLE_TARE = true (vent the sensor at startup).
//   - Resolution on a 10-bit Uno ADC: 5 V / 1024 = 4.88 mV/count, and the
//     slope is 50 psi/V, so one ADC count = ~0.24 psi. The 200 psi sensor
//     is exactly half as fine as the 100 psi version per ADC count -- if
//     you're working below ~50 psi prefer the 100 psi unit.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_OUT    = A0;
const float    VREF_VOLTS = 5.0f;      // ADC reference (Uno = 5.0)
const uint16_t ADC_COUNTS = 1024;      // 10-bit ADC

// Transfer function endpoints (Fusch-style 0.5 .. 4.5 V, 200 psi full scale).
const float    FS_PSI      = 200.0f;
const float    V_AT_0PSI   = 0.5f;
const float    V_AT_FS_PSI = 4.5f;

const unsigned long SAMPLE_WINDOW_MS = 200;    // ~5 Hz output
const float         NOISE_FLOOR_PSI  = 0.20f;  // print 0 below this

// Optional: tare against atmospheric at startup (vent the sensor first).
const bool          ENABLE_TARE      = false;
const unsigned long TARE_WINDOW_MS   = 500;

// ---------- STATE ----------
float v_offset = V_AT_0PSI;  // overwritten in setup() if ENABLE_TARE

// ---------- HELPERS ----------
static float read_v_pin_average(unsigned long window_ms, unsigned long &n_out) {
  unsigned long sum = 0;
  unsigned long n   = 0;
  const unsigned long deadline = millis() + window_ms;
  while ((long)(millis() - deadline) < 0) {
    sum += (unsigned long)analogRead(PIN_OUT);
    n++;
  }
  n_out = n;
  if (n == 0) return 0.0f;
  const float voltsPerCount = VREF_VOLTS / (float)ADC_COUNTS;
  return ((float)sum / (float)n) * voltsPerCount;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_OUT, INPUT);
  analogRead(PIN_OUT);  // discard first reading (mux settle)

  const float v_span     = V_AT_FS_PSI - V_AT_0PSI;
  const float psi_per_v  = FS_PSI / v_span;

  Serial.println(F("# Fusch 200 psi pressure transducer test"));
  Serial.print  (F("# FS_PSI="));      Serial.print(FS_PSI, 1);
  Serial.print  (F(", V_AT_0PSI="));   Serial.print(V_AT_0PSI, 3);
  Serial.print  (F(", V_AT_FS_PSI=")); Serial.print(V_AT_FS_PSI, 3);
  Serial.print  (F(", psi_per_V="));   Serial.println(psi_per_v, 3);

  if (ENABLE_TARE) {
    Serial.println(F("# taring -- keep sensor vented to atmosphere..."));
    unsigned long n = 0;
    const float v_tare = read_v_pin_average(TARE_WINDOW_MS, n);
    v_offset = v_tare;
    Serial.print(F("# tare done: v_offset = "));
    Serial.print(v_offset, 4);
    Serial.print(F(" V (samples="));
    Serial.print(n);
    Serial.println(F(")"));
  } else {
    Serial.print(F("# using nominal offset: v_offset = "));
    Serial.print(v_offset, 3);
    Serial.println(F(" V"));
  }

  Serial.println(F("# format: t_ms, v_pin, psi, samples"));
}

// ---------- LOOP ----------
void loop() {
  unsigned long n = 0;
  const float v_pin = read_v_pin_average(SAMPLE_WINDOW_MS, n);

  const float v_span    = V_AT_FS_PSI - V_AT_0PSI;
  const float psi_per_v = FS_PSI / v_span;
  float psi = (v_pin - v_offset) * psi_per_v;
  if (psi > -NOISE_FLOOR_PSI && psi < NOISE_FLOOR_PSI) psi = 0.0f;

  Serial.print(millis());   Serial.print(',');
  Serial.print(v_pin, 4);   Serial.print(',');
  Serial.print(psi, 2);     Serial.print(',');
  Serial.println(n);
}
