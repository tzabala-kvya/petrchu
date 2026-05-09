// ============================================================================
// PetrChu — 0..25 V voltage sensor module test (Arduino Uno)
// ----------------------------------------------------------------------------
// The "0-25V Voltage Sensor Module" is a passive resistor divider on a small
// breakout — no active parts, no isolation. R1 = 30 kohm (top) and R2 = 7.5
// kohm (bottom) give a ratio R2 / (R1 + R2) = 1/5, so 25 V at the screw
// terminals is attenuated to 5 V at the signal pin — exactly the max a 5 V
// Arduino ADC can read.
//
// PINOUT (typical 3-pin header + 2-screw terminal board):
//
//    Screw terminals               Header pins
//      VCC  (+) --o                o S   (signal out, Vin / 5)
//      GND  (-) --o                o +   (often tied to screw VCC — see below)
//                                  o -   (GND, tie to Arduino GND)
//
// WIRING:
//
//    [SOURCE +] -------- VCC screw
//    [SOURCE -] -------- GND screw -------- Arduino GND
//                          S  header ------ A0
//                          -  header ------ Arduino GND   (same node)
//                          +  header ------ LEAVE UNCONNECTED
//
//   The source and Arduino MUST share ground. The module does not isolate.
//   The "+" header pin is, on most clones, connected straight to the VCC
//   screw — i.e. it carries the FULL input voltage. Treat it as live and
//   never wire it to the Arduino's 5 V rail.
//
// LIMITS / SAFETY:
//   - Absolute max input: 25 V on a 5 V Arduino. Above that the signal pin
//     exceeds Vref and you will damage the ADC — the module has no clamp.
//   - On a 3.3 V board the same divider tops out at ~16.5 V. If you need a
//     full 25 V range, swap R2 to ~4.7 kohm or add an external clamp.
//   - Resistor tolerance is typically 5 %, so the 1/5 ratio can be off by a
//     few percent. Use CAL_GAIN below if you need better than ~5 % accuracy.
//   - Input impedance is R1 + R2 = 37.5 kohm — pulls ~670 uA at 25 V.
//     Negligible for batteries / supplies; noticeable on high-Z sources.
//
// HOW THE MATH WORKS:
//   raw counts -> volts at A0:   v_pin = raw * VREF / ADC_COUNTS
//   v_pin     -> volts at input: v_in  = v_pin * (R1 + R2) / R2 * CAL_GAIN
//
//   We average a window of samples to suppress ADC noise and 50/60 Hz pickup.
//   For a steady DC source the mean is the answer; an optional IIR filter
//   smooths it further for battery-monitor style readouts.
//
// CONFIG NOTES:
//   - If you replaced the resistors, update R1_OHMS / R2_OHMS to match.
//   - To calibrate: apply a known DC voltage (DMM as reference), read the
//     script's v_in, then nudge CAL_GAIN until they agree.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_S      = A0;
const float    VREF_VOLTS = 5.0f;      // ADC reference (Uno = 5.0)
const uint16_t ADC_COUNTS = 1024;      // 10-bit ADC

// Stock divider on the 0-25V module: R1 = 30k (top), R2 = 7.5k (bottom).
const float    R1_OHMS    = 30000.0f;
const float    R2_OHMS    = 7500.0f;

// Trim factor for resistor tolerance. Adjust against a known source.
const float    CAL_GAIN   = 1.0f;

const unsigned long SAMPLE_WINDOW_MS  = 200;     // averaging window
const float         NOISE_FLOOR_VOLTS = 0.02f;   // print 0 below this

// Slow IIR for steady readouts (battery monitor style). ALPHA closer to 0 =
// smoother / slower; closer to 1 = snappier.
const bool          ENABLE_IIR = false;
const float         IIR_ALPHA  = 0.10f;

// ---------- STATE ----------
float vIn_iir = 0.0f;
bool  iirSeeded = false;

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_S, INPUT);
  analogRead(PIN_S);  // discard first reading (mux settle)

  const float dividerInv = (R1_OHMS + R2_OHMS) / R2_OHMS;
  const float vMax       = VREF_VOLTS * dividerInv;

  Serial.println(F("# 0-25V voltage sensor test"));
  Serial.print  (F("# R1="));  Serial.print(R1_OHMS, 0);
  Serial.print  (F(", R2="));  Serial.print(R2_OHMS, 0);
  Serial.print  (F(", divider_inv=")); Serial.print(dividerInv, 4);
  Serial.print  (F(", cal_gain="));    Serial.println(CAL_GAIN, 4);
  Serial.print  (F("# Vmax = "));      Serial.print(vMax, 2);
  Serial.println(F(" V -- DO NOT EXCEED"));
  Serial.println(F("# format: t_ms, raw_mean, v_pin, v_in, samples"));
}

// ---------- LOOP ----------
void loop() {
  unsigned long sum = 0;
  unsigned long n   = 0;
  const unsigned long deadline = millis() + SAMPLE_WINDOW_MS;

  while ((long)(millis() - deadline) < 0) {
    sum += (unsigned long)analogRead(PIN_S);
    n++;
  }

  float rawMean = 0.0f;
  float vPin    = 0.0f;
  float vIn     = 0.0f;
  if (n > 0) {
    const float voltsPerCount = VREF_VOLTS / (float)ADC_COUNTS;
    const float dividerInv    = (R1_OHMS + R2_OHMS) / R2_OHMS;
    rawMean = (float)sum / (float)n;
    vPin    = rawMean * voltsPerCount;
    vIn     = vPin * dividerInv * CAL_GAIN;
    if (vIn < NOISE_FLOOR_VOLTS) vIn = 0.0f;

    if (ENABLE_IIR) {
      if (!iirSeeded) { vIn_iir = vIn; iirSeeded = true; }
      else            { vIn_iir += (vIn - vIn_iir) * IIR_ALPHA; }
      vIn = vIn_iir;
    }
  }

  Serial.print(millis());   Serial.print(',');
  Serial.print(rawMean, 1); Serial.print(',');
  Serial.print(vPin, 3);    Serial.print(',');
  Serial.print(vIn, 3);     Serial.print(',');
  Serial.println(n);
}
