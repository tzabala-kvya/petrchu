// ============================================================================
// PetrChu — standalone DC motor encoder test (Arduino Uno)
// ----------------------------------------------------------------------------
// Purpose: verify a 5-wire quadrature encoder is wired correctly and reports
//          counts, direction, RPM, and index pulses. No motor driver. Spin
//          the shaft by hand or run it open-loop from a separate bench supply.
//
// Assumed encoder pinout (verify against your motor datasheet — colors vary):
//   V+  (RED)    -> Arduino 5V 
//   GND (BLACK)  -> Arduino GND
//   A   (Yellow) -> i just D2  (INT0)
//   B   (White)  -> D4  (any digital input)
//   Z   (G/B)    -> D3  (INT1, index pulse — one per revolution)
// 
//  Open Serial Monitor at 115200 Baud
//
// Decoding: x1 quadrature (count rising edges of A, direction from B).
// Uno only has 2 hardware interrupts, so we use one for A and one for Z.
// To get x4 resolution, need both A and B on interrupts
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t PIN_A = 2;          // must be 2 or 3 on Uno
const uint8_t PIN_B = 4;          // any digital pin
const uint8_t PIN_Z = 3;          // must be 2 or 3 on Uno; comment out attach
                                  //   below if encoder has no index
const long   ENCODER_CPR = 4;     // counts per output-shaft revolution.
                                  //   0 = unknown -> RPM not computed.
                                  //   To find it: zero counts, spin shaft
                                  //   exactly one rev by hand, read count.
const unsigned long PRINT_INTERVAL_MS = 200;

// ---------- STATE ----------
volatile long count = 0;
volatile long indexHits = 0;
long lastCount = 0;
unsigned long lastPrintMs = 0;

// ---------- ISRs ----------
void onA() {
  // Rising edge on A. If B is low we're going one way; if high, the other.
  // Sign convention is arbitrary — flip if it reads backward from what you expect.
  if (digitalRead(PIN_B) == LOW) count++;
  else                            count--;
}

void onZ() {
  indexHits++;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PIN_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_A), onA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_Z), onZ, RISING);

  Serial.println(F("# encoder test ready"));
  Serial.println(F("# format: t_ms, count, dCount, rpm, indexHits"));
  Serial.println(F("# (rpm = 0 until ENCODER_CPR is set)"));
  lastPrintMs = millis();
}

// ---------- LOOP ----------
void loop() {
  const unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;

  // snapshot volatiles atomically
  noInterrupts();
  const long c     = count;
  const long iHits = indexHits;
  interrupts();

  const long dCount = c - lastCount;

  float rpm = 0.0f;
  if (ENCODER_CPR > 0) {
    // dCount per PRINT_INTERVAL_MS -> revs/min
    rpm = (float)dCount * 60000.0f
          / ((float)PRINT_INTERVAL_MS * (float)ENCODER_CPR);
  }

  Serial.print(now);   Serial.print(',');
  Serial.print(c);     Serial.print(',');
  Serial.print(dCount);Serial.print(',');
  Serial.print(rpm, 1);Serial.print(',');
  Serial.println(iHits);

  lastCount = c;
  lastPrintMs = now;
}
