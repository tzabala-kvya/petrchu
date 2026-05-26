// ============================================================================
// PetrChu — LED test, 5 LEDs in unison (Arduino Uno)
// ----------------------------------------------------------------------------
// Exercises five standard 5 mm / 3 mm LEDs through current-limiting resistors.
// All five are driven SIMULTANEOUSLY in every pattern — if one stays dark or
// out of sync, the issue is wiring on that channel, not the script.
//
// Patterns (each runs on all five LEDs at the same time):
//
//   1. SOLID     — every LED on for 1 s, then off for 1 s
//   2. BLINK     — every LED blinks in unison at 4 Hz for 2 s
//   3. FADE      — every LED PWM-fades up and back down together (~4 s)
//   4. HEARTBEAT — every LED does a double-pulse "lub-dub" together, x3
//
// Then it repeats. Serial prints the current pattern so you can confirm the
// script is alive even if a wire is loose.
//
// RESISTOR (locked for this test):
//   Brown-Red-Black-Black-Brown  (5-band)  =  120 ohm, 1% tolerance.
//   At 5 V:
//       Red   LED (Vf ~2.0 V):  (5 - 2.0) / 120  ~= 25 mA
//       Green LED (Vf ~2.2 V):  (5 - 2.2) / 120  ~= 23 mA
//       Yellow/Amber (~2.1 V):  (5 - 2.1) / 120  ~= 24 mA
//       Blue/White (Vf ~3.2 V): (5 - 3.2) / 120  ~= 15 mA
//   All within the typical 20 mA spec — a single 120 ohm part works for any
//   color of indicator LED off a 5 V Uno pin.
//
// CURRENT BUDGET — five LEDs at ~25 mA each = ~125 mA total sourced from the
// Uno's VCC rail when SOLID is on. That's inside the ATmega328P's 200 mA
// chip-level limit (and well inside the USB-powered 500 mA budget), but it's
// the highest steady draw this sketch produces. If you scale up to many more
// LEDs, drive them through transistors / a shift register, not direct pins.
//
// WIRING (one LED, repeat five times — one channel per pin):
//
//       Uno digital pin ----[ 120 ohm ]----|>|----+
//                                          LED    |
//                                       anode -> cathode (flat side / short leg)
//                                                 |
//                                                GND
//
//   - Long leg of the LED = anode (+), goes toward the resistor / Uno pin.
//   - Short leg / flat-side notch = cathode (-), goes to GND.
//   - The 120 ohm resistor can sit on either side of the LED electrically;
//     putting it on the anode side (as drawn) is the conventional habit.
//   - Each LED needs its OWN resistor. Do not share one resistor across
//     multiple LEDs in parallel — they don't current-share and one will hog.
//
// PIN MAP — all five are PWM-capable on the Uno so FADE works on every
// channel in unison. D11 is intentionally skipped (reserved for the buzzer
// in the planned TestStandFirmware config block).
//
//   LED 0  -> D3   (PWM)
//   LED 1  -> D5   (PWM)
//   LED 2  -> D6   (PWM)
//   LED 3  -> D9   (PWM)
//   LED 4  -> D10  (PWM)
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t LED_PINS[]  = { 3, 5, 6, 9, 10 };
const uint8_t LED_COUNT   = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

const unsigned long SOLID_ON_MS     = 1000;
const unsigned long SOLID_OFF_MS    = 1000;
const unsigned long BLINK_TOTAL_MS  = 2000;
const unsigned long BLINK_HALF_MS   = 125;   // 4 Hz
const unsigned long FADE_STEP_MS    = 8;     // 256 * 8ms * 2 = ~4 s full cycle
const unsigned long HEART_PULSE_MS  = 90;    // "lub" / "dub" width
const unsigned long HEART_GAP_MS    = 110;   // gap between lub and dub
const unsigned long HEART_REST_MS   = 600;   // pause between heartbeats
const uint8_t       HEART_CYCLES    = 3;

// ---------- HELPERS ----------
// Drive every LED to the same digital level at once.
static void allDigital(uint8_t level) {
  for (uint8_t i = 0; i < LED_COUNT; ++i) digitalWrite(LED_PINS[i], level);
}

// Drive every LED to the same PWM duty at once. All configured pins are
// PWM-capable, so this is safe — analogWrite() on the same value across pins
// gives visually-simultaneous brightness changes.
static void allAnalog(uint8_t duty) {
  for (uint8_t i = 0; i < LED_COUNT; ++i) analogWrite(LED_PINS[i], duty);
}

// ---------- PATTERNS ----------
static void patternSolid() {
  Serial.println(F("# pattern: SOLID (all 5 in unison)"));
  allDigital(HIGH); delay(SOLID_ON_MS);
  allDigital(LOW);  delay(SOLID_OFF_MS);
}

static void patternBlink() {
  Serial.println(F("# pattern: BLINK (all 5 in unison, 4 Hz)"));
  const unsigned long t0 = millis();
  while (millis() - t0 < BLINK_TOTAL_MS) {
    allDigital(HIGH); delay(BLINK_HALF_MS);
    allDigital(LOW);  delay(BLINK_HALF_MS);
  }
}

static void patternFade() {
  Serial.println(F("# pattern: FADE (all 5 in unison, PWM)"));
  for (int v = 0;   v <= 255; ++v) { allAnalog((uint8_t)v); delay(FADE_STEP_MS); }
  for (int v = 255; v >= 0;   --v) { allAnalog((uint8_t)v); delay(FADE_STEP_MS); }
  allDigital(LOW);
}

static void patternHeartbeat() {
  Serial.println(F("# pattern: HEARTBEAT (all 5 in unison, lub-dub x3)"));
  for (uint8_t c = 0; c < HEART_CYCLES; ++c) {
    allDigital(HIGH); delay(HEART_PULSE_MS);   // lub
    allDigital(LOW);  delay(HEART_GAP_MS);
    allDigital(HIGH); delay(HEART_PULSE_MS);   // dub
    allDigital(LOW);  delay(HEART_REST_MS);
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  Serial.println(F("# led test ready — 5 LEDs driven in unison"));
  Serial.print  (F("# led_count=")); Serial.println(LED_COUNT);
  Serial.print  (F("# pins="));
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    Serial.print('D'); Serial.print(LED_PINS[i]);
    if (i + 1 < LED_COUNT) Serial.print(',');
  }
  Serial.println();
  Serial.println(F("# resistor: 120 ohm 1% (brown-red-black-black-brown), per LED"));
}

// ---------- LOOP ----------
void loop() {
  patternSolid();
  patternBlink();
  patternFade();
  patternHeartbeat();
}
