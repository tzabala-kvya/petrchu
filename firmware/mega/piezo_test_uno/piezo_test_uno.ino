// ============================================================================
// PetrChu — Piezo buzzer test, 2-wire passive piezo (Arduino Uno)
// ----------------------------------------------------------------------------
// Exercises a 2-wire passive piezo element on D8. Cycles through four
// routines:
//
//   1. BEEP_SHORT   — single 100 ms chirp at 2 kHz
//   2. BEEP_LONG    — single 600 ms tone at 1 kHz ("attention")
//   3. SWEEP        — 500 Hz -> 4 kHz frequency sweep, ~1 s
//   4. ALARM        — two-tone 800/1200 Hz, six cycles ("fault" cadence)
//
// 2-WIRE PASSIVE PIEZO — what you have:
//
//   A bare 2-wire piezo element has no onboard oscillator. It needs a
//   square-wave AC drive to make sound, which tone() produces for you. That
//   means pitch is commanded by the sketch: SWEEP actually glides up in
//   frequency, ALARM warbles between two distinct tones, etc.
//
//   The two leads are not polarized — orientation doesn't matter. If your
//   element has a "+" mark or one lead is colored, that's just a convenience
//   marking for layout, not a polarity requirement.
//
//   Self-test for "is it really passive?": briefly touch the two leads
//   across the Uno's 5V and GND pins (no Arduino program running). If you
//   hear a click but no sustained tone, it's passive. If it buzzes steadily,
//   it's an active buzzer in disguise — the sketch still works, you'll just
//   hear a fixed pitch instead of the commanded one.
//
// CURRENT DRAW — small piezo elements pull <30 mA peak, well inside the Uno
// pin's 40 mA absolute max (20 mA continuous recommended). No transistor
// driver is needed for direct bench testing.
//
// WIRING (2-wire passive piezo):
//
//       Uno D8  --------------------o (lead 1) piezo
//
//       Uno GND --------------------o (lead 2) piezo
//
//   - Either lead can go to D8; the other goes to GND. Not polarized.
//   - No external resistor required.
//   - If you want a louder element later (magnetic buzzer, sounder), add a
//     2N2222 transistor between D8 and the buzzer — but for a small piezo,
//     direct drive is fine.
//
// PIN CHOICE NOTE — the planned TestStandFirmware config block reserves D11
// for the buzzer (PIN_BUZZER = 11). That pin is PWM-capable, which lets the
// real firmware use analogWrite() volume tricks if needed. This bench test
// uses D8 instead so you can run it alongside the LED test (which uses
// D3/D5/D6/D9/D10) without contention. Re-wire to D11 and update PIN_BUZZER
// below when integrating with the test stand.
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_BUZZER = 8;

const unsigned long BEEP_SHORT_MS    = 100;
const unsigned int  BEEP_SHORT_HZ    = 2000;

const unsigned long BEEP_LONG_MS     = 600;
const unsigned int  BEEP_LONG_HZ     = 1000;

const unsigned int  SWEEP_START_HZ   = 500;
const unsigned int  SWEEP_END_HZ     = 4000;
const unsigned int  SWEEP_STEPS      = 60;
const unsigned long SWEEP_STEP_MS    = 18;

const unsigned long ALARM_TONE_MS    = 150;
const unsigned int  ALARM_LO_HZ      = 800;
const unsigned int  ALARM_HI_HZ      = 1200;
const uint8_t       ALARM_CYCLES     = 6;

const unsigned long PATTERN_GAP_MS   = 600;

// ---------- PATTERNS ----------
static void beepShort() {
  Serial.println(F("# pattern: BEEP_SHORT"));
  tone(PIN_BUZZER, BEEP_SHORT_HZ, BEEP_SHORT_MS);
  delay(BEEP_SHORT_MS + 20);
  noTone(PIN_BUZZER);
}

static void beepLong() {
  Serial.println(F("# pattern: BEEP_LONG"));
  tone(PIN_BUZZER, BEEP_LONG_HZ, BEEP_LONG_MS);
  delay(BEEP_LONG_MS + 20);
  noTone(PIN_BUZZER);
}

static void sweep() {
  Serial.println(F("# pattern: SWEEP"));
  const float span = (float)SWEEP_END_HZ - (float)SWEEP_START_HZ;
  for (unsigned int i = 0; i <= SWEEP_STEPS; ++i) {
    const unsigned int hz =
        (unsigned int)((float)SWEEP_START_HZ + (span * (float)i) / (float)SWEEP_STEPS);
    tone(PIN_BUZZER, hz);
    delay(SWEEP_STEP_MS);
  }
  noTone(PIN_BUZZER);
}

static void alarm() {
  Serial.println(F("# pattern: ALARM"));
  for (uint8_t c = 0; c < ALARM_CYCLES; ++c) {
    tone(PIN_BUZZER, ALARM_LO_HZ, ALARM_TONE_MS);
    delay(ALARM_TONE_MS + 10);
    tone(PIN_BUZZER, ALARM_HI_HZ, ALARM_TONE_MS);
    delay(ALARM_TONE_MS + 10);
  }
  noTone(PIN_BUZZER);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("# piezo buzzer test ready"));
  Serial.print  (F("# pin=D")); Serial.println(PIN_BUZZER);
  Serial.println(F("# active piezo: tone changes won't affect pitch (fixed by element)"));
  Serial.println(F("# passive piezo: should produce distinct pitches in SWEEP and ALARM"));
}

// ---------- LOOP ----------
void loop() {
  beepShort(); delay(PATTERN_GAP_MS);
  beepLong();  delay(PATTERN_GAP_MS);
  sweep();     delay(PATTERN_GAP_MS);
  alarm();     delay(PATTERN_GAP_MS * 2);
}
