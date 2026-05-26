// ============================================================================
// PetrChu — MOSFET + pushbutton test, low-side N-channel switch (Arduino Uno)
// ----------------------------------------------------------------------------
// Drives a logic-level N-channel MOSFET as a low-side switch under control of
// a momentary pushbutton. Each press advances through four states so you can
// verify both digital switching and PWM gate drive in one bench session:
//
//   0. OFF        — gate held low, load fully disconnected
//   1. ON         — gate held high (full enhancement), load at full current
//   2. PWM_MID    — gate driven at ~50% duty (analogWrite 128)
//   3. PWM_LOW    — gate driven at ~10% duty (analogWrite 25)
//
// State wraps back to OFF on the next press. The Uno's onboard LED (D13)
// mirrors the gate output as a visual cue. Serial prints every transition so
// you can correlate scope traces or multimeter readings with state changes.
//
// MOSFET ASSUMPTIONS — this sketch assumes a LOGIC-LEVEL N-channel part
// (e.g. IRLZ44N, IRL540, IRLB8721, AOI518). Standard-level MOSFETs like the
// IRF540 need ~10 V Vgs to fully enhance and will run hot at 5 V gate drive —
// the part will still switch, but Rds(on) will be far above datasheet spec
// and the device may dissipate enough power to need a heatsink even at modest
// load currents. If you only have a standard-level part on the bench, use a
// gate driver IC or a small NPN level-shifter; do not just push it harder.
//
// WHY THE GATE PULL-DOWN — MOSFET gates are pure capacitance. If the Uno pin
// floats (boot, reset, sketch upload, wire popped loose), residual gate charge
// can leave the MOSFET partly-on, dissipating power and possibly damaging the
// load. A 10k pull-down from gate to GND drains that charge in microseconds
// and forces a defined OFF state any time the Uno isn't actively driving the
// gate high.
//
// WHY THE GATE SERIES RESISTOR — at the instant the Uno pin transitions, the
// MOSFET gate looks like a short to whatever's on the cap. A 220 ohm series
// resistor limits the inrush current to ~25 mA (5 V / 220), keeping the Uno
// pin inside its 40 mA absolute-max rating. It also damps any L-C ringing
// between pin trace and gate cap, which on a breadboard can otherwise cause
// the MOSFET to oscillate during the switching edge.
//
// FLYBACK DIODE — only required for INDUCTIVE loads (relay coils, solenoids,
// motor windings, the air-motor servo on the test stand). For a pure
// resistive load (light bulb, resistor, LED bank), skip it. For inductive
// loads use a 1N4007 or similar across the load with cathode (band) toward
// the supply (+) side. Without it, turn-off inductive kickback spikes the
// drain and will eventually punch through the MOSFET's avalanche rating.
//
// WIRING (low-side N-channel switch):
//
//                              +V_LOAD (external supply +)
//                                  |
//                                 [LOAD]            (with flyback diode if inductive:
//                                  |                 1N4007 across load, cathode->+V)
//                                  | drain
//                            +-----+
//                            |     |
//   Uno D9 ---[220 ohm]------+ G   |   (N-channel logic-level MOSFET, TO-220)
//                            |     |
//                            +--+--+
//                               |  source
//                              GND  (common with Uno GND AND load supply GND)
//                               |
//   Uno D9 ---[10 kOhm]-------- GND   (gate pull-down — connect at the gate
//                                      side of the 220 R, i.e. directly on
//                                      the MOSFET gate pin)
//
//   Uno D2 ----------+
//                    |
//                   [BTN]   (momentary pushbutton, normally open)
//                    |
//                   GND
//
//   - Uno GND and the external load supply GND MUST be tied together. The
//     MOSFET source is the reference point for Vgs; if grounds aren't common,
//     the gate sees an undefined voltage relative to source and switching
//     becomes unpredictable.
//   - Button uses INPUT_PULLUP — no external resistor needed. Press pulls
//     D2 to LOW.
//   - For a first sanity check, you can use a single LED + 120 ohm resistor
//     as the "load" — it'll be visibly bright in ON state and visibly dimmer
//     in the PWM states.
//
// PIN CHOICE — D9 is PWM-capable on the Uno, which is required for PWM_MID /
// PWM_LOW states. D2 is interrupt-capable (INT0), reserved here in case you
// want to convert button handling to an ISR later; this sketch polls it.
// Avoid D11 (planned PIN_BUZZER in TestStandFirmware) and the D3/5/6/9/10
// span if you want to run this alongside the LED test — D9 is shared with
// LED 3 in that test, so don't run both on the same Uno without re-pinning.
//
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_MOSFET   = 9;     // PWM-capable. Drives gate via 220 R.
const uint8_t  PIN_BUTTON   = 2;     // INPUT_PULLUP, active LOW.
const uint8_t  PIN_LED_MIRROR = LED_BUILTIN;  // D13 — visual state cue.

const unsigned long DEBOUNCE_MS    = 30;   // settling time after edge
const unsigned long HEARTBEAT_MS   = 2000; // periodic "still alive" print

// PWM duties for the two PWM states. analogWrite range is 0..255.
const uint8_t PWM_MID_DUTY = 128;  // ~50%
const uint8_t PWM_LOW_DUTY = 25;   // ~10%

// ---------- STATE MACHINE ----------
enum MosfetState : uint8_t {
  STATE_OFF      = 0,
  STATE_ON       = 1,
  STATE_PWM_MID  = 2,
  STATE_PWM_LOW  = 3,
  STATE_COUNT    = 4
};

static MosfetState    g_state           = STATE_OFF;
static uint8_t        g_button_stable   = HIGH;   // pulled up when idle
static uint8_t        g_button_last_raw = HIGH;
static unsigned long  g_button_edge_ms  = 0;
static unsigned long  g_last_heartbeat  = 0;

// ---------- HELPERS ----------
static const __FlashStringHelper* stateName(MosfetState s) {
  switch (s) {
    case STATE_OFF:     return F("OFF");
    case STATE_ON:      return F("ON");
    case STATE_PWM_MID: return F("PWM_MID (~50%)");
    case STATE_PWM_LOW: return F("PWM_LOW (~10%)");
    default:            return F("?");
  }
}

static void applyState(MosfetState s) {
  switch (s) {
    case STATE_OFF:
      // analogWrite(0) leaves the pin in PWM mode; force a clean digital LOW
      // so the gate is unambiguously held down by the pin driver.
      digitalWrite(PIN_MOSFET, LOW);
      digitalWrite(PIN_LED_MIRROR, LOW);
      break;
    case STATE_ON:
      digitalWrite(PIN_MOSFET, HIGH);
      digitalWrite(PIN_LED_MIRROR, HIGH);
      break;
    case STATE_PWM_MID:
      analogWrite(PIN_MOSFET, PWM_MID_DUTY);
      analogWrite(PIN_LED_MIRROR, PWM_MID_DUTY);
      break;
    case STATE_PWM_LOW:
      analogWrite(PIN_MOSFET, PWM_LOW_DUTY);
      analogWrite(PIN_LED_MIRROR, PWM_LOW_DUTY);
      break;
    default: break;
  }
  Serial.print(F("# state -> "));
  Serial.println(stateName(s));
}

static void advanceState() {
  g_state = (MosfetState)((g_state + 1) % STATE_COUNT);
  applyState(g_state);
}

// Debounced edge detector. Returns true exactly once per accepted press
// (HIGH -> LOW transition after the line has been stable for DEBOUNCE_MS).
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
  return (g_button_stable == LOW);  // accepted falling edge = press
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);     // boot in OFF — gate held low

  pinMode(PIN_LED_MIRROR, OUTPUT);
  digitalWrite(PIN_LED_MIRROR, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.println(F("# mosfet test ready"));
  Serial.print  (F("# gate pin = D")); Serial.println(PIN_MOSFET);
  Serial.print  (F("# button pin = D")); Serial.println(PIN_BUTTON);
  Serial.println(F("# press button to cycle: OFF -> ON -> PWM_MID -> PWM_LOW -> OFF"));
  Serial.print  (F("# initial state: "));
  Serial.println(stateName(g_state));
}

// ---------- LOOP ----------
void loop() {
  if (buttonPressed()) advanceState();

  const unsigned long now = millis();
  if (now - g_last_heartbeat >= HEARTBEAT_MS) {
    g_last_heartbeat = now;
    Serial.print(F("# heartbeat — state="));
    Serial.println(stateName(g_state));
  }
}
